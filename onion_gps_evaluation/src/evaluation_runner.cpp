#include "onion_gps_evaluation/evaluation_runner.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <variant>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatStatus.h>

namespace onion_gps_evaluation {
namespace {

constexpr double kPi = 3.14159265358979323846;

class JsonValue {
 public:
  using Object = std::map<std::string, JsonValue>;
  using Array = std::vector<JsonValue>;

  JsonValue() : value_(nullptr) {}
  JsonValue(std::nullptr_t) : value_(nullptr) {}
  JsonValue(bool value) : value_(value) {}
  JsonValue(int value) : value_(static_cast<std::int64_t>(value)) {}
  JsonValue(std::size_t value)
      : value_(static_cast<std::int64_t>(value)) {}
  JsonValue(double value)
      : value_(std::isfinite(value)
                   ? Storage(value)
                   : Storage(static_cast<std::nullptr_t>(nullptr))) {}
  JsonValue(const char* value) : value_(std::string(value)) {}
  JsonValue(std::string value) : value_(std::move(value)) {}

  static JsonValue object() {
    JsonValue value;
    value.value_ = Object();
    return value;
  }

  static JsonValue array() {
    JsonValue value;
    value.value_ = Array();
    return value;
  }

  JsonValue& operator[](const std::string& key) {
    return std::get<Object>(value_)[key];
  }

  void pushBack(JsonValue value) {
    std::get<Array>(value_).push_back(std::move(value));
  }

  std::string dump(int indent = 2) const {
    std::ostringstream stream;
    write(stream, indent, 0);
    return stream.str();
  }

  void flatten(const std::string& prefix,
               std::vector<std::pair<std::string, std::string>>* output) const {
    if (const auto* object = std::get_if<Object>(&value_)) {
      for (const auto& [key, value] : *object) {
        const std::string path = prefix.empty() ? key : prefix + "." + key;
        value.flatten(path, output);
      }
      return;
    }
    if (const auto* array = std::get_if<Array>(&value_)) {
      for (std::size_t index = 0; index < array->size(); ++index) {
        (*array)[index].flatten(
            prefix + "[" + std::to_string(index) + "]", output);
      }
      return;
    }
    output->emplace_back(prefix, dump(-1));
  }

 private:
  using Storage =
      std::variant<std::nullptr_t, bool, std::int64_t, double, std::string,
                   Object, Array>;

  static std::string escaped(const std::string& input) {
    std::ostringstream stream;
    for (const unsigned char character : input) {
      switch (character) {
        case '"':
          stream << "\\\"";
          break;
        case '\\':
          stream << "\\\\";
          break;
        case '\b':
          stream << "\\b";
          break;
        case '\f':
          stream << "\\f";
          break;
        case '\n':
          stream << "\\n";
          break;
        case '\r':
          stream << "\\r";
          break;
        case '\t':
          stream << "\\t";
          break;
        default:
          if (character < 0x20U) {
            stream << "\\u" << std::hex << std::setw(4)
                   << std::setfill('0') << static_cast<int>(character)
                   << std::dec;
          } else {
            stream << static_cast<char>(character);
          }
      }
    }
    return stream.str();
  }

  void writeIndent(std::ostream& stream, int count) const {
    for (int index = 0; index < count; ++index) {
      stream.put(' ');
    }
  }

  void write(std::ostream& stream, int indent, int level) const {
    if (std::holds_alternative<std::nullptr_t>(value_)) {
      stream << "null";
    } else if (const auto* value = std::get_if<bool>(&value_)) {
      stream << (*value ? "true" : "false");
    } else if (const auto* value = std::get_if<std::int64_t>(&value_)) {
      stream << *value;
    } else if (const auto* value = std::get_if<double>(&value_)) {
      stream << std::setprecision(15) << *value;
    } else if (const auto* value = std::get_if<std::string>(&value_)) {
      stream << '"' << escaped(*value) << '"';
    } else if (const auto* object = std::get_if<Object>(&value_)) {
      stream << '{';
      bool first = true;
      for (const auto& [key, value] : *object) {
        if (!first) {
          stream << ',';
        }
        if (indent >= 0) {
          stream << '\n';
          writeIndent(stream, (level + 1) * indent);
        }
        stream << '"' << escaped(key) << '"' << ':';
        if (indent >= 0) {
          stream << ' ';
        }
        value.write(stream, indent, level + 1);
        first = false;
      }
      if (!object->empty() && indent >= 0) {
        stream << '\n';
        writeIndent(stream, level * indent);
      }
      stream << '}';
    } else {
      const Array& array = std::get<Array>(value_);
      stream << '[';
      for (std::size_t index = 0; index < array.size(); ++index) {
        if (index != 0U) {
          stream << ',';
        }
        if (indent >= 0) {
          stream << '\n';
          writeIndent(stream, (level + 1) * indent);
        }
        array[index].write(stream, indent, level + 1);
      }
      if (!array.empty() && indent >= 0) {
        stream << '\n';
        writeIndent(stream, level * indent);
      }
      stream << ']';
    }
  }

  Storage value_;
};

struct RelocalizationResult {
  std::size_t event_index = 0;
  std::string event_source;
  double event_stamp_sec = 0.0;
  std::string event_frame_id;
  double first_onion_pose_delay_sec = kNaN;
  double initial_horizontal_error_m = kNaN;
  double initial_error_3d_m = kNaN;
  double initial_yaw_error_deg = kNaN;
  bool converged = false;
  double convergence_time_sec = kNaN;
  double convergence_threshold_m = 0.0;
  int convergence_hold_samples = 0;
  Distribution post_convergence_horizontal_error_m;
};

double messageStamp(const std_msgs::Header& header) {
  return header.stamp.isZero() ? ros::Time::now().toSec()
                               : header.stamp.toSec();
}

std::string generatedUtc() {
  const std::time_t current = std::time(nullptr);
  std::tm utc_time{};
#ifdef _WIN32
  gmtime_s(&utc_time, &current);
#else
  gmtime_r(&current, &utc_time);
#endif
  std::ostringstream stream;
  stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

std::string expandUserPath(const std::string& path) {
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char* home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }
  if (path.size() == 1U) {
    return std::string(home);
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}

std::string absolutePath(const std::string& path) {
  if (path.empty()) {
    return "";
  }
  return std::filesystem::absolute(expandUserPath(path)).lexically_normal()
      .string();
}

void writeAtomically(const std::string& path, const std::string& content) {
  const std::filesystem::path target(path);
  if (!target.parent_path().empty()) {
    std::filesystem::create_directories(target.parent_path());
  }
  const std::filesystem::path temporary = target.string() + ".tmp";
  {
    std::ofstream stream(temporary);
    if (!stream) {
      throw std::runtime_error("cannot open output file: " +
                               temporary.string());
    }
    stream << content;
    if (!stream.good()) {
      throw std::runtime_error("failed while writing: " +
                               temporary.string());
    }
  }
  std::error_code error;
  std::filesystem::remove(target, error);
  error.clear();
  std::filesystem::rename(temporary, target, error);
  if (error) {
    throw std::runtime_error("cannot replace output file " + target.string() +
                             ": " + error.message());
  }
}

std::string csvCell(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped = value;
  std::size_t position = 0;
  while ((position = escaped.find('"', position)) != std::string::npos) {
    escaped.insert(position, 1U, '"');
    position += 2U;
  }
  return '"' + escaped + '"';
}

std::string numberCell(double value) {
  if (!std::isfinite(value)) {
    return "";
  }
  std::ostringstream stream;
  stream << std::setprecision(15) << value;
  return stream.str();
}

JsonValue distributionJson(const Distribution& distribution) {
  JsonValue result = JsonValue::object();
  result["count"] = distribution.count;
  if (distribution.count > 0U) {
    result["min"] = distribution.min;
    result["mean"] = distribution.mean;
    result["median"] = distribution.median;
    result["p95"] = distribution.p95;
    result["max"] = distribution.max;
    result["rmse"] = distribution.rmse;
  }
  return result;
}

JsonValue transformJson(const PlanarTransform& transform) {
  JsonValue result = JsonValue::object();
  result["yaw_rad"] = transform.yaw;
  result["yaw_deg"] = transform.yaw * 180.0 / kPi;
  result["translation_x_m"] = transform.tx;
  result["translation_y_m"] = transform.ty;
  result["translation_z_m"] = transform.tz;
  return result;
}

JsonValue registrationJson(const AlignmentResult& result,
                           const std::vector<MatchedPose>& matches,
                           double outlier_threshold_m) {
  JsonValue registration = JsonValue::object();
  registration["matched_pose_count"] = matches.size();
  registration["inlier_count"] = static_cast<std::size_t>(std::count(
      result.inlier_mask.begin(), result.inlier_mask.end(),
      static_cast<std::uint8_t>(1U)));
  registration["horizontal_rmse_m"] = result.horizontal_rmse_m;
  registration["source_baseline_m"] = result.source_baseline_m;
  registration["start_stamp_sec"] = matches.front().estimate.stamp;
  registration["end_stamp_sec"] = matches.back().estimate.stamp;
  registration["outlier_threshold_m"] = outlier_threshold_m;
  return registration;
}

JsonValue evaluationMetricsJson(const std::vector<ErrorRow>& rows,
                                std::size_t matched_count,
                                std::size_t estimate_count,
                                std::size_t gps_count,
                                const std::vector<double>& thresholds_m,
                                const RpeResult& rpe) {
  std::vector<double> time_differences;
  std::vector<double> horizontal;
  std::vector<double> vertical;
  std::vector<double> error_3d;
  std::vector<double> yaw;
  std::size_t geodetic_count = 0U;
  for (const ErrorRow& row : rows) {
    time_differences.push_back(row.time_difference_sec);
    horizontal.push_back(row.horizontal_error_m);
    vertical.push_back(row.vertical_error_m);
    error_3d.push_back(row.error_3d_m);
    yaw.push_back(row.yaw_error_deg);
    if (std::isfinite(row.latitude_deg) &&
        std::isfinite(row.longitude_deg)) {
      ++geodetic_count;
    }
  }
  JsonValue result = JsonValue::object();
  result["performed"] = true;
  result["onion_pose_count"] = estimate_count;
  result["gps_pose_count"] = gps_count;
  result["matched_pose_count"] = matched_count;
  result["matched_geodetic_pose_count"] = geodetic_count;
  result["match_availability"] =
      estimate_count > 0U
          ? static_cast<double>(matched_count) /
                static_cast<double>(estimate_count)
          : 0.0;
  result["time_difference_sec"] =
      distributionJson(calculateDistribution(time_differences));
  result["horizontal_error_m"] =
      distributionJson(calculateDistribution(horizontal));
  result["vertical_error_m"] =
      distributionJson(calculateDistribution(vertical));
  result["error_3d_m"] =
      distributionJson(calculateDistribution(error_3d));
  result["yaw_error_deg"] =
      distributionJson(calculateDistribution(yaw));
  JsonValue pass_rates = JsonValue::object();
  for (double threshold : thresholds_m) {
    std::ostringstream key;
    key << std::fixed << std::setprecision(3) << threshold;
    const std::size_t passing = static_cast<std::size_t>(
        std::count_if(horizontal.begin(), horizontal.end(),
                      [threshold](double error) {
                        return error <= threshold;
                      }));
    pass_rates[key.str()] =
        horizontal.empty()
            ? 0.0
            : static_cast<double>(passing) /
                  static_cast<double>(horizontal.size());
  }
  result["horizontal_threshold_pass_rate"] = std::move(pass_rates);
  JsonValue rpe_json = JsonValue::object();
  rpe_json["delta_sec"] = rpe.delta_sec;
  rpe_json["translation_error_m"] =
      distributionJson(rpe.translation_error_m);
  rpe_json["yaw_error_deg"] = distributionJson(rpe.yaw_error_deg);
  result["rpe"] = std::move(rpe_json);
  return result;
}

std::string matchedTrajectoryCsv(const std::vector<ErrorRow>& rows) {
  std::ostringstream stream;
  stream
      << "phase,stamp_sec,time_difference_sec,latitude_deg,longitude_deg,"
         "altitude_m,gps_x,gps_y,gps_z,gps_yaw_rad,onion_x,onion_y,"
         "onion_z,onion_yaw_rad,aligned_onion_x,aligned_onion_y,"
         "aligned_onion_z,aligned_onion_yaw_rad,horizontal_error_m,"
         "vertical_error_m,error_3d_m,yaw_error_deg\n";
  for (const ErrorRow& row : rows) {
    stream << csvCell(row.phase) << ',' << numberCell(row.stamp_sec) << ','
           << numberCell(row.time_difference_sec) << ','
           << numberCell(row.latitude_deg) << ','
           << numberCell(row.longitude_deg) << ','
           << numberCell(row.altitude_m) << ',' << numberCell(row.gps_x)
           << ',' << numberCell(row.gps_y) << ',' << numberCell(row.gps_z)
           << ',' << numberCell(row.gps_yaw_rad) << ','
           << numberCell(row.onion_x) << ',' << numberCell(row.onion_y)
           << ',' << numberCell(row.onion_z) << ','
           << numberCell(row.onion_yaw_rad) << ','
           << numberCell(row.aligned_onion_x) << ','
           << numberCell(row.aligned_onion_y) << ','
           << numberCell(row.aligned_onion_z) << ','
           << numberCell(row.aligned_onion_yaw_rad) << ','
           << numberCell(row.horizontal_error_m) << ','
           << numberCell(row.vertical_error_m) << ','
           << numberCell(row.error_3d_m) << ','
           << numberCell(row.yaw_error_deg) << '\n';
  }
  return stream.str();
}

std::string relocalizationCsv(
    const std::vector<RelocalizationResult>& results) {
  std::ostringstream stream;
  stream
      << "event_index,event_source,event_stamp_sec,event_frame_id,"
         "first_onion_pose_delay_sec,initial_horizontal_error_m,"
         "initial_error_3d_m,initial_yaw_error_deg,converged,"
         "convergence_time_sec,convergence_threshold_m,"
         "convergence_hold_samples\n";
  for (const RelocalizationResult& result : results) {
    stream << result.event_index << ',' << csvCell(result.event_source)
           << ',' << numberCell(result.event_stamp_sec) << ','
           << csvCell(result.event_frame_id) << ','
           << numberCell(result.first_onion_pose_delay_sec) << ','
           << numberCell(result.initial_horizontal_error_m) << ','
           << numberCell(result.initial_error_3d_m) << ','
           << numberCell(result.initial_yaw_error_deg) << ','
           << (result.converged ? "true" : "false") << ','
           << numberCell(result.convergence_time_sec) << ','
           << numberCell(result.convergence_threshold_m) << ','
           << result.convergence_hold_samples << '\n';
  }
  return stream.str();
}

std::string statusCsv(
    const std::vector<EvaluationRunner::StatusEvent>& events) {
  std::ostringstream stream;
  stream << "stamp_sec,status\n";
  for (const auto& event : events) {
    stream << numberCell(event.stamp) << ',' << csvCell(event.status)
           << '\n';
  }
  return stream.str();
}

std::vector<RelocalizationResult> evaluateRelocalization(
    std::vector<EvaluationRunner::RelocalizationEvent> events,
    const std::vector<MatchedPose>& matches,
    const std::vector<ErrorRow>& rows,
    const std::vector<PoseSample>& references,
    const PlanarTransform& transform, double max_time_difference_sec,
    double max_interpolation_gap_sec, double convergence_threshold_m,
    int convergence_hold_samples, double relocalization_timeout_sec) {
  if (matches.empty()) {
    return {};
  }
  const double evaluation_start = matches.front().estimate.stamp;
  const double evaluation_end = matches.back().estimate.stamp;
  events.erase(
      std::remove_if(events.begin(), events.end(),
                     [&](const auto& event) {
                       return event.stamp < evaluation_start ||
                              event.stamp > evaluation_end;
                     }),
      events.end());
  if (events.empty()) {
    EvaluationRunner::RelocalizationEvent startup;
    startup.stamp = evaluation_start;
    startup.pose = matches.front().estimate;
    startup.frame_id = matches.front().estimate.source;
    startup.source = "evaluation_start";
    events.push_back(startup);
  }
  std::sort(events.begin(), events.end(),
            [](const auto& left, const auto& right) {
              return left.stamp < right.stamp;
            });

  std::vector<ErrorRow> ordered_rows = rows;
  std::sort(ordered_rows.begin(), ordered_rows.end(),
            [](const ErrorRow& left, const ErrorRow& right) {
              return left.stamp_sec < right.stamp_sec;
            });
  std::vector<RelocalizationResult> results;
  for (std::size_t index = 0; index < events.size(); ++index) {
    const auto& event = events[index];
    const double next_stamp =
        index + 1U < events.size()
            ? events[index + 1U].stamp
            : std::numeric_limits<double>::infinity();
    std::vector<ErrorRow> episode_rows;
    std::copy_if(ordered_rows.begin(), ordered_rows.end(),
                 std::back_inserter(episode_rows),
                 [&](const ErrorRow& row) {
                   return row.stamp_sec >= event.stamp &&
                          row.stamp_sec < next_stamp;
                 });

    RelocalizationResult result;
    result.event_index = index;
    result.event_source = event.source;
    result.event_stamp_sec = event.stamp;
    result.event_frame_id = event.frame_id;
    result.convergence_threshold_m = convergence_threshold_m;
    result.convergence_hold_samples = convergence_hold_samples;
    if (!episode_rows.empty()) {
      result.first_onion_pose_delay_sec =
          episode_rows.front().stamp_sec - event.stamp;
    }

    const std::vector<PoseSample> event_pose{event.pose};
    const std::vector<MatchedPose> event_match =
        associateTrajectories(event_pose, references,
                              max_time_difference_sec,
                              max_interpolation_gap_sec);
    if (!event_match.empty()) {
      const PoseSample aligned = transform.apply(event.pose);
      const PoseSample& reference = event_match.front().reference;
      const double dx = aligned.x - reference.x;
      const double dy = aligned.y - reference.y;
      const double dz = aligned.z - reference.z;
      result.initial_horizontal_error_m = std::hypot(dx, dy);
      result.initial_error_3d_m =
          std::sqrt(dx * dx + dy * dy + dz * dz);
      if (std::isfinite(aligned.yaw) &&
          std::isfinite(reference.yaw)) {
        result.initial_yaw_error_deg =
            std::abs(wrapAngle(aligned.yaw - reference.yaw)) * 180.0 /
            kPi;
      }
    }

    const std::size_t hold =
        static_cast<std::size_t>(std::max(1, convergence_hold_samples));
    std::optional<std::size_t> convergence_index;
    if (episode_rows.size() >= hold) {
      for (std::size_t row_index = 0;
           row_index + hold <= episode_rows.size(); ++row_index) {
        if (episode_rows[row_index + hold - 1U].stamp_sec - event.stamp >
            relocalization_timeout_sec) {
          break;
        }
        const bool stable = std::all_of(
            episode_rows.begin() + row_index,
            episode_rows.begin() + row_index + hold,
            [&](const ErrorRow& row) {
              return row.horizontal_error_m <= convergence_threshold_m;
            });
        if (stable) {
          convergence_index = row_index;
          break;
        }
      }
    }
    result.converged = convergence_index.has_value();
    if (convergence_index.has_value()) {
      result.convergence_time_sec =
          episode_rows[*convergence_index].stamp_sec - event.stamp;
      std::vector<double> post_errors;
      for (std::size_t row_index = *convergence_index;
           row_index < episode_rows.size(); ++row_index) {
        post_errors.push_back(
            episode_rows[row_index].horizontal_error_m);
      }
      result.post_convergence_horizontal_error_m =
          calculateDistribution(post_errors);
    }
    results.push_back(result);
  }
  return results;
}

JsonValue relocalizationJson(
    const std::vector<RelocalizationResult>& results) {
  JsonValue output = JsonValue::object();
  const std::size_t successes = static_cast<std::size_t>(std::count_if(
      results.begin(), results.end(),
      [](const RelocalizationResult& result) {
        return result.converged;
      }));
  output["event_count"] = results.size();
  output["successful_event_count"] = successes;
  output["success_rate"] =
      results.empty()
          ? 0.0
          : static_cast<double>(successes) /
                static_cast<double>(results.size());
  JsonValue events = JsonValue::array();
  for (const RelocalizationResult& result : results) {
    JsonValue event = JsonValue::object();
    event["event_index"] = result.event_index;
    event["event_source"] = result.event_source;
    event["event_stamp_sec"] = result.event_stamp_sec;
    event["event_frame_id"] = result.event_frame_id;
    event["first_onion_pose_delay_sec"] =
        result.first_onion_pose_delay_sec;
    event["initial_horizontal_error_m"] =
        result.initial_horizontal_error_m;
    event["initial_error_3d_m"] = result.initial_error_3d_m;
    event["initial_yaw_error_deg"] = result.initial_yaw_error_deg;
    event["converged"] = result.converged;
    event["convergence_time_sec"] = result.convergence_time_sec;
    event["convergence_threshold_m"] =
        result.convergence_threshold_m;
    event["convergence_hold_samples"] =
        result.convergence_hold_samples;
    event["post_convergence_horizontal_error_m"] =
        distributionJson(
            result.post_convergence_horizontal_error_m);
    events.pushBack(std::move(event));
  }
  output["events"] = std::move(events);
  return output;
}

}  // namespace

EvaluationRunner::EvaluationRunner(ros::NodeHandle node_handle,
                                   ros::NodeHandle private_node_handle,
                                   EvaluationWorkflow workflow)
    : node_handle_(std::move(node_handle)),
      private_node_handle_(std::move(private_node_handle)),
      workflow_(workflow) {
  switch (workflow_) {
    case EvaluationWorkflow::kTrajectoryRegistration:
      workflow_name_ = "trajectory_registration";
      break;
    case EvaluationWorkflow::kLocalizationEvaluation:
      workflow_name_ = "localization_accuracy_evaluation";
      break;
    case EvaluationWorkflow::kSegmentedRegistrationEvaluation:
      workflow_name_ = "segmented_registration_evaluation";
      break;
  }
  loadParameters();
  validateParameters();
  if (workflow_ == EvaluationWorkflow::kLocalizationEvaluation) {
    loadAlignment();
  }
  setUpRosInterfaces();
  ROS_INFO_STREAM("Onion GPS workflow started: " << workflow_name_
                  << " output=" << output_directory_);
}

EvaluationRunner::~EvaluationRunner() {
  if (!finalized_ && !finalizing_ && !onion_samples_.empty()) {
    std::string ignored;
    finalize(&ignored);
  }
}

void EvaluationRunner::loadParameters() {
  private_node_handle_.param<std::string>(
      "gps_position_source", gps_position_source_, "odom_utm");
  private_node_handle_.param<std::string>(
      "onion_odom_topic", onion_odom_topic_,
      "/onion_lo_plus_node/odometry");
  private_node_handle_.param<std::string>(
      "gps_odom_topic", gps_odom_topic_, "/gps/evaluation/odom_utm");
  private_node_handle_.param<std::string>(
      "gps_fix_topic", gps_fix_topic_, "/gps/evaluation/fix");
  private_node_handle_.param<std::string>(
      "gps_position_valid_topic", gps_position_valid_topic_,
      "/gps/evaluation/position_valid");
  private_node_handle_.param<std::string>(
      "gps_heading_valid_topic", gps_heading_valid_topic_,
      "/gps/evaluation/heading_valid");
  private_node_handle_.param<std::string>(
      "initialpose_topic", initialpose_topic_, "/initialpose");
  private_node_handle_.param<std::string>(
      "relocalization_status_topic", relocalization_status_topic_,
      "/onion_scancontext_relocalization/status");
  private_node_handle_.param(
      "require_gps_position_valid", require_gps_position_valid_, true);
  private_node_handle_.param(
      "require_gps_heading_valid_for_yaw",
      require_gps_heading_valid_for_yaw_, true);
  private_node_handle_.param("max_time_difference_sec",
                             max_time_difference_sec_, 0.15);
  private_node_handle_.param("max_interpolation_gap_sec",
                             max_interpolation_gap_sec_, 0.30);
  private_node_handle_.param("minimum_matched_poses",
                             minimum_matched_poses_, 30);
  private_node_handle_.param("minimum_registration_poses",
                             minimum_registration_poses_, 20);
  private_node_handle_.param("minimum_registration_baseline_m",
                             minimum_registration_baseline_m_, 5.0);
  private_node_handle_.param("registration_fraction",
                             registration_fraction_, 0.30);
  private_node_handle_.param("registration_duration_sec",
                             registration_duration_sec_, 0.0);
  private_node_handle_.param("robust_outlier_threshold_m",
                             robust_outlier_threshold_m_, 0.75);
  private_node_handle_.param("robust_ransac_iterations",
                             robust_ransac_iterations_, 300);
  private_node_handle_.param("random_seed", random_seed_, 7);
  private_node_handle_.param("rpe_delta_sec", rpe_delta_sec_, 1.0);
  private_node_handle_.param("convergence_threshold_m",
                             convergence_threshold_m_, 0.20);
  private_node_handle_.param("convergence_hold_samples",
                             convergence_hold_samples_, 10);
  private_node_handle_.param("relocalization_timeout_sec",
                             relocalization_timeout_sec_, 10.0);
  private_node_handle_.param("shutdown_after_finalize",
                             shutdown_after_finalize_, false);

  accuracy_thresholds_m_ = {0.05, 0.10, 0.20};
  private_node_handle_.getParam("accuracy_thresholds_m",
                                accuracy_thresholds_m_);

  std::string output_directory;
  private_node_handle_.param<std::string>("output_directory",
                                          output_directory, "");
  if (output_directory.empty()) {
    output_directory =
        "/tmp/onion_gps_evaluation_" + workflow_name_;
  }
  output_directory_ = absolutePath(output_directory);

  std::string alignment_input;
  private_node_handle_.param<std::string>("alignment_input_path",
                                          alignment_input, "");
  alignment_input_path_ = absolutePath(alignment_input);
  std::string alignment_output;
  private_node_handle_.param<std::string>("alignment_output_path",
                                          alignment_output, "");
  alignment_output_path_ =
      alignment_output.empty()
          ? (std::filesystem::path(output_directory_) /
             "map_to_gps_alignment.json")
                .string()
          : absolutePath(alignment_output);

  double reference_latitude = kNaN;
  double reference_longitude = kNaN;
  double reference_altitude = kNaN;
  private_node_handle_.getParam("navsat_reference_latitude_deg",
                                reference_latitude);
  private_node_handle_.getParam("navsat_reference_longitude_deg",
                                reference_longitude);
  private_node_handle_.getParam("navsat_reference_altitude_m",
                                reference_altitude);
  if (std::isfinite(reference_latitude) &&
      std::isfinite(reference_longitude) &&
      std::isfinite(reference_altitude)) {
    navsat_reference_ =
        NavSatReference{reference_latitude, reference_longitude,
                        reference_altitude};
  }
}

void EvaluationRunner::validateParameters() const {
  if (gps_position_source_ != "odom_utm" &&
      gps_position_source_ != "navsat_fix") {
    throw std::invalid_argument(
        "gps_position_source must be odom_utm or navsat_fix");
  }
  if (max_time_difference_sec_ <= 0.0 ||
      max_interpolation_gap_sec_ <= 0.0) {
    throw std::invalid_argument(
        "time association thresholds must be positive");
  }
  if (minimum_matched_poses_ < 2 ||
      minimum_registration_poses_ < 2) {
    throw std::invalid_argument(
        "minimum pose counts must be at least two");
  }
  if (workflow_ ==
          EvaluationWorkflow::kSegmentedRegistrationEvaluation &&
      registration_duration_sec_ <= 0.0 &&
      (registration_fraction_ <= 0.0 || registration_fraction_ >= 1.0)) {
    throw std::invalid_argument(
        "registration_fraction must be within (0, 1)");
  }
  if (workflow_ == EvaluationWorkflow::kLocalizationEvaluation &&
      alignment_input_path_.empty()) {
    throw std::invalid_argument(
        "localization evaluation requires alignment_input_path");
  }
}

void EvaluationRunner::setUpRosInterfaces() {
  summary_publisher_ =
      private_node_handle_.advertise<std_msgs::String>("summary", 1, true);
  gps_path_publisher_ =
      private_node_handle_.advertise<nav_msgs::Path>("gps_path", 1, true);
  aligned_onion_path_publisher_ =
      private_node_handle_.advertise<nav_msgs::Path>(
          "aligned_onion_path", 1, true);

  onion_subscriber_ = node_handle_.subscribe(
      onion_odom_topic_, 1000, &EvaluationRunner::onionCallback, this);
  gps_odometry_subscriber_ = node_handle_.subscribe(
      gps_odom_topic_, 1000, &EvaluationRunner::gpsOdometryCallback, this);
  gps_fix_subscriber_ = node_handle_.subscribe(
      gps_fix_topic_, 1000, &EvaluationRunner::gpsFixCallback, this);
  gps_position_validity_subscriber_ = node_handle_.subscribe(
      gps_position_valid_topic_, 100,
      &EvaluationRunner::gpsPositionValidityCallback, this);
  gps_heading_validity_subscriber_ = node_handle_.subscribe(
      gps_heading_valid_topic_, 100,
      &EvaluationRunner::gpsHeadingValidityCallback, this);
  initialpose_subscriber_ = node_handle_.subscribe(
      initialpose_topic_, 100, &EvaluationRunner::initialPoseCallback,
      this);
  relocalization_status_subscriber_ = node_handle_.subscribe(
      relocalization_status_topic_, 100,
      &EvaluationRunner::relocalizationStatusCallback, this);
  finalize_service_ = private_node_handle_.advertiseService(
      "finalize", &EvaluationRunner::finalizeService, this);
}

void EvaluationRunner::loadAlignment() {
  boost::property_tree::ptree root;
  boost::property_tree::read_json(alignment_input_path_, root);
  const int schema_version = root.get<int>("schema_version");
  if (schema_version != 1 && schema_version != 2) {
    throw std::runtime_error("unsupported alignment schema version");
  }
  const std::string saved_source =
      root.get<std::string>("gps_position_source");
  if (saved_source != gps_position_source_) {
    throw std::runtime_error(
        "alignment GPS source differs from configured source");
  }
  loaded_transform_.yaw = root.get<double>("transform.yaw_rad");
  loaded_transform_.tx =
      root.get<double>("transform.translation_x_m");
  loaded_transform_.ty =
      root.get<double>("transform.translation_y_m");
  loaded_transform_.tz =
      root.get<double>("transform.translation_z_m", 0.0);
  const auto navsat_tree = root.get_child_optional("navsat_reference");
  if (navsat_tree.has_value()) {
    const auto latitude =
        navsat_tree->get_optional<double>("latitude_deg");
    const auto longitude =
        navsat_tree->get_optional<double>("longitude_deg");
    const auto altitude =
        navsat_tree->get_optional<double>("altitude_m");
    if (latitude.has_value() && longitude.has_value() &&
        altitude.has_value()) {
      navsat_reference_ = NavSatReference{
          *latitude, *longitude, *altitude};
    }
  }
  if (gps_position_source_ == "navsat_fix" &&
      !navsat_reference_.has_value()) {
    throw std::runtime_error(
        "navsat_fix alignment does not contain its ENU origin");
  }
  has_loaded_transform_ = true;
}

PoseSample EvaluationRunner::poseSampleFromOdometry(
    const nav_msgs::Odometry& message,
    const std::string& source) const {
  PoseSample sample;
  sample.stamp = messageStamp(message.header);
  sample.x = message.pose.pose.position.x;
  sample.y = message.pose.pose.position.y;
  sample.z = message.pose.pose.position.z;
  sample.yaw = quaternionToYaw(
      message.pose.pose.orientation.x, message.pose.pose.orientation.y,
      message.pose.pose.orientation.z, message.pose.pose.orientation.w);
  sample.source = source;
  return sample;
}

void EvaluationRunner::onionCallback(
    const nav_msgs::Odometry::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!finalized_) {
    onion_samples_.push_back(
        poseSampleFromOdometry(*message, message->header.frame_id));
  }
}

bool EvaluationRunner::gpsPositionAccepted() {
  if (!require_gps_position_valid_) {
    return true;
  }
  if (latest_gps_position_validity_ == ValidityState::kValid) {
    return true;
  }
  if (latest_gps_position_validity_ == ValidityState::kInvalid) {
    ++dropped_invalid_gps_count_;
  } else {
    ++dropped_before_position_validity_count_;
  }
  return false;
}

void EvaluationRunner::gpsOdometryCallback(
    const nav_msgs::Odometry::ConstPtr& message) {
  if (gps_position_source_ != "odom_utm") {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (finalized_ || !gpsPositionAccepted()) {
    return;
  }
  PoseSample sample =
      poseSampleFromOdometry(*message, message->header.frame_id);
  if (require_gps_heading_valid_for_yaw_ &&
      latest_gps_heading_validity_ != ValidityState::kValid) {
    sample.yaw = kNaN;
    if (latest_gps_heading_validity_ == ValidityState::kInvalid) {
      ++suppressed_invalid_gps_yaw_count_;
    } else {
      ++suppressed_before_heading_validity_count_;
    }
  }
  gps_odometry_samples_.push_back(sample);
}

void EvaluationRunner::gpsFixCallback(
    const sensor_msgs::NavSatFix::ConstPtr& message) {
  if (message->status.status < sensor_msgs::NavSatStatus::STATUS_FIX ||
      !std::isfinite(message->latitude) ||
      !std::isfinite(message->longitude) ||
      !std::isfinite(message->altitude)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (finalized_) {
    return;
  }
  if (gps_position_source_ == "navsat_fix" &&
      !gpsPositionAccepted()) {
    return;
  }
  if (gps_position_source_ == "odom_utm" &&
      require_gps_position_valid_ &&
      latest_gps_position_validity_ != ValidityState::kValid) {
    return;
  }
  if (!navsat_reference_.has_value()) {
    navsat_reference_ =
        NavSatReference{message->latitude, message->longitude,
                        message->altitude};
  }
  PoseSample sample =
      geodeticToEnu(message->latitude, message->longitude,
                    message->altitude,
                    navsat_reference_->latitude_deg,
                    navsat_reference_->longitude_deg,
                    navsat_reference_->altitude_m);
  sample.stamp = messageStamp(message->header);
  sample.latitude = message->latitude;
  sample.longitude = message->longitude;
  sample.altitude = message->altitude;
  sample.source = message->header.frame_id;
  gps_fix_samples_.push_back(sample);
}

void EvaluationRunner::gpsPositionValidityCallback(
    const std_msgs::Bool::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_gps_position_validity_ =
      message->data ? ValidityState::kValid : ValidityState::kInvalid;
}

void EvaluationRunner::gpsHeadingValidityCallback(
    const std_msgs::Bool::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_gps_heading_validity_ =
      message->data ? ValidityState::kValid : ValidityState::kInvalid;
}

void EvaluationRunner::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (finalized_) {
    return;
  }
  PoseSample sample;
  sample.stamp = messageStamp(message->header);
  sample.x = message->pose.pose.position.x;
  sample.y = message->pose.pose.position.y;
  sample.z = message->pose.pose.position.z;
  sample.yaw = quaternionToYaw(
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z,
      message->pose.pose.orientation.w);
  sample.source = message->header.frame_id;
  relocalization_events_.push_back(
      RelocalizationEvent{sample.stamp, sample,
                          message->header.frame_id, "initialpose"});
}

void EvaluationRunner::relocalizationStatusCallback(
    const std_msgs::String::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!finalized_) {
    status_events_.push_back(
        StatusEvent{ros::Time::now().toSec(), message->data});
  }
}

std::vector<PoseSample> EvaluationRunner::referenceSamples() const {
  std::vector<PoseSample> references =
      gps_position_source_ == "odom_utm" ? gps_odometry_samples_
                                         : gps_fix_samples_;
  std::sort(references.begin(), references.end(),
            [](const PoseSample& left, const PoseSample& right) {
              return left.stamp < right.stamp;
            });
  if (gps_position_source_ == "odom_utm") {
    attachNavSatCoordinates(&references);
  }
  return references;
}

void EvaluationRunner::attachNavSatCoordinates(
    std::vector<PoseSample>* references) const {
  if (references->empty() || gps_fix_samples_.empty()) {
    return;
  }
  const std::vector<MatchedPose> metadata_matches =
      associateTrajectories(*references, gps_fix_samples_,
                            max_time_difference_sec_,
                            max_interpolation_gap_sec_);
  std::size_t reference_index = 0U;
  for (const MatchedPose& match : metadata_matches) {
    while (reference_index < references->size() &&
           (*references)[reference_index].stamp <
               match.estimate.stamp - 1.0e-9) {
      ++reference_index;
    }
    if (reference_index >= references->size()) {
      break;
    }
    PoseSample& reference = (*references)[reference_index];
    if (std::abs(reference.stamp - match.estimate.stamp) <= 1.0e-9) {
      reference.latitude = match.reference.latitude;
      reference.longitude = match.reference.longitude;
      reference.altitude = match.reference.altitude;
    }
  }
}

bool EvaluationRunner::finalize(std::string* result_message) {
  std::vector<PoseSample> onion_samples;
  std::vector<PoseSample> references;
  std::vector<PoseSample> gps_fix_samples;
  std::vector<RelocalizationEvent> relocalization_events;
  std::vector<StatusEvent> status_events;
  std::size_t dropped_invalid = 0U;
  std::size_t dropped_before_validity = 0U;
  std::size_t suppressed_invalid_yaw = 0U;
  std::size_t suppressed_before_heading = 0U;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (finalized_) {
      *result_message = "report already finalized";
      return true;
    }
    if (finalizing_) {
      *result_message = "report finalization is already running";
      return false;
    }
    finalizing_ = true;
    onion_samples = onion_samples_;
    references = referenceSamples();
    gps_fix_samples = gps_fix_samples_;
    relocalization_events = relocalization_events_;
    status_events = status_events_;
    dropped_invalid = dropped_invalid_gps_count_;
    dropped_before_validity =
        dropped_before_position_validity_count_;
    suppressed_invalid_yaw = suppressed_invalid_gps_yaw_count_;
    suppressed_before_heading =
        suppressed_before_heading_validity_count_;
  }

  try {
    std::vector<MatchedPose> all_matches =
        associateTrajectories(onion_samples, references,
                              max_time_difference_sec_,
                              max_interpolation_gap_sec_);
    const std::size_t required_matches =
        workflow_ == EvaluationWorkflow::kTrajectoryRegistration
            ? static_cast<std::size_t>(minimum_registration_poses_)
            : static_cast<std::size_t>(minimum_matched_poses_);
    if (all_matches.size() < required_matches) {
      throw std::runtime_error(
          "not enough timestamp-matched poses: " +
          std::to_string(all_matches.size()) + " < " +
          std::to_string(required_matches));
    }

    std::vector<MatchedPose> registration_matches;
    std::vector<MatchedPose> evaluation_matches;
    std::optional<AlignmentResult> alignment_result;
    PlanarTransform transform;
    double registration_end_stamp = kNaN;

    if (workflow_ == EvaluationWorkflow::kLocalizationEvaluation) {
      if (!has_loaded_transform_) {
        throw std::runtime_error("alignment was not loaded");
      }
      transform = loaded_transform_;
      evaluation_matches = all_matches;
    } else if (workflow_ ==
               EvaluationWorkflow::kTrajectoryRegistration) {
      registration_matches = all_matches;
    } else {
      std::sort(all_matches.begin(), all_matches.end(),
                [](const MatchedPose& left, const MatchedPose& right) {
                  return left.estimate.stamp < right.estimate.stamp;
                });
      if (registration_duration_sec_ > 0.0) {
        const double limit =
            all_matches.front().estimate.stamp +
            registration_duration_sec_;
        std::copy_if(
            all_matches.begin(), all_matches.end(),
            std::back_inserter(registration_matches),
            [limit](const MatchedPose& match) {
              return match.estimate.stamp <= limit;
            });
      } else {
        const std::size_t requested = static_cast<std::size_t>(
            std::ceil(static_cast<double>(all_matches.size()) *
                      registration_fraction_));
        const std::size_t count = std::max(
            static_cast<std::size_t>(minimum_registration_poses_),
            requested);
        if (count >= all_matches.size()) {
          throw std::runtime_error(
              "registration segment leaves no evaluation poses");
        }
        registration_matches.assign(all_matches.begin(),
                                    all_matches.begin() + count);
      }
      if (registration_matches.size() <
          static_cast<std::size_t>(minimum_registration_poses_)) {
        throw std::runtime_error(
            "registration segment contains too few matched poses");
      }
      registration_end_stamp =
          registration_matches.back().estimate.stamp;
      std::copy_if(
          all_matches.begin(), all_matches.end(),
          std::back_inserter(evaluation_matches),
          [registration_end_stamp](const MatchedPose& match) {
            return match.estimate.stamp > registration_end_stamp;
          });
      if (evaluation_matches.size() <
          static_cast<std::size_t>(minimum_matched_poses_)) {
        throw std::runtime_error(
            "post-registration evaluation segment contains too few "
            "matched poses");
      }
    }

    if (workflow_ != EvaluationWorkflow::kLocalizationEvaluation) {
      std::vector<PoseSample> source;
      std::vector<PoseSample> target;
      source.reserve(registration_matches.size());
      target.reserve(registration_matches.size());
      for (const MatchedPose& match : registration_matches) {
        source.push_back(match.estimate);
        target.push_back(match.reference);
      }
      alignment_result = estimatePlanarAlignment(
          source, target,
          static_cast<std::size_t>(minimum_registration_poses_),
          minimum_registration_baseline_m_,
          robust_outlier_threshold_m_, robust_ransac_iterations_,
          static_cast<std::uint32_t>(random_seed_));
      transform = alignment_result->transform;
    }

    std::vector<ErrorRow> all_rows =
        buildErrorRows(all_matches, transform);
    for (ErrorRow& row : all_rows) {
      if (workflow_ ==
          EvaluationWorkflow::kTrajectoryRegistration) {
        row.phase = "registration";
      } else if (workflow_ ==
                     EvaluationWorkflow::
                         kSegmentedRegistrationEvaluation &&
                 row.stamp_sec <= registration_end_stamp) {
        row.phase = "registration";
      } else {
        row.phase = "evaluation";
      }
    }

    std::vector<ErrorRow> evaluation_rows;
    if (!evaluation_matches.empty()) {
      evaluation_rows = buildErrorRows(evaluation_matches, transform);
      for (ErrorRow& row : evaluation_rows) {
        row.phase = "evaluation";
      }
    }

    JsonValue alignment = JsonValue::object();
    alignment["schema_version"] = 2;
    alignment["generated_utc"] = generatedUtc();
    alignment["transform_name"] = "gps_from_onion_map";
    alignment["source_frame"] = "onion_map";
    alignment["target_frame"] =
        gps_position_source_ == "odom_utm" ? "utm" : "gps_local_enu";
    alignment["gps_position_source"] = gps_position_source_;
    alignment["transform"] = transformJson(transform);
    if (navsat_reference_.has_value()) {
      JsonValue navsat = JsonValue::object();
      navsat["latitude_deg"] = navsat_reference_->latitude_deg;
      navsat["longitude_deg"] = navsat_reference_->longitude_deg;
      navsat["altitude_m"] = navsat_reference_->altitude_m;
      alignment["navsat_reference"] = std::move(navsat);
    } else {
      alignment["navsat_reference"] = nullptr;
    }
    if (alignment_result.has_value()) {
      alignment["registration"] =
          registrationJson(*alignment_result, registration_matches,
                           robust_outlier_threshold_m_);
      alignment["warning"] =
          "The transform removes the Onion map's arbitrary origin and "
          "yaw. Registration residuals are not an independent "
          "localization accuracy result.";
    } else {
      alignment["loaded_from"] = alignment_input_path_;
    }

    if (alignment_result.has_value()) {
      writeAtomically(alignment_output_path_, alignment.dump(2) + "\n");
    }

    const std::size_t evaluation_estimate_count =
        std::isfinite(registration_end_stamp)
            ? static_cast<std::size_t>(std::count_if(
                  onion_samples.begin(), onion_samples.end(),
                  [&](const PoseSample& sample) {
                    return sample.stamp > registration_end_stamp;
                  }))
            : onion_samples.size();
    const std::size_t evaluation_gps_count =
        std::isfinite(registration_end_stamp)
            ? static_cast<std::size_t>(std::count_if(
                  references.begin(), references.end(),
                  [&](const PoseSample& sample) {
                    return sample.stamp > registration_end_stamp;
                  }))
            : references.size();

    JsonValue evaluation = JsonValue::object();
    std::vector<RelocalizationResult> relocalization_results;
    if (workflow_ ==
        EvaluationWorkflow::kTrajectoryRegistration) {
      evaluation["performed"] = false;
      evaluation["reason"] =
          "trajectory_registration only estimates the fixed transform";
    } else {
      const RpeResult rpe =
          computeRpe(evaluation_matches, transform, rpe_delta_sec_);
      evaluation = evaluationMetricsJson(
          evaluation_rows, evaluation_matches.size(),
          evaluation_estimate_count, evaluation_gps_count,
          accuracy_thresholds_m_, rpe);
      relocalization_results = evaluateRelocalization(
          relocalization_events, evaluation_matches, evaluation_rows,
          references, transform, max_time_difference_sec_,
          max_interpolation_gap_sec_, convergence_threshold_m_,
          convergence_hold_samples_, relocalization_timeout_sec_);
    }

    JsonValue summary = JsonValue::object();
    summary["schema_version"] = 2;
    summary["generated_utc"] = generatedUtc();
    summary["workflow"] = workflow_name_;
    summary["gps_position_source"] = gps_position_source_;
    summary["coordinate_frame"] =
        gps_position_source_ == "odom_utm" ? "utm" : "gps_local_enu";
    JsonValue topics = JsonValue::object();
    topics["onion_odom"] = onion_odom_topic_;
    topics["gps_odom"] = gps_odom_topic_;
    topics["gps_fix"] = gps_fix_topic_;
    topics["gps_position_valid"] = gps_position_valid_topic_;
    topics["gps_heading_valid"] = gps_heading_valid_topic_;
    topics["initialpose"] = initialpose_topic_;
    topics["relocalization_status"] =
        relocalization_status_topic_;
    summary["topics"] = std::move(topics);
    JsonValue input = JsonValue::object();
    input["onion_pose_count"] = onion_samples.size();
    input["gps_pose_count"] = references.size();
    input["gps_fix_count"] = gps_fix_samples.size();
    input["dropped_invalid_gps_count"] = dropped_invalid;
    input["dropped_before_position_validity_count"] =
        dropped_before_validity;
    input["suppressed_invalid_gps_yaw_count"] =
        suppressed_invalid_yaw;
    input["suppressed_before_heading_validity_count"] =
        suppressed_before_heading;
    summary["input"] = std::move(input);
    summary["alignment"] = std::move(alignment);
    summary["evaluation"] = std::move(evaluation);
    summary["relocalization"] =
        relocalizationJson(relocalization_results);
    if (workflow_ ==
        EvaluationWorkflow::kLocalizationEvaluation) {
      summary["accuracy_claim"] = "fixed_alignment_evaluation";
    } else if (workflow_ ==
               EvaluationWorkflow::
                   kSegmentedRegistrationEvaluation) {
      summary["accuracy_claim"] =
          "post_registration_holdout_evaluation";
    } else {
      summary["accuracy_claim"] = "registration_transform_only";
    }
    JsonValue limitations = JsonValue::array();
    limitations.pushBack(
        "GPS is a reference only after RTK state, receiver "
        "configuration, timestamps and covariance are validated on "
        "the vehicle.");
    limitations.pushBack(
        "A transform estimated from a trajectory removes global "
        "origin and yaw error; do not refit it on each evaluation "
        "sample.");
    summary["limitations"] = std::move(limitations);

    std::filesystem::create_directories(output_directory_);
    writeAtomically(
        (std::filesystem::path(output_directory_) / "summary.json")
            .string(),
        summary.dump(2) + "\n");
    writeAtomically(
        (std::filesystem::path(output_directory_) /
         "matched_trajectory.csv")
            .string(),
        matchedTrajectoryCsv(all_rows));
    writeAtomically(
        (std::filesystem::path(output_directory_) /
         "relocalization_events.csv")
            .string(),
        relocalizationCsv(relocalization_results));
    writeAtomically(
        (std::filesystem::path(output_directory_) / "status_events.csv")
            .string(),
        statusCsv(status_events));

    std::vector<std::pair<std::string, std::string>> flattened;
    summary.flatten("", &flattened);
    std::ostringstream summary_csv;
    summary_csv << "metric,value\n";
    for (const auto& [metric, value] : flattened) {
      summary_csv << csvCell(metric) << ',' << csvCell(value) << '\n';
    }
    writeAtomically(
        (std::filesystem::path(output_directory_) / "summary.csv")
            .string(),
        summary_csv.str());

    publishPaths(all_matches, transform);
    std_msgs::String summary_message;
    summary_message.data = summary.dump(-1);
    summary_publisher_.publish(summary_message);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      finalized_ = true;
      finalizing_ = false;
    }
    ROS_INFO_STREAM("Onion GPS report written: workflow="
                    << workflow_name_ << " matched="
                    << all_matches.size() << " output="
                    << output_directory_);
    *result_message = output_directory_;
    return true;
  } catch (const std::exception& error) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      finalizing_ = false;
    }
    ROS_ERROR_STREAM("Onion GPS report finalization failed: "
                     << error.what());
    *result_message = error.what();
    return false;
  }
}

void EvaluationRunner::publishPaths(
    const std::vector<MatchedPose>& matches,
    const PlanarTransform& transform) {
  const std::string frame_id =
      gps_position_source_ == "odom_utm" ? "utm" : "gps_local_enu";
  nav_msgs::Path gps_path;
  nav_msgs::Path aligned_path;
  gps_path.header.frame_id = frame_id;
  aligned_path.header.frame_id = frame_id;
  gps_path.header.stamp = ros::Time::now();
  aligned_path.header.stamp = gps_path.header.stamp;
  for (const MatchedPose& match : matches) {
    const PoseSample aligned = transform.apply(match.estimate);
    for (const auto& item :
         {std::make_pair(&gps_path, match.reference),
          std::make_pair(&aligned_path, aligned)}) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp.fromSec(item.second.stamp);
      pose.pose.position.x = item.second.x;
      pose.pose.position.y = item.second.y;
      pose.pose.position.z = item.second.z;
      const double yaw =
          std::isfinite(item.second.yaw) ? item.second.yaw : 0.0;
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      item.first->poses.push_back(pose);
    }
  }
  gps_path_publisher_.publish(gps_path);
  aligned_onion_path_publisher_.publish(aligned_path);
}

bool EvaluationRunner::finalizeService(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response) {
  response.success = finalize(&response.message);
  if (response.success && shutdown_after_finalize_) {
    shutdown_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(0.2),
        &EvaluationRunner::shutdownTimerCallback, this, true);
  }
  return true;
}

void EvaluationRunner::shutdownTimerCallback(
    const ros::WallTimerEvent&) {
  ros::shutdown();
}

}  // namespace onion_gps_evaluation
