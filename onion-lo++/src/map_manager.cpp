/**
MIT License

Copyright (c) 2025 Xiaolong Cheng <chengxiaolong658@163.com>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "map_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

using namespace std;
namespace traj {

std::vector<Vector6d> MapManager::DownSampling(
  const std::vector<Vector6d> &points, double ds_size, bool de_factor) {
  std::vector<Vector6d> ds_result;
  if (!de_factor) {
    tsl::robin_map<Voxel, Vector6d, VoxelHash> grid;
    grid.reserve(points.size());
    for (const auto &point : points) {
      Eigen::Vector3d p = point.head(3);
      auto v_coord = Voxel((p / ds_size).template cast<int>());
      Voxel voxel(v_coord[0], v_coord[1], v_coord[2]);
      if (grid.find(voxel) != grid.end()) continue;
      grid.insert({voxel, point});
    }
    ds_result.reserve(grid.size());
    for (const auto &[voxel, point] : grid) {
      (void)voxel;
      ds_result.emplace_back(point);
    }
  } else {
    std::cout << "\033[31m[Degenerate DownSampling]\033[0m" << std::endl;
    tsl::robin_map<Voxel, Vector6d, VoxelHash> grid;
    grid.reserve(points.size());
    double voxel_size;
    for (const auto &point : points) {
		if (point[5] != 0.0)
			voxel_size = ds_size/2.0;
		else
			voxel_size = ds_size*2.0;
        Eigen::Vector3d p = point.head(3);
        auto v_coord = Voxel((p / voxel_size).template cast<int>());
        Voxel voxel(v_coord[0], v_coord[1], v_coord[2]);
        if (grid.find(voxel) != grid.end()) continue;
        grid.insert({voxel, point});
    }
    ds_result.reserve(grid.size());
    for (const auto &[voxel, point] : grid) {
      (void)voxel;
      ds_result.emplace_back(point);
    }
  }

  return ds_result;
}

void MapManager::PreProcess(const std::vector<Vector6d> &points,
                            const tStampPair &tp, double ds_size, bool de_factor) {
  map_points_database[tp] = DownSampling(points, ds_size/3.0, de_factor);
  reg_points_database[tp] =
      DownSampling(map_points_database[tp], ds_size, de_factor);
  //cout<<"map_points_database--"<<map_points_database[tp].size()<<endl;
  //cout<<"key_point--"<<reg_points_database[tp].size()<<endl;
}

void MapManager::MapInit(const std::vector<Vector6d> &points) {

  const auto &ds = points;
  std::for_each(ds.cbegin(), ds.cend(), [&](const auto &point) {
	Eigen::Vector3d p = point.head(3);
	auto v_coord = Voxel((p / voxel_size_).template cast<int>());
    Voxel voxel(v_coord[0],v_coord[1],v_coord[2]);
    auto search = map.find(voxel);
    if (search != map.end()) {
      auto &voxel_block = search.value();
      voxel_block.AddPoint(point, max_points_per_voxel_);
    } else {
      map.insert({voxel, VoxelBlock{{point}}});
    }
  });
}

bool MapManager::LoadGlobalMap(const std::string &path, bool read_only,
                               std::string *error) {
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path, blob) != 0) {
    if (error) *error = "failed to load PCD file: " + path;
    return false;
  }

  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_label = false;
  for (const auto &field : blob.fields) {
    if (field.name == "x") has_x = true;
    if (field.name == "y") has_y = true;
    if (field.name == "z") has_z = true;
    if (field.name == "label" &&
        field.datatype == pcl::PCLPointField::UINT32) {
      has_label = true;
    }
  }
  if (!has_x || !has_y || !has_z) {
    if (error) *error = "PCD file must contain x/y/z fields: " + path;
    return false;
  }

  pcl::PointCloud<MapPointXYZIL> cloud;
  pcl::fromPCLPointCloud2(blob, cloud);
  if (cloud.empty()) {
    if (error) *error = "PCD file contains no points: " + path;
    return false;
  }

  map.clear();
  std::size_t valid_points = 0;
  for (const auto &point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    Vector6d map_point;
    map_point << static_cast<double>(point.x),
                 static_cast<double>(point.y),
                 static_cast<double>(point.z),
                 0.0,
                 static_cast<double>(point.intensity),
                 has_label ? static_cast<double>(point.label) : 0.0;

    const Eigen::Vector3d p = map_point.head<3>();
    const auto v_coord = Voxel((p / voxel_size_).template cast<int>());
    const Voxel voxel(v_coord[0], v_coord[1], v_coord[2]);
    auto search = map.find(voxel);
    if (search != map.end()) {
      auto &voxel_block = search.value();
      if (max_points_per_voxel_ <= 0 ||
          voxel_block.points.size() <
              static_cast<std::size_t>(max_points_per_voxel_)) {
        voxel_block.points.emplace_back(map_point);
      }
    } else {
      map.insert({voxel, VoxelBlock{{map_point}}});
    }
    ++valid_points;
  }

  if (valid_points == 0 || map.empty()) {
    if (error) *error = "PCD file contains no finite points: " + path;
    return false;
  }

  map_has_labels_ = has_label;
  read_only_map_ = read_only;
  init_flag = true;
  return true;
}

pcl::PointCloud<MapPointXYZIL>::Ptr MapManager::ExportMapCloud() const {
  pcl::PointCloud<MapPointXYZIL>::Ptr cloud(
      new pcl::PointCloud<MapPointXYZIL>());
  cloud->points.reserve(MapPointCount());
  for (const auto &voxel_and_block : map) {
    for (const auto &point : voxel_and_block.second.points) {
      MapPointXYZIL output;
      output.x = static_cast<float>(point[0]);
      output.y = static_cast<float>(point[1]);
      output.z = static_cast<float>(point[2]);
      output.intensity = static_cast<float>(point[4]);
      output.label = static_cast<std::uint32_t>(
          std::max(0L, std::lround(point[5])));
      cloud->points.emplace_back(output);
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

std::size_t MapManager::MapPointCount() const {
  std::size_t count = 0;
  for (const auto &voxel_and_block : map) {
    count += voxel_and_block.second.points.size();
  }
  return count;
}

void MapManager::Update(const posePair &pp, const tStampPair &tp) {
  if (read_only_map_) {
    map_points_database.erase(tp);
    reg_points_database.erase(tp);
    return;
  }

  const auto &ds_points_map = map_points_database[tp];
  std::vector<Eigen::Vector3d> points_transformed(ds_points_map.size());
  std::vector<Vector6d> points_transformed6(ds_points_map.size());
  Sophus::Vector6d tau = Sophus::se3_logd(pp.first.inverse() * pp.second);
  // tbb parallel for
  tbb::parallel_for(size_t(0), ds_points_map.size(), [&](size_t i) {
    double w = ds_points_map[i][3];
	Sophus::SE3d T_b_i = Sophus::se3_expd(w * tau);
    Sophus::SE3d T_w_i = pp.first * T_b_i;

    points_transformed[i] = T_w_i * ds_points_map[i].head<3>();
    points_transformed6[i]<< points_transformed[i][0],points_transformed[i][1],points_transformed[i][2],ds_points_map[i][3], ds_points_map[i][4], ds_points_map[i][5];
  });

  std::for_each(
      points_transformed6.cbegin(), points_transformed6.cend(),
      [&](const auto &point) {
		Eigen::Vector3d p = point.head(3);
		auto v_coord = Voxel((p / voxel_size_).template cast<int>());
		Voxel voxel(v_coord[0],v_coord[1],v_coord[2]);
        auto search = map.find(voxel);
        if (search != map.end()) {
          auto &voxel_block = search.value();
          voxel_block.AddPoint(point, max_points_per_voxel_);
        } else {
          map.insert({voxel, VoxelBlock{{point}}});
        }
      });

  const auto max_distance2 = 300 * 300;
  std::vector<Voxel> voxels_to_remove;
  for (const auto &[voxel, voxel_block] : map) {
    const auto &pt = voxel_block.points.front();
    if ((pt.head<3>() - pp.first.translation()).squaredNorm() > (max_distance2)) {
      voxels_to_remove.emplace_back(voxel);
    }
  }
  for (const auto &voxel : voxels_to_remove) map.erase(voxel);

  map_points_database.erase(tp);
  reg_points_database.erase(tp);
}

bool FitPlaneFromPoints(const std::vector<Eigen::Matrix<double, 6, 1>> &neighboors,
                        double planer_threshold,
                        Eigen::Vector3d &normal) {
  const int N = neighboors.size();
  if (N < 3) return false;
  Eigen::MatrixXd A(N, 3);
  Eigen::VectorXd b(N);
  b.setOnes();
  b *= -1.0;

  for (int i = 0; i < N; ++i) {
    A.row(i) = neighboors[i].head<3>();
  }

  Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);
  double n = normvec.norm();
  if (n == 0) return false;
  normal = normvec / n;

  for (int i = 0; i < N; ++i) {
    double residual = std::abs(normal.dot(neighboors[i].head<3>()) + 1.0 / n);
    if (residual > planer_threshold) {
      return false;
    }
  }
  return true;
}
void MapManager::PointRegistrationNormal(
    const posePairLin &ppl,
    const tStampPair &tp,
    Eigen::Matrix<double, 12, 12> &H_icp,
    Eigen::Matrix<double, 12, 1> &b_icp,
    double &error, double &inliers) {
  
  const double range_thresh = reg_thresh_*2.0;
  double th = reg_thresh_/2.0;
  auto square = [](double x) { return x * x; };
  auto Weight = [&](double residual2) {
    return square(th) / (square(th) + residual2);
  };

  const posePair &pp{ppl.first.getPose(), ppl.second.getPose()};
  Sophus::Vector6d tangent = Sophus::se3_logd(pp.first.inverse() * pp.second);
  Eigen::Matrix3d R_w_b_t = pp.first.rotationMatrix().transpose();
  Eigen::Matrix3d R_w_e_t = pp.second.rotationMatrix().transpose();
  Eigen::Matrix3d R_e_b   = R_w_e_t * pp.first.rotationMatrix();

  const auto &ds_points_reg = reg_points_database[tp];

  const auto &[JTJ, JTr, e, num] = tbb::parallel_reduce(
      tbb::blocked_range<size_t>{0, ds_points_reg.size()},
      ResultTuple(),
      [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
        for (auto i = r.begin(); i < r.end(); ++i) {
          const bool no_plane_label = (ds_points_reg[i][5] == 2.0);
          double planer_threshold = planer_threshold_;
          int neighboors_num  = 6;
          const double alpha        = ds_points_reg[i][3];
          Sophus::SE3d T_b_i    = Sophus::se3_expd(alpha * tangent);
          Sophus::SE3d T_w_i    = pp.first * T_b_i;
          Eigen::Vector3d point = ds_points_reg[i].head<3>();
          Eigen::Vector3d p_in_world = T_w_i * point;
          const auto key = Voxel(
              static_cast<int>(p_in_world[0] / voxel_size_),
              static_cast<int>(p_in_world[1] / voxel_size_),
              static_cast<int>(p_in_world[2] / voxel_size_));
          std::vector<Vector6d> neighboors;
          neighboors.reserve(7 * max_points_per_voxel_);
	      for (const auto &c : coord) {
	        auto it = map.find(key + c);
	        if (it == map.end()) continue;
	        for (const auto &q : it->second.points) {
	          if (!map_has_labels_ || q[5] == ds_points_reg[i][5])
	            neighboors.emplace_back(q);
	        }
	      }

          if (neighboors.size() < static_cast<size_t>(neighboors_num)) continue;

          std::nth_element(neighboors.begin(),
                           neighboors.begin() + neighboors_num,
                           neighboors.end(),
                           [&](const Vector6d &p1, const Vector6d &p2) {
                             return (p1.head<3>() - p_in_world).squaredNorm() <
                                    (p2.head<3>() - p_in_world).squaredNorm();
                           });

          if ((neighboors[0].head<3>() - p_in_world).norm() > range_thresh) continue;

          std::vector<Vector6d> nbr(neighboors.begin(),
                                    neighboors.begin() + neighboors_num);

         
          Eigen::Vector3d normal;

          if (no_plane_label)
          		planer_threshold = 5.0*planer_threshold_;
          if (!FitPlaneFromPoints(nbr, planer_threshold, normal)) continue;
          const Eigen::Vector3d residual = nbr[0].head<3>() - p_in_world;
          if (!(residual.allFinite() && normal.allFinite())) continue;

          const double w = Weight(residual.squaredNorm());
          const Eigen::Matrix3d Information = normal * normal.transpose();
          Eigen::Vector3d p0 = nbr[0].head<3>();
          if (ppl.first.isLinearized() || ppl.second.isLinearized()) {
            tangent  = Sophus::se3_logd(ppl.first.getPoseLin().inverse() *
                                        ppl.second.getPoseLin());
            R_w_b_t  = ppl.first.getPoseLin().rotationMatrix().transpose();
            R_w_e_t  = ppl.second.getPoseLin().rotationMatrix().transpose();
            R_e_b    = R_w_e_t * ppl.first.getPoseLin().rotationMatrix();

            T_b_i    = Sophus::se3_expd(alpha * tangent);
            T_w_i    = ppl.first.getPoseLin() * T_b_i;
            p_in_world = T_w_i * point;
          }

          Eigen::Matrix<double, 3, 6> J_T_wi;
          J_T_wi.block<3, 3>(0, 0) = T_w_i.rotationMatrix();
          J_T_wi.block<3, 3>(0, 3) = -T_w_i.rotationMatrix() * Sophus::SO3d::hat(point);

          Eigen::Matrix<double, 6, 6> J_begin = Eigen::Matrix<double, 6, 6>::Zero();
          Eigen::Matrix<double, 6, 6> J_end   = Eigen::Matrix<double, 6, 6>::Zero();

          Eigen::Matrix3d Jr, Jr_inv;
          const Eigen::Vector3d omega = tangent.tail<3>();
          Sophus::rightJacobianSO3(alpha * omega, Jr);
          Sophus::rightJacobianInvSO3(omega, Jr_inv);

          J_end.topLeftCorner<3, 3>() = Sophus::SO3d::exp((1.0 - alpha) * omega).matrix() * R_w_e_t;
          J_end.bottomRightCorner<3, 3>() = Jr * Jr_inv;

          const Eigen::Matrix3d R_temp = Sophus::SO3d::exp(-alpha * omega).matrix();
          J_begin.topLeftCorner<3, 3>()   = (1.0 - alpha) * R_temp * R_w_b_t;
          J_begin.bottomRightCorner<3, 3>() = R_temp - alpha * Jr * Jr_inv * R_e_b;

          Eigen::Matrix<double, 6, 12> J_be;
          J_be.block<6, 6>(0, 0) = J_begin;
          J_be.block<6, 6>(0, 6) = alpha * J_end;

          Eigen::Matrix<double, 3, 12> J_r = J_T_wi * J_be;

          J.JTJ   += w * J_r.transpose() * Information * J_r;
          J.JTr   += w * J_r.transpose() * Information * residual;
          J.inlier += 1.0;
          J.error  += std::abs(normal.dot(residual));
        }
        return J;
      },
      [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; });

  H_icp = JTJ;
  b_icp = JTr;
  error = e;
  inliers = num;
}

}  // namespace traj
