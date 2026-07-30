#include <exception>

#include <ros/ros.h>

#include "onion_gps_evaluation/evaluation_runner.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "onion_localization_accuracy_evaluation");
  try {
    onion_gps_evaluation::EvaluationRunner runner(
        ros::NodeHandle(), ros::NodeHandle("~"),
        onion_gps_evaluation::EvaluationWorkflow::
            kLocalizationEvaluation);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Failed to start localization accuracy evaluation: "
                     << error.what());
    return 1;
  }
  return 0;
}
