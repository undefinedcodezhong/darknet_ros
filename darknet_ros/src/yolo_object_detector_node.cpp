/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <darknet_ros/YoloObjectDetector.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");
  std::string cameraTopicName;
  std::string cameraDepthTopicName;
  nodeHandle.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/rgb/image_raw"));
  nodeHandle.param("subscribers/camera_depth_reading/topic", cameraDepthTopicName, std::string("/camera/rgb/image_raw"));

  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle, cameraTopicName, cameraDepthTopicName);

  ros::spin();
  return 0;
}
