/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "CameraPoseVisualization.h"

namespace vins_multi{

void registerPub(ros::NodeHandle &n);

void pubLatestOdometry(const Estimator &estimator);

void pubTrackImage(const cv::Mat &imgTrack, const double t, const unsigned int cam_unique_id);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator);

void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const Estimator &estimator);

void pubCameraPose(const Estimator &estimator, const unsigned int unique_id);

void pubPointCloud(const Estimator &estimator, const unsigned int unique_id);

void pubTF(const Estimator &estimator);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubCar(const Estimator & estimator, const std_msgs::Header &header);

}
