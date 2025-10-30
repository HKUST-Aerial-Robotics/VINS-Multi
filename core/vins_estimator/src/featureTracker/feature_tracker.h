/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#ifdef WITH_CUDA

#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#endif

#include <eigen3/Eigen/Dense>
#include <atomic>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "estimator/parameters.h"
#include "utility/tic_toc.h"
#include "estimator/feature_data_type.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

namespace vins_multi{

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:

    FeatureTracker(bool is_depth, bool is_stereo, int feature_max_cnt);
    FeatureTracker(const FeatureTracker& s) = delete;
    ~FeatureTracker(){
        // ROS_ERROR("delete feature tracker!");
        spdlog::error("delete feature tracker!");
    }
    map<int, FeaturePerFrame> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void set_max_feature_num(int max_feature_num);
    void setMask();
    void readIntrinsicParameter(const vector<std::string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void rejectDepth(const cv::Mat &depth_img);
    void setDepth(const cv::Mat &depth_img);
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat& getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;

#ifdef WITH_CUDA

    cv::cuda::GpuMat prev_gpu_img;
    // cv::cuda::GpuMat cur_gpu_img;
    cv::cuda::GpuMat prev_gpu_pts, cur_gpu_pts;

    std::vector<cv::cuda::GpuMat> prev_pyr;

    map<int, FeaturePerFrame> trackImageGPU(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    std::vector<cv::cuda::GpuMat> buildImagePyramid(const cv::cuda::GpuMat& prevImg, int maxLevel_);

#endif

    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    vector<double> prev_depth, pts_depth;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    int n_id;
    bool hasPrediction;
    atomic<int> max_cnt;
    atomic<int> track_num;
    atomic<double> track_percentage;

    double mean_optical_flow_speed;

    // inline static std::mutex gpu_mutex;

    bool depth;
    bool stereo;
};

}