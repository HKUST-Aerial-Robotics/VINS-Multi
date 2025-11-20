/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

// #include <list>
#include <algorithm>
#include <vector>
#include <numeric>
#include <eigen3/Eigen/Dense>

// #include <ros/console.h>
// #include <ros/assert.h>


#include "parameters.h"
#include "utility/tic_toc.h"
#include "feature_data_type.h"

using namespace std;
using namespace Eigen;

namespace vins_multi{

class FeatureManager
{
  public:
    FeatureManager(bool depth, bool stereo, vector<shared_ptr<ImageFrame>>& image_frame_ptr);
    ~FeatureManager();

    void setCamInfo(camera_module_info& cam_info){
      cam_info_ptr_ = &cam_info;
    }
    // void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();
    double* getFeatureInvDepth(int feature_id);
    shared_ptr<ImageFrame> getStartFrame(int feature_id);
    bool addFeatureCheckParallax(const map<int,FeaturePerFrame> &feature_pts, double td);
    bool checkParallax(vector<shared_ptr<ImageFrame>>& frameHist);
    // vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    // //void updateDepth(const VectorXd &x);
    void setDepth();
    void removeFailures();
    // void clearDepth();
    // void getDepthVector();
    void setInvDepth();
    void triangulate(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic0, const Quaterniond& ric0, const Vector3d& tic1 = Vector3d::Zero(), const Quaterniond& ric1 = Quaterniond::Identity());
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    bool initFramePoseByICP(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic, const Quaterniond& ric);
    // bool initFramePoseByICP(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic, const Quaterniond& ric, int frameToSolveIdx);                        
    bool initFramePoseByPnP(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic, const Quaterniond& ric);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial,
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    // void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    // void removeBack();
    // void removeFront(int frame_count);
    void removeFront();
    void remove(const int idx);
    void removeSecondBack();
    void outliersRejection();
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj, double &reproj_depth);
    map<int, FeaturePerId> feature_;

    // vector<double> para_Feature_;

    int last_track_num_;
    double last_average_parallax_;
    int new_feature_num_;
    int long_track_num_;

    unsigned int num_frame_;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id);
    // const Matrix3d *Rs;
    // Matrix3d ric0_;
    // Matrix3d ric1_;
    bool stereo_;
    bool depth_;
    vector<shared_ptr<ImageFrame>>& image_frame_ptr_;
    camera_module_info* cam_info_ptr_ = nullptr;
};

}

#endif