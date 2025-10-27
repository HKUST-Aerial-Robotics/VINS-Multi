/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

// #include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

using namespace std;

namespace vins_multi{

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_POSE_EFF = 6,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_TD = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 21;
const int MAX_WINDOW_SIZE = 30;
const int MAX_NUM_OF_F = 1000;
const double MIN_OPT_INTERVAL = 0.04;
const int MIN_TRACK_FRAME_FOR_OPT = 3;
const double MIN_FRAME_INTERVAL_PER_MODULE = 0.05;
const double MIN_PRE_INTEGRATION_INTERVAL = 0.01;
const double MIN_FRAME_INTERVAL_FOR_OPT = 0.005;
const int MIN_TRACK_NUM_PER_MODULE = 30;
extern int MAX_TRACK_NUM_PER_MODULE;
const double FRAME_PRIORITY_CONST = 20.0;
//#define UNIT_SPHERE_ERROR

struct camera_module_info{
    int module_id_;
    bool depth_;
    bool stereo_;
    int img_width_;
    int img_height_;
    double td_;
    double tr_;
    vector<std::string> img_topic_;
    vector<std::string> calib_file_;
    vector<Eigen::Map<Eigen::Quaterniond>> ric_;
    vector<Eigen::Map<Eigen::Vector3d>> tic_;

    vector<double*> para_Ex_Pose_;

    // void mat2vec(){
    //     for(unsigned int i = 0; i < ric_.size(); i++){
    //         para_Ex_Pose_[i][0] = tic_[i].x();
    //         para_Ex_Pose_[i][1] = tic_[i].y();
    //         para_Ex_Pose_[i][2] = tic_[i].z();

    //         Map<MatrixXd>( &para_Ex_Pose_[i][0], 3, 1) = tic_[0];
    //         Map<MatrixXd>( &para_Ex_Pose_[i][3], 4, 1) = ric_[0].coeffs();

    //         Quaterniond q(ric_[i]);

    //         para_Ex_Pose_[i][3] = q.x();
    //         para_Ex_Pose_[i][4] = q.y();
    //         para_Ex_Pose_[i][5] = q.z();
    //         para_Ex_Pose_[i][6] = q.w();

    //     }

    // }

    // void vec2mat(){

    //     for(unsigned int i = 0; i < ric_.size(); i++){
    //         tic_[i].x() = para_Ex_Pose_[i][0]; 
    //         tic_[i].y() = para_Ex_Pose_[i][1]; 
    //         tic_[i].z() = para_Ex_Pose_[i][2];

    //         ric_[i] = Quaterniond(para_Ex_Pose_[i][6],
    //                              para_Ex_Pose_[i][3],
    //                              para_Ex_Pose_[i][4],
    //                              para_Ex_Pose_[i][5]).normalized().toRotationMatrix();
    //     }        

    // }

    void set_size(){
        if(stereo_){
            img_topic_.resize(2);
            calib_file_.resize(2);

            para_Ex_Pose_.clear();
            para_Ex_Pose_.emplace_back(new double[SIZE_POSE]);
            para_Ex_Pose_.emplace_back(new double[SIZE_POSE]);

            ric_.clear();
            ric_.emplace_back(Eigen::Map<Eigen::Quaterniond>(&para_Ex_Pose_[0][3]));
            ric_.emplace_back(Eigen::Map<Eigen::Quaterniond>(&para_Ex_Pose_[1][3]));

            tic_.clear();
            tic_.emplace_back(Eigen::Map<Eigen::Vector3d>(&para_Ex_Pose_[0][0]));
            tic_.emplace_back(Eigen::Map<Eigen::Vector3d>(&para_Ex_Pose_[1][0]));
            

        }
        else{
            if(depth_){
                img_topic_.resize(2);
                calib_file_.resize(2);
            }
            else{
                img_topic_.resize(1);
                calib_file_.resize(1);
            }
            

            para_Ex_Pose_.resize(1, new double[SIZE_POSE]);
            
            ric_.clear();
            ric_.emplace_back(Eigen::Map<Eigen::Quaterniond>(&para_Ex_Pose_[0][3]));

            tic_.clear();
            tic_.emplace_back(Eigen::Map<Eigen::Vector3d>(&para_Ex_Pose_[0][0]));
        }
    }
};

struct imu_info{
    std::string imu_topic_;
    double acc_n_;
    double gyr_n_;
    double acc_w_;
    double gyr_w_;
    // double g_norm_;
    Eigen::Matrix3d rcenterimu_;
    Eigen::Vector3d tcenterimu_;
};

extern int USE_GPU;

extern double INIT_DEPTH;
extern double MIN_PARALLAX;

extern std::vector<camera_module_info> CAM_MODULES;
extern imu_info IMU_MODULE;
extern Eigen::Vector3d G;
extern map<int, Eigen::Vector3d> pts_gt;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern int ESTIMATE_EXTRINSIC;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
extern std::string FISHEYE_MASK;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;
extern int EQUALIZE;
extern double DEPTH_MIN;
extern double DEPTH_MAX;


extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern std::mutex GPU_MUTEX;

void readParameters(std::string config_file);

}