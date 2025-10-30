/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <list>
#include <mutex>
#include <spdlog/spdlog.h>
#include "integration_base.h"

using namespace std;
using namespace Eigen;

namespace vins_multi{

typedef Matrix<double, 6, 1> Vector6d;
typedef Eigen::SparseMatrix<double> SparseMat;

class FeaturePerFrame
{
  public:
//     FeaturePerFrame(const Eigen::VectorXd &_point, double td)
//     { 
//         is_stereo = false;
//         is_depth = false;
//         point.x() = _point(0);
//         point.y() = _point(1);
//         point.z() = _point(2);
//         uv.x() = _point(3);
//         uv.y() = _point(4);
//         velocity.x() = _point(5); 
//         velocity.y() = _point(6);
//         if(_point.rows() > 7){
//           depth = _point(7);
//           is_depth = depth > 0.0;
//         } 
//         cur_td = td;
//     }
//     void rightObservation(const Eigen::VectorXd &_point)
//     {
//         pointRight.x() = _point(0);
//         pointRight.y() = _point(1);
//         pointRight.z() = _point(2);
//         uvRight.x() = _point(3);
//         uvRight.y() = _point(4);
//         velocityRight.x() = _point(5); 
//         velocityRight.y() = _point(6); 
//         is_stereo = true;
//     }
    double cur_td, depth;
    Vector3d point, pointRight;
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    bool is_stereo, is_depth;
};

class FeaturePerId
{
  public:

    enum FeatureFlag {
      UNINITIALIZED = 0,
      INITIALIZED = 1, //Initialized
      LONGTRACK = 2, //Long track
      ESTIMATED = 3, //ESTIMATE
      OUTLIER = 4
    };

    const int feature_id;
    int start_frame;
    list<FeaturePerFrame> feature_per_frame;
    int used_num;
    double estimated_depth;
    double inv_depth;
    int solve_flag;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), inv_depth(-1.0), solve_flag(UNINITIALIZED)
    {
    }

    int endFrame()
    {
        return start_frame + feature_per_frame.size() - 1;
    }
};

class ImageFrame
{
    public:
        // ImageFrame(){};
        ImageFrame(const double _t, double& _td, const int unique_id, const map<int, FeaturePerFrame>& _points):
          t_{_t}, td_{_td}, cam_module_unique_id_{unique_id}, is_key_frame_{false}, 
          R_{&para_Pose_[3]}, T_{&para_Pose_[0]},
          V_{&para_SpeedBias_[0]}, Ba_{&para_SpeedBias_[3]}, Bg_{&para_SpeedBias_[6]}
        {
            points_ = _points;
        };
        int cam_module_unique_id_;
        map<int, FeaturePerFrame> points_;
        unsigned int state_idx_;
        double t_;
        double& td_;
        double para_Pose_[SIZE_POSE];
        double para_SpeedBias_[SIZE_SPEEDBIAS];

        Map<Eigen::Quaterniond> R_;
        Map<Eigen::Vector3d> T_;
        Map<Eigen::Vector3d> V_;
        Map<Eigen::Vector3d> Ba_;
        Map<Eigen::Vector3d> Bg_;
        Vector3d prev_acc_, prev_gyr_;
        shared_ptr<IntegrationBase> pre_integration_;
        
        bool is_key_frame_;
};

class ImageFrameWindow
{
  public:
    ImageFrameWindow(){}

    void resize(unsigned int num_cam_module){
      clear();
      cam_wise_image_frame_ptr_.resize(num_cam_module);
    }

    void clear(){
      cam_wise_image_frame_ptr_.clear();
      all_image_frame_ptr_.clear();
    }

    unsigned int pop_front(const unsigned int cam_unique_id){
      auto frame_ptr = cam_wise_image_frame_ptr_[cam_unique_id].front();
      unsigned int state_idx = frame_ptr->state_idx_; 

      double remove_t = frame_ptr->t_ + frame_ptr->td_;

      all_image_frame_ptr_.erase(remove_t);

      cam_wise_image_frame_ptr_[cam_unique_id].erase(cam_wise_image_frame_ptr_[cam_unique_id].begin());

      for(auto it = all_image_frame_ptr_.begin(); it != all_image_frame_ptr_.end(); it++){
        if(it->first > remove_t){
          it->second->state_idx_ --;
        }
      }

      return state_idx;

    }

    unsigned int erase_second_new(const unsigned int cam_unique_id){
      auto frame_ptr = *next(cam_wise_image_frame_ptr_[cam_unique_id].rbegin());
      unsigned int state_idx = frame_ptr->state_idx_;

      double remove_t = frame_ptr->t_ + frame_ptr->td_;

      all_image_frame_ptr_.erase(remove_t);

      cam_wise_image_frame_ptr_[cam_unique_id].erase(cam_wise_image_frame_ptr_[cam_unique_id].begin() + cam_wise_image_frame_ptr_[cam_unique_id].size() - 2);

      for(auto it = all_image_frame_ptr_.begin(); it != all_image_frame_ptr_.end(); it++){
        if(it->first > remove_t){
          it->second->state_idx_ --;
        }
      }

      // erase_frame(frame_ptr);      
      return state_idx;
    }

    unsigned int erase(const unsigned int cam_unique_id, const unsigned int cam_wise_idx){
      auto frame_ptr = cam_wise_image_frame_ptr_[cam_unique_id][cam_wise_idx];
      unsigned int state_idx = frame_ptr->state_idx_;
      double remove_t = frame_ptr->t_ + frame_ptr->td_;

      all_image_frame_ptr_.erase(remove_t);
      cam_wise_image_frame_ptr_[cam_unique_id].erase(cam_wise_image_frame_ptr_[cam_unique_id].begin() + cam_wise_idx);

      for(auto it = all_image_frame_ptr_.begin(); it != all_image_frame_ptr_.end(); it++){
        if(it->first > remove_t){
          it->second->state_idx_ --;
        }
      }   
      return state_idx;

    }

    void reorder(){
      map<double, shared_ptr<ImageFrame>> tmp_all_image_frame = all_image_frame_ptr_;
      all_image_frame_ptr_.clear();
      for(auto frame_it = tmp_all_image_frame.begin(); frame_it != tmp_all_image_frame.end(); frame_it++){
        double real_time = frame_it->second->t_ + frame_it->second->td_;
        all_image_frame_ptr_.insert(make_pair(real_time, frame_it->second));
      }
    }

    map<double, shared_ptr<ImageFrame>>::iterator insert(shared_ptr<ImageFrame> image_frame_ptr){

      double frame_real_time = image_frame_ptr->t_ + image_frame_ptr->td_;

      if(!all_image_frame_ptr_.empty() &&  frame_real_time< all_image_frame_ptr_.begin()->first){
        // should not happen after init with proper td
        // ROS_ERROR("try to insert at front");
        spdlog::error("try to insert at front");
        return all_image_frame_ptr_.end();
      }
      
      auto insert_it_and_is_inserted = all_image_frame_ptr_.insert(make_pair(frame_real_time , image_frame_ptr));
      // ROS_DEBUG("insert image frame at t: %lf", frame_real_time);
      spdlog::debug("insert image frame at t: {}", frame_real_time);

      bool is_inserted = insert_it_and_is_inserted.second;
      if(!is_inserted){
        // ROS_ERROR("same time, no insertion");
        spdlog::error("same time, no insertion");
        return all_image_frame_ptr_.end();
      }
      auto insert_it = insert_it_and_is_inserted.first;
      cam_wise_image_frame_ptr_[image_frame_ptr->cam_module_unique_id_].emplace_back(insert_it->second);
    
      return insert_it;
    }

    map<double, shared_ptr<ImageFrame>> all_image_frame_ptr_;
    vector<vector<shared_ptr<ImageFrame>>> cam_wise_image_frame_ptr_; 

  private:

};


class State
{
  public:
    Quaterniond Q_ = Quaterniond::Identity();
    Vector3d P_ = Vector3d::Zero();
    Vector3d V_ = Vector3d::Zero();
    Vector3d Ba_ = Vector3d::Zero();
    Vector3d Bg_ = Vector3d::Zero();

    Quaterniond Q_lpf_ = Quaterniond::Identity();
    Vector3d P_lpf_ = Vector3d::Zero();
    Vector3d V_lpf_ = Vector3d::Zero();

    Vector3d un_gyr_ = Vector3d::Zero();

    double t_ = 0.0;

    Vector6d imu_data_ = Vector6d::Zero(); //store next imu data if IMAGE type
    shared_ptr<ImageFrame> image_frame_ptr_;

    enum TYPE{IMAGE, IMU};

    int type_;

  static bool compare(State& i, State& j){
    return (i.t_ < j.t_);
  }

};

}