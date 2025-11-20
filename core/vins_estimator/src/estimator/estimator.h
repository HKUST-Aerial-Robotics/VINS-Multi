/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <atomic>
#include <thread>
#include <chrono>
// #include <std_msgs/Header.h>
// #include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <list>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "parameters.h"
#include "feature_manager.h"
// #include "../utility/utility.h"
// #include "../utility/tic_toc.h"
// #include "../initial/solve_5pts.h"
// #include "../initial/initial_sfm.h"
#include "initial/initial_alignment.h"
// #include "../initial/initial_ex_rotation.h"
#include "factor/imu_factor.h"
// #include "factor/pose_local_parameterization.h"
#include "factor/pose_manifold.h"
#include "factor/marginalization_factor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"
#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/depthFactor.h"
// #include "../factor/reprojectionDepthFactor.h"
#include "factor/projectionTwoFrameOneCamDepthFactor.h"
#include "featureTracker/feature_tracker.h"
#include "publisher/PublisherCallbacks.h"

namespace vins_multi{

class Estimator
{
  public:

    class rawImageFrame{
    public:
        rawImageFrame() : t_(-1.0), img_(cv::Mat()), img1_(cv::Mat()){}
        rawImageFrame(double t, const cv::Mat& img, const cv::Mat& img1) : t_(t), img_(img), img1_(img1){}
        void setImageFrame(double t, const cv::Mat& img, const cv::Mat& img1){
            t_ = t;
            img_ = img;
            img1_ = img1;
        }

        bool valid_ = false;
        double t_;
        cv::Mat img_;
        cv::Mat img1_;
    };

    class imageBuffer{
    public:
        imageBuffer(const unsigned int buffer_size){
            unsigned int buffer_real_size = max(1U,buffer_size);
            for(unsigned int i = 0U; i < buffer_real_size; i++){
                free_memory_buffer_.emplace_back(new rawImageFrame());
            }    
        }

        void insertImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat()){
            if(free_memory_buffer_.empty()){

                if(image_buffer_.empty()){

                }
                else{
                    image_buffer_.back()->setImageFrame(t, _img, _img1);
                }

            }
            else{
                image_buffer_.emplace_back(free_memory_buffer_.front());
                free_memory_buffer_.pop_front();
                image_buffer_.back()->setImageFrame(t, _img, _img1);

            }
        }

        void releaseImage(shared_ptr<rawImageFrame> frame_ptr){

            free_memory_buffer_.emplace_back(frame_ptr);
            
        }

        shared_ptr<rawImageFrame> retrieveFrame(){
            if(image_buffer_.empty()){
                return shared_ptr<rawImageFrame>(nullptr);
            }
            else{
                shared_ptr<rawImageFrame> frame_ptr = image_buffer_.front();
                image_buffer_.pop_front();
                return frame_ptr;
            }
        }

    private:

        list<shared_ptr<rawImageFrame>> image_buffer_;
        list<shared_ptr<rawImageFrame>> free_memory_buffer_;
    };

    class imgTracker{
        public:
            imgTracker(camera_module_info& cam_module, vector<shared_ptr<ImageFrame>>& image_frame_ptr, int max_feature_per_module): cam_info_{cam_module}, featureTracker_{cam_module.depth_, cam_module.stereo_, max_feature_per_module}, f_manager_(cam_module.depth_, cam_module.stereo_, image_frame_ptr), image_buffer_(5U){
                // ROS_WARN("set tracker, id %d", cam_module.module_id_);
                // spdlog::warn("set tracker, id {}", cam_module.module_id_);
                featureTracker_.readIntrinsicParameter(cam_module.calib_file_);
            }

            ~imgTracker(){
                // spdlog::info("imgTracker destructor");
            }


            void set_f_manager_cam_info(){
                f_manager_.setCamInfo(this->cam_info_);
            }

            double get_feature_priority(){
                return static_cast<double>(featureTracker_.max_cnt) / static_cast<double>(MAX_TRACK_NUM_PER_MODULE);
            }

            void clean_frame_time_hist(const double t){
                while(frame_time_hist_.size() > 2 && *(frame_time_hist_.begin()+2) < t){
                    frame_time_hist_.pop_front();
                }
            }

            double get_this_time_priority(const double t){

                double time_priority = min(1.0, frame_time_priority_ratio_ * exp(FRAME_PRIORITY_CONST * (-1.0 - t)));

                if(frame_time_hist_.empty() || frame_time_hist_.front() >= t || frame_time_hist_.size() < 2){
                    return time_priority;
                }

                for(auto rit = frame_time_hist_.rbegin(); rit != frame_time_hist_.rend(); rit++){
                    if(*rit < t){
                        if(next(rit) == frame_time_hist_.rend()){
                            return time_priority;
                        }
                        else{
                            time_priority = min(1.0, frame_time_priority_ratio_ * exp(FRAME_PRIORITY_CONST * (*next(rit) - t)));
                        }
                    }
                }
                return time_priority;
            }

            double get_time_priority(const double t){

                double time_priority = min(1.0, frame_time_priority_ratio_ * exp(FRAME_PRIORITY_CONST * (-1.0 - t)));

                if(frame_time_hist_.empty()){
                    return time_priority;
                }

                return min(1.0, frame_time_priority_ratio_ * exp(FRAME_PRIORITY_CONST * (frame_time_hist_.back() - t)));
            }

            double get_total_priority(const double t){

                double feature_priority = get_feature_priority();
                double frame_time_priority =  min(1.0, frame_time_priority_ratio_ * exp(FRAME_PRIORITY_CONST * (last_frame_time_ - t)));
                return feature_priority + frame_time_priority;
            }

            void increase_frame_time_priority(){
                frame_time_priority_ratio_ *= 1.5;
            }

            void reset_frame_time_priority(){
                frame_time_priority_ratio_ = 1.0;
            }

            camera_module_info cam_info_;
            FeatureTracker featureTracker_;
            FeatureManager f_manager_;

            double last_frame_time_ = -1.0;
            double last_keep_frame_time_ = -1.0;
            double frame_time_priority_ratio_ = 1.0;

            double max_frame_time_priority = 1.0;

            deque<double> frame_time_hist_;

            imageBuffer image_buffer_;
            mutex image_buffer_mutex_;
    };


    // struct featureFrame{
    //     double t_;
    //     int cam_module_unique_id_;
    //     map<int, FeaturePerFrame> feature_pts_;
    // };

    Estimator();
    ~Estimator();
    void setParameter();
    static void initTrackerGPU(shared_ptr<imgTracker> img_tracker);

    void start_process_thread();
    void stop_process_thread();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputIMU(double t, const Vector6d &imu_data);
    void inputImageToBuffer(const unsigned int unique_id, double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void processImageBuffer(const unsigned int unique_id);
    void inputImage(const unsigned int unique_id, double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    
    void updateFeatureTrackerMaxCnt();
    bool CheckKeepImageUpdatePriority(const int cam_unique_id, const double t);
    deque<State>::iterator insertState(const State& state);

    void setImageIMUData(const deque<State>::iterator img_it);
    void setImageState(const deque<State>::iterator img_it);
    void setStateFromImage();
    
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const deque<State>::iterator img_it, const map<double, shared_ptr<ImageFrame>>::iterator img_frame_it);
    void processMeasurements(const deque<State>::iterator img_it);

    void processWindow(const int img_cam_unique_id);

    void constructPreintegration(const deque<State>::iterator insert_state_it, const map<double, shared_ptr<ImageFrame>>::iterator insert_frame_it);
    void reconstructPreintegration();
    void addPreintegrationToNextFrame(unsigned int remove_frame_state_idx);

    // void constructMarginalizationInfo();
    // void constructPriorFactor(shared_ptr<ImageFrame>& frame_ptr_to_margin);
    void constructMarginalizationFactor();
    // internal
    void clearState();
    // bool initialStructure();
    // bool visualInitialAlign();
    // bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow(const int img_cam_unique_id);
    void slideWindow(shared_ptr<ImageFrame>& frame_ptr);
    // void slideWindowNew();
    // void slideWindowOld();

    void reorderWindow();
    void optimization();
    void vector2double();
    void double2vector();
    // bool failureDetection();

    void getPoseInWorldFrame(const int unique_id, Eigen::Matrix4d &T);
    void getPoseInWorldFrame(const int unique_id, const int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame(const int unique_id);
    // void outliersRejection(set<int> &removeIndex);
    // double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
    //                                  Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
    //                                  double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates(const int unique_id);
    // void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    
    void repropagateIMU(const deque<State>::iterator start_it, const bool low_pass);
    void propagateIMU(const State& x, State& x_next);
    void propagateIMULowpass(const State& x, State& x_next, const double& alpha);
    bool IMUAvailable(double t);
    bool IMUInitReady(double img_time);
    // void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
    void initFirstIMUPose(const deque<State>::iterator img_it);

    inline bool needMarginalization();

    void collectPropogationData(Publisher::PropagateData& propagate_data);
    void collectOptimizedData(Publisher::FullReportData& full_report_data, const int unique_id, const bool update_latest);

    void setPublisherCallbacks(const Publisher::PublisherCallbacks& callbacks){
        publisher_callbacks_ = callbacks;
        return;
    }

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mProcess_;
    std::mutex mBuf_;
    std::mutex mPropagate_;
    // queue<pair<double, Eigen::Vector3d>> accBuf_;
    // queue<pair<double, Eigen::Vector3d>> gyrBuf_;

    deque<State> state_hist_;

    queue<ImageFrame> featureBuf_;
    double prevTime_, curTime_;
    bool openExEstimation_;

    std::thread trackThread_;
    std::thread processThread_;


    SolverFlag solver_flag_;
    MarginalizationFlag  marginalization_flag_;
    Vector3d g_;

    unsigned int total_feature_track_num_;


    // Vector3d        Ps_[(WINDOW_SIZE + 1)];
    // Vector3d        Vs_[(WINDOW_SIZE + 1)];
    // Matrix3d        Rs_[(WINDOW_SIZE + 1)];
    // Vector3d        Bas_[(WINDOW_SIZE + 1)];
    // Vector3d        Bgs_[(WINDOW_SIZE + 1)];
    // double td_;

    Matrix3d back_R0_, last_R_, last_R0_;
    Vector3d back_P0_, last_P_, last_P0_;
    Vector3d origin_R0, origin_P0;
    double Headers_[(WINDOW_SIZE + 1)];

    // IntegrationBase *pre_integrations_[(WINDOW_SIZE + 1)];
    // Vector3d acc_0_, gyr_0_;

    // vector<double> dt_buf_[(WINDOW_SIZE + 1)];
    // vector<Vector3d> linear_acceleration_buf_[(WINDOW_SIZE + 1)];
    // vector<Vector3d> angular_velocity_buf_[(WINDOW_SIZE + 1)];

    int frame_count_;
    int sum_of_outlier_, sum_of_back_, sum_of_front_, sum_of_invalid_;
    int inputImageCnt_;

    vector<shared_ptr<imgTracker>> img_trackers_;
    imu_info imu_module_;

    ImageFrameWindow image_frame_window_;
    // MotionEstimator m_estimator;
    // InitialEXRotation initial_ex_rotation;

    bool first_imu_;
    bool is_valid, is_key_;
    bool failure_occur_;

    vector<Vector3d> point_cloud_;
    vector<Vector3d> margin_cloud_;
    vector<Vector3d> key_poses_;

    // double para_Pose_[WINDOW_SIZE + 1][SIZE_POSE];
    // double para_SpeedBias_[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    // double para_Feature_[NUM_OF_F][SIZE_FEATURE];
    // double para_Ex_Pose_[3][SIZE_POSE];
    // double para_Retrive_Pose_[SIZE_POSE];
    // double para_Td_[1][1];
    // double para_Tr_[1][1];

    int loop_window_index_;

    MarginalizationInfo *last_marginalization_info_;
    ceres::Problem* problem_ptr_;
    // unique_ptr<Marginalizer> marginalizer_;
    // PriorFactor* last_prior_ptr_;
    vector<double *> last_marginalization_parameter_blocks_;

    shared_ptr<ImageFrame> frame_to_margin_;

    // IntegrationBase *tmp_pre_integration_;

    Eigen::Vector3d initP_;
    Eigen::Matrix3d initR_;

    vector<int> latest_idx_[2];

    double initial_timestamp_ = 0.0;

    double latest_time_ = 0.0;
    double latest_image_time_ = 0.0;
    double last_opt_time_;

    int lpf_idx_ = 0;

    Eigen::Vector3d latest_P_, latest_V_, latest_Ba_, latest_Bg_;
    // Eigen::Vector3d un_acc_, un_gyr_;
    Eigen::Quaterniond latest_Q_;
    // Eigen::Vector3d latest_acc_0_, latest_gyr_0_;

    bool initFirstPoseFlag_;
    bool initThreadFlag_;

    std::vector<std::thread> image_process_thread_vec_;
    std::atomic<bool> processing_threads_running_{false};
    Publisher::PublisherCallbacks publisher_callbacks_;

    //publish data related temp
    Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_omega_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond last_q_ = Eigen::Quaterniond::Identity();
};

}
