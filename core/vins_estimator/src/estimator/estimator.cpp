/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "feature_manager.h"
// #include "../utility/visualization.h"


namespace vins_multi{

#define CERES_NO_CUDA

Estimator::Estimator()
{
    // ROS_INFO("init begins");
    spdlog::info("init estimator");
    initThreadFlag_ = false;
    last_marginalization_info_ = nullptr;
    processing_threads_running_.store(false);
    clearState();
}

Estimator::~Estimator()
{
    stop_process_thread();
    if (last_marginalization_info_ != nullptr){
        delete last_marginalization_info_;
    }

}

void Estimator::clearState()
{
    mProcess_.lock();
    prevTime_ = -1;
    curTime_ = 0;
    openExEstimation_ = 0;
    initP_ = Eigen::Vector3d(0, 0, 0);
    initR_ = Eigen::Matrix3d::Identity();
    inputImageCnt_ = 0;
    initial_timestamp_ = -1.0;
    initFirstPoseFlag_ = false;
    last_opt_time_ = -1.0;

    first_imu_ = false,
    sum_of_back_ = 0;
    sum_of_front_ = 0;
    frame_count_ = 0;
    solver_flag_ = INITIAL;
    initial_timestamp_ = 0;
    image_frame_window_.clear();

    // if (tmp_pre_integration_ != nullptr)
    //     delete tmp_pre_integration_;
    if (last_marginalization_info_ != nullptr)
        delete last_marginalization_info_;

    last_marginalization_info_ = nullptr;
    last_marginalization_parameter_blocks_.clear();

    img_trackers_.clear();

    failure_occur_ = 0;

    mProcess_.unlock();
    return;
}

void Estimator::setParameter()
{
    mProcess_.lock();

    unsigned int cam_module_size = CAM_MODULES.size();
    image_frame_window_.resize(cam_module_size);
    imu_module_ = IMU_MODULE;

    int feature_num_per_module = MAX_CNT / cam_module_size;
    for(unsigned int i = 0; i < cam_module_size; i++){
        img_trackers_.emplace_back(shared_ptr<imgTracker>{new imgTracker{CAM_MODULES[i], image_frame_window_.cam_wise_image_frame_ptr_[i], feature_num_per_module}});
    }

    for(unsigned int i = 0; i < img_trackers_.size(); i++){
        img_trackers_[i]->set_f_manager_cam_info();
    }


    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // depthFactor::sqrt_info = 200;
    depthFactor::depth_covar = 5e-3;
    ProjectionTwoFrameOneCamDepthFactor::proj_sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    // ProjectionTwoFrameOneCamDepthFactor::sqrt_info(2,2) = 200;
    ProjectionTwoFrameOneCamDepthFactor::depth_covar = 5e-3;
    g_ = G;
    cout << "set g " << g_.transpose() << endl;

#ifdef WITH_CUDA

    if(USE_GPU){
        vector<std::thread> init_gpu_thread_vec;
        for(auto img_tracker : img_trackers_){
            init_gpu_thread_vec.emplace_back(initTrackerGPU, img_tracker);
        }

        for(auto& init_gpu_thread : init_gpu_thread_vec){
            init_gpu_thread.join();
        }
    }

#endif
    // std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    // if (MULTIPLE_THREAD && !initThreadFlag_)
    // {
    //     initThreadFlag_ = true;
    //     processThread_ = std::thread(&Estimator::processMeasurements, this);
    // }
    mProcess_.unlock();
    return;
}

void Estimator::start_process_thread(){
    stop_process_thread();
    if (img_trackers_.empty()) {
        return;
    }
    processing_threads_running_.store(true);
    image_process_thread_vec_.reserve(img_trackers_.size());
    for (unsigned int unique_id = 0; unique_id < img_trackers_.size(); unique_id++) {
        image_process_thread_vec_.emplace_back(&Estimator::processImageBuffer, this, unique_id);
    }
    return;
}

void Estimator::stop_process_thread() {
    processing_threads_running_.store(false);
    for (auto & thread : image_process_thread_vec_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    image_process_thread_vec_.clear();
    return;
}

#ifdef WITH_CUDA

void Estimator::initTrackerGPU(shared_ptr<imgTracker> img_tracker){
    cv::Mat _img = cv::Mat::zeros(img_tracker->cam_info_.img_height_, img_tracker->cam_info_.img_width_, CV_8UC1);
    _img.at<uchar>(img_tracker->cam_info_.img_height_/2, img_tracker->cam_info_.img_width_/2) = uchar(255);
    img_tracker->featureTracker_.trackImageGPU(-1.0, _img);
    img_tracker->featureTracker_.trackImageGPU(1.0, _img);
    img_tracker->featureTracker_.trackImageGPU(-1.0, _img);
    return;
}

#endif

void Estimator::inputImageToBuffer(const unsigned int unique_id, double t, const cv::Mat &_img, const cv::Mat &_img1){
    auto& img_tracker = img_trackers_[unique_id];
    img_tracker->image_buffer_mutex_.lock();
    img_tracker->image_buffer_.insertImage(t, _img, _img1);
    img_tracker->image_buffer_mutex_.unlock();
    return;
}

void Estimator::processImageBuffer(const unsigned int unique_id){
    auto& img_tracker = img_trackers_[unique_id];
    const std::chrono::duration<double> max_delay_time(0.002); // in seconds

    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::duration<double> process_buffer_time;

    while(processing_threads_running_.load()){

        start_time = std::chrono::system_clock::now();

        img_tracker->image_buffer_mutex_.lock();
        auto frame_ptr = img_tracker->image_buffer_.retrieveFrame();
        img_tracker->image_buffer_mutex_.unlock();

        if(frame_ptr){
            inputImage(unique_id, frame_ptr->t_, frame_ptr->img_, frame_ptr->img1_);

            img_tracker->image_buffer_mutex_.lock();
            img_tracker->image_buffer_.releaseImage(frame_ptr);
            img_tracker->image_buffer_mutex_.unlock();
        }

        process_buffer_time = std::chrono::system_clock::now() - start_time;

        // if(frame_ptr){
        //     std::cout<<"process frame in buffer "<<unique_id<<", time: "<<process_buffer_time.count() * 1000<<" ms\n";
        // }
        // else{
        //     std::cout<<"process buffer "<<unique_id<<", time: "<<process_buffer_time.count() * 1000<<" ms\n";
        // }

        if(process_buffer_time < max_delay_time){
            this_thread::sleep_for(max_delay_time - process_buffer_time);
        }
    }
    return;
}

void Estimator::inputImage(const unsigned int unique_id, double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    // inputImageCnt_++;
    map<int, FeaturePerFrame> featurePts;
    TicToc featureTracker_Time;

    if(USE_GPU){
#ifdef WITH_CUDA
        featurePts = img_trackers_[unique_id]->featureTracker_.trackImageGPU(t, _img, _img1);
#endif
    }
    else{
        featurePts = img_trackers_[unique_id]->featureTracker_.trackImage(t, _img, _img1);
    }

    // cout<<"track image time: "<<featureTracker_Time.toc()<<" ms"<<endl;

    updateFeatureTrackerMaxCnt();

    if (SHOW_TRACK)
    {
        const cv::Mat& imgTrack = img_trackers_[unique_id]->featureTracker_.getTrackImage();
        #if 0
        pubTrackImage(imgTrack, t, unique_id);
        #endif
    }

    double real_img_time = t + img_trackers_[unique_id]->cam_info_.td_;

    // make sure no imu delay
    int wait_cnt = 0;
    while(processing_threads_running_.load())
    {
        if ((!USE_IMU  || IMUAvailable(real_img_time)))
            break;
        else
        {
            if(wait_cnt == 0){
                printf("wait for imu ... \n");
            }
            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
        wait_cnt++;
        wait_cnt %= 100;
    }

    if (USE_IMU && !processing_threads_running_.load() && !IMUAvailable(real_img_time)) {
        return;
    }

    mBuf_.lock();
    if(USE_IMU){
        if(!initFirstPoseFlag_){
            if(!IMUInitReady(real_img_time)){
                mBuf_.unlock();
                return;
            }
        }
        else{
            if(real_img_time <= initial_timestamp_){
                mBuf_.unlock();
                return;
            }
        }
    }


    if(!CheckKeepImageUpdatePriority(unique_id, real_img_time)){
        mBuf_.unlock();
        return;
    }

    State img_state;
    img_state.type_ = State::IMAGE;
    img_state.image_frame_ptr_.reset(new ImageFrame{t,img_trackers_[unique_id]->cam_info_.td_, unique_id,featurePts});
    // ROS_INFO("img_state initial point size: %d", img_state.image_frame_ptr_->points_.size());
    img_state.t_ = real_img_time;

    auto img_frame_it =  image_frame_window_.insert(img_state.image_frame_ptr_);
    deque<State>::iterator insert_it = insertState(img_state);

    if(USE_IMU){
        setImageIMUData(insert_it);

        if(!initFirstPoseFlag_){
            initFirstIMUPose(insert_it);
            initial_timestamp_ = real_img_time;
            repropagateIMU(insert_it, false);
            //imu init finish
        }

    }
    TicToc processTime;
    mProcess_.lock();
    processImage(insert_it, img_frame_it);
    mProcess_.unlock();

    mBuf_.unlock();
    // printf("process time: %f\n", processTime.toc());
}

void Estimator::inputIMU(double t, const Vector6d &imu_data)
{

    State imu_state;
    imu_state.imu_data_ = imu_data;
    imu_state.type_ = State::IMU;
    imu_state.t_ = t;

    first_imu_ = true;

    // ROS_ERROR("input imu at: %lf", t);

    mBuf_.lock();
    auto insert_it = insertState(imu_state);

    if(initFirstPoseFlag_){
        lpf_idx_++;
        mPropagate_.lock();
        repropagateIMU(insert_it, true);
        mPropagate_.unlock();
    }

    if (solver_flag_ == NON_LINEAR)
    {
        Publisher::PropagateData propogate_data;
        collectPropogationData(propogate_data);
        publisher_callbacks_.on_publish_propogate_odom_cb(propogate_data);
        #if 0
        pubLatestOdometry(*this);
        #endif

        // cout<<"propagate imu for "<<t - state_hist_[image_frame_window_.all_image_frame_ptr_.rbegin()->second->state_idx_].t_<<"s"<<endl;
    }

    mBuf_.unlock();
    return;
}

void Estimator::repropagateIMU(const deque<State>::iterator start_it, const bool low_pass){
    auto last_it = start_it;
    last_it--;
    for(auto it = start_it; it != state_hist_.end(); it++, last_it++){
        switch (it->type_)
        {
        case State::IMU:{
            if(low_pass){
                propagateIMULowpass(*last_it, *it, min(1.0, 0.02*lpf_idx_));
            }
            else{
                propagateIMU(*last_it, *it);
            }
            break;
        }
        case State::IMAGE:
            break;
        default:
            break;
        }
    }
    return;
}

void Estimator::setImageIMUData(const deque<State>::iterator img_it){
    for(deque<State>::reverse_iterator rit(img_it); rit != state_hist_.rend(); rit++){
        if(rit->type_ == State::IMU){
            img_it->image_frame_ptr_->prev_acc_ = rit->imu_data_.topRows(3);
            img_it->image_frame_ptr_->prev_gyr_ = rit->imu_data_.bottomRows(3);

            img_it->Ba_ = rit->Ba_;
            img_it->Bg_ = rit->Bg_;

            break;
        }
    }

    for(auto it = img_it; it != state_hist_.end(); it++){
        if(it->type_ == State::IMU){

            img_it->imu_data_ = it->imu_data_;

            break;
        }
    }
    return;
}

void Estimator::setImageState(const deque<State>::iterator img_it){
    img_it->image_frame_ptr_->R_ = img_it->Q_;
    img_it->image_frame_ptr_->T_ = img_it->P_;
    img_it->image_frame_ptr_->V_ = img_it->V_;
    img_it->image_frame_ptr_->Ba_ = img_it->Ba_;
    img_it->image_frame_ptr_->Bg_ = img_it->Bg_;

    img_it->P_lpf_ = img_it->P_;
    img_it->Q_lpf_ = img_it->Q_;
    img_it->V_lpf_ = img_it->V_;
    return;
}

void Estimator::setStateFromImage(){
    for(auto it = state_hist_.begin(); it != state_hist_.end(); it++){
        if(it->type_ == State::IMAGE){
            it->t_ = it->image_frame_ptr_->t_ + it->image_frame_ptr_->td_;
            it->P_ = it->image_frame_ptr_->T_;
            it->Q_ = it->image_frame_ptr_->R_;
            it->V_ = it->image_frame_ptr_->V_;
            it->Ba_ = it->image_frame_ptr_->Ba_;
            it->Bg_ = it->image_frame_ptr_->Bg_;
        }
    }
    return;
}

deque<State>::iterator Estimator::insertState(const State& state){
    
    deque<State>::iterator state_it;

    if(state_hist_.size() == 0 || state.t_ >= state_hist_.back().t_){
        state_it = state_hist_.insert(state_hist_.end(), state);

    }
    else if(state.t_ < state_hist_.front().t_){
        state_it = state_hist_.insert(state_hist_.begin(), state);
    }
    else{
        //In the middle

        for(auto rit = state_hist_.rbegin(); rit!= state_hist_.rend(); rit++){
            if(state.t_ >= rit -> t_){

                state_it = state_hist_.insert(rit.base(), state);
                break;
            }
        }
    }

    if(state.type_ == State::IMAGE){

        state_it->image_frame_ptr_->state_idx_ = std::distance(state_hist_.begin(), state_it);

        for(auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++){
            if(frame_it->second->t_ + frame_it->second->td_ >state.t_){
                frame_it->second->state_idx_ ++;
            }
        }
    }

    return state_it;
}

void Estimator::updateFeatureTrackerMaxCnt(){

    if(solver_flag_ == NON_LINEAR){
        for(int i = 0; i < img_trackers_.size(); i++){

            if(img_trackers_[i]->last_frame_time_ < image_frame_window_.all_image_frame_ptr_.begin()->second->t_){
                img_trackers_[i]->featureTracker_.track_num = MIN_TRACK_NUM_PER_MODULE;
            }
            
        }
    }
    

    VectorXd track_num(img_trackers_.size());
    for(unsigned int i = 0; i< img_trackers_.size(); i++){
        track_num(i) = img_trackers_[i]->featureTracker_.track_num;
        // track_num(i) = img_trackers_[i]->f_manager_.long_track_num_;
    }

    double track_num_sum = track_num.sum();

    if(track_num_sum < 0.5)
        return;

    for(unsigned int i = 0; i< img_trackers_.size(); i++){
        img_trackers_[i]->featureTracker_.max_cnt = min(max(static_cast<int>(ceil(track_num(i) / track_num_sum * MAX_CNT)), MIN_TRACK_NUM_PER_MODULE), MAX_TRACK_NUM_PER_MODULE);
        // ROS_DEBUG("cam %d max cnt: %d, track num: %lf", i, int(img_trackers_[i]->featureTracker_.max_cnt), track_num(i));
        spdlog::debug("cam {} max cnt: {}, track num: {}", i, int(img_trackers_[i]->featureTracker_.max_cnt), track_num(i));
    }
    return;
}

bool Estimator::CheckKeepImageUpdatePriority(const int cam_unique_id, const double t){

    double this_priority = img_trackers_[cam_unique_id]->get_total_priority(t);
    
    double this_time_priority = img_trackers_[cam_unique_id]->get_this_time_priority(t);
    double this_feature_priority = img_trackers_[cam_unique_id]->get_feature_priority();
    double this_total_priority = this_time_priority * this_feature_priority;

    double this_last_frame_time = img_trackers_[cam_unique_id]->last_frame_time_;
    double this_last_keep_frame_time = img_trackers_[cam_unique_id]->last_keep_frame_time_;
    img_trackers_[cam_unique_id]->last_frame_time_ = t;

    img_trackers_[cam_unique_id]->frame_time_hist_.emplace_back(t);

    if(t - this_last_keep_frame_time < MIN_FRAME_INTERVAL_PER_MODULE){
        // ROS_DEBUG("frame too fast");
        spdlog::debug("frame too fast");
        return false;
    }

    // for(int i = 0; i < img_trackers_.size(); i++){
    //     if(cam_unique_id == i)
    //         continue;
    //     double priority = img_trackers_[i]->get_total_priority(t);
    //     double feature_priority = img_trackers_[i]->get_feature_priority();
    //     if(priority > this_priority && feature_priority > this_feature_priority){
    //         ROS_DEBUG("cam %d low priority %lf", cam_unique_id, this_priority);
    //         return false;
    //     }
    // }

    double min_last_t = t;

    for(int i = 0; i < img_trackers_.size(); i++){
        if(cam_unique_id == i)
            continue;

        if(!img_trackers_[i]->frame_time_hist_.empty()){
            min_last_t = min(min_last_t, img_trackers_[i]->frame_time_hist_.back());
        }

        double time_priority = img_trackers_[i]->get_time_priority(t);
        double feature_priority = img_trackers_[i]->get_feature_priority();

        double total_priority = time_priority * feature_priority;

        // if(feature_priority > this_feature_priority){
        // if(total_priority > this_total_priority){
        if(time_priority > this_time_priority && feature_priority > this_feature_priority){
            // ROS_INFO("cam %d low priority %lf", cam_unique_id, this_priority);
            return false;
        }
    }


    for(int i = 0; i < img_trackers_.size(); i++){

        img_trackers_[i]->clean_frame_time_hist(min_last_t);
    }

    // first frame
    if(image_frame_window_.all_image_frame_ptr_.empty()){
        img_trackers_[cam_unique_id]->reset_frame_time_priority();
        img_trackers_[cam_unique_id]->last_keep_frame_time_ = t;
        // ROS_DEBUG("first frame");
        spdlog::debug("first frame");
        return true;
    }

    // insert at back
    double last_frame_time = image_frame_window_.all_image_frame_ptr_.rbegin()->first;
    if(t >= last_frame_time){
        if(t - last_frame_time < MIN_FRAME_INTERVAL_FOR_OPT){
            img_trackers_[cam_unique_id]->increase_frame_time_priority();
            // ROS_DEBUG("inrease priority");
            spdlog::debug("inrease priority");
            return false;
        }
        else{
            img_trackers_[cam_unique_id]->reset_frame_time_priority();
            img_trackers_[cam_unique_id]->last_keep_frame_time_ = t;
            // ROS_DEBUG("insert at back");
            spdlog::debug("insert at back");
            return true;
        }
    }

    // insert at front
    double first_frame_time = image_frame_window_.all_image_frame_ptr_.begin()->first;
    if(t <= first_frame_time){
        // ROS_DEBUG("frame too old");
        spdlog::debug("frame too old");
        return false;
    }


    // insert in the middle
    for(auto rit = image_frame_window_.all_image_frame_ptr_.rbegin(); rit!= image_frame_window_.all_image_frame_ptr_ .rend(); rit++){
        if(rit->first > t){
            auto next_rit = next(rit);
            if(rit->first - t < MIN_FRAME_INTERVAL_FOR_OPT || t - next_rit->first < MIN_FRAME_INTERVAL_FOR_OPT){
                img_trackers_[cam_unique_id]->increase_frame_time_priority();
                // ROS_DEBUG("inrease priority");
                spdlog::debug("inrease priority");

                return false;
            }
            else{
                img_trackers_[cam_unique_id]->reset_frame_time_priority();
                img_trackers_[cam_unique_id]->last_keep_frame_time_ = t;
                // ROS_DEBUG("inrease in the middle");
                spdlog::debug("inrease in the middle");

                return true;
            }
        }
    }

    // ROS_ERROR("bug in keepImage");
    spdlog::error("bug in keepImage");
    // img_trackers_[cam_unique_id]->last_keep_frame_time_ = t;
    return true;
}

bool Estimator::IMUAvailable(double t)
{   
    if(!first_imu_){
        return false;
    }
    
    mBuf_.lock();
    double latest_imu_t = 0.0;
    for(auto it = state_hist_.rbegin(); it!= state_hist_.rend(); it++){
        if(it->type_ == State::IMU){
            latest_imu_t = it->t_;
            break;
        }
    }
    bool available = latest_imu_t > t;
    // cout<<"available: "<<available<<endl;
    //if(!available){
        //ROS_WARN("imu t  : %lf", latest_imu_t);
        //ROS_WARN("image t: %lf", t);
        //cout<<"dt: "<< latest_imu_t - t<<endl;
    //}
    mBuf_.unlock();

    return available;
}

bool Estimator::IMUInitReady(double img_time){
    for(auto it = state_hist_.begin(); it!=state_hist_.end(); it++)
    {
        if(it->type_ == State::IMU){
            if(img_time - it->t_ > 0.1){
                return true;
            }
            else{
                // ROS_WARN("imu not enough");
                spdlog::warn("imu not enough");
                return false;
            }
        }
    }
    // ROS_WARN("imu not arrived");
    spdlog::warn("imu not arrived");
    return false;
}

void Estimator::initFirstIMUPose(const deque<State>::iterator img_it)
{
    printf("init first imu pose\n");
    Eigen::Vector3d averAcc(0, 0, 0);

    int imu_cnt = 0;

    for(auto it = state_hist_.begin(); it!=img_it; it++)
    {
        if(it->type_ == State::IMU){
            averAcc = averAcc + it->imu_data_.topRows(3);
            imu_cnt ++;
        }
    }

    averAcc = averAcc / imu_cnt;
    // g_.z() = averAcc.norm();
    printf("averge acc %lf %lf %lf, norm: %lf\n", averAcc.x(), averAcc.y(), averAcc.z(), averAcc.norm());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    img_it->Q_ = R0;
    img_it->P_ = Vector3d::Zero();
    img_it->V_ = Vector3d::Zero();
    img_it->Ba_ = Vector3d::Zero();
    img_it->Bg_ = Vector3d::Zero();
    img_it->un_gyr_ = img_it->imu_data_.bottomRows(3);

    img_it->Q_lpf_ = R0;
    img_it->P_lpf_ = Vector3d::Zero();
    img_it->V_lpf_ = Vector3d::Zero();

    setImageState(img_it);

    img_it->image_frame_ptr_->pre_integration_.reset(new IntegrationBase{averAcc, img_it->imu_data_.bottomRows(3), Vector3d::Zero(), Vector3d::Zero(), imu_module_});

    cout << "init R0 " << endl << R0 << endl;

    initFirstPoseFlag_ = true;
    return;
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    initP_ = p;
    initR_ = r;
    return;
}

void Estimator::constructPreintegration(const deque<State>::iterator insert_state_it, const map<double, shared_ptr<ImageFrame>>::iterator insert_frame_it){
    if(insert_frame_it == image_frame_window_.all_image_frame_ptr_.end()){
        // repeat t or try to insert to the front when not empty, no insertion
        return;
    }

    if(insert_frame_it == image_frame_window_.all_image_frame_ptr_.begin()){
        // insert to the front, no preintegration before
        // should not happen after init
        return;
    }
    else{

        // add preintegration from the last frame to the current frame

        auto last_frame_state_it = insert_state_it;
        // ROS_INFO("insert state time: %lf", last_frame_state_it->t_);
        for(deque<State>::reverse_iterator rit(insert_state_it); rit != state_hist_.rend(); ++rit){
            if(rit->type_ == State::IMAGE){
                last_frame_state_it = next(rit).base();
                break;
            }
        }

        insert_frame_it->second->pre_integration_.reset(new IntegrationBase{last_frame_state_it->image_frame_ptr_->prev_acc_, last_frame_state_it->image_frame_ptr_->prev_gyr_, last_frame_state_it->Ba_, last_frame_state_it->Bg_, imu_module_});

        for(auto it = next(last_frame_state_it); it != next(insert_state_it); ++it){
            double dt = it->t_ - (it-1)->t_;
            insert_frame_it->second->pre_integration_->push_back(dt, it->imu_data_.topRows(3), it->imu_data_.bottomRows(3));
            // ROS_WARN("push back integration at time: %lf", it->t_);
        }

        // propagate to current frame
        propagateIMU(*(insert_state_it-1), *insert_state_it);
        setImageState(insert_state_it);

        // find first imu after frame
        // for(auto it = insert_state_it; it != state_hist_.end(); it++){
        //     if(it->type_ == State::IMU){
        //         double dt = insert_state_it->t_ - (insert_state_it-1)->t_;
        //         insert_frame_it->second->pre_integration_->push_back(dt, it->imu_data_.topRows(3), it->imu_data_.bottomRows(3));
        //     }
        // }


        // change preintegration from the current frame to the next frame
        if(next(insert_frame_it) == image_frame_window_.all_image_frame_ptr_.end()){
            // insert at last, no need to change preintegration after
        }
        else{
            shared_ptr<IntegrationBase> next_frame_integration(new IntegrationBase{insert_state_it->image_frame_ptr_->prev_acc_, insert_state_it->image_frame_ptr_->prev_gyr_, insert_state_it->Ba_, insert_state_it->Bg_, imu_module_});
            for(auto it = insert_state_it + 1; it != state_hist_.end(); it++){
                double dt = it->t_ - (it-1)->t_;
                next_frame_integration->push_back(dt, it->imu_data_.topRows(3), it->imu_data_.bottomRows(3));
                // ROS_ERROR("push back integration at time: %lf", it->t_);
                if(it->type_ == State::IMAGE){
                    // find first imu after frame
                    // for(auto it_find_imu = it; it_find_imu != state_hist_.end(); it_find_imu++){
                    //     if(it_find_imu->type_ == State::IMU){
                    //         next_frame_integration->push_back(dt, it_find_imu->imu_data_.topRows(3), it_find_imu->imu_data_.bottomRows(3));
                    //     }
                    // }

                    it->image_frame_ptr_->pre_integration_ = next_frame_integration;
                    break;
                }
                // propagateIMU(*(it-1), *it);
            }
        }
    }
    return;
}

void Estimator::reconstructPreintegration(){

    bool first_img = true;
    shared_ptr<IntegrationBase> next_frame_integration;
    for(auto state_it = state_hist_.begin(); state_it != state_hist_.end(); state_it++){
        Vector6d& imu_data = state_it->imu_data_;

        if(state_it->type_ == State::IMU){

            if(!first_img){
                next_frame_integration->push_back(state_it->t_ - (state_it-1)->t_, imu_data.topRows(3), imu_data.bottomRows(3));
                propagateIMU(*(state_it-1), *state_it);
            }
        }

        if(state_it->type_ == State::IMAGE){

            if(!first_img){

                next_frame_integration->push_back(state_it->t_ - (state_it-1)->t_, imu_data.topRows(3), imu_data.bottomRows(3));
                state_it->image_frame_ptr_->pre_integration_ = next_frame_integration;

            }

            next_frame_integration.reset(new IntegrationBase{state_it->image_frame_ptr_->prev_acc_, state_it->image_frame_ptr_->prev_gyr_, state_it->Ba_, state_it->Bg_, imu_module_});


            first_img = false;
        }
    }
    return;
}

void Estimator::addPreintegrationToNextFrame(unsigned int remove_frame_state_idx){
    if(remove_frame_state_idx == 0){
        return;
    }

    auto start_state_it = state_hist_.begin() + remove_frame_state_idx;
    auto& next_frame_integration = start_state_it->image_frame_ptr_->pre_integration_;
    for(auto it = next(start_state_it); it != state_hist_.end(); it++){
        double dt = it->t_ - (it-1)->t_;
        next_frame_integration->push_back(dt, it->imu_data_.topRows(3), it->imu_data_.bottomRows(3));
        if(it->type_ == State::IMAGE){
            it->image_frame_ptr_->pre_integration_ = next_frame_integration;
            break;
        }
    }
    return;
}

void Estimator::processImage(const deque<State>::iterator img_state_it, const map<double, shared_ptr<ImageFrame>>::iterator img_frame_it)
{
    // ROS_DEBUG("new image coming ------------------------------------------");
    spdlog::debug("new image coming ------------------------------------------");
    // ROS_DEBUG("Adding feature points %lu", img_state_it->image_frame_ptr_->points_.size());
    spdlog::debug("Adding feature points {}", img_state_it->image_frame_ptr_->points_.size());

    int cam_unique_id = img_state_it->image_frame_ptr_->cam_module_unique_id_;
     FeatureTracker* featureTracker_ptr = &img_trackers_[cam_unique_id]->featureTracker_;
    FeatureManager* f_manager_ptr = &img_trackers_[cam_unique_id]->f_manager_;

    TicToc t_add_feature;
    if ( f_manager_ptr->addFeatureCheckParallax(img_state_it->image_frame_ptr_->points_, img_trackers_[cam_unique_id]->cam_info_.td_))
    {
        marginalization_flag_ = MARGIN_OLD;
        img_state_it->image_frame_ptr_->is_key_frame_ = true;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag_ = MARGIN_SECOND_NEW;
        img_state_it->image_frame_ptr_->is_key_frame_ = false;
        //printf("non-keyframe\n");
    }
    // ROS_DEBUG();
    // ROS_DEBUG("%s", marginalization_flag_ ? "Non-keyframe" : "Keyframe");
    // ROS_DEBUG("Solving %d", frame_count_);
    // ROS_DEBUG("cam %d, number of feature: %d", cam_unique_id, f_manager_ptr->getFeatureCount());
    spdlog::debug("addFeatureCheckParallax costs: {}ms", t_add_feature.toc());
    spdlog::debug("%s", marginalization_flag_ ? "Non-keyframe" : "Keyframe");
    spdlog::debug("Solving {}", frame_count_);
    spdlog::debug("cam {}, number of feature: {}", cam_unique_id, f_manager_ptr->getFeatureCount());

    if(img_frame_it != image_frame_window_.all_image_frame_ptr_.begin())
        propagateIMU(*(img_state_it-1), *img_state_it);
    setImageState(img_state_it);
    constructPreintegration(img_state_it, img_frame_it);

    // ROS_WARN("insert frame at frame_cnt %d, window size %d", frame_count_, image_frame_window_.all_image_frame_.size());
    // all_image_frame_.insert(make_pair(feature_frame.t_, imageframe));
    // tmp_pre_integration_ = new IntegrationBase{acc_0_, gyr_0_, Bas_[frame_count_], Bgs_[frame_count_], imu_module_};

    // if(ESTIMATE_EXTRINSIC == 2)
    // {
    //     ROS_INFO("calibrating extrinsic param, rotation movement is needed");
    //     if (frame_count_ != 0)
    //     {
    //         vector<pair<Vector3d, Vector3d>> corres = f_manager_ptr->getCorresponding(frame_count_ - 1, frame_count_);
    //         Matrix3d calib_ric;
    //         if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations_[frame_count_]->delta_q, calib_ric))
    //         {
    //             ROS_WARN("initial extrinsic rotation calib success");
    //             ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
    //             ric[0] = calib_ric;
    //             RIC[0] = calib_ric;
    //             ESTIMATE_EXTRINSIC = 1;
    //         }
    //     }
    // }

    if (solver_flag_ == INITIAL)
    {
        // monocular + IMU initilization
        // if (!STEREO && !DEPTH && USE_IMU)
        // // if (!STEREO && USE_IMU)
        // {
        //     if (frame_count_ == WINDOW_SIZE)
        //     {
        //         bool result = false;
        //         if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
        //         {
        //             result = initialStructure();
        //             initial_timestamp = header;
        //         }
        //         if(result)
        //         {
        //             optimization();
        //             updateLatestStates();
        //             solver_flag_ = NON_LINEAR;
        //             slideWindow();
        //             ROS_INFO("Initialization finish!");
        //         }
        //         else
        //             slideWindow();
        //     }
        // }

        // stereo + IMU initilization
        if(img_trackers_[cam_unique_id]->cam_info_.stereo_ && USE_IMU)
        {
            f_manager_ptr->initFramePoseByPnP(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id], img_trackers_[cam_unique_id]->cam_info_.tic_[0], img_trackers_[cam_unique_id]->cam_info_.ric_[0]);
            f_manager_ptr->triangulate(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id],
                                        img_trackers_[cam_unique_id]->cam_info_.tic_[0],
                                        img_trackers_[cam_unique_id]->cam_info_.ric_[0],
                                        img_trackers_[cam_unique_id]->cam_info_.tic_[1],
                                        img_trackers_[cam_unique_id]->cam_info_.ric_[1]);
            if (frame_count_ == WINDOW_SIZE - 1)
            {
                // map<double, ImageFrame>::iterator frame_it;
                // int i = 0;
                // for (auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++)
                // {
                //     cout<<"Ps "<<i<<":"<< Ps_[i].transpose()<<endl;

                //     frame_it->second.R_ = Rs_[i];
                //     frame_it->second.T_ = Ps_[i];
                //     i++;

                // }

                reconstructPreintegration();
                solveGyroscopeBias(image_frame_window_.all_image_frame_ptr_);

                Vector3d& last_Bg = state_hist_.front().Bg_;

                Vector3d zero_vec = Vector3d::Zero();
                for (auto& state : state_hist_)
                {
                    if(state.type_ == State::IMAGE){
                        state.image_frame_ptr_->pre_integration_->repropagate(zero_vec, state.image_frame_ptr_->Bg_);
                        state.Bg_ = state.image_frame_ptr_->Bg_;
                        last_Bg = state.Bg_;
                    }
                    else{
                        state.Bg_ = last_Bg;
                    }
                }

                // double last_t = 0.0;
                // i = 0;
                // for(auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++){

                //     cout<<"frame "<<i<<endl;
                //     cout<<"dt: "<<frame_it->first - last_t<<endl;
                //     cout<<"pre int t: "<<frame_it->second->pre_integration_->sum_dt<<endl;

                //     cout<<"pre int acc0: "<<frame_it->second->pre_integration_->linearized_acc.transpose()<<endl;

                //     cout<<"pre int dp: "<<frame_it->second->pre_integration_->delta_p.transpose()<<endl;

                //     cout<<"Ba: "<< frame_it->second->Ba_.transpose()<<endl;
                //     cout<<"Bg: "<< frame_it->second->Bg_.transpose()<<endl<<endl;

                //     last_t = frame_it->first;
                //     i++;

                // }

                processWindow(cam_unique_id);
                updateLatestStates(cam_unique_id);
                solver_flag_ = NON_LINEAR;
                // ROS_INFO("Initialization by stereo finish!");
                spdlog::info("Initialization by stereo finish!");
                std::cout<<"tic1: "<<img_trackers_[cam_unique_id]->cam_info_.tic_[1].transpose()<<std::endl;
                std::cout<<"ric1: \n"<<img_trackers_[cam_unique_id]->cam_info_.ric_[1].toRotationMatrix()<<std::endl;
            }
        }

        // stereo only initilization
        // if(STEREO && !USE_IMU)
        // {
        //     f_manager_ptr->initFramePoseByPnP(frame_count_, Ps, Rs, tic, ric);
        //     f_manager_ptr->triangulate(frame_count_, Ps, Rs, tic, ric);
        //     optimization();

        //     if(frame_count_ == WINDOW_SIZE)
        //     {
        //         optimization();
        //         updateLatestStates();
        //         solver_flag_ = NON_LINEAR;
        //         slideWindow();
        //         ROS_INFO("Initialization finish!");
        //     }
        // }

        // depth + IMU initilization
          if(img_trackers_[cam_unique_id]->cam_info_.depth_ && USE_IMU){

            if(frame_count_ > 0){
                if(f_manager_ptr->initFramePoseByICP(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id], img_trackers_[cam_unique_id]->cam_info_.tic_[0], img_trackers_[cam_unique_id]->cam_info_.ric_[0])){
                    // if(f_manager_ptr->checkParallax(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id])){
                    //     f_manager_ptr->triangulate(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id], img_trackers_[cam_unique_id]->cam_info_.tic_[0], img_trackers_[cam_unique_id]->cam_info_.ric_[0]);
                    // }
                }
                else{
                    // ROS_ERROR("init by icp failed!");
                    spdlog::error("init by icp failed!");
                    auto second_last_frame_ptr = *next(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id].rbegin());
                    image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id].back()->R_ = second_last_frame_ptr->R_;
                    image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id].back()->T_ = second_last_frame_ptr->T_;
                }

                f_manager_ptr->triangulate(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id], img_trackers_[cam_unique_id]->cam_info_.tic_[0], img_trackers_[cam_unique_id]->cam_info_.ric_[0]);

                if (frame_count_ == WINDOW_SIZE - 1)
                {
                    // map<double, ImageFrame>::iterator frame_it;
                    // int i = 0;
                    // for (auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++)
                    // {
                    //     cout<<"Ps "<<i<<":"<< Ps_[i].transpose()<<endl;

                    //     frame_it->second.R_ = Rs_[i];
                    //     frame_it->second.T_ = Ps_[i];
                    //     i++;

                    // }

                    reconstructPreintegration();
                    solveGyroscopeBias(image_frame_window_.all_image_frame_ptr_);

                    Vector3d& last_Bg = state_hist_.front().Bg_;

                    Vector3d zero_vec = Vector3d::Zero();
                    for (auto& state : state_hist_)
                    {
                        if(state.type_ == State::IMAGE){
                            state.image_frame_ptr_->pre_integration_->repropagate(zero_vec, state.image_frame_ptr_->Bg_);
                            state.Bg_ = state.image_frame_ptr_->Bg_;
                            last_Bg = state.Bg_;
                        }
                        else{
                            state.Bg_ = last_Bg;
                        }
                    }

                    // double last_t = 0.0;
                    // i = 0;
                    // for(auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++){

                    //     cout<<"frame "<<i<<endl;
                    //     cout<<"dt: "<<frame_it->first - last_t<<endl;
                    //     cout<<"pre int t: "<<frame_it->second->pre_integration_->sum_dt<<endl;

                    //     cout<<"pre int acc0: "<<frame_it->second->pre_integration_->linearized_acc.transpose()<<endl;

                    //     cout<<"pre int dp: "<<frame_it->second->pre_integration_->delta_p.transpose()<<endl;

                    //     cout<<"Ba: "<< frame_it->second->Ba_.transpose()<<endl;
                    //     cout<<"Bg: "<< frame_it->second->Bg_.transpose()<<endl<<endl;

                    //     last_t = frame_it->first;
                    //     i++;

                    // }

                    processWindow(cam_unique_id);
                    updateLatestStates(cam_unique_id);
                    solver_flag_ = NON_LINEAR;
                    // ROS_INFO("Initialization by depth finish!");
                    spdlog::info("Initialization by depth finish!");
                }
            }
        }

        if(frame_count_ < WINDOW_SIZE)
        {
            frame_count_++;
        }

    }
    else
    {
        // for(int i = 0; i < WINDOW_SIZE+1; i++){
        //     cout<<"Ps "<<i<<": "<<Ps[i].transpose()<<endl;
        // }
        TicToc t_solve;
        // if(!USE_IMU)
        //     f_manager_ptr->initFramePoseByPnP(frame_count_, Ps, Rs, tic, ric);
        // f_manager_ptr->triangulate(frame_count_, Ps_, Rs_, tic_, ric_);
        if(img_trackers_[cam_unique_id]->cam_info_.stereo_){
            f_manager_ptr->triangulate(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id],
                                        img_trackers_[cam_unique_id]->cam_info_.tic_[0],
                                        img_trackers_[cam_unique_id]->cam_info_.ric_[0],
                                        img_trackers_[cam_unique_id]->cam_info_.tic_[1],
                                        img_trackers_[cam_unique_id]->cam_info_.ric_[1]);
        }
        else {
            f_manager_ptr->triangulate(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id], img_trackers_[cam_unique_id]->cam_info_.tic_[0], img_trackers_[cam_unique_id]->cam_info_.ric_[0]);
        }
        processWindow(cam_unique_id);
        // ROS_DEBUG("solver costs: %fms", t_solve.toc());
        spdlog::debug("solver costs: {}ms", t_solve.toc());

        // if (! MULTIPLE_THREAD)
        // {
        //     featureTracker_ptr->removeOutliers(removeIndex);
        //     predictPtsInNextFrame(cam_unique_id);
        // }

        // if (failureDetection())
        // {
        //     ROS_WARN("failure detection!");
        //     failure_occur = 1;
        //     clearState();
        //     setParameter();
        //     ROS_WARN("system reboot!");
        //     return;
        // }

        // prepare output of VINS
        key_poses_.clear();

        for(auto frame_it : image_frame_window_.all_image_frame_ptr_){
            key_poses_.push_back(frame_it.second->T_);
        }

        updateLatestStates(cam_unique_id);
    }
    return;
}

inline bool Estimator::needMarginalization(){
    return image_frame_window_.all_image_frame_ptr_.size() >= WINDOW_SIZE;
}

void Estimator::processWindow(const int img_cam_unique_id){

    TicToc tt;

    auto current_frame = image_frame_window_.cam_wise_image_frame_ptr_[img_cam_unique_id].back();
    double current_time = current_frame->t_ + current_frame->td_;
    bool need_opt = current_time - last_opt_time_ > MIN_OPT_INTERVAL;

    if(need_opt){
        // tt.tic();
        optimization();
        // printf("opt time: %lf ms\n", tt.toc());
        last_opt_time_ = current_time;

        // cout<<"td "<<img_cam_unique_id<<": "<<current_frame->td_<<endl;
    }

    // reorderWindow();
    // if(needMarginalization())
    //     slideWindow(img_cam_unique_id);
    // reconstructPreintegration();


    // tt.tic();
    // constructMarginalizationInfo();
    // ROS_ERROR("marginalization info time: %lf ms", tt.toc());

    if(marginalization_flag_ == MARGIN_OLD){
        frame_to_margin_ = image_frame_window_.all_image_frame_ptr_.begin()->second;
    }
    else if(marginalization_flag_ == MARGIN_SECOND_NEW){
        frame_to_margin_ = image_frame_window_.cam_wise_image_frame_ptr_[img_cam_unique_id][image_frame_window_.cam_wise_image_frame_ptr_[img_cam_unique_id].size()-2];
    }


    if(need_opt){
        tt.tic();
        FeatureManager* f_manager_ptr = &img_trackers_[img_cam_unique_id]->f_manager_;
        f_manager_ptr->outliersRejection();
        f_manager_ptr->removeFailures();
        // ROS_DEBUG("outlier time: %lf ms", tt.toc());
        spdlog::debug("outlier time: {} ms", tt.toc());

        if(ESTIMATE_TD){
            reorderWindow();
            reconstructPreintegration();
        }
    }

    if(needMarginalization()){
        // tt.tic();
        constructMarginalizationFactor();
        // printf("marginalization factor time: %lf ms\n", tt.toc());
        // tt.tic();

        slideWindow(frame_to_margin_);
        // printf("slide window time: %lf ms\n", tt.toc());

    }
    return;
}

void Estimator::vector2double()
{
    auto& frame0ptr = image_frame_window_.all_image_frame_ptr_.begin()->second; 
    origin_R0 = Utility::R2ypr(frame0ptr->R_.toRotationMatrix());
    origin_P0 = frame0ptr->T_;

    for (int i = 0; i < img_trackers_.size(); i++)
    {
        auto& f_manager = img_trackers_[i]->f_manager_;
        f_manager.setInvDepth();
    }
    return;
}

void Estimator::double2vector()
{

    auto& frame0ptr = image_frame_window_.all_image_frame_ptr_.begin()->second; 
    // Vector3d origin_R0 = Utility::R2ypr(frame0ptr->R_.toRotationMatrix());
    // Vector3d origin_P0 = frame0ptr->T_;

    if (failure_occur_)
    {
        origin_R0 = Utility::R2ypr(last_R0_);
        origin_P0 = last_P0_;
        failure_occur_ = false;
    }

    if(USE_IMU)
    {
        auto para_pose = frame0ptr->para_Pose_;

        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_pose[6],
                                                          para_pose[3],
                                                          para_pose[4],
                                                          para_pose[5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            // ROS_DEBUG("euler singular point!");
            spdlog::debug("euler singular point!");
            rot_diff = frame0ptr->R_ * Quaterniond(para_pose[6],
                                           para_pose[3],
                                           para_pose[4],
                                           para_pose[5]).toRotationMatrix().transpose();
        }

        Vector3d opt_P0 = frame0ptr->T_;

        int frame_i = 0;
        for(auto it = image_frame_window_.all_image_frame_ptr_.begin(); it != image_frame_window_.all_image_frame_ptr_.end(); it++, frame_i++){
            auto frame_ptr = it->second;

            // auto para_pose_i = frame_ptr->para_Pose_;

            frame_ptr->R_ = rot_diff * frame_ptr->R_.normalized();

            // frame_ptr->T_ = rot_diff * Vector3d(para_pose_i[0] - para_pose[0],
            //                         para_pose_i[1] - para_pose[1],
            //                         para_pose_i[2] - para_pose[2]) + origin_P0;

            frame_ptr->T_ = rot_diff * (frame_ptr->T_ - opt_P0) + origin_P0;


            frame_ptr->V_ = rot_diff * frame_ptr->V_;

        }

    }

    for (int i = 0; i < img_trackers_.size(); i++)
    {
        auto& f_manager = img_trackers_[i]->f_manager_;
        f_manager.setDepth();
    }

    setStateFromImage();
    return;
}

// bool Estimator::failureDetection()
// {
//     return false;
//     // if (f_manager.last_track_num < 2)
//     // {
//     //     ROS_INFO(" little feature %d", f_manager.last_track_num);
//     //     //return true;
//     // }
//     if (Bas[WINDOW_SIZE].norm() > 2.5)
//     {
//         ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
//         return true;
//     }
//     if (Bgs[WINDOW_SIZE].norm() > 1.0)
//     {
//         ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
//         return true;
//     }
//     /*
//     if (tic(0) > 1)
//     {
//         ROS_INFO(" big extri param estimation %d", tic(0) > 1);
//         return true;
//     }
//     */
//     Vector3d tmp_P = Ps[WINDOW_SIZE];
//     if ((tmp_P - last_P).norm() > 5)
//     {
//         //ROS_INFO(" big translation");
//         //return true;
//     }
//     if (abs(tmp_P.z() - last_P.z()) > 1)
//     {
//         //ROS_INFO(" big z translation");
//         //return true; 
//     }
//     Matrix3d tmp_R = Rs[WINDOW_SIZE];
//     Matrix3d delta_R = tmp_R.transpose() * last_R;
//     Quaterniond delta_Q(delta_R);
//     double delta_angle;
//     delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
//     if (delta_angle > 50)
//     {
//         ROS_INFO(" big delta_angle ");
//         //return true;
//     }
//     return false;
// }

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    problem_ptr_ = new ceres::Problem();
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    for(auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++){
        ceres::Manifold *local_parameterization = new PoseManifold();
        problem_ptr_->AddParameterBlock(frame_it->second->para_Pose_, SIZE_POSE, local_parameterization);
        if(USE_IMU)
            problem_ptr_->AddParameterBlock(frame_it->second->para_SpeedBias_, SIZE_SPEEDBIAS);

    }
    // if(!USE_IMU || !last_prior_ptr_)
    if(!USE_IMU || !last_marginalization_info_)
        problem_ptr_->SetParameterBlockConstant(image_frame_window_.all_image_frame_ptr_.begin()->second->para_Pose_);
    // else
    //     problem_ptr_->SetParameterBlockConstant(image_frame_window_.all_image_frame_ptr_.begin()->second->para_Pose_);

    bool v_enough = true;
    for(auto& frame_it : image_frame_window_.all_image_frame_ptr_){
        // auto& v = image_frame_window_.all_image_frame_ptr_.begin()->second->V_;
        auto& v = frame_it.second->V_;
        if(v.norm() < 0.2){
            v_enough = false;
            break;
        }
    }

    for (unsigned int cam_unique_id = 0; cam_unique_id < img_trackers_.size(); cam_unique_id++)
    {
        ceres::Manifold *local_parameterization = new PoseManifold();

        auto& para_Ex_Pose = img_trackers_[cam_unique_id]->cam_info_.para_Ex_Pose_;
        auto& para_Td = img_trackers_[cam_unique_id]->cam_info_.td_;

        problem_ptr_->AddParameterBlock(para_Ex_Pose[0], SIZE_POSE, local_parameterization);
        if(img_trackers_[cam_unique_id]->cam_info_.stereo_){
            problem_ptr_->AddParameterBlock(para_Ex_Pose[1], SIZE_POSE, local_parameterization);
        }

        // if ((ESTIMATE_EXTRINSIC && v_enough))
        // {
        //     // ROS_INFO("estimate extinsic param");
        //     openExEstimation_ = 1;
        // }
        // else
        // {
        //     // ROS_INFO("fix extinsic param");
        //     for(unsigned int j = 0; j < para_Ex_Pose.size(); j++){
        //         problem_ptr_->SetParameterBlockConstant(para_Ex_Pose[j]);
        //     }
        // }

        problem_ptr_->AddParameterBlock(&para_Td, 1);


        // if (!ESTIMATE_TD || !v_enough){
        //     problem_ptr_->SetParameterBlockConstant(&para_Td);
        // }
    }

    // cout<<"num param blocks: "<< problem_ptr_->NumParameterBlocks()<<endl;
    // cout<<"num param : "<< problem_ptr_->NumParameters()<<endl;

    if (last_marginalization_info_ && last_marginalization_info_->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
        problem_ptr_->AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks_);
    }

    // prior factor
    // if (last_prior_ptr_) {
    //     // last_prior_ptr_->getKeepParamsPointers();
    //     problem_ptr_->AddResidualBlock(last_prior_ptr_, nullptr, last_prior_ptr_->getKeepParamsPointers());
    //     // marginalizer_->addPrior(last_prior_ptr_);
    // }


    if(USE_IMU)
    {
        for(auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); next(frame_it) != image_frame_window_.all_image_frame_ptr_.end(); frame_it++){
            auto next_frame_it = next(frame_it);
            auto pre_integration = next_frame_it->second->pre_integration_;
            if (pre_integration->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integration);
            auto frame_ptr = frame_it->second;
            auto next_frame_ptr = next_frame_it->second;
            problem_ptr_->AddResidualBlock(imu_factor, NULL, frame_ptr->para_Pose_, frame_ptr->para_SpeedBias_, next_frame_ptr->para_Pose_, next_frame_ptr->para_SpeedBias_);
        }

    }

    int f_m_cnt = 0;
    for (unsigned int cam_unique_id = 0; cam_unique_id < img_trackers_.size(); cam_unique_id++){
        imgTracker& image_tracker = *img_trackers_[cam_unique_id];
        FeatureManager &f_manager_ref = image_tracker.f_manager_;
        camera_module_info& cam_info = image_tracker.cam_info_;
        auto& para_Ex_Pose = img_trackers_[cam_unique_id]->cam_info_.para_Ex_Pose_;
        auto& para_Td = img_trackers_[cam_unique_id]->cam_info_.td_;

        const int img_rows = img_trackers_[cam_unique_id]->cam_info_.img_height_;
        const double tr = img_trackers_[cam_unique_id]->cam_info_.tr_;

        unsigned int long_track_feature_num = 0;

        int depth_factor_cnt = 0;
        int depth_reproj_factor_cnt = 0;
        int reproj_factor_cnt = 0;
        int stereo_2f2c_factor_cnt = 0;
        int stereo_2f1c_factor_cnt = 0;


        for (auto &it_per_id : f_manager_ref.feature_)
        {
            if (it_per_id.second.solve_flag != FeaturePerId::LONGTRACK || it_per_id.second.feature_per_frame.size() < 2)
                continue;

            auto& para_Feature = it_per_id.second.inv_depth;
            it_per_id.second.solve_flag = FeaturePerId::ESTIMATED;
            long_track_feature_num++;


            int imu_i = it_per_id.second.start_frame, imu_j = imu_i - 1;

            auto frame_i_ptr = image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id][imu_i];

            Vector3d pts_i = it_per_id.second.feature_per_frame.front().point;
            double depth_i = it_per_id.second.feature_per_frame.front().depth;

            for (auto &it_per_frame : it_per_id.second.feature_per_frame)
            {
                imu_j++;
                auto frame_j_ptr = image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id][imu_j];
                if (imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;
                    double depth_j = it_per_frame.depth;
                    if( image_tracker.cam_info_.depth_ && it_per_frame.is_depth){
                        ProjectionTwoFrameOneCamDepthFactor *f_dep = new ProjectionTwoFrameOneCamDepthFactor(pts_i, pts_j, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocity,
                                                                    it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td, depth_j, it_per_id.second.feature_per_frame.front().uv.y(), it_per_frame.uv.y(), img_rows, tr);
                        problem_ptr_->AddResidualBlock(f_dep, loss_function, frame_i_ptr->para_Pose_, frame_j_ptr->para_Pose_, para_Ex_Pose[0], &para_Feature, &para_Td);
                        depth_reproj_factor_cnt++;
                    }
                    else{
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocity,
                                                                    it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td, it_per_id.second.feature_per_frame.front().uv.y(), it_per_frame.uv.y(), img_rows, tr);
                        problem_ptr_->AddResidualBlock(f_td, loss_function, frame_i_ptr->para_Pose_, frame_j_ptr->para_Pose_, para_Ex_Pose[0], &para_Feature, &para_Td);
                        reproj_factor_cnt++;
                    }

                }
                else{
                    if(image_tracker.cam_info_.depth_ && it_per_frame.is_depth){
                        depthFactor *f_dep = new depthFactor(depth_i);
                        problem_ptr_->AddResidualBlock(f_dep, loss_function, &para_Feature);
                        depth_factor_cnt++;
                    }
                }

                if(image_tracker.cam_info_.stereo_ && it_per_frame.is_stereo)
                {
                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if(imu_i != imu_j)
                    {
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocityRight,
                                                                    it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td);
                        problem_ptr_->AddResidualBlock(f, loss_function, frame_i_ptr->para_Pose_, frame_j_ptr->para_Pose_, para_Ex_Pose[0], para_Ex_Pose[1], &para_Feature, &para_Td);
                        stereo_2f1c_factor_cnt++;
                    }
                    else
                    {
                        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocityRight,
                                                                    it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td);
                        problem_ptr_->AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], &para_Feature, &para_Td);
                        stereo_2f2c_factor_cnt++;
                    }

                }
                f_m_cnt++;
            }
        }

        if(!ESTIMATE_EXTRINSIC || long_track_feature_num < 10 || !v_enough){
            for(unsigned int j = 0; j < para_Ex_Pose.size(); j++){
                problem_ptr_->SetParameterBlockConstant(para_Ex_Pose[j]);
            }
        }

        if(!ESTIMATE_TD || long_track_feature_num < 10 || !v_enough){
            problem_ptr_->SetParameterBlockConstant(&para_Td);
        }

        // ROS_WARN("cam %d depth_factor_cnt: %d, depth_reproj_factor_cnt: %d, reproj_factor_cnt: %d, long_track_feature_num: %ld, stereo_2f1c_factor_cnt: %d, stereo_2f2c_factor_cnt: %d", cam_unique_id, depth_factor_cnt, depth_reproj_factor_cnt, reproj_factor_cnt, long_track_feature_num, stereo_2f1c_factor_cnt, stereo_2f2c_factor_cnt);
    }
    // ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    spdlog::debug("visual measurement count: {}", f_m_cnt);
    // printf("prepare for ceres: %f \n", t_prepare.toc());

    // cout<<"num res blocks: "<< problem_ptr_->NumResidualBlocks()<<endl;
    // cout<<"num res: "<< problem_ptr_->NumResiduals()<<endl;

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;

#ifndef CERES_NO_CUDA
    options.dense_linear_algebra_library_type = ceres::CUDA;
#endif

    //options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag_ == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem_ptr_, &summary);
    // cout << summary.BriefReport() << endl;
    // printf("solver costs: %f \n", t_solver.toc());
    double2vector();
    return;
}

void Estimator::constructMarginalizationFactor(){

    TicToc t_whole_marginalization;
    int img_cam_unique_id = frame_to_margin_->cam_module_unique_id_;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    if (marginalization_flag_ == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        // vector2double();

        if (last_marginalization_info_ && last_marginalization_info_->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++)
            {
                if (last_marginalization_parameter_blocks_[i] == image_frame_window_.cam_wise_image_frame_ptr_[img_cam_unique_id].front()->para_Pose_ ||
                    last_marginalization_parameter_blocks_[i] == image_frame_window_.cam_wise_image_frame_ptr_[img_cam_unique_id].front()->para_SpeedBias_)
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks_,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {

            auto& frame0ptr = image_frame_window_.all_image_frame_ptr_.begin()->second;
            auto& frame1ptr = next(image_frame_window_.all_image_frame_ptr_.begin())->second;

            if (frame1ptr->pre_integration_->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(frame1ptr->pre_integration_);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{frame0ptr->para_Pose_, frame0ptr->para_SpeedBias_, frame1ptr->para_Pose_, frame1ptr->para_SpeedBias_},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            auto& image_tracker = *img_trackers_[img_cam_unique_id];
            auto& f_manager = image_tracker.f_manager_;
            auto& para_Ex_Pose = img_trackers_[img_cam_unique_id]->cam_info_.para_Ex_Pose_;
            auto& para_Td = img_trackers_[img_cam_unique_id]->cam_info_.td_;

            const int img_rows = img_trackers_[img_cam_unique_id]->cam_info_.img_height_;
            const double tr = img_trackers_[img_cam_unique_id]->cam_info_.tr_;

            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature_)
            {
                // if (it_per_id.second.solve_flag != FeaturePerId::ESTIMATED)
                //     continue;

                int feature_track_cnt = it_per_id.second.feature_per_frame.size();
                if (!(feature_track_cnt >= 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.second.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.second.feature_per_frame.front().point;
                double depth_i = it_per_id.second.feature_per_frame.front().depth;
                auto frame_i_ptr = image_frame_window_.cam_wise_image_frame_ptr_[img_cam_unique_id][imu_i];

                auto& para_Feature = it_per_id.second.inv_depth;

                for (auto &it_per_frame : it_per_id.second.feature_per_frame)
                {
                    imu_j++;
                    auto frame_j_ptr = image_frame_window_.cam_wise_image_frame_ptr_[img_cam_unique_id][imu_j];
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        double depth_j = it_per_frame.depth;

                        if(image_tracker.cam_info_.depth_ && it_per_frame.is_depth){
                            ProjectionTwoFrameOneCamDepthFactor *f_dep = new ProjectionTwoFrameOneCamDepthFactor(pts_i, pts_j, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocity,
                                                                 it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td, depth_j, it_per_id.second.feature_per_frame.front().uv.y(), it_per_frame.uv.y(), img_rows, tr);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_dep, loss_function,
                                                                                        vector<double *>{frame_i_ptr->para_Pose_, frame_j_ptr->para_Pose_, para_Ex_Pose[0], &para_Feature, &para_Td},
                                                                                        vector<int>{0, 3});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else{
                            ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocity,
                                                                          it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td, it_per_id.second.feature_per_frame.front().uv.y(), it_per_frame.uv.y(), img_rows, tr);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{frame_i_ptr->para_Pose_, frame_j_ptr->para_Pose_, para_Ex_Pose[0], &para_Feature, &para_Td},
                                                                                        vector<int>{0, 3});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                    else{
                        if(image_tracker.cam_info_.depth_ && it_per_frame.is_depth){
                            depthFactor *f_dep = new depthFactor(depth_i);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_dep, loss_function,
                                                                                        vector<double *>{&para_Feature},
                                                                                        vector<int>{0});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }

                    if(image_tracker.cam_info_.stereo_ && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocityRight,
                                                                          it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{frame_i_ptr->para_Pose_, frame_j_ptr->para_Pose_, para_Ex_Pose[0], para_Ex_Pose[1], &para_Feature, &para_Td},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.second.feature_per_frame.front().velocity, it_per_frame.velocityRight,
                                                                          it_per_id.second.feature_per_frame.front().cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], &para_Feature, &para_Td},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        // ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        spdlog::debug("pre marginalization {} ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        // ROS_DEBUG("marginalization %f ms", t_margin.toc());
        spdlog::debug("marginalization {} ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (auto frame_it = next(image_frame_window_.all_image_frame_ptr_.begin()); frame_it !=  image_frame_window_.all_image_frame_ptr_.end(); frame_it++)
        {
            addr_shift[reinterpret_cast<long>(frame_it->second->para_Pose_)] = frame_it->second->para_Pose_;
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(frame_it->second->para_SpeedBias_)] = frame_it->second->para_SpeedBias_;
        }


        for (unsigned int i = 0; i < img_trackers_.size(); i++){
            addr_shift[reinterpret_cast<long>(img_trackers_[i]->cam_info_.para_Ex_Pose_[0])] = img_trackers_[i]->cam_info_.para_Ex_Pose_[0];
            if(img_trackers_[i]->cam_info_.stereo_){
                    addr_shift[reinterpret_cast<long>(img_trackers_[i]->cam_info_.para_Ex_Pose_[1])] = img_trackers_[i]->cam_info_.para_Ex_Pose_[1];
                }
        }

        for (unsigned int i = 0; i < img_trackers_.size(); i++)
            addr_shift[reinterpret_cast<long>(&img_trackers_[i]->cam_info_.td_)] = &img_trackers_[i]->cam_info_.td_;

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info_)
            delete last_marginalization_info_;
        last_marginalization_info_ = marginalization_info;
        last_marginalization_parameter_blocks_ = parameter_blocks;

    }
    else
    {
        if (last_marginalization_info_ &&
            std::count(std::begin(last_marginalization_parameter_blocks_), std::end(last_marginalization_parameter_blocks_), frame_to_margin_->para_Pose_))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            // vector2double();
            if (last_marginalization_info_ && last_marginalization_info_->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++)
                {
                    // ROS_ASSERT(last_marginalization_parameter_blocks_[i] != frame_to_margin_->para_SpeedBias_);
                    assert(last_marginalization_parameter_blocks_[i] != frame_to_margin_->para_SpeedBias_);
                    if (last_marginalization_parameter_blocks_[i] == frame_to_margin_->para_Pose_)
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks_,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            // ROS_DEBUG("begin marginalization");
            spdlog::debug("begin marginalization");
            marginalization_info->preMarginalize();
            // ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());
            spdlog::debug("end pre marginalization, {} ms", t_pre_margin.toc());

            TicToc t_margin;
            // ROS_DEBUG("begin marginalization");
            spdlog::debug("begin marginalization");
            marginalization_info->marginalize();
            // ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            spdlog::debug("end marginalization, {} ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it !=  image_frame_window_.all_image_frame_ptr_.end(); frame_it++)
            {
                if (frame_it->second == frame_to_margin_){
                    continue;
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(frame_it->second->para_Pose_)] = frame_it->second->para_Pose_;
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(frame_it->second->para_SpeedBias_)] = frame_it->second->para_SpeedBias_;
                }
            }

            for (unsigned int i = 0; i < img_trackers_.size(); i++){
                addr_shift[reinterpret_cast<long>(img_trackers_[i]->cam_info_.para_Ex_Pose_[0])] = img_trackers_[i]->cam_info_.para_Ex_Pose_[0];
                if(img_trackers_[i]->cam_info_.stereo_){
                    addr_shift[reinterpret_cast<long>(img_trackers_[i]->cam_info_.para_Ex_Pose_[1])] = img_trackers_[i]->cam_info_.para_Ex_Pose_[1];
                }
            }

            for (unsigned int i = 0; i < img_trackers_.size(); i++)
                addr_shift[reinterpret_cast<long>(&img_trackers_[i]->cam_info_.td_)] = &img_trackers_[i]->cam_info_.td_;

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info_)
                delete last_marginalization_info_;
            last_marginalization_info_ = marginalization_info;
            last_marginalization_parameter_blocks_ = parameter_blocks;
        }
    }
    // printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    return;
}

void Estimator::slideWindow(const int img_cam_unique_id){

    if(marginalization_flag_ == MARGIN_OLD){
        img_trackers_[img_cam_unique_id]->f_manager_.removeFront();
        unsigned int remove_state_idx = image_frame_window_.pop_front(img_cam_unique_id);
        state_hist_.erase(state_hist_.begin() + remove_state_idx);

    }
    else if(marginalization_flag_ == MARGIN_SECOND_NEW){
        img_trackers_[img_cam_unique_id]->f_manager_.removeSecondBack();
        unsigned int remove_state_idx = image_frame_window_.erase_second_new(img_cam_unique_id);
        state_hist_.erase(state_hist_.begin() + remove_state_idx);
    }


    double remove_imu_min_t = image_frame_window_.all_image_frame_ptr_.begin()->first - 0.01;

    int remove_imu_cnt = 0;

    int max_remove_num = image_frame_window_.all_image_frame_ptr_.begin()->second->state_idx_ - 3;

    for(; state_hist_.front().t_ < remove_imu_min_t; state_hist_.pop_front(), remove_imu_cnt++){
        if(remove_imu_cnt >= max_remove_num){
            break;
        }
    }

    if(remove_imu_cnt > 0){
        for(auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++){
            frame_it->second->state_idx_ -= remove_imu_cnt;
        }
    }

    return;
}

void Estimator::slideWindow(shared_ptr<ImageFrame>& frame_ptr){

    TicToc tt;

    if(!frame_ptr)
        return;

    int cam_unique_id = frame_ptr->cam_module_unique_id_;
    int cam_wise_idx = 0;
    for(int i = 0; i < image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id].size(); i++){
        if(image_frame_window_.cam_wise_image_frame_ptr_[cam_unique_id][i] == frame_ptr){
            cam_wise_idx = i;
            break;
        }
    }

    img_trackers_[cam_unique_id]->f_manager_.remove(cam_wise_idx);
    unsigned int remove_state_idx = image_frame_window_.erase(cam_unique_id, cam_wise_idx);

    double remove_time = state_hist_[remove_state_idx].t_;

    if(image_frame_window_.all_image_frame_ptr_.rbegin()->first > remove_time && image_frame_window_.all_image_frame_ptr_.begin()->first < remove_time){
        tt.tic();
        addPreintegrationToNextFrame(remove_state_idx);
        // ROS_INFO("add pre integration to next frame time: %lf ms", tt.toc());
    }

    state_hist_.erase(state_hist_.begin() + remove_state_idx);


    double remove_imu_min_t = image_frame_window_.all_image_frame_ptr_.begin()->first - 0.01;

    int remove_imu_cnt = 0;

    int max_remove_num = image_frame_window_.all_image_frame_ptr_.begin()->second->state_idx_ - 3;

    for(; state_hist_.front().t_ < remove_imu_min_t; state_hist_.pop_front(), remove_imu_cnt++){
        if(remove_imu_cnt >= max_remove_num){
            break;
        }
    }

    if(remove_imu_cnt > 0){
        for(auto frame_it = image_frame_window_.all_image_frame_ptr_.begin(); frame_it != image_frame_window_.all_image_frame_ptr_.end(); frame_it++){
            frame_it->second->state_idx_ -= remove_imu_cnt;
        }
    }
    return;
}

void Estimator::reorderWindow(){

    image_frame_window_.reorder();
    std::sort(state_hist_.begin(), state_hist_.end(), State::compare);

    unsigned int state_idx = 0;
    Vector6d& prev_imu_data = state_hist_.begin()->imu_data_;
    auto img_state_it = state_hist_.begin();
    bool img_flag = false;

    for(auto state_it = state_hist_.begin(); state_it != state_hist_.end(); state_it++, state_idx++){

        if(state_it->type_ == State::IMAGE){
            state_it->image_frame_ptr_->state_idx_ = state_idx;

            state_it->image_frame_ptr_->prev_acc_ = prev_imu_data.topRows(3);
            state_it->image_frame_ptr_->prev_gyr_ = prev_imu_data.bottomRows(3);

            img_state_it = state_it;
            img_flag = true;

        }

        if(state_it->type_ == State::IMU){
            prev_imu_data = state_it->imu_data_;
            if(img_flag){
                for(auto set_state_it = img_state_it; set_state_it != state_hist_.begin() && set_state_it->type_ == State::IMAGE; set_state_it--){
                    set_state_it->imu_data_ = state_it->imu_data_;
                }
            }
            img_flag = false;
        }
    }
    return;
}

void Estimator::getPoseInWorldFrame(const int unique_id, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    // T.block<3, 3>(0, 0) = Rs_[frame_count_];
    // T.block<3, 1>(0, 3) = Ps_[frame_count_];
    T.block<3, 3>(0, 0) = image_frame_window_.cam_wise_image_frame_ptr_[unique_id].back()->R_.toRotationMatrix();
    T.block<3, 1>(0, 3) = image_frame_window_.cam_wise_image_frame_ptr_[unique_id].back()->T_;
    return;
}

void Estimator::getPoseInWorldFrame(const int unique_id, const int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = image_frame_window_.cam_wise_image_frame_ptr_[unique_id][index]->R_.toRotationMatrix();
    T.block<3, 1>(0, 3) = image_frame_window_.cam_wise_image_frame_ptr_[unique_id][index]->T_;
    return;
}

void Estimator::predictPtsInNextFrame(const int unique_id)
{
    //printf("predict pts in next frame\n");
    if(image_frame_window_.cam_wise_image_frame_ptr_[unique_id].size() < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(unique_id, curT);

    //get last frame in cam[unique_id]


    getPoseInWorldFrame(unique_id, image_frame_window_.cam_wise_image_frame_ptr_[unique_id].size() - 2, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : img_trackers_[unique_id]->f_manager_.feature_)
    {
        if(it_per_id.second.estimated_depth > 0)
        {
            int firstIndex = it_per_id.second.start_frame;
            int lastIndex = it_per_id.second.start_frame + it_per_id.second.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count_, lastIndex);
            if((int)it_per_id.second.feature_per_frame.size() >= 2 && lastIndex == image_frame_window_.cam_wise_image_frame_ptr_[unique_id].size())
            {
                double depth = it_per_id.second.estimated_depth;
                Vector3d pts_j = img_trackers_[unique_id]->cam_info_.ric_[0] * (depth * it_per_id.second.feature_per_frame.front().point) + img_trackers_[unique_id]->cam_info_.tic_[0];
                // Vector3d pts_w = Rs_[firstIndex] * pts_j + Ps_[firstIndex];
                Vector3d pts_w = image_frame_window_.cam_wise_image_frame_ptr_[unique_id][firstIndex]->R_ * pts_j + image_frame_window_.cam_wise_image_frame_ptr_[unique_id][firstIndex]->T_;
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = img_trackers_[unique_id]->cam_info_.ric_[0].toRotationMatrix().transpose() * (pts_local - img_trackers_[unique_id]->cam_info_.tic_[0]);
                int ptsIndex = it_per_id.second.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    img_trackers_[unique_id]->featureTracker_.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
    return;
}

void Estimator::propagateIMU(const State& x, State& x_next)
{
    double dt = x_next.t_ - x.t_;
    Eigen::Vector3d un_acc_0 = x.Q_ * (x.imu_data_.topRows(3) - x.Ba_) - g_;
    x_next.un_gyr_ = 0.5 * (x.imu_data_.bottomRows(3) + x_next.imu_data_.bottomRows(3)) - x.Bg_;

    Eigen::Quaterniond dQ = Utility::deltaQ(x_next.un_gyr_ * dt);

    x_next.Q_ = x.Q_ * dQ;

    Eigen::Vector3d un_acc_1 = x_next.Q_ * (x_next.imu_data_.topRows(3) - x.Ba_) - g_;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    Eigen::Vector3d acc_dt = dt * un_acc;
    Eigen::Vector3d half_acc_dt2 = 0.5 * dt * acc_dt;

    x_next.P_ = x.P_ + dt * x.V_ + half_acc_dt2;
    x_next.V_ = x.V_ + acc_dt;

    // x_next.P_lpf_ = x_next.P_;
    // x_next.V_lpf_ = x_next.V_;
    // x_next.Q_lpf_ = x_next.Q_;
    // latest_acc_0_ = linear_acceleration;
    // latest_gyr_0_ = angular_velocity;
    if(x_next.type_ == State::IMU){
        x_next.Ba_ = x.Ba_;
        x_next.Bg_ = x.Bg_;
    }
    return;
}

void Estimator::propagateIMULowpass(const State& x, State& x_next, const double& alpha)
{
    double dt = x_next.t_ - x.t_;

    Eigen::Vector3d un_acc_0 = x.Q_ * (x.imu_data_.topRows(3) - x.Ba_) - g_;
    x_next.un_gyr_ = 0.5 * (x.imu_data_.bottomRows(3) + x_next.imu_data_.bottomRows(3)) - x.Bg_;

    Eigen::Quaterniond dQ = Utility::deltaQ(x_next.un_gyr_ * dt);

    x_next.Q_ = x.Q_ * dQ;

    Eigen::Vector3d un_acc_1 = x_next.Q_ * (x_next.imu_data_.topRows(3) - x.Ba_) - g_;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    Eigen::Vector3d acc_dt = dt * un_acc;
    Eigen::Vector3d half_acc_dt2 = 0.5 * dt * acc_dt;

    x_next.P_ = x.P_ + dt * x.V_ + half_acc_dt2;
    x_next.V_ = x.V_ + acc_dt;

    x_next.P_lpf_ = x.P_lpf_ + dt * x.V_lpf_ + half_acc_dt2;
    x_next.V_lpf_ = x.V_lpf_ + acc_dt;
    x_next.Q_lpf_ = x.Q_lpf_ * dQ;

    x_next.P_lpf_ = Utility::lerp(x_next.P_lpf_, x_next.P_, alpha);
    x_next.V_lpf_ = Utility::lerp(x_next.V_lpf_, x_next.V_, alpha);
    x_next.Q_lpf_ = x_next.Q_lpf_.slerp(alpha, x_next.Q_);
    // latest_acc_0_ = linear_acceleration;
    // latest_gyr_0_ = angular_velocity;
    if(x_next.type_ == State::IMU){
        x_next.Ba_ = x.Ba_;
        x_next.Bg_ = x.Bg_;
    }
    return;
}

void Estimator::updateLatestStates(const int unique_id)
{
    // mBuf_.lock();
    auto& image_frame =image_frame_window_.all_image_frame_ptr_.rbegin()->second;
    latest_time_ = image_frame->t_ + image_frame->td_;
    latest_P_ = image_frame->T_;
    latest_Q_ = image_frame->R_;
    latest_V_ = image_frame->V_;
    latest_Ba_ = image_frame->Ba_;
    latest_Bg_ = image_frame->Bg_;

    last_R_ = image_frame_window_.all_image_frame_ptr_.rbegin()->second->R_;
    last_P_ = image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_;
    last_R0_ = image_frame_window_.all_image_frame_ptr_.begin()->second->R_;
    last_P0_ = image_frame_window_.all_image_frame_ptr_.begin()->second->T_;

    unsigned int last_image_state_idx = image_frame->state_idx_;
    repropagateIMU(state_hist_.begin()+last_image_state_idx+1, false);
    lpf_idx_ = 0;
    // printf("propagate imu time: %lf s\n", state_hist_.back().t_ -  state_hist_[last_image_state_idx].t_);

    // for(unsigned int i = 0; i< img_trackers_.size(); i++){
    //     ROS_DEBUG("td %d: %lf", i, img_trackers_[i]->cam_info_.td_);
    //     printf("td %d: %lf\n", i, img_trackers_[i]->cam_info_.td_);
    //     printf("cam %d frame cnt: %d\n", i, image_frame_window_.cam_wise_image_frame_ptr_[i].size());
    // }
    bool update_latest = false;
    if(image_frame->t_ > latest_image_time_){
        latest_image_time_ = image_frame->t_;
        update_latest = true;
    }
    Publisher::FullReportData full_report_data;
    collectOptimizedData(full_report_data, unique_id, update_latest);
    publisher_callbacks_.on_publish_full_report_cb(full_report_data);

    #if 0
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_);

    pubCameraPose(*this, unique_id);
    pubPointCloud(*this, unique_id);

    if(image_frame->t_ > latest_image_time_){
        pubTF(*this);
        pubOdometry(*this);
        pubKeyPoses(*this);
        pubKeyframe(*this);

        latest_image_time_ = image_frame->t_;

    }
    #endif
    // printStatistics(*this, 0);
    return;
}

void Estimator::collectPropogationData(Publisher::PropagateData& data){
    static const double interpolation_alpha = 0.5;
    const double t = this->state_hist_.back().t_;
    const Eigen::Vector3d& P = this->state_hist_.back().P_lpf_;
    const Eigen::Quaterniond &R= this->state_hist_.back().Q_lpf_;
    const Eigen::Vector3d &V = this->state_hist_.back().V_lpf_;
    const Eigen::Vector3d &omega =  this->state_hist_.back().un_gyr_;
    const Eigen::Matrix3d &center_R_imu = this->imu_module_.rcenterimu_;
    const Eigen::Vector3d &center_T_imu = this->imu_module_.tcenterimu_;
    Eigen::Vector3d w_T_center, v_center, a_center, omega_center;
    Eigen::Matrix3d w_R_center;
    w_R_center = R * center_R_imu;
    w_T_center = P - w_R_center * center_T_imu;
    w_T_center = Utility::lerp(last_pos_, w_T_center, interpolation_alpha);
    Eigen::Quaterniond q_center(w_R_center);
    q_center = last_q_.slerp(interpolation_alpha, q_center);
    omega_center = center_R_imu * omega;
    v_center = center_R_imu * V - omega_center.cross(center_T_imu);
    v_center = Utility::lerp(last_vel_, v_center, interpolation_alpha);
    omega_center = Utility::lerp(last_omega_, omega_center, interpolation_alpha);
    last_pos_ = w_T_center;
    last_q_ = q_center;
    last_vel_ = v_center;
    last_omega_ = omega_center;
    data.timestamp = t;
    data.position = w_T_center;
    data.orientation = q_center;
    data.velocity = v_center;
    data.angular_velocity = omega_center;
    for(unsigned int i = 0; i < img_trackers_.size(); i++){
        Vector3d P_l_cam = P + R * img_trackers_[i]->cam_info_.tic_[0];
        Quaterniond R_l_cam = Quaterniond(R * img_trackers_[i]->cam_info_.ric_[0]);
        std::vector<Vector3d> camera_pose;
        std::vector<Quaterniond> camera_ori;
        camera_pose.push_back(P_l_cam);
        camera_ori.push_back(R_l_cam);
        if(img_trackers_[i]->cam_info_.stereo_){
            Vector3d P_r_cam = P + R * img_trackers_[i]->cam_info_.tic_[1];
            Quaterniond R_r_cam = Quaterniond(R * img_trackers_[i]->cam_info_.ric_[1]);
            camera_pose.push_back(P_r_cam);
            camera_ori.push_back(R_r_cam);  
        }
        data.camera_poses.push_back(Publisher::CamerasPose{camera_pose, camera_ori});
    }
    return;
}

void Estimator::collectOptimizedData(Publisher::FullReportData& data, const int unique_id, const bool update_latest){
//camera pose
    auto& frame_ptr = this->image_frame_window_.cam_wise_image_frame_ptr_[unique_id].back();
    double camera_time = frame_ptr->t_;
    Vector3d P = frame_ptr->T_ + frame_ptr->R_ * this->img_trackers_[unique_id]->cam_info_.tic_[0];
    Quaterniond R = frame_ptr->R_ * this->img_trackers_[unique_id]->cam_info_.ric_[0];
    data.camera_time = camera_time;
    data.camera_id = unique_id;
    data.camera_pose = Publisher::Pose{P, R};


//collect pointcloud
    double report_time = this->image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_;
    data.timestamp = report_time;
    for (auto &it_per_id : img_trackers_[unique_id]->f_manager_.feature_)
    {
        int used_num =0;
        used_num = it_per_id.second.feature_per_frame.size();
        if (used_num < 2 || it_per_id.second.solve_flag == FeaturePerId::UNINITIALIZED || 
            it_per_id.second.solve_flag== FeaturePerId::OUTLIER){
            continue;
        }
        int imu_i = it_per_id.second.start_frame;
        Vector3d pts_i = it_per_id.second.feature_per_frame.front().point * it_per_id.second.estimated_depth;
        Vector3d w_pts_i = this->image_frame_window_.cam_wise_image_frame_ptr_[unique_id][imu_i]->R_ * (this->img_trackers_[unique_id]->cam_info_.ric_[0] * pts_i
             + this->img_trackers_[unique_id]->cam_info_.tic_[0]) + this->image_frame_window_.cam_wise_image_frame_ptr_[unique_id][imu_i]->T_;
        data.in_window_pointcloud.push_back(w_pts_i);
    }
//collect margin pointcloud
    auto& margin_frame_ptr = this->image_frame_window_.all_image_frame_ptr_.begin()->second;
    int margin_cam_unique_id = margin_frame_ptr->cam_module_unique_id_;

    for (auto &it_per_id : img_trackers_[margin_cam_unique_id]->f_manager_.feature_)
    {
        int used_num =0;
        used_num = it_per_id.second.feature_per_frame.size();
        if (used_num < 2 ){
            continue;
        }
        if (it_per_id.second.start_frame == 0 && it_per_id.second.feature_per_frame.size() <= 2 
            && it_per_id.second.solve_flag == FeaturePerId::ESTIMATED){
            int imu_i = it_per_id.second.start_frame;
            Vector3d pts_i = it_per_id.second.feature_per_frame.front().point * it_per_id.second.estimated_depth;
            Vector3d w_pts_i = this->image_frame_window_.cam_wise_image_frame_ptr_[margin_cam_unique_id][imu_i]->R_ * (this->img_trackers_[margin_cam_unique_id]->cam_info_.ric_[0] * pts_i 
                + this->img_trackers_[margin_cam_unique_id]->cam_info_.tic_[0]) + this->image_frame_window_.cam_wise_image_frame_ptr_[margin_cam_unique_id][imu_i]->T_;
            data.marginized_pointcloud.push_back(w_pts_i);
        }
    }
    
    data.update_latest = update_latest;

    if (update_latest){
        if (solver_flag_ != Estimator::SolverFlag::NON_LINEAR ){
            data.optimized_valid = false;
        } else {
            double time_stamp = image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_;
            data.optimized_valid = true;
            Vector3d tf_correct_t;
            Quaterniond tf_correct_q;
            //collect TF
            tf_correct_t = image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_;
            tf_correct_q = image_frame_window_.all_image_frame_ptr_.rbegin()->second->R_;
            data.TF_correct_pose = Publisher::Pose{tf_correct_t, tf_correct_q};
        }
        // Collect Odometry
        Quaterniond tmp_Q (this->image_frame_window_.all_image_frame_ptr_.rbegin()->second->R_);
        Vector3d tmp_P = this->image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_;
        Vector3d tmp_V = this->image_frame_window_.all_image_frame_ptr_.rbegin()->second->V_;
        data.latest_frame_pose.orientation = tmp_Q;
        data.latest_frame_pose.position = tmp_P;
        data.latest_frame_pose.velocity = tmp_V;
        // collect keypose
        for (int i =0 ; i < this->key_poses_.size(); i++){
            Vector3d key_p = this->key_poses_[i];
            data.key_poses.push_back(key_p);
        }
        // collect keyframe
        if (this->marginalization_flag_ == MARGIN_OLD) {
            data.have_keyframe = true;
            auto p = image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_;
            auto q = image_frame_window_.all_image_frame_ptr_.rbegin()->second->R_;
            data.keyframe_pose = Publisher::Pose{p, q};
        }
        //collect camera extrinsics
        for (unsigned int i = 0; i < img_trackers_.size(); i++){
            Vector3d ex_t = img_trackers_[i]->cam_info_.tic_[0];
            Quaterniond ex_q (img_trackers_[i]->cam_info_.ric_[0]);
            data.pub_camera_extrinsic_params.push_back(Publisher::Pose{ex_t, ex_q});
        }
    }
    return; 
}
}