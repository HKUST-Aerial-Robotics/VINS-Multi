/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "rosNode.hpp"
#include "ros/subscriber.h"

namespace vins_multi{

cv_bridge::CvImagePtr getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

        // ptr = cv_bridge::toCvShare(img_msg, "mono8");
    }
    else{
        ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
        // ptr = cv_bridge::toCvShare(img_msg, "mono8");

        // ROS_WARN("convert rgb");
    }

    return ptr;
}

cv_bridge::CvImagePtr getDepthFromMsg(const sensor_msgs::ImageConstPtr &depth_msg)
{
    cv_bridge::CvImagePtr cv_ptr_depth;

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        sensor_msgs::Image* img_ptr = new sensor_msgs::Image;
        img_ptr->header = depth_msg->header;
        img_ptr->height = depth_msg->height;
        img_ptr->width = depth_msg->width;
        img_ptr->is_bigendian = depth_msg->is_bigendian;
        img_ptr->step = depth_msg->step;
        img_ptr->data = depth_msg->data;
        img_ptr->encoding = sensor_msgs::image_encodings::MONO16;
        const sensor_msgs::ImagePtr c_img_ptr(img_ptr);
        cv_ptr_depth = cv_bridge::toCvCopy(*img_ptr, sensor_msgs::image_encodings::MONO16);

    }
    else{
        cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);

    }

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        (cv_ptr_depth->image).convertTo(cv_ptr_depth->image, CV_16UC1, 0.001);
    }
    return cv_ptr_depth;
}

void VinsNodeBaseClass::set_modules(){
    for(auto cam_module : CAM_MODULES){
        camera_modules_.emplace_back(camera_module_info_with_sub(cam_module, &estimator_));
    }

    for(unsigned int i = 0; i < camera_modules_.size(); i++){
        camera_modules_[i].unique_id_ = i;
    }

    imu_modules_.emplace_back(imu_info_with_sub(IMU_MODULE, &estimator_));
}

void VinsNodeBaseClass::registerSub(ros::NodeHandle & n){

    if(USE_IMU)
    {
        imu_modules_[0].imu_sub_ = n.subscribe(imu_modules_[0].module_info_.imu_topic_, 2000, &VinsNodeBaseClass::imu_info_with_sub::imu_callback, &(this->imu_modules_[0]), ros::TransportHints().tcpNoDelay(true));
    }

    for (unsigned int i = 0; i < camera_modules_.size(); i++){
        if(camera_modules_[i].module_info_.depth_ || camera_modules_[i].module_info_.stereo_){
            camera_modules_[i].img_sub_.img0_sync_sub_ptr_ = new message_filters::Subscriber<sensor_msgs::Image> (n, camera_modules_[i].module_info_.img_topic_[0], 10, ros::TransportHints().tcpNoDelay(true));
            camera_modules_[i].img_sub_.img1_sync_sub_ptr_ = new message_filters::Subscriber<sensor_msgs::Image> (n, camera_modules_[i].module_info_.img_topic_[1], 10, ros::TransportHints().tcpNoDelay(true));
            
            camera_modules_[i].img_sub_.sync_image_exact_.reset(new message_filters::Synchronizer<SyncPolicyImageExact>(SyncPolicyImageExact(10), *camera_modules_[i].img_sub_.img0_sync_sub_ptr_, *camera_modules_[i].img_sub_.img1_sync_sub_ptr_));
            camera_modules_[i].img_sub_.sync_image_exact_->registerCallback(boost::bind(&VinsNodeBaseClass::camera_module_info_with_sub::imgs_callback, &(this->camera_modules_[i]), _1, _2));

        }
        else{
            *camera_modules_[i].img_sub_.img0_sub_ = n.subscribe(CAM_MODULES[i].img_topic_[0], 1000, &VinsNodeBaseClass::camera_module_info_with_sub::img_callback, &(this->camera_modules_[i]), ros::TransportHints().tcpNoDelay(true));
        }
    }

    sub_restart_ = n.subscribe("/vins_restart", 100, &VinsNodeBaseClass::restart_callback, (VinsNodeBaseClass*)this, ros::TransportHints().tcpNoDelay(true));

}

void VinsNodeBaseClass::restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");

    }
    return;
}


void VinsNodeBaseClass::camera_module_info_with_sub::imgs_callback(const sensor_msgs::ImageConstPtr &img0_msg, const sensor_msgs::ImageConstPtr &img1_msg){

    cv_bridge::CvImagePtr img_0;
    cv_bridge::CvImagePtr img_1;

    img_0 = getImageFromMsg(img0_msg);
    if(module_info_.depth_){
        img_1 = getDepthFromMsg(img1_msg);
    } else if(module_info_.stereo_){
        img_1 = getImageFromMsg(img1_msg);
    }

    if(img0_msg->encoding == sensor_msgs::image_encodings::RGB8){

        if(USE_GPU){

#ifdef WITH_CUDA
            cv::cuda::GpuMat img0_gpu;
            cv::cuda::GpuMat img0_gray_gpu;
            img0_gpu.upload(img_0->image);
            cv::cuda::cvtColor(img0_gpu, img0_gray_gpu, cv::COLOR_RGB2GRAY);
            img0_gray_gpu.download(img_0->image);
#endif
        }
        else{
            cv::cvtColor(img_0->image, img_0->image, cv::COLOR_RGB2GRAY);
        }
    }
    else if(img0_msg->encoding == sensor_msgs::image_encodings::BGR8){
        if(USE_GPU){

#ifdef WITH_CUDA
            cv::cuda::GpuMat img0_gpu;
            cv::cuda::GpuMat img0_gray_gpu;
            img0_gpu.upload(img_0->image);
            cv::cuda::cvtColor(img0_gpu, img0_gray_gpu, cv::COLOR_BGR2GRAY);
            img0_gray_gpu.download(img_0->image);
#endif
        }
        else{
            cv::cvtColor(img_0->image, img_0->image, cv::COLOR_BGR2GRAY);
        }
    }


    if(module_info_.depth_)
        estimator_ptr_->inputImageToBuffer(unique_id_, img0_msg->header.stamp.toSec(), img_0->image, img_1->image);

    else if(module_info_.stereo_)
        estimator_ptr_->inputImageToBuffer(unique_id_, img0_msg->header.stamp.toSec(), img_0->image, img_1->image);

}

void VinsNodeBaseClass::camera_module_info_with_sub::img_callback(const sensor_msgs::ImageConstPtr &img0_msg){
    estimator_ptr_->inputImage(unique_id_, img0_msg->header.stamp.toSec(), getImageFromMsg(img0_msg)->image);
}

void VinsNodeBaseClass::camera_module_info_with_sub::comp_imgs_callback(const sensor_msgs::CompressedImageConstPtr &img1_msg, const sensor_msgs::CompressedImageConstPtr &img2_msg){

}

void VinsNodeBaseClass::imu_info_with_sub::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg){

    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;

    Vector6d imu_data;
    imu_data << dx, dy, dz, rx, ry, rz;
    estimator_ptr_->inputIMU(t, imu_data);

}

void VinsNodeBaseClass::Init(ros::NodeHandle & n)
{

    std::string config_file;
    n.getParam("config_file", config_file);
    std::cout << "config file:\n" << config_file << '\n';

    readParameters(config_file);
    set_modules();
    ROS_WARN("set module finish");
    registerPub(n);
    estimator_.setParameter();
    ROS_WARN("set estimator finish");

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerSub(n);

    estimator_.start_process_thread();


}

void VinsNodeBaseClass::init_node(ros::NodeHandle & n){
    this->Init(n);
}

}