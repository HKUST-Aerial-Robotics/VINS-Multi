#pragma once

#include <stdio.h>
#include <queue>
#include <map>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <boost/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CompressedImage.h>
#include <vector>

#include "estimator/parameters.h"
#include "utility/tic_toc.h"
#include "vins_estimator_ros1/CallbackManager.h"
#include "vins_estimator_ros1/visualization.h"

using namespace std;

namespace vins_multi{

class VinsNodeBaseClass {
    public:

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyImageExact;
        typedef unique_ptr<message_filters::Synchronizer<SyncPolicyImageExact>> SynchronizerImageExact;

        struct uni_image_sub_ptr{
            message_filters::Subscriber<sensor_msgs::Image>* img0_sync_sub_ptr_;
            message_filters::Subscriber<sensor_msgs::Image>* img1_sync_sub_ptr_;

            SynchronizerImageExact sync_image_exact_;

            ros::Subscriber* img0_sub_;
        };

        class camera_module_info_with_sub{
            public:

                camera_module_info_with_sub(camera_module_info cam_module, Estimator* est_ptr): module_info_(cam_module), estimator_ptr_(est_ptr){}

                uni_image_sub_ptr img_sub_;

                camera_module_info module_info_;

                unsigned int unique_id_;

                Estimator* estimator_ptr_;

                void imgs_callback(const sensor_msgs::ImageConstPtr &img0_msg, const sensor_msgs::ImageConstPtr &img1_msg);

                void img_callback(const sensor_msgs::ImageConstPtr &img0_msg);

                void comp_imgs_callback(const sensor_msgs::CompressedImageConstPtr &img1_msg, const sensor_msgs::CompressedImageConstPtr &img2_msg);
        };

        class imu_info_with_sub{
            public:

                imu_info_with_sub(imu_info imu_module, Estimator* est_ptr): module_info_(imu_module), estimator_ptr_(est_ptr){}

                ros::Subscriber imu_sub_;
                imu_info module_info_;
                Estimator* estimator_ptr_;

                void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
        };

        void init_node(ros::NodeHandle & n);


    private:

        vector<camera_module_info_with_sub> camera_modules_;
        vector<imu_info_with_sub> imu_modules_;

        Estimator estimator_;
        ros::Subscriber sub_restart_;
        std::unique_ptr<Publisher::ROS1CallbackManager> ros1_callback_manager_;

    protected:

        void registerSub(ros::NodeHandle & n);


        void set_modules();

        void restart_callback(const std_msgs::BoolConstPtr &restart_msg);

        virtual void Init(ros::NodeHandle & n);
};

}
