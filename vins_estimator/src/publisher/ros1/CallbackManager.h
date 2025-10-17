#include "publisher/PublisherCallbacks.h"
#include "estimator/parameters.h"
#include "publisher/ros1/CameraPoseVisualization.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace Publisher{
    const double k_interpolation_alpha = 0.5;
    class ROS1CallbackManager{
    public:
        //construct and add publisher and publishers to nodehandle
        ROS1CallbackManager() = delete;
        ROS1CallbackManager(ros::NodeHandle& nh):nh_(nh){
            pub_latest_odometry_ = nh_.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
            pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1000);
            pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1000);
            pub_key_poses_ = nh_.advertise<visualization_msgs::Marker>("key_poses", 1000);
            pub_keyframe_pose_ = nh_.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
            // pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
            pub_margin_cloud_ = nh_.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
            for(unsigned int i = 0; i < vins_multi::CAM_MODULES.size(); i++){
                pub_camera_pose_.emplace_back(nh_.advertise<geometry_msgs::PoseStamped>(std::string("camera_pose_")+std::to_string(i), 1000));
                pub_latest_camera_pose_.emplace_back(nh_.advertise<geometry_msgs::PoseStamped>(std::string("imu_propagate_camera_pose_")+std::to_string(i), 1000));
                pub_camera_pose_visual_.emplace_back(nh_.advertise<visualization_msgs::MarkerArray>(std::string("camera_pose_visual_")+std::to_string(i), 1000));
                pub_extrinsic_.emplace_back(nh_.advertise<nav_msgs::Odometry>(std::string("extrinsic_")+std::to_string(i), 1000));
                pub_image_track_.emplace_back(nh_.advertise<sensor_msgs::Image>(std::string("image_track_")+std::to_string(i), 1000));
                pub_point_cloud_.emplace_back(nh_.advertise<sensor_msgs::PointCloud>(std::string("point_cloud_")+std::to_string(i), 1000));
            }
            cameraposevisual_.setScale(0.1);
            cameraposevisual_.setLineWidth(0.01);
        };
        
        ~ROS1CallbackManager(){};
        void publish_propagate_cb(const Publisher::PropagateData& data);
        void publish_track_image_cb(const Publisher::TrackImageData &data);
        void publish_full_report_cb(const FullReportData &data);
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_odometry_, pub_latest_odometry_;
        ros::Publisher pub_path_;
        std::vector<ros::Publisher> pub_point_cloud_;
        ros::Publisher pub_margin_cloud_;
        ros::Publisher pub_key_poses_;
        ros::Publisher pub_keyframe_pose_;
        std::vector<ros::Publisher> pub_camera_pose_, pub_latest_camera_pose_;
        std::vector<ros::Publisher> pub_camera_pose_visual_;
        std::vector<ros::Publisher> pub_extrinsic_;
        std::vector<ros::Publisher> pub_image_track_;
        nav_msgs::Path path_;
        vins_multi::CameraPoseVisualization cameraposevisual_= vins_multi::CameraPoseVisualization(1.0f, 0.0f, 0.0f, 1.0f);
        
        static double sum_of_path_;
        static Vector3d last_path_;
        size_t pub_counter_ = 0;
        Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_vel_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_omega_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_q_ = Eigen::Quaterniond::Identity();

        const double interpolation_alpha_ = 0.5;
    };
    // ROS1CallbackManager::sum_of_path_ = 0.0;
    // ROS1CallbackManager::last_path_ = Eigen::Vector3d::Zero();
}