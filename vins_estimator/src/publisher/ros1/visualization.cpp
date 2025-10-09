/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"

using namespace ros;
using namespace Eigen;

namespace vins_multi{

ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path;
std::vector<ros::Publisher> pub_point_cloud;
ros::Publisher pub_margin_cloud;
ros::Publisher pub_key_poses;
std::vector<ros::Publisher> pub_camera_pose, pub_latest_camera_pose;
std::vector<ros::Publisher> pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_keyframe_pose;
// ros::Publisher pub_keyframe_point;
std::vector<ros::Publisher> pub_extrinsic;

std::vector<ros::Publisher> pub_image_track;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

Eigen::Vector3d last_pos = Eigen::Vector3d::Zero();
Eigen::Vector3d last_vel = Eigen::Vector3d::Zero();
Eigen::Vector3d last_omega = Eigen::Vector3d::Zero();
Eigen::Quaterniond last_q = Eigen::Quaterniond::Identity();

const double interpolation_alpha = 0.5;

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    // pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
    for(unsigned int i = 1; i <= CAM_MODULES.size(); i++){

        pub_camera_pose.emplace_back(n.advertise<geometry_msgs::PoseStamped>(std::string("camera_pose_")+std::to_string(i), 1000));
        pub_latest_camera_pose.emplace_back(n.advertise<geometry_msgs::PoseStamped>(std::string("imu_propagate_camera_pose_")+std::to_string(i), 1000));
        pub_camera_pose_visual.emplace_back(n.advertise<visualization_msgs::MarkerArray>(std::string("camera_pose_visual_")+std::to_string(i), 1000));
        pub_extrinsic.emplace_back(n.advertise<nav_msgs::Odometry>(std::string("extrinsic_")+std::to_string(i), 1000));
        pub_image_track.emplace_back(n.advertise<sensor_msgs::Image>(std::string("image_track_")+std::to_string(i), 1000));
        pub_point_cloud.emplace_back(n.advertise<sensor_msgs::PointCloud>(std::string("point_cloud_")+std::to_string(i), 1000));
    }
    

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Estimator &estimator)
{

    const double t = estimator.state_hist_.back().t_;

    const Eigen::Vector3d& P = estimator.state_hist_.back().P_lpf_;    
    const Eigen::Quaterniond &R= estimator.state_hist_.back().Q_lpf_;
    const Eigen::Vector3d &V = estimator.state_hist_.back().V_lpf_;


    const Eigen::Vector3d &omega = estimator.state_hist_.back().un_gyr_;
    
    const Eigen::Matrix3d &center_R_imu = estimator.imu_module_.rcenterimu_;
    const Eigen::Vector3d &center_T_imu = estimator.imu_module_.tcenterimu_;

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";

    Eigen::Vector3d w_T_center, v_center, a_center, omega_center;
    Eigen::Matrix3d w_R_center;

    w_R_center = R * center_R_imu;
    w_T_center = P - w_R_center * center_T_imu;

    w_T_center = Utility::lerp(last_pos, w_T_center, interpolation_alpha);

    odometry.pose.pose.position.x = w_T_center.x();
    odometry.pose.pose.position.y = w_T_center.y();
    odometry.pose.pose.position.z = w_T_center.z();

    Eigen::Quaterniond q_center(w_R_center);

    q_center = last_q.slerp(interpolation_alpha, q_center);

    odometry.pose.pose.orientation.x = q_center.x();
    odometry.pose.pose.orientation.y = q_center.y();
    odometry.pose.pose.orientation.z = q_center.z();
    odometry.pose.pose.orientation.w = q_center.w();

    omega_center = center_R_imu * omega;

    v_center = center_R_imu * V - omega_center.cross(center_T_imu);


    v_center = Utility::lerp(last_vel, v_center, interpolation_alpha);

    odometry.twist.twist.linear.x = v_center.x();
    odometry.twist.twist.linear.y = v_center.y();
    odometry.twist.twist.linear.z = v_center.z();


    omega_center = Utility::lerp(last_omega, omega_center, interpolation_alpha);

    odometry.twist.twist.angular.x = omega_center.x();
    odometry.twist.twist.angular.y = omega_center.y();
    odometry.twist.twist.angular.z = omega_center.z();
    
    pub_latest_odometry.publish(odometry);

    last_pos = w_T_center;
    last_q = q_center;
    last_vel = v_center;
    last_omega = omega_center;

    for(unsigned int i = 0; i < estimator.img_trackers_.size(); i++){

        Vector3d P_cam = P + R * estimator.img_trackers_[i]->cam_info_.tic_[0];
        Quaterniond R_cam = Quaterniond(R * estimator.img_trackers_[i]->cam_info_.ric_[0]);

        geometry_msgs::PoseStamped pose_cam;
        pose_cam.header = odometry.header;
        pose_cam.pose.position.x = P_cam.x();
        pose_cam.pose.position.y = P_cam.y();
        pose_cam.pose.position.z = P_cam.z();
        pose_cam.pose.orientation.x = R_cam.x();
        pose_cam.pose.orientation.y = R_cam.y();
        pose_cam.pose.orientation.z = R_cam.z();
        pose_cam.pose.orientation.w = R_cam.w();

        pub_latest_camera_pose[i].publish(pose_cam);

        cameraposevisual.reset();
        // if(estimator.image_frame_window_.cam_wise_image_frame_ptr_[i].empty()){
        if(false){
            cameraposevisual.publish_clear(pub_camera_pose_visual[i], odometry.header);
        }
        else{
            cameraposevisual.add_pose(P_cam, R_cam);
            if(estimator.img_trackers_[i]->cam_info_.stereo_)
            {
                Vector3d P1 = P + R * estimator.img_trackers_[i]->cam_info_.tic_[1];
                Quaterniond R1 = R * estimator.img_trackers_[i]->cam_info_.ric_[1];
                cameraposevisual.add_pose(P1, R1);
            }
            cameraposevisual.publish_by(pub_camera_pose_visual[i], odometry.header);
        }
    }

}

void pubTrackImage(const cv::Mat &imgTrack, const double t, const unsigned int cam_unique_id)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track[cam_unique_id].publish(imgTrackMsg);
}


void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag_ != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps_[WINDOW_SIZE].x(), estimator.Ps_[WINDOW_SIZE].y(), estimator.Ps_[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " <<estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_.transpose());
    // ROS_DEBUG_STREAM("orientation: " << estimator.Vs_[WINDOW_SIZE].transpose());
    if (ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < estimator.img_trackers_.size(); i++)
        {
            //ROS_DEBUG("calibration result for camera %d", i);
            
            ROS_DEBUG_STREAM("extirnsic tic: " << estimator.img_trackers_[i]->cam_info_.tic_[0].transpose());
            ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.img_trackers_[i]->cam_info_.ric_[0].toRotationMatrix()).transpose());

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.img_trackers_[i]->cam_info_.ric_[0].toRotationMatrix();
            eigen_T.block<3, 1>(0, 3) = estimator.img_trackers_[i]->cam_info_.tic_[0];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            fs << std::string("body_T_cam0")+std::to_string(estimator.img_trackers_[i]->cam_info_.module_id_) << cv_T ;

            if(estimator.img_trackers_[i]->cam_info_.stereo_){
                Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
                eigen_T.block<3, 3>(0, 0) = estimator.img_trackers_[i]->cam_info_.ric_[1].toRotationMatrix();
                eigen_T.block<3, 1>(0, 3) = estimator.img_trackers_[i]->cam_info_.tic_[1];
                cv::Mat cv_T;
                cv::eigen2cv(eigen_T, cv_T);
                fs << std::string("body_T_cam1")+std::to_string(estimator.img_trackers_[i]->cam_info_.module_id_) << cv_T ;
            }
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);
    
    // if (ESTIMATE_TD){
    //     for (int i = 0; i < estimator.img_trackers_.size(); i++)
    //     {
    //         ROS_INFO("td %d: %f", i, estimator.img_trackers_[i]->cam_info_.td_);
    //     }
    // }
        
}

void pubOdometry(const Estimator &estimator)
{
    if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
    {
        auto time_stamp = ros::Time(estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_);
        nav_msgs::Odometry odometry;
        odometry.header.stamp = time_stamp;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q (estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->R_);
        Vector3d tmp_P = estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_;
        Vector3d tmp_V = estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->V_;
        odometry.pose.pose.position.x = tmp_P.x();
        odometry.pose.pose.position.y = tmp_P.y();
        odometry.pose.pose.position.z = tmp_P.z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = tmp_V.x();
        odometry.twist.twist.linear.y = tmp_V.y();
        odometry.twist.twist.linear.z = tmp_V.z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = time_stamp;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header.stamp = time_stamp;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << time_stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << tmp_P.x() << ","
              << tmp_P.y() << ","
              << tmp_P.z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << tmp_V.x() << ","
              << tmp_V.y() << ","
              << tmp_V.z() << "," << endl;
        foutC.close();
        // Eigen::Vector3d tmp_T = estimator.Ps_[WINDOW_SIZE];
        // printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
        //                                                   tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    }
}

void pubKeyPoses(const Estimator &estimator)
{
    if (estimator.key_poses_.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header.stamp = ros::Time(estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_);
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i < estimator.key_poses_.size(); i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses_[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const unsigned int unique_id)
{
    if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
    {
        auto& frame_ptr = estimator.image_frame_window_.cam_wise_image_frame_ptr_[unique_id].back();
        
        auto stamp = ros::Time{frame_ptr->t_};
        Vector3d P = frame_ptr->T_ + frame_ptr->R_ * estimator.img_trackers_[unique_id]->cam_info_.tic_[0];
        Quaterniond R = frame_ptr->R_ * estimator.img_trackers_[unique_id]->cam_info_.ric_[0];

        geometry_msgs::PoseStamped odometry;
        odometry.header.stamp = stamp;
        odometry.header.frame_id = "world";
        odometry.pose.position.x = P.x();
        odometry.pose.position.y = P.y();
        odometry.pose.position.z = P.z();
        odometry.pose.orientation.x = R.x();
        odometry.pose.orientation.y = R.y();
        odometry.pose.orientation.z = R.z();
        odometry.pose.orientation.w = R.w();


        pub_camera_pose[unique_id].publish(odometry);

    }
}


void pubPointCloud(const Estimator &estimator, const unsigned int unique_id)
{

    auto stamp = ros::Time{estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_};

    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header.stamp = stamp;
    point_cloud.header.frame_id = "world";

    for (auto &it_per_id : estimator.img_trackers_[unique_id]->f_manager_.feature_)
    {
        int used_num;
        used_num = it_per_id.second.feature_per_frame.size();
        if (used_num < 2)
            continue;
        if (it_per_id.second.solve_flag == FeaturePerId::UNINITIALIZED || it_per_id.second.solve_flag ==FeaturePerId::OUTLIER)
            continue;
        int imu_i = it_per_id.second.start_frame;
        Vector3d pts_i = it_per_id.second.feature_per_frame.front().point * it_per_id.second.estimated_depth;
        Vector3d w_pts_i = estimator.image_frame_window_.cam_wise_image_frame_ptr_[unique_id][imu_i]->R_ * (estimator.img_trackers_[unique_id]->cam_info_.ric_[0] * pts_i + estimator.img_trackers_[unique_id]->cam_info_.tic_[0]) + estimator.image_frame_window_.cam_wise_image_frame_ptr_[unique_id][imu_i]->T_;

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud[unique_id].publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header.stamp = stamp;
    margin_cloud.header.frame_id = "world";

    auto& margin_frame_ptr = estimator.image_frame_window_.all_image_frame_ptr_.begin()->second;
    int margin_cam_unique_id = margin_frame_ptr->cam_module_unique_id_;

    for (auto &it_per_id : estimator.img_trackers_[margin_cam_unique_id]->f_manager_.feature_)
    { 
        int used_num;
        used_num = it_per_id.second.feature_per_frame.size();
        if (used_num < 2)
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.second.start_frame == 0 && it_per_id.second.feature_per_frame.size() <= 2 
            && it_per_id.second.solve_flag == FeaturePerId::ESTIMATED)
        {
            int imu_i = it_per_id.second.start_frame;
            Vector3d pts_i = it_per_id.second.feature_per_frame.front().point * it_per_id.second.estimated_depth;
            Vector3d w_pts_i = estimator.image_frame_window_.cam_wise_image_frame_ptr_[margin_cam_unique_id][imu_i]->R_ * (estimator.img_trackers_[margin_cam_unique_id]->cam_info_.ric_[0] * pts_i + estimator.img_trackers_[margin_cam_unique_id]->cam_info_.tic_[0]) + estimator.image_frame_window_.cam_wise_image_frame_ptr_[margin_cam_unique_id][imu_i]->T_;

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}


void pubTF(const Estimator &estimator)
{
    if( estimator.solver_flag_ != Estimator::SolverFlag::NON_LINEAR)
        return;

    auto stamp = ros::Time(estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_;
    correct_q = estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->R_;

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, stamp, "world", "body"));

    // camera frame
    for(unsigned int i = 0; i < estimator.img_trackers_.size(); i++){
        auto& tic = estimator.img_trackers_[i]->cam_info_.tic_[0];
        auto& ric = estimator.img_trackers_[i]->cam_info_.ric_[0];

        transform.setOrigin(tf::Vector3(tic.x(),
                                    tic.y(),
                                    tic.z()));
        q.setW(ric.w());
        q.setX(ric.x());
        q.setY(ric.y());
        q.setZ(ric.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, stamp, "body", std::string{"camera_"}+std::to_string(i+1)));

        nav_msgs::Odometry odometry;
        odometry.header.stamp = stamp;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = tic.x();
        odometry.pose.pose.position.y = tic.y();
        odometry.pose.pose.position.z = tic.z();
        odometry.pose.pose.orientation.x = ric.x();
        odometry.pose.pose.orientation.y = ric.y();
        odometry.pose.pose.orientation.z = ric.z();
        odometry.pose.pose.orientation.w = ric.w();
        pub_extrinsic[i].publish(odometry);
    }

}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag_ == estimator.MARGIN_OLD)
    {
        auto& P = estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->T_;
        auto& R = estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->R_;

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.image_frame_window_.all_image_frame_ptr_.rbegin()->second->t_);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


    }
}

}