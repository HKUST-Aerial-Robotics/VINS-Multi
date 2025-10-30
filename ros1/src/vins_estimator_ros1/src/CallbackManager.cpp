#include "vins_estimator_ros1/CallbackManager.h"
#include "publisher/PublisherCallbacks.h"
#include "utility/utility.h"
#include <ros/ros.h>
namespace Publisher{
    //TODO：we can change to Signal driven for all call back
    using namespace vins_multi;

    void ROS1CallbackManager::publish_track_image_cb(const TrackImageData &data){
        if(data.image.empty()) return;
        std_msgs::Header header;
        header.stamp = ros::Time(data.timestamp);
        header.frame_id = "world";
        sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", data.image).toImageMsg();
        this->pub_image_track_[data.camera_id].publish(imgTrackMsg);
    }

    void ROS1CallbackManager::publish_propagate_cb(const Publisher::PropagateData& data){
        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(data.timestamp);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = data.position.x();
        odometry.pose.pose.position.y = data.position.y();
        odometry.pose.pose.position.z = data.position.z();
        odometry.pose.pose.orientation.x = data.orientation.x();
        odometry.pose.pose.orientation.y = data.orientation.y();
        odometry.pose.pose.orientation.z = data.orientation.z();
        odometry.pose.pose.orientation.w = data.orientation.w();
        odometry.twist.twist.linear.x = data.velocity.x();
        odometry.twist.twist.linear.y = data.velocity.y();
        odometry.twist.twist.linear.z = data.velocity.z();
        odometry.twist.twist.angular.x = data.angular_velocity.x();
        odometry.twist.twist.angular.y = data.angular_velocity.y();
        odometry.twist.twist.angular.z = data.angular_velocity.z();
        this->pub_latest_odometry_.publish(odometry);
        for (uint32_t i = 0; i < data.camera_poses.size(); ++i) {
            geometry_msgs::PoseStamped cam_pose;
            cam_pose.header.stamp = ros::Time(data.timestamp);
            cam_pose.header.frame_id = "world";
            cam_pose.pose.position.x = data.camera_poses[i].position[0].x();
            cam_pose.pose.position.y = data.camera_poses[i].position[0].y();
            cam_pose.pose.position.z = data.camera_poses[i].position[0].z();
            cam_pose.pose.orientation.x = data.camera_poses[i].orientation[0].x();
            cam_pose.pose.orientation.y = data.camera_poses[i].orientation[0].y();
            cam_pose.pose.orientation.z = data.camera_poses[i].orientation[0].z();
            cam_pose.pose.orientation.w = data.camera_poses[i].orientation[0].w();
            this->pub_latest_camera_pose_[i].publish(cam_pose);
            cameraposevisual_.reset();
            cameraposevisual_.add_pose(data.camera_poses[i].position[0], data.camera_poses[i].orientation[0]);
            if (cam_pose.pose.position.x = data.camera_poses[i].position.size() >1){
                cameraposevisual_.add_pose(data.camera_poses[i].position[1], data.camera_poses[i].orientation[1]);
            }
            cameraposevisual_.publish_by(this->pub_camera_pose_visual_[i], odometry.header);
        }
        return;
    }

    void ROS1CallbackManager::publish_full_report_cb(const FullReportData &data){
        // Publish camera pose
        geometry_msgs::PoseStamped cam_pose_odom;
        cam_pose_odom.header.stamp = ros::Time(data.camera_time);
        cam_pose_odom.header.frame_id = "world";
        cam_pose_odom.pose.position.x = data.camera_pose.position.x();
        cam_pose_odom.pose.position.y = data.camera_pose.position.y();
        cam_pose_odom.pose.position.z = data.camera_pose.position.z();
        cam_pose_odom.pose.orientation.x = data.camera_pose.orientation.x();
        cam_pose_odom.pose.orientation.y = data.camera_pose.orientation.y();
        cam_pose_odom.pose.orientation.z = data.camera_pose.orientation.z();
        cam_pose_odom.pose.orientation.w = data.camera_pose.orientation.w();
        pub_camera_pose_[data.camera_id].publish(cam_pose_odom);

        // publish point cloud
        sensor_msgs::PointCloud in_cam_pcd, marginized_pcd;
        in_cam_pcd.header.stamp = ros::Time(data.timestamp);
        in_cam_pcd.header.frame_id = "world";
        for (const auto& point : data.in_window_pointcloud){
            geometry_msgs::Point32 p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            in_cam_pcd.points.push_back(p);
        }
        pub_point_cloud_[data.camera_id].publish(in_cam_pcd);
        // publish marginized point cloud
        
        marginized_pcd.header.stamp = ros::Time(data.timestamp);
        marginized_pcd.header.frame_id = "world";
        for (const auto& point : data.marginized_pointcloud){
            geometry_msgs::Point32 p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marginized_pcd.points.push_back(p);
        }
        pub_margin_cloud_.publish(marginized_pcd);

        if (data.update_latest){
            // publish TF
            tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin( tf::Vector3(data.TF_correct_pose.position.x(),
                 data.TF_correct_pose.position.y(), 
                 data.TF_correct_pose.position.z()) );
            q.setX(data.TF_correct_pose.orientation.x());
            q.setY(data.TF_correct_pose.orientation.y());
            q.setZ(data.TF_correct_pose.orientation.z());
            q.setW(data.TF_correct_pose.orientation.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time(data.timestamp), "world", "body"));
            for (uint32_t i = 0; i < data.pub_camera_extrinsic_params.size(); ++i) {
                tf::Transform cam_transform;
                tf::Quaternion cam_q;
                cam_transform.setOrigin(tf::Vector3(data.pub_camera_extrinsic_params[i].position.x(),
                     data.pub_camera_extrinsic_params[i].position.y(), 
                     data.pub_camera_extrinsic_params[i].position.z()) );
                    
                cam_q.setX(data.pub_camera_extrinsic_params[i].orientation.x());
                cam_q.setY(data.pub_camera_extrinsic_params[i].orientation.y());
                cam_q.setZ(data.pub_camera_extrinsic_params[i].orientation.z());
                cam_q.setW(data.pub_camera_extrinsic_params[i].orientation.w());
                cam_transform.setRotation(cam_q);
                br.sendTransform(tf::StampedTransform(cam_transform, ros::Time(data.timestamp), "world", 
                    "camera_" + std::to_string(i)));
                nav_msgs::Odometry cam_odometry;
                cam_odometry.header.stamp = ros::Time(data.timestamp);
                cam_odometry.header.frame_id = "world";
                cam_odometry.pose.pose.position.x = data.pub_camera_extrinsic_params[i].position.x();
                cam_odometry.pose.pose.position.y = data.pub_camera_extrinsic_params[i].position.y();
                cam_odometry.pose.pose.position.z = data.pub_camera_extrinsic_params[i].position.z();
                cam_odometry.pose.pose.orientation.x = data.pub_camera_extrinsic_params[i].orientation.x();
                cam_odometry.pose.pose.orientation.y = data.pub_camera_extrinsic_params[i].orientation.y();
                cam_odometry.pose.pose.orientation.z = data.pub_camera_extrinsic_params[i].orientation.z();
                cam_odometry.pose.pose.orientation.w = data.pub_camera_extrinsic_params[i].orientation.w();
                pub_extrinsic_[i].publish(cam_odometry);
            }

            //publish odometry and path
            nav_msgs::Odometry latest_odometry;
            latest_odometry.header.stamp = ros::Time(data.timestamp);
            latest_odometry.header.frame_id = "world";
            latest_odometry.child_frame_id = "world";
            latest_odometry.pose.pose.position.x = data.latest_frame_pose.position.x();
            latest_odometry.pose.pose.position.y = data.latest_frame_pose.position.y();
            latest_odometry.pose.pose.position.z = data.latest_frame_pose.position.z();
            latest_odometry.pose.pose.orientation.x = data.latest_frame_pose.orientation.x();
            latest_odometry.pose.pose.orientation.y = data.latest_frame_pose.orientation.y();
            latest_odometry.pose.pose.orientation.z = data.latest_frame_pose.orientation.z();
            latest_odometry.pose.pose.orientation.w = data.latest_frame_pose.orientation.w();
            latest_odometry.twist.twist.linear.x = data.latest_frame_pose.velocity.x();
            latest_odometry.twist.twist.linear.y = data.latest_frame_pose.velocity.y();
            latest_odometry.twist.twist.linear.z = data.latest_frame_pose.velocity.z();
            pub_latest_odometry_.publish(latest_odometry);
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time(data.timestamp);
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose = latest_odometry.pose.pose;
            path_.header.stamp = ros::Time(data.timestamp);
            path_.header.frame_id = "world";
            path_.poses.push_back(pose_stamped);
            pub_path_.publish(path_);
            
            // publish key pose
            visualization_msgs::Marker keypose_marker;
            keypose_marker.header.frame_id = "world";
            keypose_marker.header.stamp = ros::Time(data.timestamp);
            keypose_marker.ns = "key_poses";
            keypose_marker.header.frame_id = "world";
            keypose_marker.type = visualization_msgs::Marker::SPHERE_LIST;
            keypose_marker.action = visualization_msgs::Marker::ADD;
            keypose_marker.pose.orientation.w = 1.0;
            keypose_marker.lifetime = ros::Duration();
            keypose_marker.id = 0;
            keypose_marker.scale.x = 0.05;
            keypose_marker.scale.y = 0.05;
            keypose_marker.scale.z = 0.05;
            keypose_marker.color.r = 1.0;
            keypose_marker.color.a = 1.0;
            for (const auto& key_p : data.key_poses){
                geometry_msgs::Point p;
                p.x = key_p.x();
                p.y = key_p.y();
                p.z = key_p.z();
                keypose_marker.points.push_back(p);
            }
            pub_key_poses_.publish(keypose_marker);
        }
        if (data.have_keyframe){
            // publish keyframe pose
            geometry_msgs::PoseStamped keyframe_pose;
            keyframe_pose.header.stamp = ros::Time(data.timestamp);
            keyframe_pose.header.frame_id = "world";
            keyframe_pose.pose.position.x = data.keyframe_pose.position.x();
            keyframe_pose.pose.position.y = data.keyframe_pose.position.y();
            keyframe_pose.pose.position.z = data.keyframe_pose.position.z();
            keyframe_pose.pose.orientation.x = data.keyframe_pose.orientation.x();
            keyframe_pose.pose.orientation.y = data.keyframe_pose.orientation.y();
            keyframe_pose.pose.orientation.z = data.keyframe_pose.orientation.z();
            keyframe_pose.pose.orientation.w = data.keyframe_pose.orientation.w();
            pub_keyframe_pose_.publish(keyframe_pose);
        }
        return;
    }
}
