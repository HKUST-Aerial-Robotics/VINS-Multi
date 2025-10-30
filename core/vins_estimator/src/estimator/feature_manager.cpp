/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"
#include <spdlog/spdlog.h>
#include <cassert>

namespace vins_multi{

FeatureManager::FeatureManager(bool depth, bool stereo, vector<shared_ptr<ImageFrame>>& image_frame_ptr)
    :depth_(depth), stereo_(stereo), image_frame_ptr_(image_frame_ptr)
{
    num_frame_ = 0;
}


void FeatureManager::clearState()
{
    feature_.clear();
    num_frame_ = 0;
}

FeatureManager::~FeatureManager(){
    // ROS_ERROR("feature manager deleted");
    spdlog::info("feature manager deleted");

}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature_)
    {
        it.second.used_num = it.second.feature_per_frame.size();
        if (it.second.used_num >= MIN_TRACK_FRAME_FOR_OPT)
        {
            cnt++;
        }
    }
    return cnt;
}

double* FeatureManager::getFeatureInvDepth(int feature_id){

    auto it = feature_.find(feature_id);

    return it == feature_.end() ? nullptr : &(it->second.inv_depth);
}

shared_ptr<ImageFrame> FeatureManager::getStartFrame(int feature_id){
    auto it = feature_.find(feature_id);

    return it == feature_.end() ? shared_ptr<ImageFrame>() : image_frame_ptr_[it->second.start_frame];
}

// vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
// {
//     vector<pair<Vector3d, Vector3d>> corres;
//     for (auto &it : feature_)
//     {
//         if (it.second.start_frame <= frame_count_l && it.second.endFrame() >= frame_count_r)
//         {
//             Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
//             int idx_l = frame_count_l - it.second.start_frame;
//             int idx_r = frame_count_r - it.second.start_frame;

//             // auto feature_pt_it = it.second.feature_per_frame.begin();
//             // for(int i = 0; i < it.second.feature_per_frame.size(); i++, feature_pt_it = next(feature_pt_it)){
//             //     if(i == idx_l){
//             //         a = feature_pt_it->point;
//             //     }
//             //     if(i == idx_r){
//             //         b = feature_pt_it->point;
//             //         break;
//             //     }
//             // }

//             a = it.second.feature_per_frame[idx_l].point;
//             b = it.second.feature_per_frame[idx_r].point;

//             corres.emplace_back(make_pair(a, b));
//         }
//     }
//     return corres;
// }

bool FeatureManager::addFeatureCheckParallax(const map<int,FeaturePerFrame> &feature_pts, double td)
{
    // ROS_INFO("input feature_: %d", (int)feature_pts.size());
    // ROS_INFO("num of feature_: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num_ = 0;
    last_average_parallax_ = 0;
    new_feature_num_ = 0;
    long_track_num_ = 0;
    for (auto &id_pts : feature_pts)
    {
        // FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        // assert(id_pts.second[0].first == 0);
        // if(id_pts.second.size() == 2)
        // {
        //     f_per_fra.rightObservation(id_pts.second[1].second);
        //     assert(id_pts.second[1].first == 1);
        // }

        int feature_id = id_pts.first;
        // auto it = find_if(feature_.begin(), feature_.end(), [feature_id](const FeaturePerId &it)
        //                   {
        //     return it.feature_id == feature_id;
        //                   });

        // if (it == feature_.end())
        // {
        //     feature_.emplace_back(FeaturePerId(feature_id, frame_count));
        //     feature_.back().feature_per_frame.emplace_back(id_pts.second);
        //     feature_.back().feature_per_frame.back().cur_td = td;
        //     if(id_pts.second.is_depth)
        //         feature_.back().estimated_depth = id_pts.second.depth;
        //     new_feature_num_++;
        // }
        // else if (it->feature_id == feature_id)
        // {
        //     it->feature_per_frame.emplace_back(id_pts.second);
        //     it->feature_per_frame.back().cur_td = td;
        //     last_track_num_++;
        //     if( it-> feature_per_frame.size() >= MIN_TRACK_FRAME_FOR_OPT)
        //         long_track_num_++;
        // }

        auto it = feature_.find(feature_id);
        if (it == feature_.end()){
            auto emplace_it = feature_.emplace(feature_id, FeaturePerId(feature_id, num_frame_)).first;
            emplace_it->second.feature_per_frame.emplace_back(id_pts.second);
            emplace_it->second.feature_per_frame.back().cur_td = td;
            // if(id_pts.second.is_depth)
            //     emplace_it->second.estimated_depth = id_pts.second.depth;
            new_feature_num_++;
        }
        else{
            it->second.feature_per_frame.emplace_back(id_pts.second);
            it->second.feature_per_frame.back().cur_td = td;
            last_track_num_++;
            if( it->second.feature_per_frame.size() >= MIN_TRACK_FRAME_FOR_OPT)
                long_track_num_++;
        }
    }

    num_frame_++;
    //if (frame_count < 2 || last_track_num_ < 20)
    //if (frame_count < 2 || last_track_num_ < 20 || new_feature_num_ > 0.5 * last_track_num_)
    if (num_frame_ < 3 || last_track_num_ < 20 || long_track_num_ < 40 || new_feature_num_ > 0.5 * last_track_num_)
        return true;

    for (auto &it_per_id : feature_)
    {
        if (it_per_id.second.start_frame < num_frame_ - 2 &&
            it_per_id.second.endFrame() >= num_frame_-1)
        {
            parallax_sum += compensatedParallax2(it_per_id.second);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        // ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        // ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        spdlog::debug("parallax_sum: {}, parallax_num: {}", parallax_sum, parallax_num);
        spdlog::debug("current parallax: {}", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax_ = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

bool FeatureManager::checkParallax(vector<shared_ptr<ImageFrame>>& frameHist)
{
    double sum_parallax = 0.0;
    int feature_no_depth = 0;

    for (auto &it_per_id : feature_)
    {
        if(it_per_id.second.estimated_depth < 0.0){
            // int index = frameCnt - it_per_id.second.start_frame;
            double parallax = (it_per_id.second.feature_per_frame.front().point.head(2) - it_per_id.second.feature_per_frame.back().point.head(2)).norm();
            sum_parallax = sum_parallax + parallax;
            feature_no_depth ++;
        }

    }
    double average_parallax = 1.0 * sum_parallax / feature_no_depth;

    return average_parallax * 460 > 30;
}

// vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
// {
//     vector<pair<Vector3d, Vector3d>> corres;
//     for (auto &it : feature_)
//     {
//         if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
//         {
//             Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
//             int idx_l = frame_count_l - it.start_frame;
//             int idx_r = frame_count_r - it.start_frame;

//             a = it.feature_per_frame[idx_l].point;

//             b = it.feature_per_frame[idx_r].point;

//             corres.push_back(make_pair(a, b));
//         }
//     }
//     return corres;
// }

void FeatureManager::setDepth()
{
    // int feature_index = -1;
    // for (auto &it_per_id : feature_)
    // {
    //     it_per_id.second.used_num = it_per_id.second.feature_per_frame.size();
    //     if (it_per_id.second.used_num < MIN_TRACK_FRAME_FOR_OPT)
    //         continue;

    //     it_per_id.second.estimated_depth = 1.0 / para_Feature_[++feature_index];
    //     // it_per_id.second.estimated_depth = 1.0 / x(++feature_index);

    //     //ROS_INFO("feature_ id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
    //     if (it_per_id.second.estimated_depth < 0)
    //     {
    //         it_per_id.second.solve_flag = FeaturePerId::OUTLIER;
    //     }
    //     else
    //         it_per_id.second.solve_flag = FeaturePerId::ESTIMATED;
    // }

    for (auto &it_per_id : feature_){

        it_per_id.second.used_num = it_per_id.second.feature_per_frame.size();

        if (it_per_id.second.solve_flag == FeaturePerId::ESTIMATED){

            if (it_per_id.second.inv_depth > 0){
                it_per_id.second.estimated_depth = 1.0 / it_per_id.second.inv_depth;
            }
            else{
                // printf("set feature id: %d to outlier, depth:%lf, inv_depth: %lf, is depth:%d, measured depth: %lf\n", it_per_id.second.feature_id, it_per_id.second.estimated_depth, it_per_id.second.inv_depth, it_per_id.second.feature_per_frame.front().is_depth, it_per_id.second.feature_per_frame.front().depth);
                it_per_id.second.solve_flag = FeaturePerId::OUTLIER;
            }
        }
    }
}

void FeatureManager::setInvDepth(){
    // int input_outlier_cnt = 0;
    // int long_track_num = 0;
    for (auto &it_per_id : feature_){

        if(it_per_id.second.solve_flag == FeaturePerId::OUTLIER){
            // input_outlier_cnt++;
            continue;
        }

        it_per_id.second.used_num = it_per_id.second.feature_per_frame.size();

        it_per_id.second.inv_depth = 1.0 / it_per_id.second.estimated_depth;
        if (it_per_id.second.used_num >= MIN_TRACK_FRAME_FOR_OPT){
            it_per_id.second.solve_flag = FeaturePerId::LONGTRACK;
            // long_track_num++;
        }
        else{
            it_per_id.second.solve_flag = FeaturePerId::UNINITIALIZED;
        }
    }
    // cout<<"input long track: "<<long_track_num<<"\n";
    // cout<<"input outlier: "<<input_outlier_cnt<<"\n\n\n";
}

void FeatureManager::removeFailures()
{
    int remove_cnt = 0;
    // ROS_WARN("feature size before remove: %ld", feature_.size());
    for (auto it = feature_.begin(), it_next = feature_.begin();
         it != feature_.end(); it = it_next)
    {
        it_next++;
        if (it->second.solve_flag == FeaturePerId::OUTLIER){
            // printf("remove outlier feature id: %d, size: %ld\n", it->second.feature_id, it->second.feature_per_frame.size());
            it_next = feature_.erase(it);
            remove_cnt++;
        }
    }

    // ROS_WARN("feature size after remove: %ld", feature_.size());
    // ROS_WARN("cam %d remove outlier cnt: %d", this->cam_info_ptr_->module_id_, remove_cnt);
}

// void FeatureManager::clearDepth()
// {
//     for (auto &it_per_id : feature_)
//         it_per_id.estimated_depth = -1;
// }

// void FeatureManager::getDepthVector()
// {
//     // VectorXd dep_vec(getFeatureCount());

//     para_Feature_.resize(getFeatureCount());
//     int feature_index = -1;
//     for (auto &it_per_id : feature_)
//     {
//         it_per_id.second.used_num = it_per_id.second.feature_per_frame.size();
//         if (it_per_id.second.used_num < MIN_TRACK_FRAME_FOR_OPT)
//             continue;
// #if 1
//         para_Feature_[++feature_index] = 1. / it_per_id.second.estimated_depth;
// #else
//         // dep_vec(++feature_index) = it_per_id->estimated_depth;
// #endif
//     }
//     // return dep_vec;
// }


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P,
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature_ tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

bool FeatureManager::initFramePoseByPnP(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic, const Quaterniond& ric)
{
    if(frameHist.size() == 0){
        return false;
    }
    if(frameHist.size() == 1){
        return true;
    }

    vector<cv::Point2f> pts2D;
    vector<cv::Point3f> pts3D;
    for (auto &it_per_id : feature_)
    {
        if (it_per_id.second.estimated_depth > 0)
        {
            if((int)it_per_id.second.feature_per_frame.size() > 1)
            {
                Vector3d ptsInCam = ric * (it_per_id.second.feature_per_frame.front().point * it_per_id.second.feature_per_frame.front().depth) + tic;
                Vector3d ptsInWorld = frameHist[it_per_id.second.start_frame]->R_ * ptsInCam + frameHist[it_per_id.second.start_frame]->T_;

                cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                cv::Point2f point2d(it_per_id.second.feature_per_frame.back().point.x(), it_per_id.second.feature_per_frame.back().point.y());
                pts3D.push_back(point3d);
                pts2D.push_back(point2d);
            }
        }
    }
    Eigen::Matrix3d RCam;
    Eigen::Vector3d PCam;
    // trans to w_T_cam
    RCam = frameHist.back()->R_ * ric;
    PCam = frameHist.back()->R_ * tic + frameHist.back()->T_;

    if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
    {
        // trans to w_T_imu
        Eigen::Matrix3d R_ic = ric.normalized().toRotationMatrix();
        frameHist.back()->R_ = RCam * R_ic.transpose();
        frameHist.back()->T_ = -RCam * R_ic.transpose() * tic + PCam;
    }

    return true;
}

bool FeatureManager::initFramePoseByICP(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic, const Quaterniond& ric)
{
    if(frameHist.size() == 0){
        return false;
    }
    if(frameHist.size() == 1){
        return true;
    }


    vector<Vector3d> pts3D_ref;
    vector<Vector3d> pts3D;
    Vector3d mean_pts3D_ref = Vector3d::Zero();
    Vector3d mean_pts3D = Vector3d::Zero();

    for (auto &it_per_id : feature_)
    {
        if(it_per_id.second.start_frame >= it_per_id.second.feature_per_frame.size() - 1 || it_per_id.second.endFrame() < frameHist.size() - 1){
            continue;
        }

        if (it_per_id.second.feature_per_frame.front().is_depth)
        {
            // int index = frameHist.size() - it_per_id.start_frame;
            if(it_per_id.second.feature_per_frame.back().is_depth)
            {
                Vector3d ptsInCam = ric * (it_per_id.second.feature_per_frame.front().point * it_per_id.second.feature_per_frame.front().depth) + tic;
                // Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];
                Vector3d ptsInWorld = frameHist[it_per_id.second.start_frame]->R_ * ptsInCam + frameHist[it_per_id.second.start_frame]->T_;

                mean_pts3D_ref += ptsInWorld;
                pts3D_ref.emplace_back(ptsInWorld);

                // ROS_WARN("pt id: %d, %d", it_per_id.first, it_per_id.second.feature_id);
                // cout<<"pt ref in cam: "<<ptsInCam.transpose()<<endl;

                ptsInCam = ric * (it_per_id.second.feature_per_frame.back().point * it_per_id.second.feature_per_frame.back().depth) + tic;

                // cout<<"pt in cam: "<<ptsInCam.transpose()<<endl;
                mean_pts3D += ptsInCam;
                pts3D.emplace_back(ptsInCam);

            }
        }
    }

    if (pts3D_ref.size() > 4){
        mean_pts3D_ref /= pts3D_ref.size();
        mean_pts3D /= pts3D.size();
        Matrix3d W = Matrix3d::Zero();
        for(unsigned i = 0; i < pts3D_ref.size(); i++){
            W += (pts3D_ref[i] - mean_pts3D_ref) * (pts3D[i] - mean_pts3D).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd_w(W, ComputeFullU | ComputeFullV);
        frameHist.back()->R_ = svd_w.matrixU() * svd_w.matrixV().transpose();
        frameHist.back()->T_ = mean_pts3D_ref - frameHist.back()->R_ * mean_pts3D;
        // Rs[frameCnt] = svd_w.matrixU() * svd_w.matrixV().transpose();
        // Ps[frameCnt] = mean_pts3D_ref - Rs[frameCnt] * mean_pts3D;

        // cout<<"Ps "<< frameHist.back()->cam_module_unique_id_ <<" init "<<frameHist.size() - 1<<":"<< frameHist.back()->T_.transpose()<<endl;
        // cout<<"pt ref size: "<<pts3D_ref.size()<<endl;

        return true;
    }


    return false;
    // return initFramePoseByICP(frameHist, tic, ric, frameHist.size() - 1);
}

// bool FeatureManager::initFramePoseByICP(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic, const Quaterniond& ric, int frameToSolveIdx)
// {
//     if(frameHist.size() == 0 || frameToSolveIdx < 0){
//         return false;
//     }
//     if(frameHist.size() == 1 || frameToSolveIdx == 0){
//         return true;
//     }


//     vector<Vector3d> pts3D_ref;
//     vector<Vector3d> pts3D;
//     Vector3d mean_pts3D_ref = Vector3d::Zero();
//     Vector3d mean_pts3D = Vector3d::Zero();

//     for (auto &it_per_id : feature_)
//     {
//         if(it_per_id.second.start_frame >= frameToSolveIdx || it_per_id.second.endFrame() < frameToSolveIdx){
//             continue;
//         }

//         if (it_per_id.second.feature_per_frame.front().is_depth)
//         {
//             int index = frameToSolveIdx - it_per_id.second.start_frame;
//             auto& this_feature_pt = it_per_id.second.feature_per_frame[index];
//             if(this_feature_pt.is_depth)
//             {
//                 Vector3d ptsInCam = ric * (it_per_id.second.feature_per_frame.front().point * it_per_id.second.estimated_depth) + tic;
//                 // Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];
//                 Vector3d ptsInWorld = frameHist[it_per_id.second.start_frame]->R_ * ptsInCam + frameHist[it_per_id.second.start_frame]->T_;

//                 mean_pts3D_ref += ptsInWorld;
//                 pts3D_ref.emplace_back(ptsInWorld);

//                 // ROS_WARN("pt id: %d, %d", it_per_id.first, it_per_id.second.feature_id);
//                 // cout<<"pt ref in cam: "<<ptsInCam.transpose()<<endl;

//                 ptsInCam = ric * (this_feature_pt.point * this_feature_pt.depth) + tic;

//                 // cout<<"pt in cam: "<<ptsInCam.transpose()<<endl;
//                 mean_pts3D += ptsInCam;
//                 pts3D.emplace_back(ptsInCam);

//             }
//         }
//     }

//     if (pts3D_ref.size() > 4){
//         mean_pts3D_ref /= pts3D_ref.size();
//         mean_pts3D /= pts3D.size();
//         Matrix3d W = Matrix3d::Zero();
//         for(unsigned i = 0; i < pts3D_ref.size(); i++){
//             W += (pts3D_ref[i] - mean_pts3D_ref) * (pts3D[i] - mean_pts3D).transpose();
//         }
//         Eigen::JacobiSVD<Eigen::Matrix3d> svd_w(W, ComputeFullU | ComputeFullV);
//         frameHist[frameToSolveIdx]->R_ = svd_w.matrixU() * svd_w.matrixV().transpose();
//         frameHist[frameToSolveIdx]->T_ = mean_pts3D_ref - frameHist[frameToSolveIdx]->R_ * mean_pts3D;
//         // Rs[frameCnt] = svd_w.matrixU() * svd_w.matrixV().transpose();
//         // Ps[frameCnt] = mean_pts3D_ref - Rs[frameCnt] * mean_pts3D;

//         // cout<<"Ps "<< frameHist.back()->cam_module_unique_id_ <<" init "<<frameHist.size() - 1<<":"<< frameHist.back()->T_.transpose()<<endl;
//         // cout<<"pt ref size: "<<pts3D_ref.size()<<endl;

//         return true;
//     }

//     return false;
// }

void FeatureManager::triangulate(vector<shared_ptr<ImageFrame>>& frameHist, const Vector3d& tic0, const Quaterniond& ric0, const Vector3d& tic1, const Quaterniond& ric1)
{
    int outlier_cnt = 0;
    int outlier_depth_cnt = 0;
    int outlier_proj_cnt = 0;

    for (auto &it_per_id : feature_)
    {
        if (it_per_id.second.estimated_depth > 0){
            // continue;
        }
        else{
            // if(STEREO && it_per_id.feature_per_frame[0].is_stereo)
            if(it_per_id.second.feature_per_frame.front().is_stereo)
            {
                int imu_i = it_per_id.second.start_frame;
                Vector3d Ps_imui = frameHist[imu_i]->T_;
                Matrix3d Rs_imui = frameHist[imu_i]->R_.toRotationMatrix();

                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps_imui + Rs_imui * tic0;
                Eigen::Matrix3d R0 = Rs_imui * ric0;
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;
                // cout << "left pose " << leftPose << endl;

                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps_imui + Rs_imui * tic1;
                Eigen::Matrix3d R1 = Rs_imui * ric1;
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;
                // cout << "right pose " << rightPose << endl;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.second.feature_per_frame.front().point.head(2);
                point1 = it_per_id.second.feature_per_frame.front().pointRight.head(2);
                // cout << "point0 " << point0.transpose() << endl;
                // cout << "point1 " << point1.transpose() << endl;

                triangulatePoint(leftPose, rightPose, point0, point1, point3d);
                Eigen::Vector3d localPoint;
                localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
                double depth = localPoint.z();
                if (depth > 0)
                    it_per_id.second.estimated_depth = depth;
                else
                    it_per_id.second.estimated_depth = INIT_DEPTH;

                // Vector3d ptsGt = pts_gt[it_per_id.second.feature_id];
                // printf("stereo %d pts: %f %f %f depth: %f\n",it_per_id.second.feature_id, point3d.x(), point3d.y(), point3d.z(), depth);
                continue;
            }
            else if(it_per_id.second.feature_per_frame.size() > 1)
            {
                int imu_i = it_per_id.second.start_frame;
                Vector3d Ps_imui = frameHist[imu_i]->T_;
                Matrix3d Rs_imui = frameHist[imu_i]->R_.toRotationMatrix();

                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps_imui + Rs_imui * tic0;
                Eigen::Matrix3d R0 = Rs_imui * ric0;
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;

                imu_i++;
                Ps_imui = frameHist[imu_i]->T_;
                Rs_imui = frameHist[imu_i]->R_;

                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps_imui + Rs_imui * tic0;
                Eigen::Matrix3d R1 = Rs_imui * ric0;
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.second.feature_per_frame.front().point.head(2);
                auto next_frame_feature_ptr = next(it_per_id.second.feature_per_frame.begin());
                point1 = next_frame_feature_ptr->point.head(2);
                triangulatePoint(leftPose, rightPose, point0, point1, point3d);
                Eigen::Vector3d localPoint;
                localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
                double depth = localPoint.z();

                if(depth > 0.1){
                    if(it_per_id.second.feature_per_frame.front().is_depth){
                        // check if valid depth in the first frame
                        double measured_depth = it_per_id.second.feature_per_frame.front().depth;
                        double depth_error_rate = fabs((depth - measured_depth) / measured_depth);
                        if(depth_error_rate > 0.5){
                            it_per_id.second.feature_per_frame.front().is_depth = false;
                            // ROS_ERROR("init rej first depth");
                            // cout<<"pt: "<<it_per_id.second.feature_per_frame.front().uv.transpose()<<endl;
                            // cout<<"triangulated depth: "<<depth<<", measured depth: "<<measured_depth<<endl;
                        }
                        else{
                            depth = measured_depth;
                        }
                    }


                    if(next_frame_feature_ptr->is_depth){
                        // check if valid depth in the second frame
                        Eigen::Vector3d local_pt_right = rightPose.leftCols<3>() * point3d + rightPose.rightCols<1>();
                        double trangulated_depth = local_pt_right.z();
                        double measured_depth = next_frame_feature_ptr->depth;
                        double depth_error_rate = fabs((trangulated_depth - measured_depth) / measured_depth);
                        if(depth_error_rate > 0.5){
                            next_frame_feature_ptr->is_depth = false;
                            // ROS_ERROR("init rej second depth");

                        }
                    }

                    it_per_id.second.estimated_depth = depth;
                }
                else{
                    it_per_id.second.estimated_depth = INIT_DEPTH;
                }

                // if (depth > 0)
                //     it_per_id.second.estimated_depth = depth;
                // else
                //     it_per_id.second.estimated_depth = INIT_DEPTH;
                /*
                Vector3d ptsGt = pts_gt[it_per_id.feature_id];
                printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                                ptsGt.x(), ptsGt.y(), ptsGt.z());
                */
                continue;
            }
            it_per_id.second.used_num = it_per_id.second.feature_per_frame.size();
            if (it_per_id.second.used_num < 2)
                continue;

            int imu_i = it_per_id.second.start_frame, imu_j = imu_i - 1;
            Vector3d Ps_imui = frameHist[imu_i]->T_;
            Matrix3d Rs_imui = frameHist[imu_i]->R_.toRotationMatrix();

            Eigen::MatrixXd svd_A(2 * it_per_id.second.feature_per_frame.size(), 4);
            int svd_idx = 0;

            Eigen::Matrix<double, 3, 4> P0;
            Eigen::Vector3d t0 = Ps_imui + Rs_imui * tic0;
            Eigen::Matrix3d R0 = Rs_imui * ric0;
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();

            for (auto &it_per_frame : it_per_id.second.feature_per_frame)
            {
                imu_j++;

                Vector3d Ps_imuj = frameHist[imu_j]->T_;
                Matrix3d Rs_imuj = frameHist[imu_j]->R_.toRotationMatrix();

                Eigen::Vector3d t1 = Ps_imuj + Rs_imuj * tic0;
                Eigen::Matrix3d R1 = Rs_imuj * ric0;
                Eigen::Vector3d t = R0.transpose() * (t1 - t0);
                Eigen::Matrix3d R = R0.transpose() * R1;
                Eigen::Matrix<double, 3, 4> P;
                P.leftCols<3>() = R.transpose();
                P.rightCols<1>() = -R.transpose() * t;
                Eigen::Vector3d f = it_per_frame.point.normalized();
                svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
                svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

                if (imu_i == imu_j)
                    continue;
            }
            // ROS_ASSERT(svd_idx == svd_A.rows());
            assert(svd_idx == svd_A.rows()); // efficiency sensetive point
            Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
            double svd_method_depth = svd_V[2] / svd_V[3];
            //it_per_id->estimated_depth = -b / A;
            //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

            if(svd_method_depth > 0.1){
                if(it_per_id.second.feature_per_frame.front().is_depth){
                    // check if valid depth in the first frame
                    double measured_depth = it_per_id.second.feature_per_frame.front().depth;
                    double depth_error_rate = fabs((svd_method_depth - measured_depth) / measured_depth);
                    if(depth_error_rate > 0.2){
                        it_per_id.second.feature_per_frame.front().is_depth = false;
                            // ROS_ERROR("init svd rej first depth");
                    }
                    else{
                        svd_method_depth = measured_depth;
                    }
                }

                it_per_id.second.estimated_depth = svd_method_depth;

            }
            else{
                it_per_id.second.estimated_depth = INIT_DEPTH;
            }

            // it_per_id.second.estimated_depth = svd_method;
            // //it_per_id->estimated_depth = INIT_DEPTH;

            // if (it_per_id.second.estimated_depth < 0.1)
            // {
            //     it_per_id.second.estimated_depth = INIT_DEPTH;
            // }
        }

    }

    // cout<<"total feature num: "<<feature_.size()<<endl;
    // cout<<"outlier cnt: "<<outlier_cnt<<endl;
    // cout<<"outlier depth cnt: "<<outlier_depth_cnt<<endl;
    // cout<<"outlier proj cnt: "<<outlier_proj_cnt<<endl;
    // cout<<"\n\n"<<endl;
}

double FeatureManager::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj, double &reproj_depth)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();

    reproj_depth = pts_cj.z();
    return sqrt(rx * rx + ry * ry);
}

void FeatureManager::outliersRejection()
{
    //return;

    int estimated_feature_cnt = 0;
    int outlier_cnt = 0;

    int feature_index = -1;
    for (auto &it_per_id : feature_)
    {
        double err = 0;
        int errCnt = 0;
        // it_per_id.second.used_num = it_per_id.second.feature_per_frame.size();
        // if (it_per_id.second.used_num < MIN_TRACK_FRAME_FOR_OPT)

        if(it_per_id.second.solve_flag != FeaturePerId::ESTIMATED){
            continue;
        }
        estimated_feature_cnt++;

        feature_index ++;
        double depth = it_per_id.second.estimated_depth;

        // check first frame depth measurement
        auto first_frame_feature_ptr = it_per_id.second.feature_per_frame.begin();
        if(first_frame_feature_ptr->is_depth){
            double depth_err = (first_frame_feature_ptr->depth - depth);
            double normalized_depth_err = fabs(depth_err / depth);

            if(normalized_depth_err > 0.1){
                first_frame_feature_ptr->is_depth = false;
                // ROS_ERROR("opt rej first depth");
                // cout<<"pt: "<<it_per_id.second.feature_per_frame.front().uv.transpose()<<endl;
                // cout<<"opt first depth: "<<depth<<", measured depth: "<<first_frame_feature_ptr->depth<<endl;

            }
        }
        else if(first_frame_feature_ptr->depth > 0.0){
            double depth_err = (first_frame_feature_ptr->depth - depth);
            double normalized_depth_err = fabs(depth_err / depth);

            if(normalized_depth_err < 0.1){
                first_frame_feature_ptr->is_depth = true;
                // ROS_WARN("opt reaccept first depth");
            }
        }

        int imu_i = it_per_id.second.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.second.feature_per_frame.front().point;
        Matrix3d R_i = image_frame_ptr_[imu_i]->R_.toRotationMatrix();
        Vector3d P_i = image_frame_ptr_[imu_i]->T_;
        Matrix3d ric_0 = cam_info_ptr_->ric_[0].toRotationMatrix();
        Vector3d tic_0 = cam_info_ptr_->tic_[0];

        double proj_depth_j = -1.0;
        for (auto &it_per_frame : it_per_id.second.feature_per_frame)
        {
            imu_j++;
            Matrix3d R_j = image_frame_ptr_[imu_j]->R_.toRotationMatrix();
            Vector3d P_j = image_frame_ptr_[imu_j]->T_;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(R_i, P_i, ric_0, tic_0,
                                                    R_j, P_j, ric_0, tic_0,
                                                    depth, pts_i, pts_j, proj_depth_j);

                if(it_per_frame.is_depth){
                    double depth_err = (it_per_frame.depth - proj_depth_j);
                    double normalized_depth_err = fabs(depth_err / proj_depth_j);

                    if(normalized_depth_err > 0.1){
                        it_per_frame.is_depth = false;
                        // ROS_ERROR("opt rej reproj depth");
                        // cout<<"pt: "<<it_per_id.second.feature_per_frame.front().uv.transpose()<<endl;
                        // cout<<"opt reproj depth: "<<proj_depth_j<<", measured depth: "<<it_per_frame.depth<<endl;
                    }
                }
                else if(it_per_frame.depth > 0.0){
                    double depth_err = (it_per_frame.depth - proj_depth_j);
                    double normalized_depth_err = fabs(depth_err / proj_depth_j);

                    if(normalized_depth_err < 0.1){
                        it_per_frame.is_depth = true;
                        // ROS_WARN("opt reaccept reproj depth");
                    }
                }

                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(it_per_frame.is_stereo)
            {
                Matrix3d ric_1 = cam_info_ptr_->ric_[1].toRotationMatrix();
                Vector3d tic_1 = cam_info_ptr_->tic_[1];

                Vector3d pts_j_right = it_per_frame.pointRight;
                // proj_depth_j : occupied parameter
                if(imu_i != imu_j)
                {
                    double tmp_error = reprojectionError(R_i, P_i, ric_0, tic_0,
                                                        R_j, P_j, ric_1, tic_1,
                                                        depth, pts_i, pts_j_right, proj_depth_j);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(R_i, P_i, ric_0, tic_0,
                                                        R_j, P_j, ric_1, tic_1,
                                                        depth, pts_i, pts_j_right, proj_depth_j);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3){
            it_per_id.second.solve_flag = FeaturePerId::OUTLIER;
            outlier_cnt++;
        }

    }

    // ROS_INFO("estimated_feature_cnt: %d, outlier_cnt: %d", estimated_feature_cnt, outlier_cnt);
}

// void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
// {
//     for (auto it = feature_.begin(), it_next = feature_.begin();
//          it != feature_.end(); it = it_next)
//     {
//         it_next++;

//         if (it->start_frame != 0)
//             it->start_frame--;
//         else
//         {
//             Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
//             it->feature_per_frame.erase(it->feature_per_frame.begin());
//             if (it->feature_per_frame.size() < 2)
//             {
//                 feature_.erase(it);
//                 continue;
//             }
//             else
//             {
//                 Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
//                 Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
//                 Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
//                 double dep_j = pts_j(2);
//                 if (dep_j > 0)
//                     it->estimated_depth = dep_j;
//                 else
//                     it->estimated_depth = INIT_DEPTH;
//             }
//         }
//         // remove tracking-lost feature_ after marginalize
//         /*
//         if (it->endFrame() < WINDOW_SIZE - 1)
//         {
//             feature_.erase(it);
//         }
//         */
//     }
// }

// void FeatureManager::removeBack()
// {
//     for (auto it = feature_.begin(), it_next = feature_.begin();
//          it != feature_.end(); it = it_next)
//     {
//         it_next++;

//         if (it->start_frame != 0)
//             it->start_frame--;
//         else
//         {
//             it->feature_per_frame.erase(it->feature_per_frame.begin());
//             if (it->feature_per_frame.size() == 0)
//                 feature_.erase(it);
//         }
//     }
// }

// void FeatureManager::removeFront(int frame_count)
// {
    // for (auto it = feature_.begin(), it_next = feature_.begin(); it != feature_.end(); it = it_next)
    // {
    //     it_next++;

    //     if (it->second.start_frame == frame_count)
    //     {
    //         it->second.start_frame--;
    //     }
    //     else
    //     {
    //         int j = WINDOW_SIZE - 1 - it->second.start_frame;
    //         if (it->second.endFrame() < frame_count - 1)
    //             continue;
    //         it->second.feature_per_frame.erase(it->second.feature_per_frame.begin() + j);
    //         if (it->second.feature_per_frame.size() == 0)
    //             feature_.erase(it);
    //     }
    // }
// }
void FeatureManager::remove(const int idx){

    for (auto it = feature_.begin(); it != feature_.end();)
    {

        if (it->second.start_frame > idx)
        {
            it->second.start_frame--;
        }
        else
        {
            if(it->second.endFrame() < idx){

            }else{
                auto it2erase = it->second.feature_per_frame.begin();
                advance(it2erase,  idx - it->second.start_frame);
                it->second.feature_per_frame.erase(it2erase);

                if (it->second.feature_per_frame.empty()){
                    it = feature_.erase(it);
                    continue;
                }
            }

        }

        it++;
    }

    num_frame_--;

}

void FeatureManager::removeFront(){
    for (auto it = feature_.begin(); it != feature_.end();){
        if (it->second.start_frame > 0){
            it->second.start_frame--;
        }
        else{
            it->second.feature_per_frame.pop_front();
            if (it->second.feature_per_frame.empty()){
                it = feature_.erase(it);
                continue;
            }
        }
        it++;
    }

    num_frame_--;
}

void FeatureManager::removeSecondBack(){

    for (auto it = feature_.begin(); it != feature_.end(); ){
        if (it->second.start_frame == num_frame_ - 1){
            it->second.start_frame--;
        }
        else{
            if(it->second.endFrame() < num_frame_ - 1){

            }else{
                auto it2erase = it->second.feature_per_frame.begin();
                advance(it2erase,  num_frame_ - 2 - it->second.start_frame);
                it->second.feature_per_frame.erase(it2erase);

                if (it->second.feature_per_frame.empty()){
                    it = feature_.erase(it);
                    continue;
                }
            }

        }

        it++;
    }

    num_frame_--;

}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    // const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    // const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    const FeaturePerFrame &frame_i = *next(next(it_per_id.feature_per_frame.rbegin()));
    const FeaturePerFrame &frame_j = *next(it_per_id.feature_per_frame.rbegin());

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

}