/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_tracker.h"
#include <Eigen/src/Core/Matrix.h>

namespace vins_multi{

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker(bool is_depth, bool is_stereo, int feature_max_cnt): depth{is_depth}, stereo{is_stereo}, max_cnt{feature_max_cnt}, track_num{feature_max_cnt}, track_percentage{1.0}
{
    n_id = 0;
    hasPrediction = false;

    n_pts.reserve(feature_max_cnt);
    predict_pts.reserve(feature_max_cnt);
    predict_pts_debug.reserve(feature_max_cnt);
    prev_pts.reserve(feature_max_cnt);
    cur_pts.reserve(feature_max_cnt);
    cur_right_pts.reserve(feature_max_cnt);
    prev_un_pts.reserve(feature_max_cnt);
    cur_un_pts.reserve(feature_max_cnt);
    cur_un_right_pts.reserve(feature_max_cnt);
    pts_velocity.reserve(feature_max_cnt);
    right_pts_velocity.reserve(feature_max_cnt);
    ids.reserve(feature_max_cnt);
    ids_right.reserve(feature_max_cnt);
    track_cnt.reserve(feature_max_cnt);
    pts_depth.reserve(feature_max_cnt);
}

void FeatureTracker::set_max_feature_num(int max_feature_num){
    max_cnt = max_feature_num;
}

void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

map<int, FeaturePerFrame> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;

    if (EQUALIZE)
    {
        cv::Mat img_tmp;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img_tmp);
        // ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
        spdlog::debug("CLAHE costs: {}ms", t_c.toc());
        cur_img = img_tmp;

        if(stereo && !_img1.empty())
        {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(_img1, rightImg);
        }
    }
    else{
        cur_img = _img;
    }

    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();

    if (prev_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        auto prev_pt_size = prev_pts.size();
        if(hasPrediction)
        {
            cur_pts = predict_pts;
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(15, 15), 1,
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(15, 15), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(15, 15), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(15, 15), 1,
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(15, 15), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts.size()); i++){
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        }
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        spdlog::debug("temporal optical flow costs: {}ms", t_o.toc());
        // printf("track cnt %d\n", (int)ids.size());

        track_num = static_cast<double>(cur_pts.size());
        track_percentage = track_num / static_cast<double>(prev_pt_size);
    }

    for (auto &n : track_cnt)
        n++;

    if (1)
    {
        rejectWithF();
        // ROS_DEBUG("set mask begins");
        spdlog::debug("set mask begins");
        TicToc t_m;
        setMask();
        // ROS_DEBUG("set mask costs %fms", t_m.toc());
        spdlog::debug("set mask costs {}ms", t_m.toc());

        // ROS_DEBUG("detect feature begins");
        spdlog::debug("detect feature begins");
        TicToc t_t;
        int n_max_cnt = max_cnt - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        // ROS_DEBUG("detect feature %d costs: %f ms",n_max_cnt, t_t.toc());
        spdlog::debug("detect feature {} costs: {} ms",n_max_cnt, t_t.toc());

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);

            // if(cur_pts.size() > 0.7 * max_cnt){
            //     break;
            // }
        }

        // return map<int,FeaturePerFrame>();
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);

    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo)
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            // printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(15, 15), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(15, 15), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    else if(!_img1.empty() && depth){
        // rejectDepth(_img1);
        setDepth(_img1);
    }

    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    // map<int, vector<pair<int, Eigen::VectorXd>>> featureFrame;
    FeaturePerFrame featurePt;
    map<int,FeaturePerFrame> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        // double x, y ,z;
        // x = cur_un_pts[i].x;
        // y = cur_un_pts[i].y;
        // z = 1;
        // double p_u, p_v;
        // p_u = cur_pts[i].x;
        // p_v = cur_pts[i].y;
        // int camera_id = 0;
        // double velocity_x, velocity_y;
        // velocity_x = pts_velocity[i].x;
        // velocity_y = pts_velocity[i].y;

        featurePt.point.x() = cur_un_pts[i].x;
        featurePt.point.y() = cur_un_pts[i].y;
        featurePt.point.z() = 1.0;

        featurePt.uv.x() = cur_pts[i].x;
        featurePt.uv.y() = cur_pts[i].y;

        featurePt.velocity.x() = pts_velocity[i].x;
        featurePt.velocity.y() = pts_velocity[i].y;

        featurePt.is_depth = featurePt.is_stereo = false;


        if(depth && pts_depth[i] > 0.0){
            // Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
            // xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, pts_depth[i];
            // featurePt.pt_data = xyz_uv_velocity_depth;
            featurePt.depth = pts_depth[i];
            featurePt.is_depth = true;
            // featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
        } else {
            featurePt.depth = -1.0;
            // Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            // xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            // featurePt.pt_data = xyz_uv_velocity;
            // featurePt.depth = false;
            // featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
        featureFrame[feature_id]=featurePt;
    }

    if (!_img1.empty() && stereo)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            // double x, y ,z;
            // x = cur_un_right_pts[i].x;
            // y = cur_un_right_pts[i].y;
            // z = 1;
            // double p_u, p_v;
            // p_u = cur_right_pts[i].x;
            // p_v = cur_right_pts[i].y;
            // int camera_id = 1;
            // double velocity_x, velocity_y;
            // velocity_x = right_pts_velocity[i].x;
            // velocity_y = right_pts_velocity[i].y;

            featureFrame[feature_id].pointRight.x() = cur_un_right_pts[i].x;
            featureFrame[feature_id].pointRight.y() = cur_un_right_pts[i].y;
            featureFrame[feature_id].pointRight.z() = 1.0;

            featureFrame[feature_id].uvRight.x() = cur_right_pts[i].x;
            featureFrame[feature_id].uvRight.y() = cur_right_pts[i].y;

            featureFrame[feature_id].velocityRight.x() = right_pts_velocity[i].x;
            featureFrame[feature_id].velocityRight.y() = right_pts_velocity[i].y;

            // Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            // xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            // featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            // featureFrame[feature_id].pt_data_right = xyz_uv_velocity;
            featureFrame[feature_id].is_stereo = true;

        }
    }

    // printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

#ifdef WITH_CUDA

map<int, FeaturePerFrame> FeatureTracker::trackImageGPU(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    row = _img.rows;
    col = _img.cols;
    cv::Mat rightImg = _img1;

    cv::cuda::GpuMat cur_gpu_img(_img);

    cv::cuda::GpuMat cur_gpu_short_img;
    cur_gpu_img.convertTo(cur_gpu_short_img, CV_16UC1);

    auto cur_pyr = buildImagePyramid(cur_gpu_short_img, 3);

    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();

    if (prev_pts.size() > 0 && cur_time > 0.0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        auto prev_pt_size = prev_pts.size();

        // gpu_mutex.lock();
        // GPU_MUTEX.lock();
        TicToc t_og;


        // TicToc gpu_mat_time;
        cv::cuda::GpuMat prev_gpu_pts(prev_pts);
        cv::cuda::GpuMat cur_gpu_pts(cur_pts);
        cv::cuda::GpuMat gpu_status;
        // cout<<"gpu mat construct time: "<<gpu_mat_time.toc()<<endl;

        if(hasPrediction)
        {
            cur_gpu_pts = cv::cuda::GpuMat(predict_pts);
            cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(15, 15), 3, 30, true);
            // d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);
            d_pyrLK_sparse->calc(prev_pyr, cur_pyr, prev_gpu_pts, cur_gpu_pts, gpu_status);

            cur_gpu_pts.download(cur_pts);
            gpu_status.download(status);

            // vector<cv::Point2f> tmp_cur_pts(cur_gpu_pts.cols);
            // cur_gpu_pts.download(tmp_cur_pts);
            // cur_pts = tmp_cur_pts;

            // vector<uchar> tmp_status(gpu_status.cols);
            // gpu_status.download(tmp_status);
            // status = tmp_status;

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
            {
                cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
                cv::Size(15, 15), 3, 30, false);
                d_pyrLK_sparse->calc(prev_pyr, cur_pyr, prev_gpu_pts, cur_gpu_pts, gpu_status);

                // vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
                // cur_gpu_pts.download(tmp1_cur_pts);
                // cur_pts = tmp1_cur_pts;

                // vector<uchar> tmp1_status(gpu_status.cols);
                // gpu_status.download(tmp1_status);
                // status = tmp1_status;
                cur_gpu_pts.download(cur_pts);
                gpu_status.download(status);
            }
        }
        else
        {
            // ROS_ERROR("GPU optical flow!");
            TicToc gpu_of_time;
            cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
            cv::Size(15, 15), 3, 30, false);
            // d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);
            d_pyrLK_sparse->calc(prev_pyr, cur_pyr, prev_gpu_pts, cur_gpu_pts, gpu_status);
            // cout<<"gpu of cal time: "<<gpu_of_time.toc()<<endl;

            // TicToc gpu_mat_download_time;
            cur_gpu_pts.download(cur_pts);
            gpu_status.download(status);
            // cout<<"gpu mat download time: "<<gpu_mat_download_time.toc()<<endl;


            // vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
            // cur_gpu_pts.download(tmp1_cur_pts);
            // cur_pts = tmp1_cur_pts;

            // vector<uchar> tmp1_status(gpu_status.cols);
            // gpu_status.download(tmp1_status);
            // status = tmp1_status;

            // d_pyrLK_sparse.release();
        }
        if(FLOW_BACK)
        {
            // TicToc rev_gpu_mat_time;
            cv::cuda::GpuMat reverse_gpu_status;
            cv::cuda::GpuMat reverse_gpu_pts = prev_gpu_pts;
            // cout<<"rev gpu mat construct time: "<<rev_gpu_mat_time.toc()<<endl;


            // ROS_ERROR("GPU optical flow!");
            TicToc gpu_of_back_time;
            cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
            cv::Size(15, 15), 3, 30, true);
            // d_pyrLK_sparse->calc(cur_gpu_img, prev_gpu_img, cur_gpu_pts, reverse_gpu_pts, reverse_gpu_status);
            d_pyrLK_sparse->calc(cur_pyr, prev_pyr, cur_gpu_pts, reverse_gpu_pts, reverse_gpu_status);
            // cout<<"gpu of back time: "<<gpu_of_back_time.toc()<<endl;


            // TicToc rev_gpu_mat_download_time;
            vector<cv::Point2f> reverse_pts(reverse_gpu_pts.cols);
            reverse_gpu_pts.download(reverse_pts);

            vector<uchar> reverse_status(reverse_gpu_status.cols);
            reverse_gpu_status.download(reverse_status);
            // cout<<"rev gpu mat download time: "<<rev_gpu_mat_download_time.toc()<<endl;


            // d_pyrLK_sparse.release();

            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        // gpu_mutex.unlock();
        // GPU_MUTEX.unlock();
        // printf("gpu temporal optical flow costs: %f ms\n",t_og.toc());

        for (int i = 0; i < int(cur_pts.size()); i++){
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;

            for(int j = i+1;  j < int(cur_pts.size()); j++ ){
                auto dist = cur_pts[i] - cur_pts[j];

                if(dist.x * dist.x + dist.y * dist.y < MIN_DIST * MIN_DIST){
                    if(track_cnt[i] < track_cnt[j]){
                        status[i] = 0;
                    }
                    else{
                        status[j] = 0;
                    }
                }
            }
        }
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());

        track_num = static_cast<double>(cur_pts.size());
        track_percentage = track_num / static_cast<double>(prev_pt_size);
    }

    prev_pyr = cur_pyr;

    for (auto &n : track_cnt)
        n++;

    if (1)
    {
        // rejectWithF();
        // ROS_DEBUG("set mask begins");
        TicToc t_m;
        // setMask();
        // ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = max_cnt;
        // int n_max_cnt = max_cnt - static_cast<int>(cur_pts.size());

        if (n_max_cnt > 0)
        {
            // if(mask.empty())
            //     cout << "mask is empty " << endl;
            // if (mask.type() != CV_8UC1)
            //     cout << "mask type wrong " << endl;

            // gpu_mutex.lock();
            // GPU_MUTEX.lock();
            TicToc t_g;
            cv::cuda::GpuMat d_prevPts;
            TicToc t_gg;
            // cv::cuda::GpuMat gpu_mask(mask);
            // printf("gpumat cost: %fms\n",t_gg.toc());
            // ROS_ERROR("GPU feature!");
            cv::Ptr<cv::cuda::CornersDetector> detector = cv::cuda::createGoodFeaturesToTrackDetector(cur_gpu_img.type(), n_max_cnt, 0.01, MIN_DIST);
            // cout << "new gpu points: "<< MAX_CNT - cur_pts.size()<<endl;
            // detector->detect(cur_gpu_img, d_prevPts, gpu_mask);

            // cout<<"create gpu feature detector time: "<<t_gg.toc()<<endl;

            t_gg.tic();

            detector->detect(cur_gpu_img, d_prevPts);

            // cout<<"gpu feature detect time: "<<t_gg.toc()<<endl;

            // cout<<"gpu feature time: "<<t_g.toc()<<endl;

            // std::cout << "d_prevPts size: "<< d_prevPts.size()<<std::endl;
            if(!d_prevPts.empty()){
                // TicToc feature_gpu_mat_download_time;
                n_pts = cv::Mat_<cv::Point2f>(cv::Mat(d_prevPts));
                // cout<<"feature_gpu_mat_download_time: "<<feature_gpu_mat_download_time.toc()<<endl;

            }
            else
                n_pts.clear();

            // gpu_mutex.unlock();
            // GPU_MUTEX.unlock();

            // detector.release();
            // sum_n += n_pts.size();
            // printf("total point from gpu: %d\n",sum_n);
            // printf("gpu good feature to track cost: %fms\n", t_g.toc());
        }
        else
            n_pts.clear();

        ROS_DEBUG("detect feature %d costs: %f ms",n_max_cnt, t_t.toc());

        unsigned int cur_pt_size = cur_pts.size();

        for (auto &p : n_pts)
        {
            if(cur_pts.size() >= max_cnt){
                break;
            }

            bool close_new_pt = false;
            for(auto &cur_p : cur_pts){
                auto dist = cur_p - p;

                if(dist.x * dist.x + dist.y * dist.y <= MIN_DIST * MIN_DIST){
                    close_new_pt = true;
                    break;
                }
            }

            if(!close_new_pt){
                cur_pts.emplace_back(p);
                ids.emplace_back(n_id++);
                track_cnt.emplace_back(1);
            }

            // if(cur_pts.size() > 0.7 * max_cnt){
            //     break;
            // }
        }

        // return map<int,FeaturePerFrame>();
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);

    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo)
    {
        // ids_right.clear();
        // cur_right_pts.clear();
        // cur_un_right_pts.clear();
        // right_pts_velocity.clear();
        // cur_un_right_pts_map.clear();
        // if(!cur_pts.empty())
        // {
        //     //printf("stereo image; track feature on right image\n");
        //     vector<cv::Point2f> reverseLeftPts;
        //     vector<uchar> status, statusRightLeft;
        //     vector<float> err;
        //     // cur left ---- cur right
        //     cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(15, 15), 3);
        //     // reverse check cur right ---- cur left
        //     if(FLOW_BACK)
        //     {
        //         cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(15, 15), 3);
        //         for(size_t i = 0; i < status.size(); i++)
        //         {
        //             if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
        //                 status[i] = 1;
        //             else
        //                 status[i] = 0;
        //         }
        //     }

        //     ids_right = ids;
        //     reduceVector(cur_right_pts, status);
        //     reduceVector(ids_right, status);
        //     // only keep left-right pts
        //     /*
        //     reduceVector(cur_pts, status);
        //     reduceVector(ids, status);
        //     reduceVector(track_cnt, status);
        //     reduceVector(cur_un_pts, status);
        //     reduceVector(pts_velocity, status);
        //     */
        //     cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
        //     right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        // }
        // prev_un_right_pts_map = cur_un_right_pts_map;
    }
    else if(!_img1.empty() && depth){
        // rejectDepth(_img1);
        setDepth(_img1);
    }

    if(SHOW_TRACK)
        drawTrack(_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    // prev_img = _img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    // map<int, vector<pair<int, Eigen::VectorXd>>> featureFrame;
    FeaturePerFrame featurePt;
    map<int,FeaturePerFrame> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        // double x, y ,z;
        // x = cur_un_pts[i].x;
        // y = cur_un_pts[i].y;
        // z = 1;
        // double p_u, p_v;
        // p_u = cur_pts[i].x;
        // p_v = cur_pts[i].y;
        // int camera_id = 0;
        // double velocity_x, velocity_y;
        // velocity_x = pts_velocity[i].x;
        // velocity_y = pts_velocity[i].y;

        featurePt.point.x() = cur_un_pts[i].x;
        featurePt.point.y() = cur_un_pts[i].y;
        featurePt.point.z() = 1.0;

        featurePt.uv.x() = cur_pts[i].x;
        featurePt.uv.y() = cur_pts[i].y;

        featurePt.velocity.x() = pts_velocity[i].x;
        featurePt.velocity.y() = pts_velocity[i].y;

        featurePt.is_depth = featurePt.is_stereo = false;


        if(depth && pts_depth[i] > 0.0){
            // Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
            // xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, pts_depth[i];
            // featurePt.pt_data = xyz_uv_velocity_depth;
            featurePt.depth = pts_depth[i];
            featurePt.is_depth = true;
            // featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
        }
        else{
            featurePt.depth = -1.0;
            // Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            // xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            // featurePt.pt_data = xyz_uv_velocity;
            // featurePt.depth = false;
            // featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
        featureFrame[feature_id]=featurePt;
    }

    if (!_img1.empty() && stereo)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            // double x, y ,z;
            // x = cur_un_right_pts[i].x;
            // y = cur_un_right_pts[i].y;
            // z = 1;
            // double p_u, p_v;
            // p_u = cur_right_pts[i].x;
            // p_v = cur_right_pts[i].y;
            // int camera_id = 1;
            // double velocity_x, velocity_y;
            // velocity_x = right_pts_velocity[i].x;
            // velocity_y = right_pts_velocity[i].y;

            featureFrame[feature_id].pointRight.x() = cur_un_right_pts[i].x;
            featureFrame[feature_id].pointRight.y() = cur_un_right_pts[i].y;
            featureFrame[feature_id].pointRight.z() = 1.0;

            featureFrame[feature_id].uvRight.x() = cur_right_pts[i].x;
            featureFrame[feature_id].uvRight.y() = cur_right_pts[i].y;

            featureFrame[feature_id].velocityRight.x() = right_pts_velocity[i].x;
            featureFrame[feature_id].velocityRight.y() = right_pts_velocity[i].y;

            // Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            // xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            // featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            // featureFrame[feature_id].pt_data_right = xyz_uv_velocity;
            featureFrame[feature_id].is_stereo = true;

        }
    }

    printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

std::vector<cv::cuda::GpuMat> FeatureTracker::buildImagePyramid(const cv::cuda::GpuMat& img, int maxLevel_) {
    std::vector<cv::cuda::GpuMat> pyr;
    pyr.resize(maxLevel_ + 1);

    int cn = img.channels();

    CV_Assert(cn == 1 || cn == 3 || cn == 4);

    pyr[0] = img;
    for (int level = 1; level <= maxLevel_; ++level) {
        cv::cuda::pyrDown(pyr[level - 1], pyr[level]);
    }

    return pyr;
}

#endif

void FeatureTracker::rejectWithF()
  {
    if (cur_pts.size() >= 8)
    {
        // ROS_DEBUG("FM ransac begins");
        spdlog::debug("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        spdlog::debug("FM ransac: {} -> {}: {}", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        // ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
        spdlog::debug("FM ransac costs: {}ms", t_f.toc());
    }
}

void FeatureTracker::rejectDepth(const cv::Mat &depth_img){

    // ROS_WARN("rej depth");
    // ROS_WARN("cur_pts size init: %d", cur_pts.size());

    // ros::Time t0 = ros::Time::now();

    std::vector<uchar> valid_status(cur_pts.size(), 0);
    for (unsigned int i = 0; i < cur_pts.size(); i++){
        double dep = depth_img.at<uint16_t>(cur_pts[i].y, cur_pts[i].x) * 0.001;
        valid_status[i] = (dep < DEPTH_MAX && dep > DEPTH_MIN) ? 1 : 0;
    }
    // std::multimap<double, unsigned int> cur_pts_idx_sorted;
    // unsigned int num_valid_depth = 0.0;
    // double cur_pts_lap = std::numeric_limits<double>::max();

    // for (unsigned int i = 0; i < cur_pts.size(); cur_pts_idx_sorted.emplace(cur_pts_lap, i), i++)
    // {
    //     float depth[5];
    //     cur_pts_lap = std::numeric_limits<double>::max();

    //     if(cur_pts[i].y < 1.0f || cur_pts[i].x < 1.0f || cur_pts[i].y > depth_img.rows - 2.0f || cur_pts[i].x > depth_img.cols - 2.0f){
    //         continue;
    //     }
    //     depth[0] = depth_img.at<uint16_t>(cur_pts[i].y, cur_pts[i].x) * 0.001;
    //     depth[1] = depth_img.at<uint16_t>(cur_pts[i].y-1, cur_pts[i].x) * 0.001;
    //     depth[2] = depth_img.at<uint16_t>(cur_pts[i].y, cur_pts[i].x-1) * 0.001;
    //     depth[3] = depth_img.at<uint16_t>(cur_pts[i].y, cur_pts[i].x+1) * 0.001;
    //     depth[4] = depth_img.at<uint16_t>(cur_pts[i].y+1, cur_pts[i].x) * 0.001;

    //     bool depth_valid = true;
    //     for(int j = 0; j < 5; j++){
    //         if(depth[j] < DEPTH_MIN || depth[j] > DEPTH_MAX){
    //             depth_valid = false;
    //             break;
    //         }
    //     }


    //     if(depth_valid){
    //         cur_pts_lap = fabs(depth[1] + depth[2] + depth[3] + depth[4] - 4.0 * depth[0]);
    //         num_valid_depth++;
    //     }
    // }

    // int num_depth_use = min(min(30, MAX_CNT/2), static_cast<int>(cur_pts.size()));
    // int num_feature_use = min(max(30, MAX_CNT), static_cast<int>(cur_pts.size()));

    // for(int i = 0; i < num_feature_use; i++){
    //     valid_status[i] = 1;
    // }

    // auto it = cur_pts_idx_sorted.begin();

    // int depth_valid_cnt = 0;
    // for(int i = 0; i < num_depth_use; advance(it, 1), i++){
    //     valid_status[it->second] |= 1;
    //     depth_valid_cnt += valid_status[it->second];
    // }

    // ROS_WARN("depth cnt: %d", depth_valid_cnt);

    reduceVector(cur_pts, valid_status);
    reduceVector(ids, valid_status);
    reduceVector(track_cnt, valid_status);
    reduceVector(cur_un_pts, valid_status);
    reduceVector(pts_velocity, valid_status);
}

void FeatureTracker::setDepth(const cv::Mat &depth_img){

    pts_depth.resize(cur_pts.size());
    for (unsigned int i = 0; i < cur_pts.size(); i++){
        double dep = depth_img.at<uint16_t>(cur_pts[i].y, cur_pts[i].x) * 0.001;
        pts_depth[i] = (dep < DEPTH_MAX && dep > DEPTH_MIN) ? dep : -1.0;
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<std::string> &calib_file)
{

    // ROS_INFO("reading paramerter of camera %s", calib_file[0].c_str());
    spdlog::info("reading paramerter of camera {}", calib_file[0].c_str());
    camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[0]);
    m_camera.push_back(camera);

    if(stereo){
        // ROS_INFO("reading paramerter of camera %s", calib_file[1].c_str());
        spdlog::info("reading paramerter of camera {}", calib_file[1].c_str());
        camodocal::CameraPtr camera_1 = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[1]);
        m_camera.push_back(camera_1);
    }
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.emplace_back(b.x() / b.z(), b.y() / b.z());
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    mean_optical_flow_speed = 0.0;
    int optical_flow_pt_cnt = 0;

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;

        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.emplace_back(v_x, v_y);

                mean_optical_flow_speed += sqrt(v_x * v_x + v_y * v_y);
                optical_flow_pt_cnt++;
            }
            else
                pts_velocity.emplace_back(0, 0);

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.emplace_back(0, 0);
        }
    }

    if(optical_flow_pt_cnt > 0){
        mean_optical_flow_speed /= optical_flow_pt_cnt;
    }
    else{
        mean_optical_flow_speed = std::numeric_limits<double>::max();
    }

    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts,
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo)
        cv::hconcat(imLeft, imRight, imTrack);
    else{
        if(USE_GPU){

#ifdef WITH_CUDA
            cv::cuda::GpuMat gray_img, bgr_img;
            gray_img.upload(imLeft);
            cv::cuda::cvtColor(gray_img, bgr_img, cv::COLOR_GRAY2BGR);
            bgr_img.download(imTrack);

#endif

        }
        else{
            imTrack = imLeft.clone();
        }
    }
    cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2BGR);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }

    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.emplace_back(0);
        else
            status.emplace_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat& FeatureTracker::getTrackImage()
{
    return imTrack;
}

}