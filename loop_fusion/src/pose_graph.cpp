/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "pose_graph.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>

PoseGraph::PoseGraph()
{
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;
    use_imu = 0;
    restart_index_ = -1;
}

PoseGraph::~PoseGraph()
{
	{
	    std::lock_guard<std::mutex> lock(m_distance_log_);
	    if (distance_log_.is_open())
	        distance_log_.close();
	}
    t_optimization.detach();
}

bool PoseGraph::initializeDistanceAuditLog(const std::string &path)
{
    {
        std::lock_guard<std::mutex> lock(m_distance_log_);
        distance_log_path_ = path;
        distance_log_.open(path, std::ios::out | std::ios::trunc);
        if (!distance_log_.is_open())
        {
            ROS_ERROR("[POSEGRAPH] Failed to open relative recovery audit log: %s", path.c_str());
            return false;
        }
        distance_log_ << "wall_time,ros_time,robot,event,mode,range_received_total,"
                         "factor_count,neighbor_count,temporal_span,self_motion,"
                         "normalized_rms,event_success,used_in_final_recovery,detail\n";
        distance_log_.flush();
    }
    ROS_INFO("[POSEGRAPH] Relative recovery audit log: %s", path.c_str());
    logDistanceAudit("LOGGER_INITIALIZED",
                     dist_config_.use_distance_recovery ? "range-enabled" : "range-disabled",
                     0, 0, 0.0, 0.0, 0.0, dist_config_.use_distance_recovery,
                     "used_in_final_recovery=1 proves range affected the stitched result");
    return true;
}

void PoseGraph::logDistanceAudit(const std::string &event, const std::string &mode,
                                 size_t factor_count, int neighbor_count,
                                 double temporal_span, double self_motion,
                                 double normalized_rms, bool accepted,
                                 const std::string &detail, bool throttle)
{
    std::lock_guard<std::mutex> lock(m_distance_log_);
    if (!distance_log_.is_open())
        return;
    const double wall_time = ros::WallTime::now().toSec();
    const std::string throttle_key = event + ":" + mode;
    if (throttle)
    {
        auto previous = distance_log_last_wall_time_.find(throttle_key);
        if (previous != distance_log_last_wall_time_.end() && wall_time - previous->second < 1.0)
            return;
        distance_log_last_wall_time_[throttle_key] = wall_time;
    }
    std::string safe_detail = detail;
    std::replace(safe_detail.begin(), safe_detail.end(), ',', ';');
    std::replace(safe_detail.begin(), safe_detail.end(), '\n', ' ');
    const bool used_in_final_recovery =
        event == "RECOVERY_COMPLETE" && accepted && mode.find("range") != std::string::npos;
    distance_log_ << std::fixed << std::setprecision(6)
                  << wall_time << ',' << ros::Time::now().toSec() << ','
                  << dist_config_.robot_name << ',' << event << ',' << mode << ','
                  << distance_received_total_.load() << ',' << factor_count << ','
                  << neighbor_count << ',' << temporal_span << ',' << self_motion << ','
                  << normalized_rms << ',' << (accepted ? 1 : 0) << ','
                  << (used_in_final_recovery ? 1 : 0) << ',' << safe_detail << '\n';
    distance_log_.flush();
}

void PoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 1000);
    pub_base_path = n.advertise<nav_msgs::Path>("base_path", 1000);
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);
    for (int i = 1; i < 10; i++)
        pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 1000);
    pub_recovery_stitch = n.advertise<visualization_msgs::MarkerArray>("recovery_stitch", 100);
    pub_recovery_cancel = n.advertise<std_msgs::Bool>("/failure_recovery/trigger", 10);
    pub_combined_path = n.advertise<nav_msgs::Path>("combined_path", 1000);
}

void PoseGraph::setIMUFlag(bool _use_imu)
{
    use_imu = _use_imu;
    if(use_imu)
    {
        printf("[POSEGRAPH]: VIO input, perfrom 4 DoF (x, y, z, yaw) pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    }
    else
    {
        printf("[POSEGRAPH]: VO input, perfrom 6 DoF pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize6DoF, this);
    }

}

void PoseGraph::setInitialAlignment(const Eigen::Vector3d &world_position,
                                    double yaw_degrees)
{
    initial_world_position_ = world_position;
    initial_world_rotation_ = Utility::ypr2R(Vector3d(yaw_degrees, 0.0, 0.0));
    initial_alignment_enabled_ = true;
    initial_alignment_pending_ = true;
    w_r_vio = initial_world_rotation_;
    // This is exact when VIO initializes at the origin and is refined against
    // the first keyframe below when its initial position is slightly nonzero.
    w_t_vio = initial_world_position_;
    ROS_INFO("[POSEGRAPH] Initial world alignment: p=(%.3f, %.3f, %.3f), yaw=%.3f deg",
             world_position.x(), world_position.y(), world_position.z(), yaw_degrees);
}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;
        sequence_loop.push_back(0);
        if (sequence_cnt == 1 && initial_alignment_enabled_)
        {
            w_t_vio = initial_world_position_;
            w_r_vio = initial_world_rotation_;
        }
        else
        {
            // A restarted VIO creates a new local frame. It must remain local
            // until visual/range recovery estimates its transform.
            w_t_vio = Eigen::Vector3d(0, 0, 0);
            w_r_vio = Eigen::Matrix3d::Identity();
        }
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    if (sequence_cnt == 1 && initial_alignment_pending_)
    {
        // A freshly initialized VIO frame has an arbitrary yaw gauge.  The
        // configured yaw describes the robot's physical initial heading, so
        // remove the first VIO yaw before applying that known world heading.
        const double initial_world_yaw = Utility::R2ypr(initial_world_rotation_).x();
        const double first_vio_yaw = Utility::R2ypr(vio_R_cur).x();
        const double world_from_vio_yaw =
            Utility::normalizeAngle(initial_world_yaw - first_vio_yaw);
        w_r_vio = Utility::ypr2R(Vector3d(world_from_vio_yaw, 0.0, 0.0));
        w_t_vio = initial_world_position_ - w_r_vio * vio_P_cur;
        initial_alignment_pending_ = false;
        ROS_INFO("[POSEGRAPH] Latched first VIO yaw %.3f deg; world-frame yaw offset %.3f deg",
                 first_vio_yaw, world_from_vio_yaw);
    }
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
    if (flag_detect_loop)
    {
        TicToc tmp_t;
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
	if (loop_index != -1)
	{
        //printf("[POSEGRAPH]:  %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);

        if ((cur_kf->has_loop && loop_index == old_kf->index) || cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t; 
            if(use_imu)
            {
                shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
                shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            }
            else
                shift_r = w_R_cur * vio_R_cur.transpose();
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; 
            // shift vio pose of whole sequence to the world frame
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {  
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio *  vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)   
                {
                    if((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio *  vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;

                // Visualize stitch point
                Vector3d old_P, cur_P_vis;
                Matrix3d old_R, cur_R_vis;
                old_kf->getPose(old_P, old_R);
                cur_kf->getPose(cur_P_vis, cur_R_vis);
                publishStitchMarkers(old_P, cur_P_vis);

                // Loop closure stitched a cross-sequence match — cancel failure recovery if active
                std_msgs::Bool cancel_msg;
                cancel_msg.data = false;
                pub_recovery_cancel.publish(cancel_msg);
                cancelDistanceRecovery("cross-sequence visual loop closure");
                ROS_INFO("[POSEGRAPH] Cross-sequence loop closure — recovery mode cancelled.");
            }
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
	}
	m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "global";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(0);
        loop_path_file << cur_kf->time_stamp * 1e9 << ",";
        loop_path_file.precision(5);
        loop_path_file  << P.x() << ","
              << P.y() << ","
              << P.z() << ","
              << Q.w() << ","
              << Q.x() << ","
              << Q.y() << ","
              << Q.z() << ","
              << endl;
        loop_path_file.close();
    }
    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                auto getSequenceColor = [](int seq) -> std_msgs::ColorRGBA {
                    std_msgs::ColorRGBA c;
                    c.a = 1.0;
                    switch(seq % 6) {
                        case 0: c.r = 1.0; c.g = 0.0; c.b = 0.0; break;
                        case 1: c.r = 0.0; c.g = 1.0; c.b = 0.0; break;
                        case 2: c.r = 0.0; c.g = 0.0; c.b = 1.0; break;
                        case 3: c.r = 1.0; c.g = 1.0; c.b = 0.0; break;
                        case 4: c.r = 0.0; c.g = 1.0; c.b = 1.0; break;
                        case 5: c.r = 1.0; c.g = 0.0; c.b = 1.0; break;
                    }
                    return c;
                };

                if (cur_kf->sequence > 1) {
                    (*rit)->getPose(conncected_P, connected_R);
                    posegraph_visualization->add_edge(P, conncected_P, getSequenceColor(cur_kf->sequence));
                }
            }
            rit++;
        }
    }
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)
        {
            //printf("[POSEGRAPH]: has loop \n");
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if(cur_kf->sequence > 0)
            {
                std_msgs::ColorRGBA loop_color;
                loop_color.r = 1.0; loop_color.g = 1.0; loop_color.b = 1.0; loop_color.a = 1.0;
                //printf("[POSEGRAPH]: add loop into visual \n");
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), loop_color);
            }
            
        }
    }
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);

	keyframelist.push_back(cur_kf);
    addSelfPoseSample(cur_kf, vio_P_cur, vio_R_cur);
    publish();
	m_keyframelist.unlock();

	// A visual recovery result can arrive before this first post-restart
	// keyframe. Apply the retained result only after the keyframe is visible to
	// updateRecoveryPose(), avoiding the former one-shot race.
	tryApplyPendingVisualRecovery();
}


void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    if (flag_detect_loop)
       loop_index = detectLoop(cur_kf, cur_kf->index);
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
    if (loop_index != -1)
    {
        printf("[POSEGRAPH]:  %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "global";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;

    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 1; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                auto getSequenceColor = [](int seq) -> std_msgs::ColorRGBA {
                    std_msgs::ColorRGBA c;
                    c.a = 1.0;
                    switch(seq % 6) {
                        case 0: c.r = 1.0; c.g = 0.0; c.b = 0.0; break;
                        case 1: c.r = 0.0; c.g = 1.0; c.b = 0.0; break;
                        case 2: c.r = 0.0; c.g = 0.0; c.b = 1.0; break;
                        case 3: c.r = 1.0; c.g = 1.0; c.b = 0.0; break;
                        case 4: c.r = 0.0; c.g = 1.0; c.b = 1.0; break;
                        case 5: c.r = 1.0; c.g = 0.0; c.b = 1.0; break;
                    }
                    return c;
                };
                
                if (cur_kf->sequence > 1) {
                    (*rit)->getPose(conncected_P, connected_R);
                    posegraph_visualization->add_edge(P, conncected_P, getSequenceColor(cur_kf->sequence));
                }
            }
            rit++;
        }
    }
    /*
    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P,  connected_R);
        posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
    }
    */

    keyframelist.push_back(cur_kf);
    //publish();
    m_keyframelist.unlock();
}

KeyFrame* PoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    int max_frame_id_allowed = std::max(0, frame_index - RECALL_IGNORE_RECENT_COUNT);
    db.query(keyframe->brief_descriptors, ret, 3, max_frame_id_allowed);
    //printf("[POSEGRAPH]: query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    //printf("[POSEGRAPH]: add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result 
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }

    //for (unsigned int i = 0; i < ret.size(); i++)
    //    cout << i << " - " <<  ret[i].Score << endl;

    // a good match with its nerghbour
    //if (ret.size() >= 1 && ret[0].Score > MIN_SCORE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > MIN_SCORE && ret[i].Id < max_frame_id_allowed)
            {
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }

    }
/*
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
    //if (find_loop && frame_index > 50)
    if (frame_index < 50)
        return -1;

    // Loop through all, and see if we have one that is a good match!
    std::vector<int> done_ids;
    while(done_ids.size() < ret.size())
    {

        // First find the oldest that we have not tried yet
        int min_index = INFINITY;
        bool has_min = false;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (ret[i].Id < min_index && ret[i].Id < max_frame_id_allowed && ret[i].Score > MIN_SCORE && std::find(done_ids.begin(),done_ids.end(),ret[i].Id)==done_ids.end())
            {
                min_index = ret[i].Id;
                has_min = true;
            }
        }

        // Break out if we have not found a min
        if(!has_min)
            return -1;

        // Then try to see if we can loop close with it
        KeyFrame* old_kf = getKeyFrame(min_index);
        if(keyframe->findConnection(old_kf)) return min_index;
        else done_ids.push_back(min_index);

    }

    // failure
    return -1;


}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

void PoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            //printf("[POSEGRAPH]: optimize pose graph \n");
            TicToc tmp_t1;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            options.max_solver_time_in_seconds = 5;
            options.max_num_iterations = 20;
            options.num_threads = 1;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {   
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
                                            t_array[i-j], 
                                            euler_array[i], 
                                            t_array[i]);
                  }
                }

                //add loop edge
                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
                                                                  t_array[connected_index], 
                                                                  euler_array[i], 
                                                                  t_array[i]);
                    
                }
                
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();
            double t_create = tmp_t1.toc();


            TicToc tmp_t2;
            ceres::Solve(options, &problem, &summary);
            double t_opt = tmp_t2.toc();
            //std::cout << summary.BriefReport() << "\n";

            //printf("[POSEGRAPH]: pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("[POSEGRAPH]: optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            TicToc tmp_t3;
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
            double t_update = tmp_t3.toc();

            // Nice debug print
            printf("[POSEGRAPH]: creation %.3f ms | optimization %.3f ms | update %.3f ms | %.3f dyaw, %.3f dpos\n", t_create, t_opt, t_update, yaw_drift, t_drift.norm());


        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}


void PoseGraph::optimize6DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            //printf("[POSEGRAPH]: optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            double q_array[max_length][4];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //ptions.minimizer_progress_to_stdout = true;
            options.max_solver_time_in_seconds = 5;
            options.max_num_iterations = 20;
            options.num_threads = 1;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i][0] = tmp_q.w();
                q_array[i][1] = tmp_q.x();
                q_array[i][2] = tmp_q.y();
                q_array[i][3] = tmp_q.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {   
                    problem.SetParameterBlockConstant(q_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                    if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                    {
                        Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                        Quaterniond q_i_j = Quaterniond(q_array[i-j][0], q_array[i-j][1], q_array[i-j][2], q_array[i-j][3]);
                        Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                        relative_t = q_i_j.inverse() * relative_t;
                        Quaterniond relative_q = q_i_j.inverse() * q_i;
                        ceres::CostFunction* vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                0.1, 0.01);
                        problem.AddResidualBlock(vo_function, NULL, q_array[i-j], t_array[i-j], q_array[i], t_array[i]);
                    }
                }

                //add loop edge
                
                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    Quaterniond relative_q;
                    relative_q = (*it)->getLoopRelativeQ();
                    ceres::CostFunction* loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);                    
                }
                
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
            
            //printf("[POSEGRAPH]: pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("[POSEGRAPH]: optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            r_drift = cur_r * vio_r.transpose();
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();

            // Nice debug print
            printf("[POSEGRAPH]: pose optimization in %.3f seconds | %.3f dori, %.3f dpos\n", tmp_t.toc(), Utility::R2ypr(r_drift).norm(), t_drift.norm());

        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}

void PoseGraph::updatePath()
{
    m_keyframelist.lock();
    list<KeyFrame*>::iterator it;
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    base_path.poses.clear();
    combined_path.poses.clear();
    combined_path.header.frame_id = "global";
    posegraph_visualization->reset();

    auto getSequenceColor = [](int seq) -> std_msgs::ColorRGBA {
        std_msgs::ColorRGBA c;
        c.a = 1.0;
        switch(seq % 6) {
            case 0: c.r = 1.0; c.g = 0.0; c.b = 0.0; break; // Red
            case 1: c.r = 0.0; c.g = 1.0; c.b = 0.0; break; // Green
            case 2: c.r = 0.0; c.g = 0.0; c.b = 1.0; break; // Blue
            case 3: c.r = 1.0; c.g = 1.0; c.b = 0.0; break; // Yellow
            case 4: c.r = 0.0; c.g = 1.0; c.b = 1.0; break; // Cyan
            case 5: c.r = 1.0; c.g = 0.0; c.b = 1.0; break; // Magenta
        }
        return c;
    };

    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
//        printf("[POSEGRAPH]: path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
        pose_stamped.header.frame_id = "global";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        if((*it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
        }
        // Always add to the combined (unified) path regardless of sequence
        combined_path.poses.push_back(pose_stamped);
        combined_path.header = pose_stamped.header;

        if (SAVE_LOOP_PATH)
        {
            ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(0);
            loop_path_file << (*it)->time_stamp * 1e9 << ",";
            loop_path_file.precision(5);
            loop_path_file  << P.x() << ","
                  << P.y() << ","
                  << P.z() << ","
                  << Q.w() << ","
                  << Q.x() << ","
                  << Q.y() << ","
                  << Q.z() << ","
                  << endl;
            loop_path_file.close();
        }
        //draw local connection
        if (SHOW_S_EDGE)
        {
            list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame*>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)  
            {  
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            if ((*it)->sequence > 1) {
                                (*lrit)->getPose(conncected_P, connected_R);
                                posegraph_visualization->add_edge(P, conncected_P, getSequenceColor((*it)->sequence));
                            }
                        }
                        lrit++;
                    }
                    break;
                }
            } 
        }
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {
                
                KeyFrame* connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (*it)->getPose(P, R);
                if((*it)->sequence > 0)
                {
                    std_msgs::ColorRGBA loop_color;
                    loop_color.r = 1.0; loop_color.g = 1.0; loop_color.b = 1.0; loop_color.a = 1.0;
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), loop_color);
                }
            }
        }

    }
    publish();
    m_keyframelist.unlock();
}


void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile;
    printf("[POSEGRAPH]: pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("[POSEGRAPH]: pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen (file_path.c_str(),"w");
    //fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;

        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n",(*it)->index, (*it)->time_stamp, 
                                    VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(), 
                                    PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(), 
                                    VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), 
                                    PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(), 
                                    (*it)->loop_index, 
                                    (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                                    (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                                    (int)(*it)->keypoints.size());

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y, 
                                                     (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);
    }
    fclose(pFile);

    printf("[POSEGRAPH]: save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}
void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("[POSEGRAPH]: lode pose graph from: %s \n", file_path.c_str());
    printf("[POSEGRAPH]: pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("[POSEGRAPH]: lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp, 
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                                    &PG_Tx, &PG_Ty, &PG_Tz, 
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, 
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF) 
    {
        /*
        printf("[POSEGRAPH]: I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp, 
                                    VIO_Tx, VIO_Ty, VIO_Tz, 
                                    PG_Tx, PG_Ty, PG_Tz, 
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz, 
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz, 
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3, 
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors   
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf("[POSEGRAPH]:  fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    fclose (pFile);
    printf("[POSEGRAPH]: load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void PoseGraph::publish()
{
    for (int i = 1; i <= sequence_cnt; i++)
    {
        //if (sequence_loop[i] == true || i == base_sequence)
        if (1 || i == base_sequence)
        {
            pub_pg_path.publish(path[i]);
            pub_path[i].publish(path[i]);
            posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        }
    }
    pub_base_path.publish(base_path);
    pub_combined_path.publish(combined_path);
    //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}

void PoseGraph::publishStitchMarkers(const Eigen::Vector3d &pre_P, const Eigen::Vector3d &post_P)
{
    visualization_msgs::MarkerArray markers;
    ros::Time now = ros::Time::now();

    // Thick cyan line between the two stitch points
    visualization_msgs::Marker line;
    line.header.frame_id = "global";
    line.header.stamp = now;
    line.ns = "recovery_stitch";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.08;
    line.color.r = 0.0f; line.color.g = 1.0f; line.color.b = 1.0f; line.color.a = 1.0f;
    geometry_msgs::Point p0, p1;
    p0.x = pre_P.x();  p0.y = pre_P.y();  p0.z = pre_P.z();
    p1.x = post_P.x(); p1.y = post_P.y(); p1.z = post_P.z();
    line.points.push_back(p0);
    line.points.push_back(p1);
    markers.markers.push_back(line);

    // Sphere at pre-restart stitch point (yellow)
    visualization_msgs::Marker sphere_pre;
    sphere_pre.header.frame_id = "global";
    sphere_pre.header.stamp = now;
    sphere_pre.ns = "recovery_stitch";
    sphere_pre.id = 1;
    sphere_pre.type = visualization_msgs::Marker::SPHERE;
    sphere_pre.action = visualization_msgs::Marker::ADD;
    sphere_pre.pose.position.x = pre_P.x();
    sphere_pre.pose.position.y = pre_P.y();
    sphere_pre.pose.position.z = pre_P.z();
    sphere_pre.pose.orientation.w = 1.0;
    sphere_pre.scale.x = sphere_pre.scale.y = sphere_pre.scale.z = 0.1;
    sphere_pre.color.r = 1.0f; sphere_pre.color.g = 1.0f; sphere_pre.color.b = 0.0f; sphere_pre.color.a = 1.0f;
    markers.markers.push_back(sphere_pre);

    // Sphere at post-restart stitch point (orange)
    visualization_msgs::Marker sphere_post;
    sphere_post.header.frame_id = "global";
    sphere_post.header.stamp = now;
    sphere_post.ns = "recovery_stitch";
    sphere_post.id = 2;
    sphere_post.type = visualization_msgs::Marker::SPHERE;
    sphere_post.action = visualization_msgs::Marker::ADD;
    sphere_post.pose.position.x = post_P.x();
    sphere_post.pose.position.y = post_P.y();
    sphere_post.pose.position.z = post_P.z();
    sphere_post.pose.orientation.w = 1.0;
    sphere_post.scale.x = sphere_post.scale.y = sphere_post.scale.z = 0.1;
    sphere_post.color.r = 1.0f; sphere_post.color.g = 0.5f; sphere_post.color.b = 0.0f; sphere_post.color.a = 1.0f;
    markers.markers.push_back(sphere_post);

    // Text label at midpoint
    Vector3d mid = (pre_P + post_P) * 0.5;
    visualization_msgs::Marker text;
    text.header.frame_id = "global";
    text.header.stamp = now;
    text.ns = "recovery_stitch";
    text.id = 3;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x = mid.x();
    text.pose.position.y = mid.y();
    text.pose.position.z = mid.z() + 0.4;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
    text.text = "RECOVERED";
    markers.markers.push_back(text);

    pub_recovery_stitch.publish(markers);
}

void PoseGraph::onRestart()
{
    std::lock_guard<std::mutex> solver_lock(m_recovery_solver_);
    bool anchor_valid = false;
    Vector3d anchor_position = Vector3d::Zero();
    Matrix3d anchor_rotation = Matrix3d::Identity();
    int boundary = -1;
    restart_index_.store(-1);
    clearPendingVisualRecovery();
    {
        std::lock_guard<std::mutex> keyframe_lock(m_keyframelist);
        anchor_valid = !keyframelist.empty();
        if (anchor_valid)
            keyframelist.back()->getPose(anchor_position, anchor_rotation);
        boundary = global_index;
    }
    {
        std::lock_guard<std::mutex> distance_lock(m_distance_);
        distance_constraints_.clear();
        last_distance_timestamp_by_neighbor_.clear();
        self_pose_history_.clear();
        neighbor_pose_history_.clear();
        first_post_restart_timestamp_ = -1.0;
        recovery_anchor_valid_ = anchor_valid;
        recovery_anchor_position_ = anchor_position;
        recovery_anchor_rotation_ = anchor_rotation;
    }
    restart_index_.store(boundary);
    recovery_data_generation_.fetch_add(1);
    logDistanceAudit("RECOVERY_STARTED", "none", 0, 0, 0.0, 0.0, 0.0,
                     false, "boundary=" + std::to_string(boundary));
    ROS_WARN("[POSEGRAPH] onRestart: recovery boundary set at keyframe index %d", boundary);
}

void PoseGraph::updateRecoveryPose(const Eigen::Vector3d &P_rec, const Eigen::Quaterniond &Q_rec, double timestamp)
{
    std::lock_guard<std::mutex> solver_lock(m_recovery_solver_);
    const int boundary = restart_index_.load();
    if (boundary < 0)
        return;
    if (!P_rec.allFinite() || !Q_rec.coeffs().allFinite() || Q_rec.norm() < 1e-6)
    {
        ROS_WARN("[POSEGRAPH] Ignoring invalid visual recovery pose.");
        return;
    }

    Vector3d vio_position;
    Matrix3d vio_rotation;
    bool sequence_already_stitched = false;
    double reference_time_difference = std::numeric_limits<double>::infinity();
    {
        std::lock_guard<std::mutex> keyframe_lock(m_keyframelist);
        KeyFrame* reference_keyframe = nullptr;
        for (auto &keyframe : keyframelist)
        {
            if (keyframe->index < boundary)
                continue;
            if (keyframe->sequence > 0 && keyframe->sequence < (int)sequence_loop.size() && sequence_loop[keyframe->sequence])
                sequence_already_stitched = true;
            const double difference = dist_config_.use_header_timestamps
                                          ? std::fabs(keyframe->time_stamp - timestamp)
                                          : -static_cast<double>(keyframe->index);
            if (difference < reference_time_difference)
            {
                reference_time_difference = difference;
                reference_keyframe = keyframe;
            }
        }
        if (reference_keyframe == nullptr)
        {
            std::lock_guard<std::mutex> pending_lock(m_pending_visual_recovery_);
            if (restart_index_.load() == boundary)
            {
                pending_visual_recovery_.valid = true;
                pending_visual_recovery_.position = P_rec;
                pending_visual_recovery_.orientation = Q_rec.normalized();
                pending_visual_recovery_.timestamp = timestamp;
                pending_visual_recovery_.boundary = boundary;
                logDistanceAudit("VISUAL_RECOVERY_DEFERRED", "visual", 0, 0,
                                 0.0, 0.0, 0.0, false,
                                 "waiting_for_post_restart_keyframe");
                ROS_WARN("[POSEGRAPH] Visual recovery retained until the first post-restart keyframe.");
            }
            return;
        }
        reference_keyframe->getVioPose(vio_position, vio_rotation);
    }

    clearPendingVisualRecovery();

    if (dist_config_.use_header_timestamps &&
        reference_time_difference > dist_config_.visual_sync_tolerance)
    {
        logDistanceAudit("VISUAL_RECOVERY_REJECTED", "visual", 0, 0,
                         reference_time_difference, 0.0, 0.0, false,
                         "image_keyframe_time_mismatch");
        ROS_WARN_THROTTLE(1.0,
                          "[POSEGRAPH] Visual recovery candidate is %.3fs from the closest post-restart keyframe (limit %.3fs); waiting for a fresh candidate.",
                          reference_time_difference, dist_config_.visual_sync_tolerance);
        return;
    }

    if (sequence_already_stitched)
    {
        cancelDistanceRecovery("post-restart sequence already stitched by visual loop closure");
        std_msgs::Bool message;
        message.data = false;
        pub_recovery_cancel.publish(message);
        return;
    }

    const double recovered_yaw = Utility::R2ypr(Q_rec.toRotationMatrix()).x();
    const double vio_yaw = Utility::R2ypr(vio_rotation).x();
    const double visual_yaw_drift = Utility::normalizeAngle(recovered_yaw - vio_yaw);
    const Matrix3d visual_rotation = Utility::ypr2R(Vector3d(visual_yaw_drift, 0.0, 0.0));
    const Vector3d visual_translation = P_rec - visual_rotation * vio_position;

    std::vector<AlignedRangeFactor> range_factors;
    int unique_neighbors = 0;
    double temporal_span = 0.0;
    double self_motion = 0.0;
    if (dist_config_.use_distance_recovery)
        collectAlignedRangeFactors(timestamp, range_factors, unique_neighbors, temporal_span, self_motion);
    logDistanceAudit("VISUAL_RECOVERY_CANDIDATE", "visual", range_factors.size(),
                     unique_neighbors, temporal_span, self_motion, 0.0, false,
                     range_factors.empty() ? "no_aligned_range_factors" : "range_factors_available");

    double solved_yaw = visual_yaw_drift;
    Vector3d solved_translation = visual_translation;
    double range_rms = 0.0;
    std::string source = "visual-only";
    if (!range_factors.empty())
    {
        bool range_solution_valid = solveRecoveryTransform(
            range_factors, true, P_rec, visual_yaw_drift, vio_position,
            visual_yaw_drift, visual_translation,
            solved_yaw, solved_translation, range_rms);
        double visual_position_deviation = std::numeric_limits<double>::infinity();
        double visual_yaw_deviation = std::numeric_limits<double>::infinity();
        if (range_solution_valid)
        {
            const Matrix3d solved_rotation = Utility::ypr2R(Vector3d(solved_yaw, 0.0, 0.0));
            visual_position_deviation =
                (solved_rotation * vio_position + solved_translation - P_rec).norm();
            visual_yaw_deviation = std::fabs(
                Utility::normalizeAngle(solved_yaw - visual_yaw_drift));
            range_solution_valid =
                visual_position_deviation <= dist_config_.max_restart_position_gap &&
                visual_yaw_deviation <= dist_config_.max_yaw_deviation;
        }
        if (range_solution_valid)
        {
            source = "visual+range";
            logDistanceAudit("RANGE_OPTIMIZATION_ACCEPTED", "visual+range",
                             range_factors.size(), unique_neighbors, temporal_span,
                             self_motion, range_rms, true,
                             "yaw=" + std::to_string(solved_yaw) +
                             ";tx=" + std::to_string(solved_translation.x()) +
                             ";ty=" + std::to_string(solved_translation.y()) +
                             ";tz=" + std::to_string(solved_translation.z()) +
                             ";visual_position_deviation=" + std::to_string(visual_position_deviation) +
                             ";visual_yaw_deviation=" + std::to_string(visual_yaw_deviation));
            ROS_INFO("[POSEGRAPH] Combined recovery accepted: %zu factors, %d neighbors, span %.2fs, range RMS %.2f sigma",
                     range_factors.size(), unique_neighbors, temporal_span, range_rms);
        }
        else
        {
            solved_yaw = visual_yaw_drift;
            solved_translation = visual_translation;
            logDistanceAudit("RANGE_OPTIMIZATION_REJECTED", "visual+range",
                             range_factors.size(), unique_neighbors, temporal_span,
                             self_motion, range_rms, false,
                             "fallback_to_visual;visual_position_deviation=" +
                             std::to_string(visual_position_deviation) +
                             ";visual_yaw_deviation=" + std::to_string(visual_yaw_deviation));
            ROS_WARN("[POSEGRAPH] Range fusion rejected; falling back to the visual recovery pose.");
        }
    }

    completeRecovery(solved_yaw, solved_translation, source);
}

void PoseGraph::tryApplyPendingVisualRecovery()
{
    PendingVisualRecovery pending;
    {
        std::lock_guard<std::mutex> lock(m_pending_visual_recovery_);
        if (!pending_visual_recovery_.valid ||
            pending_visual_recovery_.boundary != restart_index_.load())
            return;
        pending = pending_visual_recovery_;
        pending_visual_recovery_.valid = false;
    }
    ROS_INFO("[POSEGRAPH] Applying retained visual recovery to a post-restart keyframe.");
    updateRecoveryPose(pending.position, pending.orientation, pending.timestamp);
}

void PoseGraph::clearPendingVisualRecovery()
{
    std::lock_guard<std::mutex> lock(m_pending_visual_recovery_);
    pending_visual_recovery_ = PendingVisualRecovery();
}

// ─── Distance recovery: neighbor pose storage ─────────────────────────────────

void PoseGraph::addSelfPoseSample(const KeyFrame *keyframe, const Eigen::Vector3d &position,
                                  const Eigen::Matrix3d &rotation)
{
    const int boundary = restart_index_.load();
    if (!dist_config_.use_distance_recovery || boundary < 0 || keyframe->index < boundary)
        return;
    const double timestamp = dist_config_.use_header_timestamps ? keyframe->time_stamp : ros::Time::now().toSec();
    if (!std::isfinite(timestamp) || !position.allFinite() || !rotation.allFinite())
        return;

    std::lock_guard<std::mutex> lock(m_distance_);
    self_pose_history_.push_back({position, rotation, timestamp, keyframe->index});
    if (first_post_restart_timestamp_ < 0.0)
        first_post_restart_timestamp_ = timestamp;
    while (self_pose_history_.size() > 1000)
        self_pose_history_.pop_front();
    recovery_data_generation_.fetch_add(1);
}

void PoseGraph::cancelDistanceRecovery(const std::string &reason)
{
    const int old_boundary = restart_index_.exchange(-1);
    if (old_boundary < 0)
        return;
    {
        std::lock_guard<std::mutex> lock(m_distance_);
        distance_constraints_.clear();
        last_distance_timestamp_by_neighbor_.clear();
        self_pose_history_.clear();
        neighbor_pose_history_.clear();
        first_post_restart_timestamp_ = -1.0;
        recovery_anchor_valid_ = false;
    }
    clearPendingVisualRecovery();
    ROS_INFO("[POSEGRAPH] Recovery state cleared: %s", reason.c_str());
    logDistanceAudit("RECOVERY_CANCELLED", "none", 0, 0, 0.0, 0.0, 0.0,
                     false, reason);
}

void PoseGraph::updateNeighborPose(const std::string &name, const Eigen::Vector3d &position,
                                   double timestamp, const std::string &frame_id)
{
    if (!dist_config_.use_distance_recovery || !position.allFinite() || !std::isfinite(timestamp))
        return;
    auto normalized_frame = [](const std::string &frame) {
        return !frame.empty() && frame.front() == '/' ? frame.substr(1) : frame;
    };
    if (!dist_config_.global_frame.empty() && !frame_id.empty() &&
        normalized_frame(frame_id) != normalized_frame(dist_config_.global_frame))
    {
        ROS_WARN_THROTTLE(2.0, "[POSEGRAPH] Ignoring neighbor pose for %s: frame '%s' != recovery frame '%s'",
                          name.c_str(), frame_id.c_str(), dist_config_.global_frame.c_str());
        return;
    }

    std::lock_guard<std::mutex> lock(m_distance_);
    auto &history = neighbor_pose_history_[name];
    history.push_back({name, position, timestamp, frame_id});
    while (history.size() > 1000)
        history.pop_front();
    recovery_data_generation_.fetch_add(1);
}

void PoseGraph::addDistanceConstraint(const std::string &robot_a, const std::string &robot_b,
                                       double distance, double timestamp)
{
    if (!dist_config_.use_distance_recovery || !std::isfinite(distance) || !std::isfinite(timestamp) ||
        distance <= 0.0 || distance > dist_config_.max_distance)
        return;

    std::string other;
    if (robot_a == dist_config_.robot_name)
        other = robot_b;
    else if (robot_b == dist_config_.robot_name)
        other = robot_a;
    else
        return;
    if (other.empty() || other == dist_config_.robot_name)
        return;

    {
        std::lock_guard<std::mutex> lock(m_distance_);
        auto previous = last_distance_timestamp_by_neighbor_.find(other);
        if (previous != last_distance_timestamp_by_neighbor_.end() &&
            timestamp <= previous->second + dist_config_.sample_interval)
        {
            logDistanceAudit("RANGE_DUPLICATE_DROPPED", "input", 0, 0,
                             0.0, 0.0, 0.0, false,
                             "neighbor=" + other, true);
            return;
        }
        last_distance_timestamp_by_neighbor_[other] = timestamp;
        distance_constraints_.push_back({other, distance, timestamp});
        while (distance_constraints_.size() > 2000)
            distance_constraints_.pop_front();
    }
    distance_received_total_.fetch_add(1);
    recovery_data_generation_.fetch_add(1);
    logDistanceAudit("RANGE_RECEIVED", "input", 0, 0, 0.0, 0.0, 0.0,
                     true, "neighbor=" + other + ";distance=" + std::to_string(distance), true);
}

// ─── Distance-only recovery attempt ───────────────────────────────────────────

bool PoseGraph::collectAlignedRangeFactors(double reference_timestamp,
                                           std::vector<AlignedRangeFactor> &factors,
                                           int &unique_neighbors, double &temporal_span,
                                           double &self_motion)
{
    factors.clear();
    unique_neighbors = 0;
    temporal_span = 0.0;
    self_motion = 0.0;

    std::lock_guard<std::mutex> lock(m_distance_);
    if (self_pose_history_.empty() || distance_constraints_.empty())
        return false;
    if (!std::isfinite(reference_timestamp) || reference_timestamp <= 0.0)
        reference_timestamp = self_pose_history_.back().timestamp;

    auto closest_self = [this](double timestamp) -> const SelfPoseSample* {
        const SelfPoseSample* best = nullptr;
        double best_difference = std::numeric_limits<double>::infinity();
        for (const auto &sample : self_pose_history_)
        {
            const double difference = std::fabs(sample.timestamp - timestamp);
            if (difference < best_difference)
            {
                best = &sample;
                best_difference = difference;
            }
        }
        return best_difference <= dist_config_.sync_tolerance ? best : nullptr;
    };

    auto closest_neighbor = [this](const std::string &name, double timestamp) -> const NeighborPose* {
        auto history_it = neighbor_pose_history_.find(name);
        if (history_it == neighbor_pose_history_.end())
            return nullptr;
        const NeighborPose* best = nullptr;
        double best_difference = std::numeric_limits<double>::infinity();
        for (const auto &sample : history_it->second)
        {
            const double difference = std::fabs(sample.timestamp - timestamp);
            if (difference < best_difference)
            {
                best = &sample;
                best_difference = difference;
            }
        }
        return best_difference <= dist_config_.sync_tolerance ? best : nullptr;
    };

    std::map<std::string, double> last_factor_time;
    std::set<std::string> neighbor_set;
    double minimum_time = std::numeric_limits<double>::infinity();
    double maximum_time = -std::numeric_limits<double>::infinity();
    for (const auto &constraint : distance_constraints_)
    {
        if (std::fabs(constraint.timestamp - reference_timestamp) > dist_config_.max_constraint_age)
            continue;
        auto previous = last_factor_time.find(constraint.other_robot);
        if (previous != last_factor_time.end() &&
            std::fabs(constraint.timestamp - previous->second) < dist_config_.sample_interval)
            continue;

        const SelfPoseSample* self = closest_self(constraint.timestamp);
        const NeighborPose* neighbor = closest_neighbor(constraint.other_robot, constraint.timestamp);
        if (self == nullptr || neighbor == nullptr)
            continue;

        factors.push_back({constraint.other_robot, self->position, neighbor->position,
                           constraint.distance, constraint.timestamp});
        last_factor_time[constraint.other_robot] = constraint.timestamp;
        neighbor_set.insert(constraint.other_robot);
        minimum_time = std::min(minimum_time, constraint.timestamp);
        maximum_time = std::max(maximum_time, constraint.timestamp);
    }

    unique_neighbors = static_cast<int>(neighbor_set.size());
    if (!factors.empty())
    {
        temporal_span = maximum_time - minimum_time;
        const Vector3d first_position = factors.front().self_vio_position;
        for (const auto &factor : factors)
            self_motion = std::max(self_motion, (factor.self_vio_position - first_position).norm());
    }
    return !factors.empty();
}

bool PoseGraph::rangeGeometryObservable(const std::vector<AlignedRangeFactor> &factors,
                                        double yaw_degrees, const Eigen::Vector3d &translation,
                                        double &condition_number) const
{
    condition_number = std::numeric_limits<double>::infinity();
    if (factors.size() < 4)
        return false;

    const double yaw = yaw_degrees * M_PI / 180.0;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Matrix3d rotation;
    rotation << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
    const int parameter_count = dist_config_.planar_mode ? 3 : 4;
    Eigen::MatrixXd jacobian(factors.size(), parameter_count);
    for (size_t i = 0; i < factors.size(); ++i)
    {
        const Vector3d corrected = rotation * factors[i].self_vio_position + translation;
        const Vector3d delta = corrected - factors[i].neighbor_global_position;
        const double norm = delta.norm();
        if (norm < 1e-6)
            return false;
        const Vector3d unit = delta / norm;
        const Vector3d &position = factors[i].self_vio_position;
        const Vector3d position_yaw_derivative(-s * position.x() - c * position.y(),
                                                c * position.x() - s * position.y(), 0.0);
        jacobian(i, 0) = unit.dot(position_yaw_derivative);
        jacobian(i, 1) = unit.x();
        jacobian(i, 2) = unit.y();
        if (!dist_config_.planar_mode)
            jacobian(i, 3) = unit.z();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular_values = svd.singularValues();
    if (singular_values.size() < parameter_count ||
        singular_values(parameter_count - 1) < 1e-6)
        return false;
    condition_number = singular_values(0) / singular_values(parameter_count - 1);
    return std::isfinite(condition_number) && condition_number <= dist_config_.max_condition_number;
}

bool PoseGraph::solveRecoveryTransform(const std::vector<AlignedRangeFactor> &factors,
                                       bool use_visual, const Eigen::Vector3d &visual_position,
                                       double visual_yaw_drift, const Eigen::Vector3d &visual_vio_position,
                                       double initial_yaw, const Eigen::Vector3d &initial_translation,
                                       double &solved_yaw, Eigen::Vector3d &solved_translation,
                                       double &normalized_range_rms)
{
    double yaw_variable[1] = {initial_yaw};
    double translation_variable[3] = {initial_translation.x(), initial_translation.y(), initial_translation.z()};
    ceres::Problem problem;
    problem.AddParameterBlock(yaw_variable, 1, AngleLocalParameterization::Create());
    problem.AddParameterBlock(translation_variable, 3);
    if (dist_config_.planar_mode)
    {
        // Inter-robot ranges from coplanar UWB tags do not independently
        // observe vertical translation. Keep tz at the continuity/visual
        // initialization instead of allowing an artificial height change to
        // reduce the range residual.
        problem.SetParameterization(
            translation_variable,
            new ceres::SubsetParameterization(3, std::vector<int>{2}));
    }

    if (use_visual)
    {
        problem.AddResidualBlock(
            VisualRecoveryFactor::Create(visual_position, visual_vio_position,
                                         dist_config_.visual_position_sigma),
            nullptr, yaw_variable, translation_variable);
        problem.AddResidualBlock(
            YawPriorFactor::Create(visual_yaw_drift, dist_config_.visual_yaw_sigma),
            nullptr, yaw_variable);
    }
    else
    {
        // Range measurements alone can admit a second, globally incorrect
        // transform with a small residual. The last valid pre-failure pose and
        // the first post-restart pose select the physically continuous branch,
        // while ranges are still free to refine that estimate.
        problem.AddResidualBlock(
            VisualRecoveryFactor::Create(visual_position, visual_vio_position,
                                         dist_config_.range_only_position_prior_sigma),
            new ceres::HuberLoss(1.5), yaw_variable, translation_variable);
        problem.AddResidualBlock(
            YawPriorFactor::Create(visual_yaw_drift,
                                   dist_config_.range_only_yaw_prior_sigma),
            new ceres::HuberLoss(1.5), yaw_variable);
    }
    for (const auto &factor : factors)
    {
        problem.AddResidualBlock(
            DistanceFactor::Create(factor.distance, factor.neighbor_global_position,
                                   factor.self_vio_position, dist_config_.distance_sigma),
            new ceres::HuberLoss(1.5), yaw_variable, translation_variable);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 80;
    options.num_threads = 1;
    options.function_tolerance = 1e-8;
    options.gradient_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable() || !std::isfinite(yaw_variable[0]) ||
        !std::isfinite(translation_variable[0]) || !std::isfinite(translation_variable[1]) ||
        !std::isfinite(translation_variable[2]))
    {
        ROS_WARN("[POSEGRAPH] Recovery optimization failed: %s", summary.BriefReport().c_str());
        return false;
    }

    solved_yaw = Utility::normalizeAngle(yaw_variable[0]);
    solved_translation = Vector3d(translation_variable[0], translation_variable[1], translation_variable[2]);
    const Matrix3d solved_rotation = Utility::ypr2R(Vector3d(solved_yaw, 0.0, 0.0));
    double squared_error = 0.0;
    for (const auto &factor : factors)
    {
        const double predicted_distance =
            (solved_rotation * factor.self_vio_position + solved_translation - factor.neighbor_global_position).norm();
        const double normalized_error = (predicted_distance - factor.distance) / dist_config_.distance_sigma;
        squared_error += normalized_error * normalized_error;
    }
    normalized_range_rms = factors.empty() ? 0.0 : std::sqrt(squared_error / factors.size());
    if (!std::isfinite(normalized_range_rms) || normalized_range_rms > dist_config_.max_normalized_rms)
    {
        ROS_WARN_THROTTLE(1.0,
                          "[POSEGRAPH] Recovery range residual rejected: RMS %.2f sigma (limit %.2f)",
                          normalized_range_rms, dist_config_.max_normalized_rms);
        return false;
    }
    return true;
}

bool PoseGraph::completeRecovery(double solved_yaw, const Eigen::Vector3d &solved_translation,
                                 const std::string &source)
{
    const int boundary = restart_index_.load();
    if (boundary < 0)
        return false;

    const Matrix3d solved_rotation = Utility::ypr2R(Vector3d(solved_yaw, 0.0, 0.0));
    Vector3d stitch_pre_position = Vector3d::Zero();
    Matrix3d stitch_pre_rotation = Matrix3d::Identity();
    Vector3d stitch_post_position = Vector3d::Zero();
    bool found_pre = false;
    bool found_post = false;
    {
        std::lock_guard<std::mutex> keyframe_lock(m_keyframelist);
        if (restart_index_.load() != boundary)
            return false;
        for (auto &keyframe : keyframelist)
        {
            if (keyframe->index >= boundary)
            {
                found_post = true;
                break;
            }
        }
        if (!found_post)
            return false;
        found_post = false;
        std::set<int> recovered_sequences;
        for (auto &keyframe : keyframelist)
        {
            if (keyframe->index < boundary)
            {
                keyframe->getPose(stitch_pre_position, stitch_pre_rotation);
                found_pre = true;
                continue;
            }
            Vector3d position;
            Matrix3d rotation;
            keyframe->getVioPose(position, rotation);
            position = solved_rotation * position + solved_translation;
            rotation = solved_rotation * rotation;
            // Promote the recovered sequence itself into the global frame.
            // Leaving this correction only in r_drift would make a later
            // cross-sequence loop closure apply a second, duplicate shift.
            keyframe->updateVioPose(position, rotation);
            keyframe->updatePose(position, rotation);
            recovered_sequences.insert(keyframe->sequence);
            if (!found_post)
            {
                stitch_post_position = position;
                found_post = true;
            }
        }
        for (int recovered_sequence : recovered_sequences)
        {
            if (recovered_sequence > 0 && recovered_sequence < (int)sequence_loop.size())
                sequence_loop[recovered_sequence] = true;
        }
        w_r_vio = solved_rotation;
        w_t_vio = solved_translation;
        {
            std::lock_guard<std::mutex> drift_lock(m_drift);
            yaw_drift = 0.0;
            r_drift = Matrix3d::Identity();
            t_drift = Vector3d::Zero();
        }
        restart_index_.store(-1);
    }

    {
        std::lock_guard<std::mutex> distance_lock(m_distance_);
        distance_constraints_.clear();
        last_distance_timestamp_by_neighbor_.clear();
        self_pose_history_.clear();
        neighbor_pose_history_.clear();
        first_post_restart_timestamp_ = -1.0;
        recovery_anchor_valid_ = false;
    }
    clearPendingVisualRecovery();
    std_msgs::Bool message;
    message.data = false;
    pub_recovery_cancel.publish(message);
    if (found_pre && found_post)
        publishStitchMarkers(stitch_pre_position, stitch_post_position);
    {
        std::lock_guard<std::mutex> path_lock(m_path);
        updatePath();
    }
    ROS_WARN("[POSEGRAPH] %s recovery complete: yaw %.3f deg, translation (%.3f, %.3f, %.3f)",
             source.c_str(), solved_yaw, solved_translation.x(), solved_translation.y(), solved_translation.z());
    logDistanceAudit("RECOVERY_COMPLETE", source, 0, 0, 0.0, 0.0, 0.0,
                     found_post, found_post ? "trajectory_stitched" : "no_post_restart_keyframe");
    return found_post;
}

void PoseGraph::attemptDistanceRecovery()
{
    if (!dist_config_.use_distance_recovery || restart_index_.load() < 0)
        return;

    // The worker may overlap a visual callback. Never allow two recovery
    // transforms to be solved/applied concurrently, and do not re-solve when
    // no self/neighbor/range input changed since the previous worker tick.
    std::unique_lock<std::mutex> solver_lock(m_recovery_solver_, std::try_to_lock);
    if (!solver_lock.owns_lock())
        return;
    const uint64_t generation = recovery_data_generation_.load();
    if (generation == last_range_attempt_generation_.load())
        return;
    last_range_attempt_generation_.store(generation);

    double reference_timestamp = -1.0;
    double first_timestamp = -1.0;
    SelfPoseSample first_self;
    SelfPoseSample latest_self;
    bool anchor_valid = false;
    Vector3d anchor_position;
    Matrix3d anchor_rotation;
    {
        std::lock_guard<std::mutex> lock(m_distance_);
        if (self_pose_history_.empty())
            return;
        first_self = self_pose_history_.front();
        latest_self = self_pose_history_.back();
        reference_timestamp = latest_self.timestamp;
        first_timestamp = first_post_restart_timestamp_;
        anchor_valid = recovery_anchor_valid_;
        anchor_position = recovery_anchor_position_;
        anchor_rotation = recovery_anchor_rotation_;
    }
    if (!anchor_valid || first_timestamp < 0.0 ||
        reference_timestamp - first_timestamp < dist_config_.wait_for_visual)
        return;

    std::vector<AlignedRangeFactor> factors;
    int unique_neighbors = 0;
    double temporal_span = 0.0;
    double self_motion = 0.0;
    const bool has_factors = collectAlignedRangeFactors(reference_timestamp, factors, unique_neighbors,
                                                        temporal_span, self_motion);
    if (!has_factors || unique_neighbors < dist_config_.min_neighbors ||
        static_cast<int>(factors.size()) < dist_config_.min_factors ||
        temporal_span < dist_config_.min_temporal_span || self_motion < dist_config_.min_self_motion)
    {
        logDistanceAudit("RANGE_WAITING", "range-only", factors.size(), unique_neighbors,
                         temporal_span, self_motion, 0.0, false,
                         "insufficient_aligned_factors", true);
        ROS_WARN_THROTTLE(2.0,
                          "[POSEGRAPH] Range recovery waiting: factors %zu/%d, neighbors %d/%d, span %.2f/%.2fs, motion %.2f/%.2fm",
                          factors.size(), dist_config_.min_factors, unique_neighbors,
                          dist_config_.min_neighbors, temporal_span, dist_config_.min_temporal_span,
                          self_motion, dist_config_.min_self_motion);
        return;
    }

    const double anchor_yaw = Utility::R2ypr(anchor_rotation).x();
    const double first_vio_yaw = Utility::R2ypr(first_self.rotation).x();
    const double initial_yaw = Utility::normalizeAngle(anchor_yaw - first_vio_yaw);
    const Matrix3d initial_rotation = Utility::ypr2R(Vector3d(initial_yaw, 0.0, 0.0));
    const Vector3d initial_translation = anchor_position - initial_rotation * first_self.position;
    double condition_number = 0.0;
    if (!rangeGeometryObservable(factors, initial_yaw, initial_translation, condition_number))
    {
        logDistanceAudit("RANGE_GEOMETRY_REJECTED", "range-only", factors.size(),
                         unique_neighbors, temporal_span, self_motion, 0.0, false,
                         "condition=" + std::to_string(condition_number), true);
        ROS_WARN_THROTTLE(2.0, "[POSEGRAPH] Range recovery waiting for observable geometry (condition %.3g)",
                          condition_number);
        return;
    }

    double solved_yaw = initial_yaw;
    Vector3d solved_translation = initial_translation;
    double normalized_rms = 0.0;
    if (!solveRecoveryTransform(factors, false, anchor_position, initial_yaw, first_self.position,
                                initial_yaw, initial_translation,
                                solved_yaw, solved_translation, normalized_rms))
    {
        logDistanceAudit("RANGE_OPTIMIZATION_REJECTED", "range-only", factors.size(),
                         unique_neighbors, temporal_span, self_motion, normalized_rms,
                         false, "residual_or_solver_rejected", true);
        return;
    }

    double solved_condition = 0.0;
    if (!rangeGeometryObservable(factors, solved_yaw, solved_translation, solved_condition))
    {
        logDistanceAudit("RANGE_GEOMETRY_REJECTED", "range-only-solved", factors.size(),
                         unique_neighbors, temporal_span, self_motion, normalized_rms,
                         false, "condition=" + std::to_string(solved_condition), true);
        return;
    }
    const Matrix3d solved_rotation = Utility::ypr2R(Vector3d(solved_yaw, 0.0, 0.0));
    const Vector3d first_recovered = solved_rotation * first_self.position + solved_translation;
    const Vector3d latest_recovered = solved_rotation * latest_self.position + solved_translation;
    const double restart_position_gap = (first_recovered - anchor_position).norm();
    const double yaw_deviation = std::fabs(Utility::normalizeAngle(solved_yaw - initial_yaw));
    const double position_jump = (latest_recovered - anchor_position).norm();
    const std::string solution_detail =
        "condition=" + std::to_string(solved_condition) +
        ";initial_yaw=" + std::to_string(initial_yaw) +
        ";yaw=" + std::to_string(solved_yaw) +
        ";yaw_deviation=" + std::to_string(yaw_deviation) +
        ";tx=" + std::to_string(solved_translation.x()) +
        ";ty=" + std::to_string(solved_translation.y()) +
        ";tz=" + std::to_string(solved_translation.z()) +
        ";restart_position_gap=" + std::to_string(restart_position_gap) +
        ";latest_position_jump=" + std::to_string(position_jump);
    if (restart_position_gap > dist_config_.max_restart_position_gap ||
        yaw_deviation > dist_config_.max_yaw_deviation ||
        position_jump > dist_config_.max_position_jump)
    {
        logDistanceAudit("RANGE_CONTINUITY_REJECTED", "range-only", factors.size(),
                         unique_neighbors, temporal_span, self_motion, normalized_rms,
                         false, solution_detail, true);
        ROS_WARN("[POSEGRAPH] Range recovery rejected by continuity: first gap %.2f/%.2fm, yaw change %.1f/%.1fdeg, latest jump %.2f/%.2fm",
                 restart_position_gap, dist_config_.max_restart_position_gap,
                 yaw_deviation, dist_config_.max_yaw_deviation,
                 position_jump, dist_config_.max_position_jump);
        return;
    }

    ROS_WARN("[POSEGRAPH] Range-only fallback accepted: %zu factors, %d neighbors, span %.2fs, motion %.2fm, condition %.1f, RMS %.2f sigma",
             factors.size(), unique_neighbors, temporal_span, self_motion, solved_condition, normalized_rms);
    logDistanceAudit("RANGE_OPTIMIZATION_ACCEPTED", "range-only", factors.size(),
                     unique_neighbors, temporal_span, self_motion, normalized_rms,
                     true, solution_detail);
    completeRecovery(solved_yaw, solved_translation, "range-only fallback");
}
