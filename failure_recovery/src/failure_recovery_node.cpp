#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <mutex>
#include <queue>
#include <thread>
#include <map>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "keyframe.h"
#include "ThirdParty/DBoW/DBoW2.h"

using namespace std;
using namespace Eigen;

// Missing externs from libloop_fusion_lib
std::string POSE_GRAPH_SAVE_PATH = "";
std::string VINS_RESULT_PATH = "";
std::string BRIEF_PATTERN_FILE = "";
double MAX_THETA_DIFF = 180.0;
double MAX_POS_DIFF = 200.0;
int MIN_LOOP_NUM = 15;
double MIN_SCORE = 0.030;
double PNP_INFLATION = 10.0;
int RECALL_IGNORE_RECENT_COUNT = 50;
int VISUALIZATION_SHIFT_X = 0;
int VISUALIZATION_SHIFT_Y = 0;
int ROW = 480;
int COL = 752;
int DEBUG_IMAGE = 0;
double max_focallength = 460.0;
camodocal::CameraPtr m_camera;
Eigen::Vector3d tic(0,0,0);
Eigen::Matrix3d qic = Eigen::Matrix3d::Identity();
ros::Publisher pub_match_img;

// Mutexes and queues for synchronization
mutex m_buf;
mutex m_db;
queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;

// Database for PnP
BriefVocabulary* voc;
BriefDatabase db;
map<int, KeyFrame*> keyframe_db;
int global_index = 0;

ros::Publisher pub_recovered_pose;
bool recovery_active = false;
double last_recovery_time = 0.0;
mutex m_recovery;

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
    m_buf.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_buf.unlock();
}

void intrinsics_callback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    m_buf.lock();
    cv::Size imageSize(msg->width, msg->height);
    if(msg->distortion_model == "plumb_bob") {
        m_camera = camodocal::CameraFactory::instance()->generateCamera(camodocal::Camera::ModelType::PINHOLE, "cam0", imageSize);
        std::vector<double> parameters;
        parameters.push_back(msg->D.at(0));
        parameters.push_back(msg->D.at(1));
        parameters.push_back(msg->D.at(2));
        parameters.push_back(msg->D.at(3));
        parameters.push_back(msg->K.at(0));
        parameters.push_back(msg->K.at(4));
        parameters.push_back(msg->K.at(2));
        parameters.push_back(msg->K.at(5));
        m_camera.get()->readParameters(parameters);
        max_focallength = std::max(msg->K.at(0), msg->K.at(4));
    } else if(msg->distortion_model == "equidistant") {
        m_camera = camodocal::CameraFactory::instance()->generateCamera(camodocal::Camera::ModelType::KANNALA_BRANDT, "cam0", imageSize);
        std::vector<double> parameters;
        parameters.push_back(msg->D.at(0));
        parameters.push_back(msg->D.at(1));
        parameters.push_back(msg->D.at(2));
        parameters.push_back(msg->D.at(3));
        parameters.push_back(msg->K.at(0));
        parameters.push_back(msg->K.at(4));
        parameters.push_back(msg->K.at(2));
        parameters.push_back(msg->K.at(5));
        m_camera.get()->readParameters(parameters);
        max_focallength = std::max(msg->K.at(0), msg->K.at(4));
    }
    ROW = msg->height;
    COL = msg->width;
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg) {
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg) {
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void trigger_callback(const std_msgs::Bool::ConstPtr &msg) {
    lock_guard<mutex> lock(m_recovery);
    if (msg->data) {
        if (!recovery_active && ros::Time::now().toSec() - last_recovery_time > 2.0) {
            ROS_WARN("[FAILURE_RECOVERY] Recovery mode ON (DB size: %d)", global_index);
            recovery_active = true;
        }
    } else {
        recovery_active = false;
    }
}

void perform_recovery() {
    while (ros::ok()) {
        bool active = false;
        {
            lock_guard<mutex> lock(m_recovery);
            active = recovery_active;
        }

        if (!active) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Grab the absolute latest image from buffer
        sensor_msgs::ImageConstPtr latest_image = NULL;
        m_buf.lock();
        if (image_buf.empty()) {
            m_buf.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        while(image_buf.size() > 1) image_buf.pop();
        latest_image = image_buf.front();
        m_buf.unlock();

        // Convert image
        cv_bridge::CvImageConstPtr ptr;
        try {
            if (latest_image->encoding == "8UC1") {
                sensor_msgs::Image img = *latest_image;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            } else {
                ptr = cv_bridge::toCvCopy(latest_image, sensor_msgs::image_encodings::MONO8);
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            continue;
        }

        cv::Mat image = ptr->image.clone();
        
        // Create a dummy KeyFrame for the current live image
        Vector3d dummy_T(0, 0, 0);
        Matrix3d dummy_R = Matrix3d::Identity();
        vector<cv::Point3f> empty_3d;
        vector<cv::Point2f> empty_2d;
        vector<double> empty_id;
        
        KeyFrame* live_kf = new KeyFrame(latest_image->header.stamp.toSec(), -1, dummy_T, dummy_R, image,
                                        empty_3d, empty_2d, empty_2d, empty_id, 0);

        // Query Database
        m_db.lock();
        if(keyframe_db.empty()) {
            m_db.unlock();
            delete live_kf;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        DBoW2::QueryResults ret;
        db.query(live_kf->brief_descriptors, ret, 4, global_index - 1);
        m_db.unlock();

        if (!ret.empty()) {
            ROS_INFO_THROTTLE(2.0, "[FAILURE_RECOVERY] searching... best score %.3f", ret[0].Score);
        }

        bool recovered = false;
        for (size_t i = 0; i < ret.size(); i++) {
            if (ret[i].Score > MIN_SCORE) { 
                int match_index = ret[i].Id;
                m_db.lock();
                KeyFrame* old_kf = keyframe_db[match_index];
                m_db.unlock();
                
                if (old_kf->findConnection(live_kf)) {
                    ROS_WARN("[FAILURE_RECOVERY] Recovered! kf=%d score=%.3f", match_index, ret[i].Score);

                    // Retrieve old_kf pose and loop relative transform under lock
                    // findConnection stores: relative_t = R_old^T*(T_cur - T_old), relative_q = R_old^T*R_cur
                    m_db.lock();
                    Vector3d w_P_db;
                    Matrix3d w_R_db;
                    old_kf->getPose(w_P_db, w_R_db);
                    Vector3d dT = old_kf->getLoopRelativeT();
                    Quaterniond dQ = old_kf->getLoopRelativeQ();
                    m_db.unlock();

                    // T_cur = T_old + R_old * dT
                    // R_cur = R_old * dQ
                    Matrix3d w_R_cur = w_R_db * dQ.toRotationMatrix();
                    Vector3d w_P_cur = w_P_db + w_R_db * dT;
                    Quaterniond w_Q_cur(w_R_cur);

                    geometry_msgs::PoseWithCovarianceStamped recovery_msg;
                    recovery_msg.header.stamp = latest_image->header.stamp;
                    recovery_msg.header.frame_id = "global";
                    recovery_msg.pose.pose.position.x = w_P_cur.x();
                    recovery_msg.pose.pose.position.y = w_P_cur.y();
                    recovery_msg.pose.pose.position.z = w_P_cur.z();
                    recovery_msg.pose.pose.orientation.w = w_Q_cur.w();
                    recovery_msg.pose.pose.orientation.x = w_Q_cur.x();
                    recovery_msg.pose.pose.orientation.y = w_Q_cur.y();
                    recovery_msg.pose.pose.orientation.z = w_Q_cur.z();
                    
                    for(int d=0; d<36; d++) recovery_msg.pose.covariance[d] = 0.0;
                    recovery_msg.pose.covariance[0] = 0.01;
                    recovery_msg.pose.covariance[7] = 0.01;
                    recovery_msg.pose.covariance[14] = 0.01;
                    recovery_msg.pose.covariance[21] = 0.03;
                    recovery_msg.pose.covariance[28] = 0.03;
                    recovery_msg.pose.covariance[35] = 0.05;

                    pub_recovered_pose.publish(recovery_msg);
                    recovered = true;
                    {
                        lock_guard<mutex> lock(m_recovery);
                        recovery_active = false; // Stop after success
                        last_recovery_time = ros::Time::now().toSec();
                    }
                    break;
                }
            }
        }

        delete live_kf;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void process_db() {
    while (ros::ok()) {
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;

        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty()) {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec()) {
                pose_buf.pop();
            } else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec()) {
                point_buf.pop();
            } else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec()) {
                
                pose_msg = pose_buf.front(); pose_buf.pop();
                while (!pose_buf.empty()) pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec()) image_buf.pop();
                image_msg = image_buf.front(); image_buf.pop();
                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec()) point_buf.pop();
                point_msg = point_buf.front(); point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL) {
            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1") {
                sensor_msgs::Image img = *image_msg;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            } else {
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            }
            
            Vector3d T(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z).toRotationMatrix();
            
            vector<cv::Point3f> point_3d; 
            vector<cv::Point2f> point_2d_uv, point_2d_normal;
            vector<double> point_id;

            for (unsigned int i = 0; i < point_msg->points.size(); i++) {
                point_3d.emplace_back(point_msg->points[i].x, point_msg->points[i].y, point_msg->points[i].z);
                point_2d_normal.emplace_back(point_msg->channels[i].values[0], point_msg->channels[i].values[1]);
                point_2d_uv.emplace_back(point_msg->channels[i].values[2], point_msg->channels[i].values[3]);
                point_id.push_back(point_msg->channels[i].values[4]);
            }

            cv::Mat image = ptr->image.clone();
            KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), global_index, T, R, image,
                                              point_3d, point_2d_uv, point_2d_normal, point_id, 1);
            
            m_db.lock();
            keyframe_db[global_index] = keyframe;
            db.add(keyframe->brief_descriptors);
            global_index++;
            if (global_index % 50 == 0) {
                ROS_INFO("[FAILURE_RECOVERY] DB size: %d", global_index);
            }
            m_db.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "failure_recovery_node");
    ros::NodeHandle n("~");

    std::string pkg_path = ros::package::getPath("loop_fusion");
    std::string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";

    if (argc >= 2) {
        std::string config_file = argv[1];
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (fsSettings.isOpened()) {
            fsSettings["min_loop_feat_num"] >> MIN_LOOP_NUM;
            fsSettings["min_score"]         >> MIN_SCORE;
            fsSettings["pnp_inflation"]     >> PNP_INFLATION;
            fsSettings["max_theta_diff"]    >> MAX_THETA_DIFF;
            fsSettings["max_pos_diff"]      >> MAX_POS_DIFF;
            ROS_INFO("[FAILURE_RECOVERY] Config loaded: min_loop_feat_num=%d, min_score=%.3f", MIN_LOOP_NUM, MIN_SCORE);
        } else {
            ROS_WARN("[FAILURE_RECOVERY] Failed to open config file: %s, using defaults.", config_file.c_str());
        }
    }
    
    voc = new BriefVocabulary(vocabulary_file);
    db.setVocabulary(*voc, false, 0);

    ros::Subscriber sub_image = n.subscribe("/cam0/image_raw", 2000, image_callback);
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    ros::Subscriber sub_trigger = n.subscribe("/failure_recovery/trigger", 10, trigger_callback);
    ros::Subscriber sub_restart = n.subscribe("/restart", 10, trigger_callback);
    ros::Subscriber sub_vio_reset = n.subscribe("/ov_msckf/vio_reset", 10, trigger_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    ros::Subscriber sub_intrinsics = n.subscribe("/vins_estimator/intrinsics", 2000, intrinsics_callback);

    auto msg1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vins_estimator/intrinsics", n);
    intrinsics_callback(msg1);
    auto msg2 = ros::topic::waitForMessage<nav_msgs::Odometry>("/vins_estimator/extrinsic", n);
    extrinsic_callback(msg2);

    pub_recovered_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/failure_recovery/recovered_pose", 10);

    std::thread process_thread(process_db);
    std::thread recovery_thread(perform_recovery);

    ROS_INFO("[FAILURE_RECOVERY] Ready. DB building...");
    ros::spin();

    process_thread.join();
    recovery_thread.join();
    return 0;
}
