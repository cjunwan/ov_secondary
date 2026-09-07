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

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <loop_fusion/RobotPairDistances.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#include <cerrno>
#include <cstdlib>
#include <sys/stat.h>
#define SKIP_FIRST_CNT 2
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;
double MIN_SCORE = 0.015;
double PNP_INFLATION = 1.0;
int RECALL_IGNORE_RECENT_COUNT = 50;
double MAX_THETA_DIFF = 30.0;
double MAX_POS_DIFF = 20.0;
int MIN_LOOP_NUM = 20;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;

camodocal::CameraPtr m_camera;
double max_focallength = 460.0;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;
ros::Publisher pub_pose_rect;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;

bool ensure_directory(const std::string &path)
{
    if (path.empty()) return false;
    std::string normalized = path;
    if (normalized.back() != '/') normalized.push_back('/');
    for (size_t position = 1; position < normalized.size(); ++position)
    {
        if (normalized[position] != '/') continue;
        const std::string directory = normalized.substr(0, position);
        if (!directory.empty() && ::mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST)
        {
            ROS_ERROR("[POSEGRAPH] Failed to create directory %s: errno %d",
                      directory.c_str(), errno);
            return false;
        }
    }
    return true;
}

void new_sequence()
{
    printf("[POSEGRAPH]: new sequence\n");
    sequence++;
    printf("[POSEGRAPH]: sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    //ROS_INFO("image_callback!");
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf("[POSEGRAPH]:  image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("[POSEGRAPH]: %d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);
}

// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_margin_cloud.publish(point_cloud);
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("[POSEGRAPH]: pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "global";
    odometry.pose.pose.position.x = vio_t.x();
    odometry.pose.pose.position.y = vio_t.y();
    odometry.pose.pose.position.z = vio_t.z();
    odometry.pose.pose.orientation.x = vio_q.x();
    odometry.pose.pose.orientation.y = vio_q.y();
    odometry.pose.pose.orientation.z = vio_q.z();
    odometry.pose.pose.orientation.w = vio_q.w();
    odometry.twist = pose_msg->twist;
    odometry.pose.covariance = pose_msg->pose.covariance;
    pub_odometry_rect.publish(odometry);

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    cameraposevisual.reset();
    cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);


}


void vio_callback_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    geometry_msgs::PoseWithCovarianceStamped odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "global";
    odometry.pose.pose.position.x = vio_t.x();
    odometry.pose.pose.position.y = vio_t.y();
    odometry.pose.pose.position.z = vio_t.z();
    odometry.pose.pose.orientation.x = vio_q.x();
    odometry.pose.pose.orientation.y = vio_q.y();
    odometry.pose.pose.orientation.z = vio_q.z();
    odometry.pose.pose.orientation.w = vio_q.w();
    odometry.pose.covariance = pose_msg->pose.covariance;
    pub_pose_rect.publish(odometry);

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    cameraposevisual.reset();
    cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);


}

void recovery_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    Vector3d P_rec(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond Q_rec(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
    double timestamp = posegraph.dist_config_.use_header_timestamps && !pose_msg->header.stamp.isZero()
                           ? pose_msg->header.stamp.toSec()
                           : ros::Time::now().toSec();
    posegraph.updateRecoveryPose(P_rec, Q_rec, timestamp);
}

void restart_callback(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
    {
        static ros::WallTime last_restart;
        const ros::WallTime now = ros::WallTime::now();
        if (!last_restart.isZero() && (now - last_restart).toSec() < 1.0)
        {
            ROS_WARN("[POSEGRAPH] Ignoring duplicate reset signal received within 1 second.");
            return;
        }
        last_restart = now;
        ROS_WARN("[POSEGRAPH] VIO reset received — starting new sequence for recovery.");
        new_sequence();
        posegraph.onRestart();
    }
}

// ─── Distance recovery callbacks ──────────────────────────────────────────────

void distance_callback(const loop_fusion::RobotPairDistances::ConstPtr &msg)
{
    if (msg->robot_a_names.size() != msg->robot_b_names.size() ||
        msg->robot_a_names.size() != msg->distances.size())
    {
        ROS_ERROR_THROTTLE(1.0, "[POSEGRAPH] Malformed RobotPairDistances: a=%zu b=%zu d=%zu",
                           msg->robot_a_names.size(), msg->robot_b_names.size(), msg->distances.size());
        return;
    }
    double ts = posegraph.dist_config_.use_header_timestamps && !msg->header.stamp.isZero()
                    ? msg->header.stamp.toSec()
                    : ros::Time::now().toSec();
    for (size_t k = 0; k < msg->distances.size(); k++)
    {
        posegraph.addDistanceConstraint(msg->robot_a_names[k], msg->robot_b_names[k],
                                         msg->distances[k], ts);
    }
}

void neighbor_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg,
                             const std::string &robot_name)
{
    Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    const double timestamp = posegraph.dist_config_.use_header_timestamps && !msg->header.stamp.isZero()
                                 ? msg->header.stamp.toSec()
                                 : ros::Time::now().toSec();
    posegraph.updateNeighborPose(robot_name, pos, timestamp, msg->header.frame_id);
}

void neighbor_pose_cov_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                                 const std::string &robot_name)
{
    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const double timestamp = posegraph.dist_config_.use_header_timestamps && !msg->header.stamp.isZero()
                                 ? msg->header.stamp.toSec()
                                 : ros::Time::now().toSec();
    posegraph.updateNeighborPose(robot_name, pos, timestamp, msg->header.frame_id);
}

void neighbor_odom_callback(const nav_msgs::Odometry::ConstPtr &msg,
                            const std::string &robot_name)
{
    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const double timestamp = posegraph.dist_config_.use_header_timestamps && !msg->header.stamp.isZero()
                                 ? msg->header.stamp.toSec()
                                 : ros::Time::now().toSec();
    posegraph.updateNeighborPose(robot_name, pos, timestamp, msg->header.frame_id);
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}


void intrinsics_callback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    m_process.lock();
    assert(msg->K.size()==9);
    assert(msg->D.size()==4);
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
    } else {
        throw std::runtime_error("Invalid distorition model, unable to parse (plumb_bob, equidistant)");
    }
    m_process.unlock();
}

void process()
{
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("[POSEGRAPH]: throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("[POSEGRAPH]: throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf("[POSEGRAPH]:  pose time %f \n", pose_msg->header.stamp.toSec());
            //printf("[POSEGRAPH]:  point time %f \n", point_msg->header.stamp.toSec());
            //printf("[POSEGRAPH]:  image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;
            //cv::equalizeHist(image, image);

            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("[POSEGRAPH]: u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("[POSEGRAPH]: save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("[POSEGRAPH]: program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_fusion");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);
    
    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

    if(argc != 2)
    {
        printf("[POSEGRAPH]: please intput: rosrun loop_fusion loop_fusion_node [config file] \n"
               "for example: rosrun loop_fusion loop_fusion_node "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }
    
    string config_file = argv[1];
    printf("[POSEGRAPH]: config_file: %s\n", argv[1]);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);


    std::string pkg_path = ros::package::getPath("loop_fusion");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);
    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;


    //ROW = fsSettings["image_height"];
    //COL = fsSettings["image_width"];
    //int pn = config_file.find_last_of('/');
    //std::string configPath = config_file.substr(0, pn);
    //std::string cam0Calib;
    //fsSettings["cam0_calib"] >> cam0Calib;
    //std::string cam0Path = configPath + "/" + cam0Calib;
    //printf("[POSEGRAPH]: cam calib path: %s\n", cam0Path.c_str());
    //m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());


    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    if (!ensure_directory(VINS_RESULT_PATH) || !ensure_directory(POSE_GRAPH_SAVE_PATH))
        return 1;
    fsSettings["save_image"] >> DEBUG_IMAGE;
    fsSettings["skip_dist"] >> SKIP_DIS;
    fsSettings["skip_cnt"] >> SKIP_CNT;
    fsSettings["min_score"] >> MIN_SCORE;
    fsSettings["pnp_inflation"] >> PNP_INFLATION;
    fsSettings["recall_ignore_recent_ct"] >> RECALL_IGNORE_RECENT_COUNT;
    fsSettings["max_theta_diff"] >> MAX_THETA_DIFF;
    fsSettings["max_pos_diff"] >> MAX_POS_DIFF;
    fsSettings["min_loop_feat_num"] >> MIN_LOOP_NUM;

    int LOAD_PREVIOUS_POSE_GRAPH;
    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.csv";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    int USE_IMU = fsSettings["imu"];

    // Known initial target pose anchors the pre-failure trajectory in world.
    // ROS params override YAML so each experiment can supply measured values
    // without editing the checked-in config.
    std::vector<double> initial_world_position(3, 0.0);
    cv::FileNode initial_position_node = fsSettings["initial_world_position"];
    if (initial_position_node.type() == cv::FileNode::SEQ && initial_position_node.size() == 3)
    {
        size_t i = 0;
        for (auto it = initial_position_node.begin(); it != initial_position_node.end(); ++it)
            initial_world_position[i++] = (double)*it;
    }
    double initial_world_yaw_deg = 0.0;
    cv::FileNode initial_yaw_node = fsSettings["initial_world_yaw_deg"];
    if (!initial_yaw_node.empty()) initial_world_yaw_deg = (double)initial_yaw_node;
    n.getParam("initial_world_position", initial_world_position);
    n.param("initial_world_yaw_deg", initial_world_yaw_deg, initial_world_yaw_deg);
    if (initial_world_position.size() != 3)
    {
        ROS_FATAL("[POSEGRAPH] initial_world_position must contain exactly three values.");
        return 1;
    }
    posegraph.setInitialAlignment(
        Vector3d(initial_world_position[0], initial_world_position[1], initial_world_position[2]),
        initial_world_yaw_deg);
    posegraph.setIMUFlag(USE_IMU);

    // ── Distance recovery config ──
    {
        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        if (fs.isOpened())
        {
            std::string rname;
            fs["robot_name"] >> rname;
            if (!rname.empty())
            {
                posegraph.dist_config_.robot_name = rname;
                posegraph.dist_config_.use_distance_recovery = true;
                printf("[POSEGRAPH]: Distance recovery enabled for robot: %s\n", rname.c_str());
            }

            std::string global_frame;
            fs["distance_global_frame"] >> global_frame;
            if (!global_frame.empty()) posegraph.dist_config_.global_frame = global_frame;

            auto read_positive_double = [&fs](const char *key, double &value) {
                cv::FileNode node = fs[key];
                if (!node.empty()) {
                    double candidate = (double)node;
                    if (candidate > 0.0) value = candidate;
                }
            };
            auto read_positive_int = [&fs](const char *key, int &value) {
                cv::FileNode node = fs[key];
                if (!node.empty()) {
                    int candidate = (int)node;
                    if (candidate > 0) value = candidate;
                }
            };
            read_positive_double("distance_sigma", posegraph.dist_config_.distance_sigma);
            read_positive_double("distance_max_age", posegraph.dist_config_.max_constraint_age);
            read_positive_double("distance_sync_tolerance", posegraph.dist_config_.sync_tolerance);
            read_positive_double("distance_sample_interval", posegraph.dist_config_.sample_interval);
            read_positive_double("distance_recovery_rate_hz", posegraph.dist_config_.optimization_rate_hz);
            read_positive_double("distance_wait_for_visual", posegraph.dist_config_.wait_for_visual);
            read_positive_double("distance_min_temporal_span", posegraph.dist_config_.min_temporal_span);
            read_positive_double("distance_min_self_motion", posegraph.dist_config_.min_self_motion);
            read_positive_double("distance_max_condition_number", posegraph.dist_config_.max_condition_number);
            read_positive_double("distance_max_normalized_rms", posegraph.dist_config_.max_normalized_rms);
            read_positive_double("distance_max_position_jump", posegraph.dist_config_.max_position_jump);
            read_positive_double("distance_max_restart_position_gap", posegraph.dist_config_.max_restart_position_gap);
            read_positive_double("distance_max_yaw_deviation", posegraph.dist_config_.max_yaw_deviation);
            read_positive_double("distance_range_only_position_prior_sigma", posegraph.dist_config_.range_only_position_prior_sigma);
            read_positive_double("distance_range_only_yaw_prior_sigma", posegraph.dist_config_.range_only_yaw_prior_sigma);
            read_positive_double("distance_max_value", posegraph.dist_config_.max_distance);
            read_positive_double("visual_recovery_position_sigma", posegraph.dist_config_.visual_position_sigma);
            read_positive_double("visual_recovery_yaw_sigma", posegraph.dist_config_.visual_yaw_sigma);
            read_positive_double("visual_recovery_sync_tolerance", posegraph.dist_config_.visual_sync_tolerance);
            read_positive_int("distance_min_neighbors", posegraph.dist_config_.min_neighbors);
            read_positive_int("distance_min_factors", posegraph.dist_config_.min_factors);

            cv::FileNode timestamp_node = fs["distance_use_header_timestamps"];
            if (!timestamp_node.empty())
                posegraph.dist_config_.use_header_timestamps = ((int)timestamp_node != 0);
            cv::FileNode planar_node = fs["distance_planar_mode"];
            if (!planar_node.empty())
                posegraph.dist_config_.planar_mode = ((int)planar_node != 0);
        }
    }

    // Allow an otherwise identical launch to be used for visual-only and
    // visual+range A/B trials.  The YAML-derived value remains the default.
    n.param("use_distance_recovery", posegraph.dist_config_.use_distance_recovery,
            posegraph.dist_config_.use_distance_recovery);
    n.param("distance_recovery_rate_hz", posegraph.dist_config_.optimization_rate_hz,
            posegraph.dist_config_.optimization_rate_hz);
    n.param("distance_planar_mode", posegraph.dist_config_.planar_mode,
            posegraph.dist_config_.planar_mode);
    if (posegraph.dist_config_.optimization_rate_hz <= 0.0)
    {
        ROS_WARN("[POSEGRAPH] Invalid distance_recovery_rate_hz; using 3.0 Hz.");
        posegraph.dist_config_.optimization_rate_hz = 3.0;
    }
    ROS_INFO("[POSEGRAPH] Distance recovery: %s (%s)",
             posegraph.dist_config_.use_distance_recovery ? "enabled" : "disabled",
             posegraph.dist_config_.planar_mode ? "planar yaw/x/y solve" : "full yaw/x/y/z solve");

    // One persistent CSV per robot and launch.  The timestamp prevents A/B
    // experiments from overwriting each other.
    std::string relative_log_dir;
    n.param<std::string>("relative_recovery_log_dir", relative_log_dir, "");
    if (relative_log_dir.empty())
    {
        const char *home = std::getenv("HOME");
        relative_log_dir = std::string(home != nullptr ? home : "/tmp") +
                           "/microswarm_ws/logs/relative_recovery";
    }
    if (!ensure_directory(relative_log_dir))
        return 1;
    const std::string audit_path = relative_log_dir + "/" +
        (posegraph.dist_config_.robot_name.empty() ? "unknown_robot" : posegraph.dist_config_.robot_name) +
        "_relative_recovery_" + std::to_string(ros::WallTime::now().toNSec()) + ".csv";
    if (!posegraph.initializeDistanceAuditLog(audit_path))
        return 1;

    fsSettings.release();

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("[POSEGRAPH]: load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("[POSEGRAPH]: load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("[POSEGRAPH]: no previous pose graph\n");
        load_flag = 1;
    }


    // Get camera information
    printf("[POSEGRAPH]: waiting for camera info topic...\n");
    auto msg1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vins_estimator/intrinsics", ros::Duration(ros::DURATION_MAX));
    intrinsics_callback(msg1);
    printf("[POSEGRAPH]: received camera info message!\n");
    std::cout << m_camera.get()->parametersToString() << std::endl;

    // Get camera to imu information
    printf("[POSEGRAPH]: waiting for camera to imu extrinsics topic...\n");
    auto msg2 = ros::topic::waitForMessage<nav_msgs::Odometry>("/vins_estimator/extrinsic", ros::Duration(ros::DURATION_MAX));
    extrinsic_callback(msg2);
    printf("[POSEGRAPH]: received camera to imu extrinsics message!\n");
    std::cout << qic.transpose() << std::endl;
    std::cout << tic.transpose() << std::endl;

    // Setup the rest of the publishers
    ros::Subscriber sub_vio1 = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
    ros::Subscriber sub_vio2 = n.subscribe("/vins_estimator/pose", 2000, vio_callback_pose);
    ros::Subscriber sub_image = n.subscribe("/cam0/image_raw", 2000, image_callback);
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    ros::Subscriber sub_intrinsics = n.subscribe("/vins_estimator/intrinsics", 2000, intrinsics_callback);
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    ros::Subscriber sub_margin_point = n.subscribe("/vins_estimator/margin_cloud", 2000, margin_point_callback);
    ros::Subscriber sub_recovery = n.subscribe("/failure_recovery/recovered_pose", 2000, recovery_callback);
    ros::Subscriber sub_restart  = n.subscribe("/restart", 10, restart_callback);
    ros::Subscriber sub_vio_reset = n.subscribe("/ov_msckf/vio_reset", 10, restart_callback);

    // ── Distance recovery subscribers ──
    std::vector<ros::Subscriber> neighbor_subs;
    ros::Subscriber sub_distance;
    if (posegraph.dist_config_.use_distance_recovery)
    {
        std::string dist_topic;
        {
            cv::FileStorage fs(config_file, cv::FileStorage::READ);
            fs["distance_topic"] >> dist_topic;
        }
        if (dist_topic.empty()) dist_topic = "/group1/structure_distances";
        sub_distance = n.subscribe(dist_topic, 200, distance_callback);
        printf("[POSEGRAPH]: Subscribed to distance topic: %s\n", dist_topic.c_str());

        // Subscribe to neighbor poses. Explicit topics/types are preferred; the
        // suffix convention remains available for simple deployments.
        std::string suffix;
        std::vector<std::string> neighbor_names;
        std::vector<std::string> neighbor_topics;
        std::vector<std::string> neighbor_types;
        {
            cv::FileStorage fs(config_file, cv::FileStorage::READ);
            fs["neighbor_pose_suffix"] >> suffix;
            auto read_string_sequence = [&fs](const char *key, std::vector<std::string> &values) {
                cv::FileNode node = fs[key];
                if (node.type() != cv::FileNode::SEQ)
                    return;
                for (auto it = node.begin(); it != node.end(); ++it)
                    values.push_back((std::string)*it);
            };
            read_string_sequence("neighbor_names", neighbor_names);
            read_string_sequence("neighbor_pose_topics", neighbor_topics);
            read_string_sequence("neighbor_pose_types", neighbor_types);
        }
        if (suffix.empty()) suffix = "/global_pose";

        if (!neighbor_topics.empty() && neighbor_topics.size() != neighbor_names.size())
        {
            ROS_FATAL("[POSEGRAPH] neighbor_pose_topics (%zu) must match neighbor_names (%zu).",
                      neighbor_topics.size(), neighbor_names.size());
            return 1;
        }
        if (!neighbor_types.empty() && neighbor_types.size() != neighbor_names.size())
        {
            ROS_FATAL("[POSEGRAPH] neighbor_pose_types (%zu) must match neighbor_names (%zu).",
                      neighbor_types.size(), neighbor_names.size());
            return 1;
        }

        for (size_t index = 0; index < neighbor_names.size(); ++index)
        {
            const std::string &name = neighbor_names[index];
            const std::string topic = neighbor_topics.empty() ? "/" + name + suffix : neighbor_topics[index];
            const std::string type = neighbor_types.empty() ? "PoseStamped" : neighbor_types[index];
            if (type == "PoseStamped" || type == "geometry_msgs/PoseStamped")
                neighbor_subs.push_back(n.subscribe<geometry_msgs::PoseStamped>(
                    topic, 200, boost::bind(neighbor_pose_callback, _1, name)));
            else if (type == "PoseWithCovarianceStamped" || type == "geometry_msgs/PoseWithCovarianceStamped")
                neighbor_subs.push_back(n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
                    topic, 200, boost::bind(neighbor_pose_cov_callback, _1, name)));
            else if (type == "Odometry" || type == "nav_msgs/Odometry")
                neighbor_subs.push_back(n.subscribe<nav_msgs::Odometry>(
                    topic, 200, boost::bind(neighbor_odom_callback, _1, name)));
            else
            {
                ROS_FATAL("[POSEGRAPH] Unsupported neighbor pose type '%s' for %s.",
                          type.c_str(), name.c_str());
                return 1;
            }
            printf("[POSEGRAPH]: Subscribed to neighbor pose: %s (%s, robot=%s)\n",
                   topic.c_str(), type.c_str(), name.c_str());
        }
    }

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
    pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);
    pub_pose_rect = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_rect", 1000);

    std::thread measurement_process;
    std::thread keyboard_command_process;
    std::thread distance_recovery_process;

    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);
    if (posegraph.dist_config_.use_distance_recovery)
    {
        const double rate_hz = posegraph.dist_config_.optimization_rate_hz;
        ROS_INFO("[POSEGRAPH] Range recovery worker running at %.2f Hz", rate_hz);
        distance_recovery_process = std::thread([rate_hz]() {
            ros::WallRate rate(rate_hz);
            while (ros::ok())
            {
                posegraph.attemptDistanceRecovery();
                rate.sleep();
            }
        });
    }
    
    ros::spin();

    if (distance_recovery_process.joinable())
        distance_recovery_process.join();

    return 0;
}
