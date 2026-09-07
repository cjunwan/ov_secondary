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

#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include <assert.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdio.h>
#include <ros/ros.h>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"


#define SHOW_S_EDGE true
#define SHOW_L_EDGE true
#define SAVE_LOOP_PATH true

using namespace DVision;
using namespace DBoW2;

class PoseGraph
{
public:
	PoseGraph();
	~PoseGraph();
	void registerPub(ros::NodeHandle &n);
	void addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
	void loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
	void loadVocabulary(std::string voc_path);
	void setIMUFlag(bool _use_imu);
	void setInitialAlignment(const Eigen::Vector3d &world_position, double yaw_degrees);
	KeyFrame* getKeyFrame(int index);
	nav_msgs::Path path[10];
	nav_msgs::Path base_path;
	nav_msgs::Path combined_path;  ///< All sequences merged into one trajectory
	CameraPoseVisualization* posegraph_visualization;
	void savePoseGraph();
	void loadPoseGraph();
	void publish();
	void updateRecoveryPose(const Eigen::Vector3d &P_rec, const Eigen::Quaterniond &Q_rec, double timestamp);
	void onRestart();
	bool initializeDistanceAuditLog(const std::string &path);

	// ─── Distance-based recovery ────────────────────────────────────────
	struct NeighborPose {
		std::string name;
		Eigen::Vector3d position;
		double timestamp;
		std::string frame_id;
	};

	struct DistanceConstraint {
		std::string other_robot;
		double distance;
		double timestamp;
	};

	struct SelfPoseSample {
		Eigen::Vector3d position;
		Eigen::Matrix3d rotation;
		double timestamp;
		int keyframe_index;
	};

	struct AlignedRangeFactor {
		std::string other_robot;
		Eigen::Vector3d self_vio_position;
		Eigen::Vector3d neighbor_global_position;
		double distance;
		double timestamp;
	};

	struct DistanceRecoveryConfig {
		std::string robot_name = "";
		std::string global_frame = "global";
		double distance_sigma = 0.1;       // meters
		bool use_distance_recovery = false;
		bool use_header_timestamps = true;
		double max_constraint_age = 5.0;   // seconds of measurements used by one recovery
		double sync_tolerance = 0.15;      // max self/neighbor/range time mismatch
		double sample_interval = 0.05;     // per-neighbor range downsampling interval
		double optimization_rate_hz = 3.0; // asynchronous range-only solve rate
		double wait_for_visual = 3.0;      // range-only fallback delay after first post-reset pose
		double min_temporal_span = 0.5;    // minimum range-factor time span
		double min_self_motion = 0.15;     // motion needed to make yaw observable
		double max_condition_number = 1e4;
		double max_normalized_rms = 3.0;   // RMS of range residuals divided by sigma
		double max_position_jump = 3.0;    // latest recovered pose vs pre-reset anchor
		double max_restart_position_gap = 1.0; // first recovered pose vs pre-reset anchor
		double max_yaw_deviation = 30.0;   // solved transform yaw vs continuity/visual prior
		double range_only_position_prior_sigma = 0.50; // pre/post restart continuity prior [m]
		double range_only_yaw_prior_sigma = 15.0;      // pre/post restart yaw prior [deg]
		double max_distance = 100.0;
		double visual_position_sigma = 0.10;
		double visual_yaw_sigma = 3.0;     // degrees
		double visual_sync_tolerance = 0.30; // recovered image vs post-reset keyframe
		bool planar_mode = true;           // ground robots: ranges solve yaw/x/y; z follows continuity/visual prior
		int min_neighbors = 2;
		int min_factors = 8;
	};

	DistanceRecoveryConfig dist_config_;

	void updateNeighborPose(const std::string &name, const Eigen::Vector3d &position,
	                        double timestamp, const std::string &frame_id = "");
	void addDistanceConstraint(const std::string &robot_a, const std::string &robot_b, double distance, double timestamp);
	void attemptDistanceRecovery();
	Vector3d t_drift;
	double yaw_drift;
	Matrix3d r_drift;
	// world frame( base sequence or first sequence)<----> cur sequence frame  
	Vector3d w_t_vio;
	Matrix3d w_r_vio;
	Eigen::Vector3d initial_world_position_ = Eigen::Vector3d::Zero();
	Eigen::Matrix3d initial_world_rotation_ = Eigen::Matrix3d::Identity();
	bool initial_alignment_enabled_ = false;
	bool initial_alignment_pending_ = false;


private:
	int detectLoop(KeyFrame* keyframe, int frame_index);
	void addKeyFrameIntoVoc(KeyFrame* keyframe);
	void optimize4DoF();
	void optimize6DoF();
	void updatePath();
	void publishStitchMarkers(const Eigen::Vector3d &pre_P, const Eigen::Vector3d &post_P);
	void addSelfPoseSample(const KeyFrame *keyframe, const Eigen::Vector3d &position,
	                      const Eigen::Matrix3d &rotation);
	void tryApplyPendingVisualRecovery();
	void clearPendingVisualRecovery();
	void cancelDistanceRecovery(const std::string &reason);
	bool collectAlignedRangeFactors(double reference_timestamp,
	                               std::vector<AlignedRangeFactor> &factors,
	                               int &unique_neighbors, double &temporal_span,
	                               double &self_motion);
	bool rangeGeometryObservable(const std::vector<AlignedRangeFactor> &factors,
	                            double yaw_degrees, const Eigen::Vector3d &translation,
	                            double &condition_number) const;
	bool solveRecoveryTransform(const std::vector<AlignedRangeFactor> &factors,
	                           bool use_visual, const Eigen::Vector3d &visual_position,
	                           double visual_yaw_drift, const Eigen::Vector3d &visual_vio_position,
	                           double initial_yaw, const Eigen::Vector3d &initial_translation,
	                           double &solved_yaw, Eigen::Vector3d &solved_translation,
	                           double &normalized_range_rms);
	bool completeRecovery(double solved_yaw, const Eigen::Vector3d &solved_translation,
	                     const std::string &source);
	void logDistanceAudit(const std::string &event, const std::string &mode,
	                      size_t factor_count, int neighbor_count,
	                      double temporal_span, double self_motion,
	                      double normalized_rms, bool accepted,
	                      const std::string &detail = "", bool throttle = false);
	list<KeyFrame*> keyframelist;
	std::mutex m_keyframelist;
	std::mutex m_optimize_buf;
	std::mutex m_path;
	std::mutex m_drift;
	std::thread t_optimization;
	std::queue<int> optimize_buf;

	int global_index;
	int sequence_cnt;
	vector<bool> sequence_loop;
	map<int, cv::Mat> image_pool;
	int earliest_loop_index;
	int base_sequence;
	bool use_imu;
	std::atomic<int> restart_index_;   // first global keyframe index after /restart (-1 = not in recovery)

	struct PendingVisualRecovery {
		bool valid = false;
		Eigen::Vector3d position = Eigen::Vector3d::Zero();
		Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
		double timestamp = -1.0;
		int boundary = -1;
	};
	std::mutex m_pending_visual_recovery_;
	PendingVisualRecovery pending_visual_recovery_;
	std::mutex m_recovery_solver_;

	// Distance recovery storage
	std::mutex m_distance_;
	std::map<std::string, std::deque<NeighborPose>> neighbor_pose_history_;
	std::deque<SelfPoseSample> self_pose_history_;
	std::deque<DistanceConstraint> distance_constraints_;
	std::map<std::string, double> last_distance_timestamp_by_neighbor_;
	double first_post_restart_timestamp_ = -1.0;
	bool recovery_anchor_valid_ = false;
	Eigen::Vector3d recovery_anchor_position_ = Eigen::Vector3d::Zero();
	Eigen::Matrix3d recovery_anchor_rotation_ = Eigen::Matrix3d::Identity();

	// Persistent audit trail proving whether range information was merely
	// received, entered the optimizer, or was accepted in the final recovery.
	std::mutex m_distance_log_;
	std::ofstream distance_log_;
	std::string distance_log_path_;
	std::atomic<uint64_t> distance_received_total_{0};
	std::atomic<uint64_t> recovery_data_generation_{0};
	std::atomic<uint64_t> last_range_attempt_generation_{0};
	std::map<std::string, double> distance_log_last_wall_time_;

	BriefDatabase db;
	BriefVocabulary* voc;

	ros::Publisher pub_pg_path;
	ros::Publisher pub_base_path;
	ros::Publisher pub_pose_graph;
	ros::Publisher pub_path[10];
	ros::Publisher pub_recovery_stitch;
	ros::Publisher pub_recovery_cancel;
	ros::Publisher pub_combined_path;
};

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
  	return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
  	return angle_degrees + T(360.0);
  else
  	return angle_degrees;
};

class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

template <typename T> 
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{

	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);


	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};

template <typename T> 
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

template <typename T> 
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

struct FourDOFError
{
	FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
				  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}

	template <typename T>
	bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		// euler to rotation
		T w_R_i[9];
		YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
		// rotation transpose
		T i_R_w[9];
		RotationMatrixTranspose(w_R_i, i_R_w);
		// rotation matrix rotate point
		T t_i_ij[3];
		RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x));
		residuals[1] = (t_i_ij[1] - T(t_y));
		residuals[2] = (t_i_ij[2] - T(t_z));
		residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double relative_yaw, const double pitch_i, const double roll_i) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          FourDOFError, 4, 1, 3, 1, 3>(
	          	new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
	}

	double t_x, t_y, t_z;
	double relative_yaw, pitch_i, roll_i;

};

struct FourDOFWeightError
{
	FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
				  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
				  	weight = 1;
				  }

	template <typename T>
	bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		// euler to rotation
		T w_R_i[9];
		YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
		// rotation transpose
		T i_R_w[9];
		RotationMatrixTranspose(w_R_i, i_R_w);
		// rotation matrix rotate point
		T t_i_ij[3];
		RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
		residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
		residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
		residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double relative_yaw, const double pitch_i, const double roll_i) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          FourDOFWeightError, 4, 1, 3, 1, 3>(
	          	new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
	}

	double t_x, t_y, t_z;
	double relative_yaw, pitch_i, roll_i;
	double weight;

};

struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w);

		T t_i_ij[3];
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q); 

		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};

// ═══════════════════════════════════════════════════════════════════════════════
// Distance-based recovery cost functions
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Distance factor: penalizes deviation from measured inter-robot distance.
 *
 * Optimization variables: yaw_drift (1), t_drift (3)
 * Corrected position: P_corrected = R(yaw_drift) * VIO_P + t_drift
 * Residual: (d_measured - ||P_corrected - P_neighbor||) / sigma
 */
struct DistanceFactor
{
	DistanceFactor(double d_measured, const Eigen::Vector3d &P_neighbor,
	               const Eigen::Vector3d &VIO_P, double sigma)
	    : d_measured_(d_measured), P_neighbor_(P_neighbor), VIO_P_(VIO_P), sigma_(sigma) {}

	template <typename T>
	bool operator()(const T* const yaw_drift, const T* const t_drift, T* residuals) const
	{
		// R_drift = Rz(yaw_drift), applied to VIO position
		T cy = cos(yaw_drift[0] * T(M_PI) / T(180.0));
		T sy = sin(yaw_drift[0] * T(M_PI) / T(180.0));

		// P_corrected = R_yaw * VIO_P + t_drift
		T px = cy * T(VIO_P_.x()) - sy * T(VIO_P_.y()) + t_drift[0];
		T py = sy * T(VIO_P_.x()) + cy * T(VIO_P_.y()) + t_drift[1];
		T pz = T(VIO_P_.z()) + t_drift[2];

		// Distance to neighbor
		T dx = px - T(P_neighbor_.x());
		T dy = py - T(P_neighbor_.y());
		T dz = pz - T(P_neighbor_.z());
		T dist = sqrt(dx * dx + dy * dy + dz * dz + T(1e-8));

		residuals[0] = (dist - T(d_measured_)) / T(sigma_);
		return true;
	}

	static ceres::CostFunction* Create(double d_measured, const Eigen::Vector3d &P_neighbor,
	                                    const Eigen::Vector3d &VIO_P, double sigma)
	{
		return new ceres::AutoDiffCostFunction<DistanceFactor, 1, 1, 3>(
		    new DistanceFactor(d_measured, P_neighbor, VIO_P, sigma));
	}

	double d_measured_;
	Eigen::Vector3d P_neighbor_;
	Eigen::Vector3d VIO_P_;
	double sigma_;
};

/**
 * @brief Visual recovery factor: penalizes deviation from the PnP-recovered pose.
 *
 * Residual: P_rec - (R(yaw_drift) * VIO_P + t_drift)
 */
struct VisualRecoveryFactor
{
	VisualRecoveryFactor(const Eigen::Vector3d &P_rec, const Eigen::Vector3d &VIO_P,
	                     double sigma)
	    : P_rec_(P_rec), VIO_P_(VIO_P), sigma_(sigma) {}

	template <typename T>
	bool operator()(const T* const yaw_drift, const T* const t_drift, T* residuals) const
	{
		T cy = cos(yaw_drift[0] * T(M_PI) / T(180.0));
		T sy = sin(yaw_drift[0] * T(M_PI) / T(180.0));

		T px = cy * T(VIO_P_.x()) - sy * T(VIO_P_.y()) + t_drift[0];
		T py = sy * T(VIO_P_.x()) + cy * T(VIO_P_.y()) + t_drift[1];
		T pz = T(VIO_P_.z()) + t_drift[2];

		residuals[0] = (T(P_rec_.x()) - px) / T(sigma_);
		residuals[1] = (T(P_rec_.y()) - py) / T(sigma_);
		residuals[2] = (T(P_rec_.z()) - pz) / T(sigma_);
		return true;
	}

	static ceres::CostFunction* Create(const Eigen::Vector3d &P_rec, const Eigen::Vector3d &VIO_P,
	                                    double sigma)
	{
		return new ceres::AutoDiffCostFunction<VisualRecoveryFactor, 3, 1, 3>(
		    new VisualRecoveryFactor(P_rec, VIO_P, sigma));
	}

	Eigen::Vector3d P_rec_;
	Eigen::Vector3d VIO_P_;
	double sigma_;
};

/**
 * @brief Weak prior on yaw drift (used for distance-only recovery where yaw is unobservable).
 *
 * Residual: (yaw_drift - yaw_prior) / sigma
 */
struct YawPriorFactor
{
	YawPriorFactor(double yaw_prior, double sigma)
	    : yaw_prior_(yaw_prior), sigma_(sigma) {}

	template <typename T>
	bool operator()(const T* const yaw_drift, T* residuals) const
	{
		residuals[0] = NormalizeAngle(yaw_drift[0] - T(yaw_prior_)) / T(sigma_);
		return true;
	}

	static ceres::CostFunction* Create(double yaw_prior, double sigma)
	{
		return new ceres::AutoDiffCostFunction<YawPriorFactor, 1, 1>(
		    new YawPriorFactor(yaw_prior, sigma));
	}

	double yaw_prior_;
	double sigma_;
};
