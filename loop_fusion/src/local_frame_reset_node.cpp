#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <string>
#include <sys/stat.h>

namespace {

bool ensureParentDirectory(const std::string &file_path) {
  const std::size_t slash = file_path.find_last_of('/');
  if (slash == std::string::npos || slash == 0) {
    return true;
  }

  const std::string parent = file_path.substr(0, slash);
  std::string current;
  if (!parent.empty() && parent.front() == '/') {
    current = "/";
  }
  std::size_t begin = current.empty() ? 0 : 1;
  while (begin <= parent.size()) {
    const std::size_t end = parent.find('/', begin);
    const std::string component = parent.substr(
        begin, end == std::string::npos ? std::string::npos : end - begin);
    if (!component.empty()) {
      if (current.size() > 1 && current.back() != '/') {
        current.push_back('/');
      }
      current += component;
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        ROS_ERROR("[LOCAL_FRAME_RESET] Cannot create %s (errno=%d)",
                  current.c_str(), errno);
        return false;
      }
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return true;
}

Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::Quaternion &msg) {
  Eigen::Quaterniond q(msg.w, msg.x, msg.y, msg.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

void quaternionToMsg(const Eigen::Quaterniond &q_in,
                     geometry_msgs::Quaternion &msg) {
  Eigen::Quaterniond q = q_in.normalized();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
}

Eigen::Vector3d positionFromMsg(const geometry_msgs::Point &msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

void positionToMsg(const Eigen::Vector3d &p, geometry_msgs::Point &msg) {
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
}

}  // namespace

class LocalFrameResetNode {
 public:
  LocalFrameResetNode() : nh_(), private_nh_("~") {
    private_nh_.param<std::string>("robot_name", robot_name_, "unknown");
    private_nh_.param<std::string>("rotation_mode", rotation_mode_, "yaw_only");
    private_nh_.param<std::string>("output_frame_id", output_frame_id_,
                                   "vio_local");
    private_nh_.param<std::string>("audit_path", audit_path_, "");

    if (rotation_mode_ != "yaw_only" && rotation_mode_ != "full_3d") {
      ROS_WARN("[LOCAL_FRAME_RESET:%s] Unknown rotation_mode '%s'; using yaw_only.",
               robot_name_.c_str(), rotation_mode_.c_str());
      rotation_mode_ = "yaw_only";
    }

    poseimu_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "poseimu_out", 200);
    loop_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("loop_pose_out", 100);
    loop_feats_pub_ = nh_.advertise<sensor_msgs::PointCloud>("loop_feats_out", 100);
    restart_pub_ = nh_.advertise<std_msgs::Bool>("restart_out", 10);

    poseimu_sub_ = nh_.subscribe("poseimu_in", 400,
                                 &LocalFrameResetNode::poseimuCallback, this);
    loop_pose_sub_ = nh_.subscribe("loop_pose_in", 200,
                                   &LocalFrameResetNode::loopPoseCallback, this);
    loop_feats_sub_ = nh_.subscribe("loop_feats_in", 200,
                                    &LocalFrameResetNode::loopFeatsCallback, this);
    restart_sub_ = nh_.subscribe("restart_in", 10,
                                 &LocalFrameResetNode::restartCallback, this);

    initializeAudit();
    ROS_INFO("[LOCAL_FRAME_RESET:%s] Ready (rotation_mode=%s). Input is pass-through until restart.",
             robot_name_.c_str(), rotation_mode_.c_str());
  }

 private:
  void initializeAudit() {
    if (audit_path_.empty()) {
      return;
    }
    if (!ensureParentDirectory(audit_path_)) {
      audit_path_.clear();
      return;
    }
    audit_.open(audit_path_, std::ios::out | std::ios::trunc);
    if (!audit_.is_open()) {
      ROS_ERROR("[LOCAL_FRAME_RESET:%s] Cannot open audit CSV: %s",
                robot_name_.c_str(), audit_path_.c_str());
      audit_path_.clear();
      return;
    }
    audit_ << "wall_time,ros_time,event,epoch,origin_stamp,origin_x,origin_y,origin_z,"
              "origin_yaw_deg,rotation_mode,detail\n";
    audit_.flush();
    ROS_INFO("[LOCAL_FRAME_RESET:%s] Audit CSV: %s", robot_name_.c_str(),
             audit_path_.c_str());
  }

  void logEvent(const std::string &event, const std::string &detail) {
    if (!audit_.is_open()) {
      return;
    }
    audit_ << std::fixed << std::setprecision(9)
           << ros::WallTime::now().toSec() << ',' << ros::Time::now().toSec()
           << ',' << event << ',' << epoch_ << ',' << epoch_start_stamp_.toSec()
           << ',' << origin_position_raw_.x() << ',' << origin_position_raw_.y()
           << ',' << origin_position_raw_.z() << ',' << origin_yaw_raw_deg_ << ','
           << rotation_mode_ << ',' << detail << '\n';
    audit_.flush();
  }

  void poseimuCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    const Eigen::Vector3d p = positionFromMsg(msg->pose.pose.position);
    const Eigen::Quaterniond q = quaternionFromMsg(msg->pose.pose.orientation);
    if (!p.allFinite()) {
      ROS_WARN_THROTTLE(1.0, "[LOCAL_FRAME_RESET:%s] Invalid poseimu ignored.",
                        robot_name_.c_str());
      return;
    }

    latest_pose_valid_ = true;
    latest_position_raw_ = p;
    latest_orientation_raw_ = q;
    latest_pose_stamp_ = msg->header.stamp;

    if (shouldDropAfterReset(msg->header.stamp, "poseimu")) {
      return;
    }

    geometry_msgs::PoseWithCovarianceStamped output = *msg;
    if (epoch_ > 0) {
      const Eigen::Vector3d transformed_position =
          origin_raw_to_local_ * (p - origin_position_raw_);
      const Eigen::Quaterniond transformed_orientation(
          origin_raw_to_local_ * q.toRotationMatrix());
      positionToMsg(transformed_position, output.pose.pose.position);
      quaternionToMsg(transformed_orientation, output.pose.pose.orientation);
      rotateCovariance(msg->pose.covariance, output.pose.covariance);
      output.header.frame_id = epochFrameId();
    }
    poseimu_pub_.publish(output);
  }

  void loopPoseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (shouldDropAfterReset(msg->header.stamp, "loop_pose")) {
      return;
    }
    nav_msgs::Odometry output = *msg;
    if (epoch_ > 0) {
      const Eigen::Vector3d p = positionFromMsg(msg->pose.pose.position);
      const Eigen::Quaterniond q = quaternionFromMsg(msg->pose.pose.orientation);
      if (!p.allFinite()) {
        return;
      }
      positionToMsg(origin_raw_to_local_ * (p - origin_position_raw_),
                    output.pose.pose.position);
      quaternionToMsg(
          Eigen::Quaterniond(origin_raw_to_local_ * q.toRotationMatrix()),
          output.pose.pose.orientation);
      rotateCovariance(msg->pose.covariance, output.pose.covariance);
      output.header.frame_id = epochFrameId();
    }
    loop_pose_pub_.publish(output);
  }

  void loopFeatsCallback(const sensor_msgs::PointCloud::ConstPtr &msg) {
    if (shouldDropAfterReset(msg->header.stamp, "loop_feats")) {
      return;
    }
    sensor_msgs::PointCloud output = *msg;
    if (epoch_ > 0) {
      for (geometry_msgs::Point32 &point : output.points) {
        const Eigen::Vector3d raw(point.x, point.y, point.z);
        const Eigen::Vector3d transformed =
            origin_raw_to_local_ * (raw - origin_position_raw_);
        point.x = transformed.x();
        point.y = transformed.y();
        point.z = transformed.z();
      }
      output.header.frame_id = epochFrameId();
    }
    loop_feats_pub_.publish(output);
  }

  void restartCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (!msg->data) {
      restart_pub_.publish(*msg);
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    if (!last_restart_wall_.isZero() &&
        (now - last_restart_wall_).toSec() < 1.0) {
      ROS_WARN("[LOCAL_FRAME_RESET:%s] Duplicate restart ignored.",
               robot_name_.c_str());
      logEvent("RESET_REJECTED", "duplicate_within_one_second");
      return;
    }
    if (!latest_pose_valid_) {
      ROS_ERROR("[LOCAL_FRAME_RESET:%s] Restart rejected: no poseimu sample has arrived.",
                robot_name_.c_str());
      logEvent("RESET_REJECTED", "no_poseimu_sample");
      return;
    }
    last_restart_wall_ = now;

    origin_position_raw_ = latest_position_raw_;
    const Eigen::Matrix3d latest_rotation = latest_orientation_raw_.toRotationMatrix();
    if (rotation_mode_ == "full_3d") {
      origin_raw_to_local_ = latest_rotation.transpose();
    } else {
      const double yaw = std::atan2(latest_rotation(1, 0), latest_rotation(0, 0));
      origin_raw_to_local_ =
          Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    epoch_start_stamp_ = latest_pose_stamp_.isZero() ? ros::Time::now()
                                                     : latest_pose_stamp_;
    ++epoch_;
    dropped_poseimu_ = dropped_loop_pose_ = dropped_loop_feats_ = 0;

    const double raw_yaw =
        std::atan2(latest_rotation(1, 0), latest_rotation(0, 0)) * 180.0 /
        M_PI;
    origin_yaw_raw_deg_ = raw_yaw;
    ROS_WARN("[LOCAL_FRAME_RESET:%s] Epoch %u starts at %.6f: raw origin "
             "p=(%.3f, %.3f, %.3f), yaw=%.2f deg. VIO process remains running.",
             robot_name_.c_str(), epoch_, epoch_start_stamp_.toSec(),
             origin_position_raw_.x(), origin_position_raw_.y(),
             origin_position_raw_.z(), raw_yaw);
    logEvent("RESET_ACCEPTED", "downstream_restart_published");

    // Publish only after the transform is latched. Consumers clear their old
    // queues on this event, and every subsequently published sample is in the
    // new local frame.
    restart_pub_.publish(*msg);
  }

  bool shouldDropAfterReset(const ros::Time &stamp, const char *stream) {
    if (epoch_ == 0 || stamp.isZero() || stamp > epoch_start_stamp_) {
      return false;
    }
    std::uint64_t *counter = nullptr;
    if (std::string(stream) == "poseimu") {
      counter = &dropped_poseimu_;
    } else if (std::string(stream) == "loop_pose") {
      counter = &dropped_loop_pose_;
    } else {
      counter = &dropped_loop_feats_;
    }
    ++(*counter);
    ROS_WARN_THROTTLE(
        2.0,
        "[LOCAL_FRAME_RESET:%s] Dropping stale %s sample at %.6f "
        "(epoch boundary %.6f).",
        robot_name_.c_str(), stream, stamp.toSec(), epoch_start_stamp_.toSec());
    return true;
  }

  template <typename Covariance>
  void rotateCovariance(const Covariance &input, Covariance &output) const {
    Eigen::Matrix<double, 6, 6> covariance;
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        covariance(row, col) = input[6 * row + col];
      }
    }
    Eigen::Matrix<double, 6, 6> transform =
        Eigen::Matrix<double, 6, 6>::Zero();
    transform.block<3, 3>(0, 0) = origin_raw_to_local_;
    transform.block<3, 3>(3, 3) = origin_raw_to_local_;
    covariance = transform * covariance * transform.transpose();
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        output[6 * row + col] = covariance(row, col);
      }
    }
  }

  std::string epochFrameId() const {
    return output_frame_id_ + "_epoch_" + std::to_string(epoch_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber poseimu_sub_;
  ros::Subscriber loop_pose_sub_;
  ros::Subscriber loop_feats_sub_;
  ros::Subscriber restart_sub_;
  ros::Publisher poseimu_pub_;
  ros::Publisher loop_pose_pub_;
  ros::Publisher loop_feats_pub_;
  ros::Publisher restart_pub_;

  std::string robot_name_;
  std::string rotation_mode_;
  std::string output_frame_id_;
  std::string audit_path_;
  std::ofstream audit_;

  bool latest_pose_valid_ = false;
  Eigen::Vector3d latest_position_raw_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond latest_orientation_raw_ = Eigen::Quaterniond::Identity();
  ros::Time latest_pose_stamp_;

  unsigned int epoch_ = 0;
  Eigen::Vector3d origin_position_raw_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d origin_raw_to_local_ = Eigen::Matrix3d::Identity();
  double origin_yaw_raw_deg_ = 0.0;
  ros::Time epoch_start_stamp_;
  ros::WallTime last_restart_wall_;
  std::uint64_t dropped_poseimu_ = 0;
  std::uint64_t dropped_loop_pose_ = 0;
  std::uint64_t dropped_loop_feats_ = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_frame_reset");
  LocalFrameResetNode node;
  ros::spin();
  return 0;
}
