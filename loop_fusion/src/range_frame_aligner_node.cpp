#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <loop_fusion/RobotPairDistances.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace {

struct TimedPose {
  double stamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct TimedRange {
  double stamp;
  std::string a;
  std::string b;
  double distance;
};

template <typename T>
Eigen::Matrix<T, 3, 1> transformPoint(const Eigen::Vector3d& p, const T* x) {
  const T c = ceres::cos(x[0]);
  const T s = ceres::sin(x[0]);
  return Eigen::Matrix<T, 3, 1>(
      c * T(p.x()) - s * T(p.y()) + x[1],
      s * T(p.x()) + c * T(p.y()) + x[2],
      T(p.z()) + x[3]);
}

struct AnchorRangeCost {
  AnchorRangeCost(const Eigen::Vector3d& anchor,
                  const Eigen::Vector3d& neighbor,
                  double distance,
                  double sigma)
      : anchor_(anchor), neighbor_(neighbor), distance_(distance), sigma_(sigma) {}

  template <typename T>
  bool operator()(const T* const transform, T* residual) const {
    const Eigen::Matrix<T, 3, 1> p = transformPoint(neighbor_, transform);
    const T dx = p.x() - T(anchor_.x());
    const T dy = p.y() - T(anchor_.y());
    const T dz = p.z() - T(anchor_.z());
    residual[0] = (ceres::sqrt(dx * dx + dy * dy + dz * dz + T(1e-12)) -
                   T(distance_)) /
                  T(sigma_);
    return true;
  }

  Eigen::Vector3d anchor_;
  Eigen::Vector3d neighbor_;
  double distance_;
  double sigma_;
};

struct NeighborRangeCost {
  NeighborRangeCost(const Eigen::Vector3d& first,
                    const Eigen::Vector3d& second,
                    double distance,
                    double sigma)
      : first_(first), second_(second), distance_(distance), sigma_(sigma) {}

  template <typename T>
  bool operator()(const T* const first_transform,
                  const T* const second_transform,
                  T* residual) const {
    const Eigen::Matrix<T, 3, 1> p1 = transformPoint(first_, first_transform);
    const Eigen::Matrix<T, 3, 1> p2 = transformPoint(second_, second_transform);
    residual[0] = ((p1 - p2).norm() - T(distance_)) / T(sigma_);
    return true;
  }

  Eigen::Vector3d first_;
  Eigen::Vector3d second_;
  double distance_;
  double sigma_;
};

struct ZPriorCost {
  ZPriorCost(double value, double sigma) : value_(value), sigma_(sigma) {}
  template <typename T>
  bool operator()(const T* const transform, T* residual) const {
    residual[0] = (transform[3] - T(value_)) / T(sigma_);
    return true;
  }
  double value_;
  double sigma_;
};

std::string canonicalPair(const std::string& a, const std::string& b) {
  return a < b ? a + "|" + b : b + "|" + a;
}

}  // namespace

class RangeFrameAligner {
 public:
  RangeFrameAligner() : nh_(), private_nh_("~") {
    private_nh_.param<std::string>("anchor_name", anchor_name_, "omo2");
    private_nh_.param<std::string>("global_frame", global_frame_, "global");
    private_nh_.param<double>("window_seconds", window_seconds_, 20.0);
    private_nh_.param<double>("sync_tolerance", sync_tolerance_, 0.30);
    private_nh_.param<double>("sample_interval", sample_interval_, 0.05);
    private_nh_.param<double>("range_sigma", range_sigma_, 0.15);
    private_nh_.param<double>("z_prior_sigma", z_prior_sigma_, 0.50);
    private_nh_.param<double>("solve_period", solve_period_, 1.0);
    private_nh_.param<double>("min_span", min_span_, 3.0);
    private_nh_.param<double>("min_motion", min_motion_, 0.20);
    private_nh_.param<double>("max_normalized_rms", max_normalized_rms_, 3.5);
    private_nh_.param<int>("min_factors", min_factors_, 40);

    if (!private_nh_.getParam("neighbor_names", neighbor_names_) ||
        !private_nh_.getParam("neighbor_pose_topics", neighbor_topics_) ||
        neighbor_names_.empty() || neighbor_names_.size() != neighbor_topics_.size()) {
      ROS_FATAL("~neighbor_names and ~neighbor_pose_topics must be non-empty parallel lists.");
      throw std::runtime_error("invalid neighbor configuration");
    }

    std::string anchor_topic = "/ov_secondary_" + anchor_name_ + "/poseimu";
    std::string distance_topic = "/group1/structure_distances";
    std::string restart_topic = "/" + anchor_name_ + "/restart";
    private_nh_.param<std::string>("anchor_pose_topic", anchor_topic, anchor_topic);
    private_nh_.param<std::string>("distance_topic", distance_topic, distance_topic);
    private_nh_.param<std::string>("restart_topic", restart_topic, restart_topic);

    anchor_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        anchor_topic, 500, &RangeFrameAligner::anchorCallback, this);
    distance_sub_ = nh_.subscribe(
        distance_topic, 2000, &RangeFrameAligner::distanceCallback, this);
    restart_sub_ = nh_.subscribe(
        restart_topic, 10, &RangeFrameAligner::restartCallback, this);

    for (size_t i = 0; i < neighbor_names_.size(); ++i) {
      const std::string name = neighbor_names_[i];
      neighbor_subs_.push_back(
          nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
              neighbor_topics_[i], 500,
              boost::bind(&RangeFrameAligner::neighborCallback, this, _1, name)));
      publishers_[name] =
          nh_.advertise<geometry_msgs::PoseStamped>("/range_frame_aligner/" + name +
                                                        "/global_pose",
                                                    100);
      transforms_[name] = {{0.0, 0.0, 0.0, 0.0}};
    }
    status_pub_ = private_nh_.advertise<std_msgs::Bool>("valid", 1, true);
    timer_ = nh_.createTimer(ros::Duration(solve_period_),
                             &RangeFrameAligner::solveCallback, this);
    publishStatus(false);
    ROS_INFO("[RANGE_ALIGNER] anchor=%s, neighbors=%zu, window=%.1fs",
             anchor_name_.c_str(), neighbor_names_.size(), window_seconds_);
  }

 private:
  static TimedPose toTimedPose(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    const auto& p = msg->pose.pose;
    return {msg->header.stamp.toSec(),
            Eigen::Vector3d(p.position.x, p.position.y, p.position.z),
            Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                               p.orientation.y, p.orientation.z)};
  }

  void anchorCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (msg->header.stamp.isZero()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!frozen_) appendPose(anchor_history_, toTimedPose(msg));
  }

  void neighborCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
      const std::string& name) {
    if (msg->header.stamp.isZero()) return;
    TimedPose pose = toTimedPose(msg);
    std::array<double, 4> transform;
    bool publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      appendPose(neighbor_history_[name], pose);
      const auto found = transforms_.find(name);
      if (valid_ && found != transforms_.end()) {
        transform = found->second;
        publish = true;
      }
    }
    if (publish) publishAligned(name, pose, transform);
  }

  void distanceCallback(
      const loop_fusion::RobotPairDistances::ConstPtr& msg) {
    if (msg->robot_a_names.size() != msg->robot_b_names.size() ||
        msg->distances.size() != msg->robot_a_names.size()) {
      ROS_WARN_THROTTLE(1.0, "[RANGE_ALIGNER] malformed distance message.");
      return;
    }
    const double stamp =
        msg->header.stamp.isZero() ? ros::Time::now().toSec()
                                   : msg->header.stamp.toSec();
    std::lock_guard<std::mutex> lock(mutex_);
    if (frozen_) return;
    for (size_t i = 0; i < msg->distances.size(); ++i) {
      if (knownRobot(msg->robot_a_names[i]) &&
          knownRobot(msg->robot_b_names[i]) &&
          msg->robot_a_names[i] != msg->robot_b_names[i] &&
          std::isfinite(msg->distances[i]) && msg->distances[i] > 0.0) {
        ranges_.push_back({stamp, msg->robot_a_names[i],
                           msg->robot_b_names[i], msg->distances[i]});
      }
    }
    trim(stamp);
  }

  void restartCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) return;
    std::lock_guard<std::mutex> lock(mutex_);
    frozen_ = true;
    ROS_WARN("[RANGE_ALIGNER] target restart: freezing %zu transforms (%s).",
             transforms_.size(),
             valid_ ? "valid" : "not yet valid");
  }

  bool knownRobot(const std::string& name) const {
    return name == anchor_name_ ||
           std::find(neighbor_names_.begin(), neighbor_names_.end(), name) !=
               neighbor_names_.end();
  }

  void appendPose(std::deque<TimedPose>& history, const TimedPose& pose) {
    if (!history.empty() && pose.stamp < history.back().stamp) return;
    history.push_back(pose);
    trim(pose.stamp);
  }

  void trim(double newest) {
    const double cutoff = newest - window_seconds_ - sync_tolerance_;
    while (!ranges_.empty() && ranges_.front().stamp < cutoff)
      ranges_.pop_front();
    while (!anchor_history_.empty() && anchor_history_.front().stamp < cutoff)
      anchor_history_.pop_front();
    for (auto& item : neighbor_history_)
      while (!item.second.empty() && item.second.front().stamp < cutoff)
        item.second.pop_front();
  }

  static bool nearest(const std::deque<TimedPose>& poses, double stamp,
                      double tolerance, TimedPose& result) {
    if (poses.empty()) return false;
    auto it = std::lower_bound(
        poses.begin(), poses.end(), stamp,
        [](const TimedPose& pose, double value) { return pose.stamp < value; });
    auto best = it;
    if (it == poses.end()) best = std::prev(poses.end());
    if (it != poses.begin()) {
      auto previous = std::prev(it);
      if (best == poses.end() ||
          std::abs(previous->stamp - stamp) < std::abs(best->stamp - stamp))
        best = previous;
    }
    if (std::abs(best->stamp - stamp) > tolerance) return false;
    result = *best;
    return true;
  }

  static double trajectoryMotion(const std::deque<TimedPose>& poses) {
    double motion = 0.0;
    if (poses.empty()) return motion;
    for (size_t i = 1; i < poses.size(); ++i)
      motion = std::max(motion,
                        (poses[i].position - poses.front().position).norm());
    return motion;
  }

  const std::deque<TimedPose>* historyFor(
      const std::string& name,
      const std::deque<TimedPose>& anchor,
      const std::map<std::string, std::deque<TimedPose>>& neighbors) const {
    if (name == anchor_name_) return &anchor;
    auto found = neighbors.find(name);
    return found == neighbors.end() ? nullptr : &found->second;
  }

  void solveCallback(const ros::TimerEvent&) {
    std::deque<TimedPose> anchor;
    std::map<std::string, std::deque<TimedPose>> neighbors;
    std::deque<TimedRange> ranges;
    std::map<std::string, std::array<double, 4>> transforms;
    bool was_valid;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (frozen_) return;
      anchor = anchor_history_;
      neighbors = neighbor_history_;
      ranges = ranges_;
      transforms = transforms_;
      was_valid = valid_;
    }
    if (anchor.empty() || ranges.empty()) return;
    if (trajectoryMotion(anchor) < min_motion_) {
      ROS_INFO_THROTTLE(5.0,
                        "[RANGE_ALIGNER] waiting for anchor motion (%.2f/%.2fm).",
                        trajectoryMotion(anchor), min_motion_);
      return;
    }
    for (const std::string& name : neighbor_names_) {
      auto found = neighbors.find(name);
      if (found == neighbors.end() ||
          trajectoryMotion(found->second) < min_motion_) {
        ROS_INFO_THROTTLE(
            5.0, "[RANGE_ALIGNER] waiting for observable motion from %s.",
            name.c_str());
        return;
      }
    }

    ceres::Problem problem;
    std::map<std::string, double> last_pair_stamp;
    int factor_count = 0;
    double first_stamp = std::numeric_limits<double>::infinity();
    double last_stamp = -std::numeric_limits<double>::infinity();

    if (!was_valid)
      initializeTransforms(anchor, neighbors, ranges, transforms, 0.0, 1.0);
    for (const std::string& name : neighbor_names_) {
      problem.AddParameterBlock(transforms[name].data(), 4);
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<ZPriorCost, 1, 4>(
              new ZPriorCost(transforms[name][3], z_prior_sigma_)),
          nullptr, transforms[name].data());
    }

    for (const TimedRange& range : ranges) {
      const std::string pair = canonicalPair(range.a, range.b);
      if (last_pair_stamp.count(pair) &&
          range.stamp - last_pair_stamp[pair] < sample_interval_)
        continue;
      const auto* history_a = historyFor(range.a, anchor, neighbors);
      const auto* history_b = historyFor(range.b, anchor, neighbors);
      TimedPose pose_a, pose_b;
      if (!history_a || !history_b ||
          !nearest(*history_a, range.stamp, sync_tolerance_, pose_a) ||
          !nearest(*history_b, range.stamp, sync_tolerance_, pose_b))
        continue;

      ceres::LossFunction* loss = new ceres::HuberLoss(2.0);
      if (range.a == anchor_name_) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AnchorRangeCost, 1, 4>(
                new AnchorRangeCost(pose_a.position, pose_b.position,
                                    range.distance, range_sigma_)),
            loss, transforms[range.b].data());
      } else if (range.b == anchor_name_) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AnchorRangeCost, 1, 4>(
                new AnchorRangeCost(pose_b.position, pose_a.position,
                                    range.distance, range_sigma_)),
            loss, transforms[range.a].data());
      } else {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<NeighborRangeCost, 1, 4, 4>(
                new NeighborRangeCost(pose_a.position, pose_b.position,
                                      range.distance, range_sigma_)),
            loss, transforms[range.a].data(), transforms[range.b].data());
      }
      last_pair_stamp[pair] = range.stamp;
      first_stamp = std::min(first_stamp, range.stamp);
      last_stamp = std::max(last_stamp, range.stamp);
      ++factor_count;
    }

    if (factor_count < min_factors_ || last_stamp - first_stamp < min_span_) {
      ROS_INFO_THROTTLE(
          5.0, "[RANGE_ALIGNER] collecting data: factors=%d/%d span=%.2f/%.2fs",
          factor_count, min_factors_, last_stamp - first_stamp, min_span_);
      return;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 80;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    if (was_valid) {
      ceres::Solve(options, &problem, &summary);
    } else {
      // Ranges have a reflected/rotated initialization ambiguity.  A single
      // arbitrary placement often converges to a geometrically valid but
      // wrong local minimum, so test both triangle handednesses and yaw seeds.
      double best_cost = std::numeric_limits<double>::infinity();
      std::map<std::string, std::array<double, 4>> best_transforms;
      ceres::Solver::Summary best_summary;
      for (double handedness : {-1.0, 1.0}) {
        for (int seed = 0; seed < 8; ++seed) {
          const double yaw_seed = seed * M_PI / 4.0;
          initializeTransforms(anchor, neighbors, ranges, transforms, yaw_seed,
                               handedness);
          ceres::Solver::Summary candidate;
          ceres::Solve(options, &problem, &candidate);
          if (candidate.IsSolutionUsable() &&
              candidate.final_cost < best_cost) {
            best_cost = candidate.final_cost;
            best_transforms = transforms;
            best_summary = candidate;
          }
        }
      }
      if (!best_transforms.empty()) {
        transforms = best_transforms;
        summary = best_summary;
      }
    }
    const double normalized_rms =
        std::sqrt(2.0 * summary.final_cost / std::max(1, factor_count));
    const bool accepted =
        summary.IsSolutionUsable() && std::isfinite(normalized_rms) &&
        normalized_rms <= max_normalized_rms_;
    if (!accepted) {
      ROS_WARN_THROTTLE(
          3.0,
          "[RANGE_ALIGNER] rejected alignment: factors=%d span=%.1fs rms/sigma=%.2f (%s)",
          factor_count, last_stamp - first_stamp, normalized_rms,
          summary.BriefReport().c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (frozen_) return;
      transforms_ = transforms;
      valid_ = true;
    }
    publishStatus(true);
    ROS_INFO_THROTTLE(
        3.0,
        "[RANGE_ALIGNER] valid: factors=%d span=%.1fs normalized_rms=%.2f",
        factor_count, last_stamp - first_stamp, normalized_rms);
  }

  void initializeTransforms(
      const std::deque<TimedPose>& anchor,
      const std::map<std::string, std::deque<TimedPose>>& neighbors,
      const std::deque<TimedRange>& ranges,
      std::map<std::string, std::array<double, 4>>& transforms,
      double yaw_seed, double handedness) const {
    const TimedPose& anchor_pose = anchor.front();
    std::map<std::string, double> anchor_ranges;
    double first_pair_distance = -1.0;
    for (const auto& range : ranges) {
      if (range.a == anchor_name_) anchor_ranges.emplace(range.b, range.distance);
      if (range.b == anchor_name_) anchor_ranges.emplace(range.a, range.distance);
      if (neighbor_names_.size() >= 2 &&
          canonicalPair(range.a, range.b) ==
              canonicalPair(neighbor_names_[0], neighbor_names_[1]) &&
          first_pair_distance < 0.0)
        first_pair_distance = range.distance;
    }

    std::map<std::string, Eigen::Vector3d> local_triangle;
    if (neighbor_names_.size() >= 2 &&
        anchor_ranges.count(neighbor_names_[0]) &&
        anchor_ranges.count(neighbor_names_[1]) && first_pair_distance > 0.0) {
      const double d0 = anchor_ranges[neighbor_names_[0]];
      const double d1 = anchor_ranges[neighbor_names_[1]];
      const double x1 =
          (d1 * d1 + d0 * d0 - first_pair_distance * first_pair_distance) /
          (2.0 * d0);
      const double y1 =
          handedness * std::sqrt(std::max(0.0, d1 * d1 - x1 * x1));
      local_triangle[neighbor_names_[0]] = Eigen::Vector3d(d0, 0.0, 0.0);
      local_triangle[neighbor_names_[1]] = Eigen::Vector3d(x1, y1, 0.0);
    }

    const Eigen::AngleAxisd seed_rotation(yaw_seed,
                                          Eigen::Vector3d::UnitZ());
    for (size_t index = 0; index < neighbor_names_.size(); ++index) {
      const std::string& name = neighbor_names_[index];
      auto history = neighbors.find(name);
      if (history == neighbors.end() || history->second.empty()) continue;
      TimedPose raw;
      if (!nearest(history->second, anchor_pose.stamp, sync_tolerance_, raw))
        raw = history->second.front();
      const double radius =
          anchor_ranges.count(name) ? anchor_ranges[name] : 1.0;
      Eigen::Vector3d relative;
      auto triangle_point = local_triangle.find(name);
      if (triangle_point != local_triangle.end())
        relative = triangle_point->second;
      else {
        const double bearing =
            2.0 * M_PI * static_cast<double>(index) /
            static_cast<double>(std::max<size_t>(1, neighbor_names_.size()));
        relative =
            Eigen::Vector3d(radius * std::cos(bearing),
                            handedness * radius * std::sin(bearing), 0.0);
      }
      const Eigen::Vector3d guessed =
          anchor_pose.position + seed_rotation * relative;
      const double c = std::cos(yaw_seed);
      const double s = std::sin(yaw_seed);
      const Eigen::Vector3d rotated_raw(
          c * raw.position.x() - s * raw.position.y(),
          s * raw.position.x() + c * raw.position.y(), raw.position.z());
      transforms[name] = {{yaw_seed, guessed.x() - rotated_raw.x(),
                           guessed.y() - rotated_raw.y(),
                           anchor_pose.position.z() - raw.position.z()}};
    }
  }

  void publishAligned(const std::string& name, const TimedPose& raw,
                      const std::array<double, 4>& x) {
    const double c = std::cos(x[0]);
    const double s = std::sin(x[0]);
    const Eigen::Vector3d position(
        c * raw.position.x() - s * raw.position.y() + x[1],
        s * raw.position.x() + c * raw.position.y() + x[2],
        raw.position.z() + x[3]);
    const Eigen::Quaterniond yaw(
        Eigen::AngleAxisd(x[0], Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond orientation = yaw * raw.orientation;

    geometry_msgs::PoseStamped output;
    output.header.stamp.fromSec(raw.stamp);
    output.header.frame_id = global_frame_;
    output.pose.position.x = position.x();
    output.pose.position.y = position.y();
    output.pose.position.z = position.z();
    output.pose.orientation.w = orientation.w();
    output.pose.orientation.x = orientation.x();
    output.pose.orientation.y = orientation.y();
    output.pose.orientation.z = orientation.z();
    publishers_[name].publish(output);
  }

  void publishStatus(bool valid) {
    std_msgs::Bool message;
    message.data = valid;
    status_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber anchor_sub_, distance_sub_, restart_sub_;
  std::vector<ros::Subscriber> neighbor_subs_;
  std::map<std::string, ros::Publisher> publishers_;
  ros::Publisher status_pub_;
  ros::Timer timer_;

  std::string anchor_name_, global_frame_;
  std::vector<std::string> neighbor_names_, neighbor_topics_;
  double window_seconds_, sync_tolerance_, sample_interval_, range_sigma_;
  double z_prior_sigma_, solve_period_, min_span_, min_motion_,
      max_normalized_rms_;
  int min_factors_;

  std::mutex mutex_;
  std::deque<TimedPose> anchor_history_;
  std::map<std::string, std::deque<TimedPose>> neighbor_history_;
  std::deque<TimedRange> ranges_;
  std::map<std::string, std::array<double, 4>> transforms_;
  bool valid_ = false;
  bool frozen_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "range_frame_aligner");
  try {
    RangeFrameAligner node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("range_frame_aligner failed: %s", error.what());
    return 1;
  }
  return 0;
}
