// Robot pose node: expresses every robot marker in the ground frame defined by
// the reference marker and publishes its 2D pose (x, y, heading):
//   - /positions_raw: all robots, unfiltered
//   - /positions: all robots, filtered
//   - /robot_<id>/pose: each robot's filtered pose on its own topic
// Rotations are handled as rotation matrices (no quaternions).
//
// Filter (exponential, per robot): filtered = alpha * previous + (1 - alpha) * new.
// The heading is filtered as a unit vector (cos, sin) so the +-pi wrap does not break it.

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "aruco_interfaces/msg/marker.hpp"
#include "aruco_interfaces/msg/marker_array.hpp"
#include "aruco_interfaces/msg/robot_pose2_d.hpp"
#include "aruco_interfaces/msg/robot_pose2_d_array.hpp"

using aruco_interfaces::msg::Marker;
using aruco_interfaces::msg::MarkerArray;
using aruco_interfaces::msg::RobotPose2D;
using aruco_interfaces::msg::RobotPose2DArray;
using geometry_msgs::msg::Pose2D;

class RobotPosNode : public rclcpp::Node {
public:
  RobotPosNode() : Node("robot_pos_node") {
    reference_id_ = declare_parameter<int>("reference_id", 1);
    // Marker IDs that belong to robots; any other ID is ignored (empty = accept all).
    declare_parameter<std::vector<int64_t>>("robot_ids", std::vector<int64_t>{});
    // Filter weight of the previous estimate: 0 = no filtering, closer to 1 = smoother but slower.
    declare_parameter<double>("alpha", 0.0);
    // If a robot is not seen for longer than this [s], its filter restarts from the new measurement.
    declare_parameter<double>("reset_timeout", 0.5);
    // All of them are read on every frame, so they can be changed live with `ros2 param set`.

    reference_sub_ = create_subscription<Marker>(
      "/reference_tf", 10,
      [this](const Marker::SharedPtr msg) {
        reference_ = *msg;
        has_reference_ = true;
      });
    markers_sub_ = create_subscription<MarkerArray>(
      "/aruco/markers", 10,
      std::bind(&RobotPosNode::markers_callback, this, std::placeholders::_1));
    positions_pub_ = create_publisher<RobotPose2DArray>("/positions", 10);
    positions_raw_pub_ = create_publisher<RobotPose2DArray>("/positions_raw", 10);

    RCLCPP_INFO(get_logger(), "Robot pose node ready");
  }

private:
  struct FilterState {
    double x, y, cos_h, sin_h;
    rclcpp::Time last_seen;
  };

  // OpenCV rotation vector -> 3x3 rotation matrix
  static cv::Matx33d to_rotation_matrix(const std::array<double, 3> & rvec) {
    cv::Matx33d R;
    cv::Rodrigues(cv::Vec3d(rvec[0], rvec[1], rvec[2]), R);
    return R;
  }

  // Publisher of /robot_<id>/pose, created the first time that robot is seen
  rclcpp::Publisher<Pose2D>::SharedPtr pose_publisher(int id) {
    auto it = pose_pubs_.find(id);
    if (it != pose_pubs_.end()) {
      return it->second;
    }
    const std::string topic = "/robot_" + std::to_string(id) + "/pose";
    auto pub = create_publisher<Pose2D>(topic, 10);
    pose_pubs_[id] = pub;
    RCLCPP_INFO(get_logger(), "Robot %d detected, publishing its pose on %s", id, topic.c_str());
    return pub;
  }

  // Exponential filter of one robot's pose
  RobotPose2D filter(const RobotPose2D & raw, double alpha, double reset_timeout) {
    const rclcpp::Time now = this->now();
    const double c = std::cos(raw.heading);
    const double s = std::sin(raw.heading);

    auto it = filters_.find(raw.id);
    if (it == filters_.end() || (now - it->second.last_seen).seconds() > reset_timeout) {
      filters_[raw.id] = FilterState{raw.x, raw.y, c, s, now};  // (re)start from this measurement
    } else {
      FilterState & f = it->second;
      f.x = alpha * f.x + (1.0 - alpha) * raw.x;
      f.y = alpha * f.y + (1.0 - alpha) * raw.y;
      f.cos_h = alpha * f.cos_h + (1.0 - alpha) * c;
      f.sin_h = alpha * f.sin_h + (1.0 - alpha) * s;
      f.last_seen = now;
    }

    const FilterState & f = filters_[raw.id];
    RobotPose2D out;
    out.id = raw.id;
    out.x = f.x;
    out.y = f.y;
    out.heading = std::atan2(f.sin_h, f.cos_h);
    return out;
  }

  void markers_callback(const MarkerArray::SharedPtr msg) {
    if (!has_reference_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /reference_tf...");
      return;
    }

    // Reference (ground) frame expressed in the camera frame
    const cv::Matx33d R_ref = to_rotation_matrix(reference_.rotation);
    const cv::Vec3d t_ref(reference_.translation[0], reference_.translation[1], reference_.translation[2]);
    const cv::Matx33d R_ref_inv = R_ref.t();  // inverse of a rotation = its transpose

    const auto robot_ids = get_parameter("robot_ids").as_integer_array();
    const double alpha = std::clamp(get_parameter("alpha").as_double(), 0.0, 0.99);
    const double reset_timeout = get_parameter("reset_timeout").as_double();

    RobotPose2DArray out, raw_out;
    out.header = msg->header;
    out.header.frame_id = "ground";
    raw_out.header = out.header;

    for (const auto & m : msg->markers) {
      if (m.id == reference_id_) {
        continue;  // the reference is not a robot
      }
      if (!robot_ids.empty() &&
          std::find(robot_ids.begin(), robot_ids.end(), m.id) == robot_ids.end()) {
        continue;  // not a known robot: false detection or unused marker
      }

      // Position: camera frame -> ground frame
      const cv::Vec3d t_robot(m.translation[0], m.translation[1], m.translation[2]);
      const cv::Vec3d p = R_ref_inv * (t_robot - t_ref);

      // Heading: robot marker X axis expressed in the ground frame
      const cv::Matx33d R_robot = to_rotation_matrix(m.rotation);
      const cv::Vec3d x_axis_cam(R_robot(0, 0), R_robot(1, 0), R_robot(2, 0));
      const cv::Vec3d x_axis_ground = R_ref_inv * x_axis_cam;

      RobotPose2D raw;
      raw.id = m.id;
      raw.x = p[0];
      raw.y = p[1];
      raw.heading = std::atan2(x_axis_ground[1], x_axis_ground[0]);
      raw_out.robots.push_back(raw);

      const RobotPose2D filtered = filter(raw, alpha, reset_timeout);
      out.robots.push_back(filtered);

      // Filtered pose on the robot's own topic
      Pose2D pose;
      pose.x = filtered.x;
      pose.y = filtered.y;
      pose.theta = filtered.heading;
      pose_publisher(filtered.id)->publish(pose);
    }

    positions_raw_pub_->publish(raw_out);
    positions_pub_->publish(out);
  }

  int reference_id_;
  bool has_reference_ = false;
  Marker reference_;

  rclcpp::Subscription<Marker>::SharedPtr reference_sub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Publisher<RobotPose2DArray>::SharedPtr positions_pub_;
  rclcpp::Publisher<RobotPose2DArray>::SharedPtr positions_raw_pub_;
  std::map<int, rclcpp::Publisher<Pose2D>::SharedPtr> pose_pubs_;
  std::map<int, FilterState> filters_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPosNode>());
  rclcpp::shutdown();
  return 0;
}
