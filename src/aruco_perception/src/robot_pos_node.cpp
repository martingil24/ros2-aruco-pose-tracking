// Robot pose node: expresses every robot marker in the ground frame defined by
// the reference marker and publishes its 2D pose (x, y, heading) on /positions.
// Rotations are handled as rotation matrices (no quaternions).

#include <array>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "aruco_interfaces/msg/marker.hpp"
#include "aruco_interfaces/msg/marker_array.hpp"
#include "aruco_interfaces/msg/robot_pose2_d.hpp"
#include "aruco_interfaces/msg/robot_pose2_d_array.hpp"

using aruco_interfaces::msg::Marker;
using aruco_interfaces::msg::MarkerArray;
using aruco_interfaces::msg::RobotPose2D;
using aruco_interfaces::msg::RobotPose2DArray;

class RobotPosNode : public rclcpp::Node {
public:
  RobotPosNode() : Node("robot_pos_node") {
    reference_id_ = declare_parameter<int>("reference_id", 1);

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

    RCLCPP_INFO(get_logger(), "Robot pose node ready");
  }

private:
  // OpenCV rotation vector -> 3x3 rotation matrix
  static cv::Matx33d to_rotation_matrix(const std::array<double, 3> & rvec) {
    cv::Matx33d R;
    cv::Rodrigues(cv::Vec3d(rvec[0], rvec[1], rvec[2]), R);
    return R;
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

    RobotPose2DArray out;
    out.header = msg->header;
    out.header.frame_id = "ground";

    for (const auto & m : msg->markers) {
      if (m.id == reference_id_) {
        continue;  // the reference is not a robot
      }

      // Position: camera frame -> ground frame
      const cv::Vec3d t_robot(m.translation[0], m.translation[1], m.translation[2]);
      const cv::Vec3d p = R_ref_inv * (t_robot - t_ref);

      // Heading: robot marker X axis expressed in the ground frame
      const cv::Matx33d R_robot = to_rotation_matrix(m.rotation);
      const cv::Vec3d x_axis_cam(R_robot(0, 0), R_robot(1, 0), R_robot(2, 0));
      const cv::Vec3d x_axis_ground = R_ref_inv * x_axis_cam;

      RobotPose2D robot;
      robot.id = m.id;
      robot.x = p[0];
      robot.y = p[1];
      robot.heading = std::atan2(x_axis_ground[1], x_axis_ground[0]);
      out.robots.push_back(robot);
    }

    positions_pub_->publish(out);
  }

  int reference_id_;
  bool has_reference_ = false;
  Marker reference_;

  rclcpp::Subscription<Marker>::SharedPtr reference_sub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Publisher<RobotPose2DArray>::SharedPtr positions_pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPosNode>());
  rclcpp::shutdown();
  return 0;
}
