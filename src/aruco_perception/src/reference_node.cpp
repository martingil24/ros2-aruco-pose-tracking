// Reference node: picks the floor reference marker out of the detections and
// publishes its pose on /reference_tf. The reference marker lies flat on the
// floor, so its plane defines the ground plane (its Z axis is the floor normal).
// The last known pose is kept, so a robot briefly covering the marker does not
// break the system.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "aruco_interfaces/msg/marker.hpp"
#include "aruco_interfaces/msg/marker_array.hpp"

using aruco_interfaces::msg::Marker;
using aruco_interfaces::msg::MarkerArray;

class ReferenceNode : public rclcpp::Node {
public:
  ReferenceNode() : Node("reference_node") {
    reference_id_ = declare_parameter<int>("reference_id", 1);

    markers_sub_ = create_subscription<MarkerArray>(
      "/aruco/markers", 10,
      std::bind(&ReferenceNode::markers_callback, this, std::placeholders::_1));
    reference_pub_ = create_publisher<Marker>("/reference_tf", 10);

    RCLCPP_INFO(get_logger(), "Reference node ready (reference ID %d)", reference_id_);
  }

private:
  void markers_callback(const MarkerArray::SharedPtr msg) {
    for (const auto & m : msg->markers) {
      if (m.id == reference_id_) {
        if (!has_reference_) {
          RCLCPP_INFO(get_logger(), "Reference marker found, ground plane defined");
        }
        reference_ = m;
        has_reference_ = true;
      }
    }

    if (has_reference_) {
      reference_pub_->publish(reference_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Reference marker %d not seen yet", reference_id_);
    }
  }

  int reference_id_;
  bool has_reference_ = false;
  Marker reference_;

  rclcpp::Subscription<MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Publisher<Marker>::SharedPtr reference_pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceNode>());
  rclcpp::shutdown();
  return 0;
}
