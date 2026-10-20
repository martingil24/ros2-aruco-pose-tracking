// Detector node: detects ArUco markers in the camera image and publishes
// the ID and pose (camera frame) of every marker on /aruco/markers.

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "aruco_interfaces/msg/marker.hpp"
#include "aruco_interfaces/msg/marker_array.hpp"

using aruco_interfaces::msg::Marker;
using aruco_interfaces::msg::MarkerArray;

class ArucoDetector : public rclcpp::Node {
public:
  ArucoDetector() : Node("aruco_detector_node") {
    // Parameters (can be changed at launch time without recompiling)
    reference_id_   = declare_parameter<int>("reference_id", 1);
    reference_size_ = declare_parameter<double>("reference_marker_size", 0.195);  // metres
    robot_size_     = declare_parameter<double>("robot_marker_size", 0.10);       // metres

    // The lab markers use a custom dictionary: 10 markers, 4x4 bits, seed 0
    dictionary_ = cv::aruco::generateCustomDictionary(10, 4, 0);

    // Placeholder camera intrinsics (to be replaced by a real calibration)
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 800.0, 0, 320.0, 0, 800.0, 240.0, 0, 0, 1);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10,
      std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));
    image_pub_   = create_publisher<sensor_msgs::msg::Image>("/aruco/image_out", 10);
    markers_pub_ = create_publisher<MarkerArray>("/aruco/markers", 10);

    RCLCPP_INFO(get_logger(), "ArUco detector ready (reference ID %d)", reference_id_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat & frame = cv_ptr->image;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids);

    MarkerArray markers_msg;
    markers_msg.header = msg->header;  // same timestamp as the image

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);

      for (size_t i = 0; i < ids.size(); ++i) {
        // Each marker type has a different physical size
        double size = (ids[i] == reference_id_) ? reference_size_ : robot_size_;

        std::vector<std::vector<cv::Point2f>> single = {corners[i]};
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(single, size, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvecs[0], tvecs[0], size * 0.5);

        Marker m;
        m.id = ids[i];
        for (int k = 0; k < 3; ++k) {
          m.translation[k] = tvecs[0][k];
          m.rotation[k] = rvecs[0][k];
        }
        markers_msg.markers.push_back(m);
      }
    }

    markers_pub_->publish(markers_msg);
    image_pub_->publish(*cv_ptr->toImageMsg());
  }

  int reference_id_;
  double reference_size_;
  double robot_size_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}
