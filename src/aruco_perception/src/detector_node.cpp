#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// NUEVAS LIBRERÍAS TF2
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class ArucoDetector : public rclcpp::Node {
public:
    ArucoDetector() : Node("aruco_detector_node") {
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));
            
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco/image_out", 10);
        
        // Inicializar el emisor de transformaciones TF2
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "ArUco TF2 Broadcaster initialized...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(10, 4, 0);
            
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
            
            if (!markerIds.empty()) {
                cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                
                cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 800.0, 0, 320.0, 0, 800.0, 240.0, 0, 0, 1);
                cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
                
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.10f, cameraMatrix, distCoeffs, rvecs, tvecs);
                
                for (size_t i = 0; i < markerIds.size(); ++i) {
                    cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05f);
                    
                    // --- LÓGICA TF2 ---
                    geometry_msgs::msg::TransformStamped t;
                    t.header.stamp = this->now();
                    t.header.frame_id = "camera_frame";
                    // Creamos un nombre dinámico (ej: "marker_0", "marker_45")
                    t.child_frame_id = "marker_" + std::to_string(markerIds[i]); 
                    
                    // 1. Traslación (Posición X, Y, Z)
                    t.transform.translation.x = tvecs[i][0];
                    t.transform.translation.y = tvecs[i][1];
                    t.transform.translation.z = tvecs[i][2];
                    
                    // 2. Rotación: Convertir el vector 'rvec' de OpenCV a una Matriz de 3x3
                    cv::Mat rot_mat;
                    cv::Rodrigues(rvecs[i], rot_mat);
                    
                    // 3. Pasar la matriz de OpenCV a TF2
                    tf2::Matrix3x3 tf_rot(
                        rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                        rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                        rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)
                    );
                    
                    // 4. Extraer el Cuaternión (X, Y, Z, W)
                    tf2::Quaternion q;
                    tf_rot.getRotation(q);
                    
                    t.transform.rotation.x = q.x();
                    t.transform.rotation.y = q.y();
                    t.transform.rotation.z = q.z();
                    t.transform.rotation.w = q.w();
                    
                    // Publicar la transformación en el árbol de ROS 2
                    tf_broadcaster_->sendTransform(t);
                    
                    RCLCPP_INFO(this->get_logger(), "TF Broadcasted: camera_frame -> marker_%d", markerIds[i]);
                }
            }
            
            image_publisher_->publish(*cv_ptr->toImageMsg());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // Puntero al emisor de TF
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
