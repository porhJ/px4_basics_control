/*
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoVisionNode : public rclcpp::Node
{
public:
    ArucoVisionNode()
        : Node("aruco_vision_node")
    {
        auto qos = rclcpp::QoS(1).best_effort();

        // Set parameters
        declare_parameter("marker_id", 0);
        declare_parameter("marker_size", 0.20); // meters
        declare_parameter("fx", 500.0);
        declare_parameter("fy", 500.0);
        declare_parameter("cx", 320.0);
        declare_parameter("cy", 240.0);

        get_parameter("marker_id", marker_id_);
        get_parameter("marker_size", marker_size_);
        get_parameter("fx", fx_);
        get_parameter("fy", fy_);
        get_parameter("cx", cx_);
        get_parameter("cy", cy_);

        // Publisher for marker pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/vision/aruco_pose", qos);

        // Subscribe to camera
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", qos, std::bind(&ArucoVisionNode::imageCallback, this, std::placeholders::_1));

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        RCLCPP_INFO(this->get_logger(), "Aruco Vision Node started.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge failed.");
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, dictionary_, corners, ids);

        if (ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "Not detecting any arucomark");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Detecting");
        }

        // Find our desired marker ID
        for (size_t i = 0; i < ids.size(); i++)
        {
            if (ids[i] == marker_id_)
            {
                // Camera matrix
                cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx_, 0, cx_,
                                                           0, fy_, cy_,
                                                           0, 0, 1);
                cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

                cv::Vec3d rvec, tvec;
                cv::aruco::estimatePoseSingleMarkers(
                    std::vector<std::vector<cv::Point2f>>{corners[i]},
                    marker_size_, cameraMatrix, distCoeffs, rvec, tvec
                );

                // Publish pose
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = now();
                pose_msg.header.frame_id = "camera_link";

                pose_msg.pose.position.x = tvec[0];
                pose_msg.pose.position.y = tvec[1];
                pose_msg.pose.position.z = tvec[2];

                // Convert rotation to quaternion
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                cv::Mat rot;
                cv::Rodrigues(rvec, rot);
                cv::Vec4d q = rotationMatrixToQuaternion(R);

                pose_msg.pose.orientation.x = q[0];
                pose_msg.pose.orientation.y = q[1];
                pose_msg.pose.orientation.z = q[2];
                pose_msg.pose.orientation.w = q[3];

                pose_pub_->publish(pose_msg);

                return;
            }
        }
    }

    // Convert rotation matrix → quaternion
    cv::Vec4d rotationMatrixToQuaternion(const cv::Mat &R)
    {
        cv::Vec4d q;
        double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);

        if (trace > 0.0) {
            double s = sqrt(trace + 1.0) * 2.0;
            q[3] = 0.25 * s;
            q[0] = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
            q[1] = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
            q[2] = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
        }
        return q;
    }

    int marker_id_;
    double marker_size_;
    double fx_, fy_, cx_, cy_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoVisionNode>());
    rclcpp::shutdown();
    return 0;
}
*/