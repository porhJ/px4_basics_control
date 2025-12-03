#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/camera_info.pb.h>

class CameraInfoBridge : public rclcpp::Node
{
public:
  CameraInfoBridge() : Node("gz_caminfo_bridge")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS());

    gz_node_.Subscribe(
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info",
      &CameraInfoBridge::gzCallback, this);

    RCLCPP_INFO(this->get_logger(), "CameraInfo bridge running");
  }

private:
  void gzCallback(const gz::msgs::CameraInfo &msg)
  {
    sensor_msgs::msg::CameraInfo ros;

    ros.header.stamp = this->now();
    ros.header.frame_id = "camera_link";

    ros.width = msg.width();
    ros.height = msg.height();

    // -----------------------------
    // DISTORTION
    // -----------------------------
    ros.distortion_model = "plumb_bob";
    ros.d.clear();
    for (int i = 0; i < msg.distortion().k_size(); i++)
      ros.d.push_back(msg.distortion().k(i));

    // -----------------------------
    // INTRINSICS → K matrix
    // -----------------------------
    for (int i = 0; i < msg.intrinsics().k_size() && i < 9; i++)
      ros.k[i] = msg.intrinsics().k(i);

    // -----------------------------
    // RECTIFICATION → R matrix
    // -----------------------------
    for (int i = 0; i < msg.rectification_matrix_size() && i < 9; i++)
      ros.r[i] = msg.rectification_matrix(i);

    // -----------------------------
    // PROJECTION → P matrix
    // -----------------------------
    for (int i = 0; i < msg.projection().p_size() && i < 12; i++)
      ros.p[i] = msg.projection().p(i);

    pub_->publish(ros);
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoBridge>());
  rclcpp::shutdown();
  return 0;
}
