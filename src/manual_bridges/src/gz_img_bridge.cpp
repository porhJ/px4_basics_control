#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gz/transport13/gz/transport/Node.hh>
#include <gz/msgs9/gz/msgs/image.pb.h>

class ManualImageBridge : public rclcpp::Node
{
public:
  ManualImageBridge()
  : Node("manual_gz_image_bridge")
  {
    auto qos = rclcpp::QoS(1).best_effort();
    ros_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);

    std::string gz_topic =
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image";

    if (!gz_node_.Subscribe(gz_topic, &ManualImageBridge::gzCallback, this))
    {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to Gazebo topic: %s", gz_topic.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Subscribed to Gazebo topic: %s", gz_topic.c_str());
    }
  }

private:
  void gzCallback(const gz::msgs::Image &msg)
  {
    sensor_msgs::msg::Image ros_msg;

    ros_msg.header.stamp = now();
    ros_msg.header.frame_id = "camera";
    ros_msg.width = msg.width();
    ros_msg.height = msg.height();
    ros_msg.step = msg.step();
    ros_msg.encoding = "rgb8";

    ros_msg.data.assign(msg.data().begin(), msg.data().end());

    ros_pub_->publish(ros_msg);
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualImageBridge>());
  rclcpp::shutdown();
  return 0;
}
