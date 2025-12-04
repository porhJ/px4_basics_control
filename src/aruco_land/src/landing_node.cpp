#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class LandingNode : public rclcpp::Node
{
public:
    LandingNode() : Node("landing_node")
    {   
        mission_node_stopped = false;
        marker_detected_ = false;
        start_land = false;
        auto qos = rclcpp::QoS(1).best_effort();
        // Publishers
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ext_setpoint/vel", 10);
        pos_publiser_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ext_setpoint/pos", 10);
        land_ = this->create_publisher<std_msgs::msg::Bool>("/land_command", 10);

        // Subscribers
        start_presland_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/land_command/pres_land", 10,
            std::bind(&LandingNode::startPresLandCallback, this, std::placeholders::_1)
        );
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", qos,
            std::bind(&LandingNode::poseCallback, this, std::placeholders::_1)
        );
        

        // Main Control Loop Timer (Runs at 20Hz)
        timer_ = this->create_wall_timer(50ms, std::bind(&LandingNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Landing Node started. Waiting for mission node...");
    }

private:
    bool mission_node_stopped;
    bool marker_detected_;
    bool start_land;
    float last_vel_x = 0.0f;
    float last_vel_y = 0.0f;
    float last_vel_z = 0.0f;
    rclcpp::Time last_valid_pose_time_;
    
    
    // Last known marker position relative to camera
    float err_x, err_y, err_z;
    rclcpp::Time last_detection_time_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_publiser_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_presland_sub_;

    // Receive detection from Vision Node
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_detection_time_ = this->now();
        marker_detected_ = true;

        err_x = msg->pose.position.x;
        err_y = msg->pose.position.y;
        err_z = msg->pose.position.z;

        last_valid_pose_time_ = this->now();
    }

    void timerCallback()
    {
        if (!start_land) return;
        RCLCPP_INFO_ONCE(get_logger(), "Precision landing sequence initiated.");

        bool seen_recent = (this->now() - last_valid_pose_time_).seconds() < 0.5;

        geometry_msgs::msg::TwistStamped vel_msg{};
        vel_msg.header.stamp = now();

        if (seen_recent) {
            float k_p = 0.6f; // reduce gain to be less oscillatory
            float vel_x = k_p * err_y;
            float vel_y = k_p * err_x;
            float vel_z = -0.2f; // descent (NED: negative down)

            // clamp...
            last_vel_x = vel_x; last_vel_y = vel_y; last_vel_z = vel_z;
        } else {
            // fallback: send last good velocity but reduce magnitude (keeps control stable)
            last_vel_x *= 0.6f;
            last_vel_y *= 0.6f;
            // keep descent so we don't hover back to mission
            last_vel_z = -0.2f;
        }

        vel_msg.twist.linear.x = last_vel_x;
        vel_msg.twist.linear.y = last_vel_y;
        vel_msg.twist.linear.z = last_vel_z;

        velocity_publisher_->publish(vel_msg);

        // Only trigger nav-land when close
        if (err_z < 0.6) {
            std_msgs::msg::Bool msg; msg.data = true;
            RCLCPP_INFO(get_logger(), "Close enough! Triggering LAND mode.");
            land_->publish(msg);
            timer_->cancel();
        }
    }

    void startPresLandCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(get_logger(), "Precision landing sequence started.");
            start_land = true;
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandingNode>());
    rclcpp::shutdown();
    return 0;
}