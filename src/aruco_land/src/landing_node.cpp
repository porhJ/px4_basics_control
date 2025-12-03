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
        auto qos = rclcpp::QoS(1).best_effort();
        // Publishers
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ext_setpoint/vel", 10);
        pos_publiser_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ext_setpoint/pos", 10);
        land_ = this->create_publisher<std_msgs::msg::Bool>("/land_command", 10);

        // Subscribers
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
    
    
    // Last known marker position relative to camera
    float err_x, err_y, err_z;
    rclcpp::Time last_detection_time_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_publiser_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_;

    // Receive detection from Vision Node
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_detection_time_ = this->now();
        marker_detected_ = true;

        // Vision Coordinate (Camera Frame)
        // Usually for Down Facing Camera: X=Right, Y=Down(Forward in image), Z=Depth
        // We need to map this to Drone Body Frame: X=Forward, Y=Right, Z=Down
        // Save the error to use in the timer loop
        err_x = msg->pose.position.x; 
        err_y = msg->pose.position.y;
        err_z = msg->pose.position.z;
    }

    void timerCallback()
    {
        bool is_valid_detection = false;
        
        if (marker_detected_) {
            auto time_now = this->now();
            auto time_diff = time_now - last_detection_time_;
            
            // Only consider it valid if seen within the last 0.5 seconds
            if (time_diff.seconds() < 0.5) {
                is_valid_detection = true;
            }
        }

        geometry_msgs::msg::TwistStamped vel_msg{};
        
        vel_msg.header.stamp = now();

        if (!is_valid_detection) {
            return;
        } 
        else {
            // VISUAL SERVOING LOGIC
            float k_p = 1.0; 
            
            // Camera Frame -> Body Frame Mapping
            // Cam X (Right) -> Body Y (Right)
            // Cam Y (Down)  -> Body X (Forward)
            
            float vel_x = k_p * err_y; // Move Forward/Back
            float vel_y = k_p * err_x; // Move Left/Right
            float vel_z = 0.2;         // Descend speed

            // Clamp velocities
            if (vel_x > 1.0) vel_x = 1.0; if (vel_x < -1.0) vel_x = -1.0;
            if (vel_y > 1.0) vel_y = 1.0; if (vel_y < -1.0) vel_y = -1.0;

            vel_msg.twist.linear.x = vel_x; 
            vel_msg.twist.linear.y = vel_y;
            vel_msg.twist.linear.z = vel_z; 

            RCLCPP_INFO(get_logger(), "Aligning: Vx=%.2f Vy=%.2f Dist=%.2f", vel_x, vel_y, err_z);

            // Landing Trigger
            if (err_z < 0.6) { 
                std_msgs::msg::Bool msg;
                msg.data = true;
                RCLCPP_INFO(get_logger(), "Close enough! Triggering LAND mode.");
                land_->publish(msg);
                timer_->cancel(); 
            }
        }

        velocity_publisher_->publish(vel_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandingNode>());
    rclcpp::shutdown();
    return 0;
}