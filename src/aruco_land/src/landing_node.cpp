#include <rclcpp/rclcpp.hpp>
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
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        arm_timer_ = this->create_wall_timer(
            1000ms, std::bind(&LandingNode::startOffboard, this)
        );

        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", qos,
            std::bind(&LandingNode::poseCallback, this, std::placeholders::_1)
        );
        
        /*
        mission_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mission/reached_final_waypoint", 10, 
            std::bind(&LandingNode::controlCallback, this, std::placeholders::_1)
        );
        */

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_status_sub_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr arm_timer_;
    rclcpp::TimerBase::SharedPtr timer_;

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
        publish_offboard_control_mode();

        bool is_valid_detection = false;
        
        if (marker_detected_) {
            auto time_now = this->now();
            auto time_diff = time_now - last_detection_time_;
            
            // Only consider it valid if seen within the last 0.5 seconds
            if (time_diff.seconds() < 0.5) {
                is_valid_detection = true;
            }
        }

        // 4. Calculate Setpoints based on validity
        TrajectorySetpoint setpoint{};
        setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        if (!is_valid_detection) {
            // Failsafe behavior: HOLD VELOCITY 0
            // We print only once every second to avoid flooding logs (optional improvement)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No Marker detected! Holding position.");
            
            setpoint.velocity[0] = 0.0;
            setpoint.velocity[1] = 0.0;
            setpoint.velocity[2] = 0.0; // Hold altitude
            setpoint.yawspeed = 0.0;
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

            setpoint.velocity[0] = vel_x; 
            setpoint.velocity[1] = vel_y;
            setpoint.velocity[2] = vel_z; 
            setpoint.yawspeed = 0.0;

            RCLCPP_INFO(get_logger(), "Aligning: Vx=%.2f Vy=%.2f Dist=%.2f", vel_x, vel_y, err_z);

            // Landing Trigger
            if (err_z < 0.6) { 
                RCLCPP_INFO(get_logger(), "Close enough! Triggering LAND mode.");
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
                timer_->cancel(); 
            }
        }

        trajectory_setpoint_publisher_->publish(setpoint);
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = false;      // WE ARE USING VELOCITY NOW
        msg.velocity = true;       // ENABLE VELOCITY CONTROL
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);     
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void startOffboard()
    {
        RCLCPP_INFO(this->get_logger(), "Sending ARM + OFFBOARD commands...");

        // 1. Arm
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

        // 2. Request OFFBOARD
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        // px4 mode 6 = offboard
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandingNode>());
    rclcpp::shutdown();
    return 0;
}