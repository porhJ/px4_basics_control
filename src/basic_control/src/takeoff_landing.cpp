#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class takeoffLandingNode : public rclcpp::Node
{
public:
    takeoffLandingNode() : Node("takeoffLanding")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        pos_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&takeoffLandingNode::pos_callback, this, std::placeholders::_1)
        );
        /*
        STATE
        0: ARMING
        1: TAKEOFF
        2: LANDING
        */

        STATE_ = 0;
        offboard_setpoint_counter_ = 0;
        target_pos_.position = {0.0, 0.0, 0.0};
        target_pos_.yaw = -3.14;
        timer_ = this->create_wall_timer(100ms, std::bind(&takeoffLandingNode::takeoffLanding, this));
        
        RCLCPP_INFO(this->get_logger(), "takeoffLandingNode has started");
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr pos_;
    
    px4_msgs::msg::TrajectorySetpoint target_pos_;
    float altitude_;
    int STATE_;
    float target_alt;
    float upward_vel;

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped

    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

    void pos_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        // PX4 NED coordinates: z is negative when above ground.
        /*
        float x = msg->position[0];
        float y = msg->position[1];
        */
        float z = msg->position[2];
        float vz = msg->velocity[0];
        altitude_ = -z;
        upward_vel = -vz;
        

    }

    void takeoffLanding() {
        publish_offboard_control_mode();
        publish_trajectory_setpoint(target_pos_);

        if (STATE_ == 0 && offboard_setpoint_counter_ >= 50) {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            RCLCPP_INFO(this->get_logger(), "Offboard mode started");

            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            RCLCPP_INFO(this->get_logger(), "The vehicle is armed");

            STATE_ = 1;
            target_pos_.position = {0.0, 0.0, -10.0};
            RCLCPP_INFO(this->get_logger(), "Current target_position: (%.2f, %.2f, %.2f)", target_pos_.position[0], target_pos_.position[1], target_pos_.position[2]);
        } 
        else if (STATE_ == 1 && upward_vel == 0) {
            RCLCPP_INFO(this->get_logger(), "The vehicle reached the desired altitude");
            STATE_ = 2;
            target_pos_.position = {0.0, 0.0, 0.0};
            RCLCPP_INFO(this->get_logger(), "The vehicle is landing");
        } else if (STATE_ == 2 && upward_vel == 0) {
            RCLCPP_INFO(this->get_logger(), "The vehicle is landed");
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        }


        offboard_setpoint_counter_++;
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint(px4_msgs::msg::TrajectorySetpoint &msg)
    {
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<takeoffLandingNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}