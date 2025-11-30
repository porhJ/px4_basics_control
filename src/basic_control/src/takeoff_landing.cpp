#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <cmath>

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
        mission_status_ = this->create_publisher<std_msgs::msg::Bool>("/mission/reached_final_waypoint", 10);
        pos_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&takeoffLandingNode::pos_callback, this, std::placeholders::_1)
        );

        float tmp[2][3] = {
        {0.0, 0.0,  0.0},
        {10.0,-10.0, -10.0}
        };
        memcpy(waypoints, tmp, sizeof(waypoints));

        num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
 
        STATE_ = 0; // current waypoint
        offboard_setpoint_counter_ = 0;
        target_pos_.position[0] = waypoints[STATE_][0];
        target_pos_.position[1] = waypoints[STATE_][1];
        target_pos_.position[2] = waypoints[STATE_][2];
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_status_;

    px4_msgs::msg::TrajectorySetpoint target_pos_;
    float altitude_;
    int STATE_;
    float target_alt;
    float upward_vel;
    float local_pos[3];
    float local_vel[3];
    float waypoints[2][3];
    int num_waypoints;
    float distance_;

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped

    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

    void pos_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        // PX4 NED coordinates: z is negative when above ground.
        local_pos[0] = msg->position[0];
        local_pos[1] = msg->position[1];
        local_pos[2] = msg->position[2];
        altitude_ = local_pos[2];
        

    }

    void takeoffLanding() {
        publish_offboard_control_mode();
        publish_trajectory_setpoint(target_pos_);

        if (STATE_ == 0 && offboard_setpoint_counter_ >= 50) {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            RCLCPP_INFO(this->get_logger(), "Offboard mode started");

            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            RCLCPP_INFO(this->get_logger(), "The vehicle is armed");

            STATE_++;
            RCLCPP_INFO(this->get_logger(), "Current target_position: (%.2f, %.2f, %.2f)", target_pos_.position[0], target_pos_.position[1], target_pos_.position[2]);
        } 
        RCLCPP_INFO(this->get_logger(), "Current State: %i, Current waypoint: (%.2f, %.2f, %.2f)", STATE_, target_pos_.position[0], target_pos_.position[1], target_pos_.position[2]);
        target_pos_.position[0] = waypoints[STATE_][0];
        target_pos_.position[1] = waypoints[STATE_][1];
        target_pos_.position[2] = waypoints[STATE_][2];

        distance_ = this->cal_distance(local_pos, waypoints[STATE_]);
        if (STATE_ > 0 && STATE_ < num_waypoints) {
            if (distance_ <= 0.01) {
            RCLCPP_INFO(this->get_logger(), "Reached the #%i waypoint: (%.2f, %.2f, %.2f)", STATE_, target_pos_.position[0], target_pos_.position[1], target_pos_.position[2]);
            STATE_++;
            }
        } 
        else if (STATE_ == num_waypoints) { // if it reaches the final goal
            RCLCPP_INFO(this->get_logger(), "Reached the final waypoint: (%.2f, %.2f, %.2f)", target_pos_.position[0], target_pos_.position[1], target_pos_.position[2]);
            RCLCPP_INFO(this->get_logger(), "The vehicle starts landing by using aruco landing node");
            publish_mission_status();
            // this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            timer_->cancel();
        }
        
        
        
        offboard_setpoint_counter_++;
    }


    void publish_mission_status()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        mission_status_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publish that we have reached the last waypoint.");
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

    float cal_distance(float current_pos[3], float target_pos[3]) {
        float dx = current_pos[0] - target_pos[0];
        float dy = current_pos[1] - target_pos[1];
        float dz = current_pos[2] - target_pos[2];
        float distance =  sqrt(pow(dx, 2)+ pow(dy, 2) + pow(dz, 2));
        return distance;
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