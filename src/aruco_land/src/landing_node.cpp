// safe_landing_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <limits>
#include <math.h>

using namespace std::chrono_literals;

class SafeLandingNode : public rclcpp::Node
{
public:
    SafeLandingNode() : Node("safe_landing_node")
    {
        start_land = false;
        land_sent_ = false;

        auto qos = rclcpp::QoS(10).best_effort();

        pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ext_setpoint/pos", 10);
        land_pub_ = this->create_publisher<std_msgs::msg::Bool>("/land_command", 10);

        start_presland_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/land_command/pres_land", 10,
            std::bind(&SafeLandingNode::startPresLandCallback, this, std::placeholders::_1)
        );

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose",
            qos,
            std::bind(&SafeLandingNode::poseCallback, this, std::placeholders::_1)
        );

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&SafeLandingNode::odomCallback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(50ms, std::bind(&SafeLandingNode::timerCallback, this));

        // defaults
        err_x = err_y = err_z = 1000.0f;
        last_valid_pose_time_ = this->now() - rclcpp::Duration::from_seconds(10.0); // mark old initially

        RCLCPP_INFO(this->get_logger(), "SafeLandingNode started");
    }

private:
    // state
    bool start_land;
    bool land_sent_;

    // pose and odom
    float err_x{1000.0f}, err_y{1000.0f}, err_z{1000.0f};
    float current_yaw_{0.0f};
    rclcpp::Time last_valid_pose_time_;
    float target_x, target_y, target_z;

    // publishers/subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_presland_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time current_time_;

    // safety params
    const double POSE_TIMEOUT_S = 0.5;   // how long a pose is considered fresh
    const float MAX_HORIZ_VEL = 0.6f;    // m/s (very conservative)
    const float DESCENT_VEL = -0.2f;     // m/s (negative = down in NED)
    const float LAND_TRIGGER_Z = 0.6f;   // depth at which to trigger nav-land (meters)
    float current_x, current_y, current_z;
    bool marker_visible_;
    

    void startPresLandCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Received pres_land -> starting precision landing");
            start_land = true;
            land_sent_ = false;
            last_valid_pose_time_ = this->now();
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                     "Vision received: err = (%.2f, %.2f, %.2f)", err_x, err_y, err_z);
        last_valid_pose_time_ = this->now();
        err_x = -msg->pose.position.x;
        err_y = -msg->pose.position.y;
        err_z = -msg->pose.position.z;
    }

    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // extract yaw from quaternion
        double qw = msg->q[0];
        double qx = msg->q[1];
        double qy = msg->q[2];
        double qz = msg->q[3];
        current_x = msg->position[0];
        current_y = msg->position[1];
        current_z = msg->position[2];
        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    void timerCallback()
    {   
        if (!start_land) {
            return; // not started yet
        }
        
        float dx_world = err_x * std::cos(current_yaw_) - err_y * std::sin(current_yaw_);
        float dy_world = err_x * std::sin(current_yaw_) + err_y * std::cos(current_yaw_);
        RCLCPP_INFO(this->get_logger(),
                     "Vision in world frame: dx = %.2f, dy = %.2f", dx_world, dy_world);
        float max_step = 0.2f; // meters per cycle
        dx_world = std::clamp(dx_world, -max_step, max_step);
        dy_world = std::clamp(dy_world, -max_step, max_step);
        float k = 0.1f; 
        dx_world = k * dx_world;
        dy_world = k * dy_world;


        
        target_x = current_x + dx_world; 
        target_y = current_y + dy_world;
        target_z = current_z + 0.1f; 
        RCLCPP_INFO(this->get_logger(),
                     "target setpoint: x = %.2f, y = %.2f, z = %.2f", target_x, target_y, target_z);
        geometry_msgs::msg::PoseStamped pos_setpoint;
        pos_setpoint.header.stamp = this->now();
        pos_setpoint.pose.position.x = target_x;
        pos_setpoint.pose.position.y = target_y;
        pos_setpoint.pose.position.z = target_z;
        double yaw_ned = -current_yaw_;
        setYawInPose(pos_setpoint.pose, yaw_ned);
        pos_pub_->publish(pos_setpoint);

        if (-current_z <= LAND_TRIGGER_Z && !land_sent_) {
            RCLCPP_INFO(this->get_logger(), "Triggering landing command at z=%.2f m", current_z);
            std_msgs::msg::Bool land_msg;
            land_msg.data = true;
            land_pub_->publish(land_msg);
            RCLCPP_INFO(this->get_logger(), "Landing command sent");
            land_sent_ = true;
            timer_->cancel();
        }
    }

    void setYawInPose(geometry_msgs::msg::Pose &pose, double yaw_ned)
    {
        double cy = std::cos(yaw_ned * 0.5);
        double sy = std::sin(yaw_ned * 0.5);

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = sy;
        pose.orientation.w = cy;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeLandingNode>());
    rclcpp::shutdown();
    return 0;
}
