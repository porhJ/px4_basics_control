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

class MissionNode : public rclcpp::Node
{
public:
    MissionNode() : Node("mission_node")
    {
       // Publishers
        pos_publiser_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ext_setpoint/pos", 10);
        start_presland_sub_ = this->create_publisher<std_msgs::msg::Bool>("/land_command/pres_land", 10);

        // Subscribers
        pos_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&MissionNode::pos_callback, this, std::placeholders::_1)
        );

        float tmp[2][3] = {
        {0.0, 0.0,  -5.0},
        {10.0, 10.0, -5.0}
        };
        memcpy(waypoints, tmp, sizeof(waypoints));

        num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
        STATE_ = 0; // current waypoint
        target_pos_.pose.position.x = waypoints[STATE_][0];
        target_pos_.pose.position.y = waypoints[STATE_][1];
        target_pos_.pose.position.z = waypoints[STATE_][2];
        

        // Main Control Loop Timer (Runs at 20Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&MissionNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Mission Node started...");
    }

private:
    float waypoints[2][3];
    int num_waypoints;
    int STATE_;
    float altitude_;
    float distance_;
    float local_pos[3];

    rclcpp::Time last_detection_time_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_publiser_;
    geometry_msgs::msg::PoseStamped target_pos_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_presland_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr pos_;

    void pos_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        // PX4 NED coordinates: z is negative when above ground.
        local_pos[0] = msg->position[0];
        local_pos[1] = msg->position[1];
        local_pos[2] = msg->position[2];
        altitude_ = local_pos[2];
        

    }

    void timerCallback()
    {   
        if (STATE_ < num_waypoints) {
            // Update target from waypoint list
            target_pos_.pose.position.x = waypoints[STATE_][0];
            target_pos_.pose.position.y = waypoints[STATE_][1];
            target_pos_.pose.position.z = waypoints[STATE_][2];
        }
        geometry_msgs::msg::PoseStamped pos_setpoint;
        pos_setpoint.header.stamp = this->now();
        pos_setpoint.pose = target_pos_.pose;
        pos_publiser_->publish(pos_setpoint);

        RCLCPP_INFO(this->get_logger(), "Current State: %i, Current waypoint: (%.2f, %.2f, %.2f)", STATE_, target_pos_.pose.position.x, target_pos_.pose.position.y, target_pos_.pose.position.z);

        distance_ = this->cal_distance(local_pos, waypoints[STATE_]);
        if (STATE_ < num_waypoints) {
            if (distance_ <= 0.1) {
                RCLCPP_INFO(this->get_logger(), "Reached the #%i waypoint: (%.2f, %.2f, %.2f)", STATE_, target_pos_.pose.position.x, target_pos_.pose.position.y, target_pos_.pose.position.z);
                STATE_++;
            }
        } 
        else { // if it reaches the final goal
            RCLCPP_INFO(this->get_logger(), "Reached the final waypoint: (%.2f, %.2f, %.2f)", target_pos_.pose.position.x, target_pos_.pose.position.y, target_pos_.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "The vehicle starts landing by using aruco landing node");
            // this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            std_msgs::msg::Bool msg;
            msg.data = true;
            start_presland_sub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published pre-landing command.");
            pos_publiser_.reset();
            timer_->cancel();
        }
        
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
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}