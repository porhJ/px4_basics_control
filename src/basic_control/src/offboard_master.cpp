#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <std_msgs/msg/bool.hpp>


using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class OffboardMaster : public rclcpp::Node
{
public:
    OffboardMaster() : Node("offboard_master")
    {
        // Publishers
        offboard_pub_   = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_pub_       = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        cmd_pub_        = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        last_pos_time_ = this->now();
        last_vel_time_ = this->now();
        offboard_setpoint_counter_ = 0;

        ext_land_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/land_command", 10,
            std::bind(&OffboardMaster::landCallback, this, std::placeholders::_1)
        );
        // External setpoint subscriber
        ext_pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ext_setpoint/pos", 10,
            std::bind(&OffboardMaster::externalSetpointPos, this, std::placeholders::_1)
        );

        ext_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/ext_setpoint/vel", 10,
            std::bind(&OffboardMaster::externalSetpointVel, this, std::placeholders::_1)
        );

        landed_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardMaster::landedCallback, this, std::placeholders::_1)
        );


        // Timer (20 Hz)
        timer_ = create_wall_timer(50ms, std::bind(&OffboardMaster::loop, this));

        // Start after 1 second (arm + offboard)
        start_timer_ = create_wall_timer(1000ms, std::bind(&OffboardMaster::startOffboard, this));

        RCLCPP_INFO(get_logger(), "Offboard Master Started");
    }

private:

    bool ignore_position_setpoints_ = false;
    bool landing_in_progress_ = false;
    bool offboard_active = true;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ext_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ext_vel_sub_;
    uint64_t offboard_setpoint_counter_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::Time last_pos_time_;
    rclcpp::Time last_vel_time_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ext_land_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landed_sub_;

    float hover_x_ = 0.0, hover_y_ = 0.0, hover_z_ = -2.0; // 2m altitude

    void externalSetpointPos(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_pos_time_ = now();

        external_x_ = msg->pose.position.x;
        external_y_ = msg->pose.position.y;
        external_z_ = msg->pose.position.z;
    }

    void externalSetpointVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        last_vel_time_ = now();
        external_vx_ = msg->twist.linear.x;
        external_vy_ = msg->twist.linear.y;
        external_vz_ = msg->twist.linear.z;
    }

    float external_x_, external_y_, external_z_, external_vx_, external_vy_, external_vz_;

    void loop()
    {

        if (!offboard_active) {
            // stop sending attitude/position/velocity setpoints
            return;
        }
        OffboardControlMode mode{};
        mode.timestamp = now().nanoseconds() / 1000;

        TrajectorySetpoint sp{};
        sp.timestamp = now().nanoseconds() / 1000;

        bool has_pos = (now() - last_pos_time_).seconds() < 0.3;
        bool has_vel = (now() - last_vel_time_).seconds() < 0.3;
        if (has_vel)
        {
            RCLCPP_INFO(this->get_logger(), "Current external velocity setpoints: vx: %.2f, vy: %.2f, vz: %.2f", external_vx_, external_vy_, external_vz_);
            mode.position = false;
            mode.velocity = true;
            sp.velocity[0] = external_vx_;
            sp.velocity[1] = external_vy_;
            sp.velocity[2] = external_vz_;
        }
        else if (has_pos)
        {   
            RCLCPP_INFO(this->get_logger(), "Current external setpoints: x: %.2f, y: %.2f, z: %.2f", external_x_, external_y_, external_z_);
            mode.position = true;
            mode.velocity = false;
            sp.position[0] = external_x_;
            sp.position[1] = external_y_;
            sp.position[2] = external_z_;
            hover_x_ = external_x_;
            hover_y_ = external_y_;
            hover_z_ = external_z_;
        }
        else
        {
            // regular hover
            RCLCPP_INFO(this->get_logger(), "No external setpoints received, hovering at last position.");
            mode.position = true;
            sp.position[0] = hover_x_;
            sp.position[1] = hover_y_;
            sp.position[2] = hover_z_;
        }


        offboard_pub_->publish(mode);
        traj_pub_->publish(sp);
        offboard_setpoint_counter_++;
    }

    void landCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(get_logger(), "Landing Triggered from LandingNode");

            landing_in_progress_ = true;
            offboard_active = true;   
            publishCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        }
    }


    void landedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
    {
        if (msg->landed && landing_in_progress_) {
            RCLCPP_INFO(get_logger(), "Drone has landed, disarming...");
            publishCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
            landing_in_progress_ = false;
        }
    }

    void publishOffboardMode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = now().nanoseconds() / 1000;
        offboard_pub_->publish(msg);
    }

    void startOffboard()
    {  
        if (offboard_setpoint_counter_ > 10 ){
            RCLCPP_INFO(get_logger(), "Arming & switching to OFFBOARD...");

            publishCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            publishCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            RCLCPP_INFO(get_logger(), "Armed and OFFBOARD enabled.");
            start_timer_->cancel();
        } else {
            RCLCPP_WARN(get_logger(), "Waiting for offboard setpoints... (%lu)", offboard_setpoint_counter_);
        }
    }

    void publishCommand(uint16_t cmd, float p1=0, float p2=0)
    {
        VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now().nanoseconds() / 1000;
        cmd_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardMaster>());
    rclcpp::shutdown();
}
