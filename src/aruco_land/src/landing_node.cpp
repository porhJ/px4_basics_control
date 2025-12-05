// safe_landing_node_v2.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

using namespace std::chrono_literals;

class SafeLandingNode : public rclcpp::Node
{
public:
    SafeLandingNode() : Node("safe_landing_node_v2")
    {
        // ---- PARAMETERS ----
        declare_parameter("descent_vel", -0.4);
        declare_parameter("vel_p_gain", 1.2);
        declare_parameter("vel_i_gain", 0.0);
        declare_parameter("max_velocity", 1.0);
        declare_parameter("target_timeout", 1.0);
        declare_parameter("delta_position", 0.25);
        declare_parameter("delta_velocity", 0.25);

        get_parameter("descent_vel",  descent_vel_);
        get_parameter("vel_p_gain",   p_gain_);
        get_parameter("vel_i_gain",   i_gain_);
        get_parameter("max_velocity", max_vel_);
        get_parameter("target_timeout", timeout_sec_);
        get_parameter("delta_position", delta_pos_);
        get_parameter("delta_velocity", delta_vel_);

        auto qos = rclcpp::QoS(10).best_effort();
        // ---- PUB / SUB ----
        pos_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/ext_setpoint/pos", rclcpp::SensorDataQoS());
        land_pub_ = create_publisher<std_msgs::msg::Bool>("/land_command", 10);

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", qos,
            std::bind(&SafeLandingNode::targetPoseCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SensorDataQoS(),
            std::bind(&SafeLandingNode::odomCallback, this, std::placeholders::_1));

        start_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/land_command/pres_land",
            rclcpp::QoS(1).best_effort(),
            std::bind(&SafeLandingNode::startCallback, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(50ms, std::bind(&SafeLandingNode::updateLoop, this));

        initStateMachine();
        RCLCPP_INFO(get_logger(), "Precision landing node (offboard-style) started");
    }

private:

    // ========================
    // ====== STATE ENUM ======
    // ========================
    enum class State { Idle, Search, Approach, Descend, Finished };
    State state_;

    void initStateMachine()
    {
        state_ = State::Idle;
        last_tag_time_ = now() - rclcpp::Duration::from_seconds(5);
        tag_seen_ = false;

        vel_int_x_ = vel_int_y_ = 0;
        search_index_ = 0;

        generateSearchSpiral();
    }

    // ========================
    // ====== VARIABLES ======
    // ========================
    // Tag pose world frame
    Eigen::Vector3d tag_pos_world_;
    Eigen::Quaterniond tag_q_world_;

    // Drone odometry
    Eigen::Vector3d drone_pos_;
    Eigen::Quaterniond drone_q_;
    float drone_yaw_{0};

    // PI control integrators
    float vel_int_x_, vel_int_y_;

    // Params
    double descent_vel_;
    double p_gain_, i_gain_, max_vel_;
    double timeout_sec_, delta_pos_, delta_vel_;

    // Tag timing
    rclcpp::Time last_tag_time_;
    bool tag_seen_;

    // Search spiral
    std::vector<Eigen::Vector3d> search_points_;
    int search_index_;

    // ========================
    // ====== PUB/SUB =========
    // ========================
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;

    // ==================================================
    // =============== TAG CALLBACK =====================
    // ==================================================
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Save tag time
        tag_seen_ = true;
        last_tag_time_ = now();

        // Convert optical → camera → world frame (same as original logic)
        Eigen::Vector3d pos(msg->pose.position.x,
                            msg->pose.position.y,
                            msg->pose.position.z);

        Eigen::Quaterniond q(msg->pose.orientation.w,
                             msg->pose.orientation.x,
                             msg->pose.orientation.y,
                             msg->pose.orientation.z);

        // Optical → NED rotation
        Eigen::Matrix3d R;
        R << 0, -1, 0,
             1,  0, 0,
             0,  0, 1;

        Eigen::Quaterniond q_opt_to_ned(R);

        // Camera offset transform (example: 0,0,-0.1)
        Eigen::Affine3d T_cam = 
            Eigen::Translation3d(0, 0, -0.1) * q_opt_to_ned;

        Eigen::Affine3d T_drone = 
            Eigen::Translation3d(drone_pos_) * drone_q_;

        Eigen::Affine3d T_tag =
            Eigen::Translation3d(pos) * q;

        auto T_world = T_drone * T_cam * T_tag;

        tag_pos_world_ = T_world.translation();
        tag_q_world_   = Eigen::Quaterniond(T_world.rotation());

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Tag detected at: %.2f %.2f %.2f",
            tag_pos_world_.x(), tag_pos_world_.y(), tag_pos_world_.z());
    }

    void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(get_logger(), "Switching IDLE → SEARCH");
            state_ = State::Search;
        }
    }

    // ==================================================
    // ================= ODOM CALLBACK ==================
    // ==================================================
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        drone_pos_ = Eigen::Vector3d(msg->position[0],
                                     msg->position[1],
                                     msg->position[2]);

        drone_q_ = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

        drone_yaw_ = std::atan2(
            2*(drone_q_.w()*drone_q_.z() + drone_q_.x()*drone_q_.y()),
            1 - 2*(drone_q_.y()*drone_q_.y() + drone_q_.z()*drone_q_.z()));
    }


    // ==================================================
    // ================= MAIN LOOP ======================
    // ==================================================
    void updateLoop()
    {
        if (state_ == State::Idle)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting...");
            return;
        }

        bool lost_target = checkTargetLost();

        switch (state_)
        {
        case State::Search:
            runSearchState(lost_target);
            break;

        case State::Approach:
            runApproachState(lost_target);
            break;

        case State::Descend:
            runDescendState(lost_target);
            break;

        case State::Finished:
            break;

        default:
            break;
        }
    }

    // ==================================================
    // =============== STATE HANDLERS ===================
    // ==================================================

    void runSearchState(bool lost)
    {
        if (!lost)
        {
            RCLCPP_INFO(get_logger(), "Target found. → Approach");
            state_ = State::Approach;
            return;
        }

        // Follow spiral
        Eigen::Vector3d wp = search_points_[search_index_];

        publishPose(wp, drone_yaw_);
        RCLCPP_INFO(get_logger(), "Setpoint sent..")

        if ((wp - drone_pos_).norm() < 0.3)
            search_index_ = (search_index_ + 1) % search_points_.size();
    }


    void runApproachState(bool lost)
    {
        if (lost)
        {
            RCLCPP_WARN(get_logger(), "Target lost during approach → abort");
            state_ = State::Idle;
            return;
        }

        Eigen::Vector3d target(tag_pos_world_.x(),
                               tag_pos_world_.y(),
                               drone_pos_.z());

        publishPose(target, drone_yaw_);

        if ((target - drone_pos_).norm() < delta_pos_)
            state_ = State::Descend;
    }


    void runDescendState(bool lost)
    {
        if (lost)
        {
            RCLCPP_WARN(get_logger(), "Target lost during descent → abort");
            state_ = State::Idle;
            return;
        }

        Eigen::Vector2d vel_xy = computePIControlXY();

        geometry_msgs::msg::PoseStamped sp;
        sp.header.stamp = now();
        sp.pose.position.x = drone_pos_.x() + vel_xy.x() * 0.05;
        sp.pose.position.y = drone_pos_.y() + vel_xy.y() * 0.05;
        sp.pose.position.z = drone_pos_.z() + descent_vel_ * 0.05;
        setYaw(sp.pose, drone_yaw_);
        pos_pub_->publish(sp);

        if (drone_pos_.z() > -0.6)  // near ground
        {
            std_msgs::msg::Bool msg;
            msg.data = true;
            land_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "Landing triggered");
            state_ = State::Finished;
        }
    }

    // ==================================================
    // =============== HELPERS ==========================
    // ==================================================

    bool checkTargetLost()
    {
        if (!tag_seen_) return true;

        double dt = (now() - last_tag_time_).seconds();
        return dt > timeout_sec_;
    }

    void publishPose(const Eigen::Vector3d& p, double yaw)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.pose.position.x = p.x();
        msg.pose.position.y = p.y();
        msg.pose.position.z = p.z();
        setYaw(msg.pose, yaw);
        pos_pub_->publish(msg);
    }

    void setYaw(geometry_msgs::msg::Pose& pose, double yaw)
    {
        pose.orientation.w = cos(yaw * 0.5);
        pose.orientation.z = sin(yaw * 0.5);
        pose.orientation.x = 0;
        pose.orientation.y = 0;
    }

    Eigen::Vector2d computePIControlXY()
    {
        double dx = drone_pos_.x() - tag_pos_world_.x();
        double dy = drone_pos_.y() - tag_pos_world_.y();

        vel_int_x_ += dx;
        vel_int_y_ += dy;

        vel_int_x_ = std::clamp(vel_int_x_, -float(max_vel_), float(max_vel_));
        vel_int_y_ = std::clamp(vel_int_y_, -float(max_vel_), float(max_vel_));

        double vx = -(p_gain_ * dx + i_gain_ * vel_int_x_);
        double vy = -(p_gain_ * dy + i_gain_ * vel_int_y_);

        vx = std::clamp(vx, -max_vel_, max_vel_);
        vy = std::clamp(vy, -max_vel_, max_vel_);

        return {vx, vy};
    }

    // Generate spiral search path
    void generateSearchSpiral()
    {
        double z = -2.5;
        double radius = 2.0;

        for (int i = 0; i < 20; i++)
        {
            double angle = i * 0.5;
            double r = 0.1 * i;
            search_points_.push_back(
                Eigen::Vector3d(r*cos(angle), r*sin(angle), z)
            );
        }
    }
};

// ========================
// ======== MAIN ==========
// ========================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeLandingNode>());
    rclcpp::shutdown();
    return 0;
}
