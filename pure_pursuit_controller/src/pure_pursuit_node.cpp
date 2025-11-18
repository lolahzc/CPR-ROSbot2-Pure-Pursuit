#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>


class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // Parámetros
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 1.0);
        this->declare_parameter("goal_tolerance", 0.1);
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        // Subsciptores
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
            
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_pose", 10,
            std::bind(&PurePursuitNode::goalCallback, this, std::placeholders::_1));

        // Publicadores
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer para el control
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PurePursuitNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
    }

    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        goal_point_ = msg->point;
        has_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", 
                   goal_point_.x, goal_point_.y);
    }

    void controlLoop()
    {
        if (!has_odom_ || !has_goal_) return;

        double dx = goal_point_.x - current_pose_.position.x;
        double dy = goal_point_.y - current_pose_.position.y;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);

        if (distance_to_goal < goal_tolerance_) {
            stopRobot();
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            has_goal_ = false;
            return;
        }

        double current_yaw = getYawFromQuaternion(current_pose_.orientation);
        
        double target_angle = std::atan2(dy, dx);
        double angle_error = normalizeAngle(target_angle - current_yaw);

        geometry_msgs::msg::Twist cmd_vel;
        
        double angular_vel = 2.0 * angle_error;  
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        
        double linear_vel = max_linear_vel_ * (1.0 - std::abs(angle_error)/M_PI);
        linear_vel = std::max(0.0, std::min(linear_vel, max_linear_vel_));

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;

        cmd_vel_pub_->publish(cmd_vel);
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Variables
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Point goal_point_;
    bool has_odom_ = false;
    bool has_goal_ = false;
    
    // Parámetros
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;

    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}