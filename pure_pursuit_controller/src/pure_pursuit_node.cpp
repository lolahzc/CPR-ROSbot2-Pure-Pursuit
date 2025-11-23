#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // Parámetros
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.05);
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publicador de odometría simulada
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Subscriptores
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_pose", 10,
            std::bind(&PurePursuitNode::goalCallback, this, std::placeholders::_1));

        // Publicadores
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Posición inicial del robot
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;

        // Timer para el control (más rápido para mejor respuesta)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz para control
            std::bind(&PurePursuitNode::controlLoop, this));
            
        // Timer para TF y odometría (MUCHO más rápido para RViz)
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz para TF/odom
            std::bind(&PurePursuitNode::publishOdomAndTF, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized - High frequency mode");
    }

private:
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        goal_point_ = msg->point;
        has_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", 
                   goal_point_.x, goal_point_.y);
    }

    void controlLoop()
    {
        if (!has_goal_) {
            // Solo publicar comandos de velocidad cero, no detener el timer de TF
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        double dx = goal_point_.x - robot_x_;
        double dy = goal_point_.y - robot_y_;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);

        if (distance_to_goal < goal_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            has_goal_ = false;
            return;
        }

        double target_angle = std::atan2(dy, dx);
        double angle_error = normalizeAngle(target_angle - robot_yaw_);

        geometry_msgs::msg::Twist cmd_vel;
        
        // Control más suave
        double angular_vel = 1.5 * angle_error;  // Reducir ganancia para más suavidad
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        
        double linear_vel = max_linear_vel_ * (1.0 - std::abs(angle_error)/M_PI);
        linear_vel = std::max(0.1, std::min(linear_vel, max_linear_vel_)); // Velocidad mínima de 0.1

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;

        cmd_vel_pub_->publish(cmd_vel);

        // Actualizar posición del robot con paso de tiempo más pequeño
        updateRobotPose(cmd_vel.linear.x, cmd_vel.angular.z);
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Robot Pos: (%.2f, %.2f), Goal: (%.2f, %.2f), Lin: %.2f, Ang: %.2f",
                    robot_x_, robot_y_,
                    goal_point_.x, goal_point_.y,
                    cmd_vel.linear.x, cmd_vel.angular.z);
    }

    void updateRobotPose(double linear_vel, double angular_vel)
    {
        double dt = 0.05; // 50ms - igual que el timer de control

        // Actualizar orientación
        robot_yaw_ += angular_vel * dt;
        robot_yaw_ = normalizeAngle(robot_yaw_);

        // Actualizar posición
        robot_x_ += linear_vel * cos(robot_yaw_) * dt;
        robot_y_ += linear_vel * sin(robot_yaw_) * dt;
    }

    void publishOdomAndTF()
    {
        auto now = this->now();

        // Publicar Odometría (alta frecuencia)
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.pose.pose.position.x = robot_x_;
        odom_msg.pose.pose.position.y = robot_y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = createQuaternionFromYaw(robot_yaw_);
        
        // Añadir pequeña velocidad para suavizar
        if (has_goal_) {
            odom_msg.twist.twist.linear.x = 0.1;
            odom_msg.twist.twist.angular.z = 0.0;
        } else {
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;
        }
        
        odom_pub_->publish(odom_msg);

        // Publicar TF (alta frecuencia)
        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = now;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = robot_x_;
        transform.transform.translation.y = robot_y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = createQuaternionFromYaw(robot_yaw_);
        
        tf_broadcaster_->sendTransform(transform);
    }

    geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Variables
    double robot_x_, robot_y_, robot_yaw_;
    geometry_msgs::msg::Point goal_point_;
    bool has_goal_ = false;
    
    // Parámetros
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;

    // ROS2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}