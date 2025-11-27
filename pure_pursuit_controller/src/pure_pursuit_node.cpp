#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // Parámetros
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.1);
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publicadores
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);

        // Subscriptores
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PurePursuitNode::goalCallback, this, std::placeholders::_1));
            
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoints_path", 10,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

        // Publicadores
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Posición inicial del robot
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;

        // Timer para el control
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuitNode::controlLoop, this));
            
        // Timer para TF y odometría
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PurePursuitNode::publishOdomAndTF, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized - Continuous path following mode");
        RCLCPP_INFO(this->get_logger(), "Lookahead distance: %.2f", lookahead_distance_);
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_goal_ = msg->pose;
        has_goal_ = true;
        goal_reached_ = false;
        
        RCLCPP_DEBUG(this->get_logger(), "Goal updated: (%.2f, %.2f)", 
                    current_goal_.position.x, current_goal_.position.y);
    }

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Guardar la ruta completa para Pure Pursuit
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose.position);
        }
        has_path_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Received path with %zu points", path_points_.size());
    }

    void controlLoop()
    {
        if (!has_goal_ || !has_path_ || path_points_.empty()) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        // Verificar si se alcanzó el goal final
        double dx = current_goal_.position.x - robot_x_;
        double dy = current_goal_.position.y - robot_y_;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);

        if (distance_to_goal < goal_tolerance_) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "Final goal reached! Distance: %.3f", distance_to_goal);
                goal_reached_ = true;
                has_goal_ = false;
                
                auto goal_reached_msg = std_msgs::msg::Bool();
                goal_reached_msg.data = true;
                goal_reached_pub_->publish(goal_reached_msg);
            }
            
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        // PURE PURSUIT CON SEGUIMIENTO DE TRAYECTORIA CONTINUA
        
        // 1. Encontrar el punto más cercano en la trayectoria
        int closest_point_index = findClosestPoint();
        
        // 2. Buscar el punto lookahead a lo largo de la trayectoria
        geometry_msgs::msg::Point lookahead_point;
        bool lookahead_found = findLookaheadPoint(closest_point_index, lookahead_point);
        
        if (!lookahead_found) {
            // Si no encontramos punto lookahead, ir directamente al goal
            lookahead_point = current_goal_.position;
        }

        // 3. Publicar marcador del punto lookahead
        publishLookaheadMarker(lookahead_point);
        
        // 4. Calcular curvatura usando Pure Pursuit
        double curvature = calculateCurvature(lookahead_point);
        
        // 5. Calcular velocidades de control
        geometry_msgs::msg::Twist cmd_vel = calculateControlCommands(curvature, distance_to_goal);
        cmd_vel_pub_->publish(cmd_vel);

        // 6. Actualizar posición del robot (simulación)
        updateRobotPose(cmd_vel.linear.x, cmd_vel.angular.z);
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "PurePursuit: Curvature: %.3f, Lin: %.2f, Ang: %.2f, Dist2Goal: %.3f",
                    curvature, cmd_vel.linear.x, cmd_vel.angular.z, distance_to_goal);
    }

    int findClosestPoint()
    {
        int closest_index = 0;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < path_points_.size(); ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        
        return closest_index;
    }

    bool findLookaheadPoint(int start_index, geometry_msgs::msg::Point& lookahead_point)
    {
        // Buscar el primer punto que esté al menos a lookahead_distance del robot
        for (size_t i = start_index; i < path_points_.size(); ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance >= lookahead_distance_) {
                lookahead_point = path_points_[i];
                return true;
            }
        }
        
        // Si no encontramos ningún punto, usar el último punto de la trayectoria
        if (!path_points_.empty()) {
            lookahead_point = path_points_.back();
            return true;
        }
        
        return false;
    }

    double calculateCurvature(const geometry_msgs::msg::Point& lookahead_point)
    {
        // Transformar punto lookahead al marco del robot
        double rel_x = lookahead_point.x - robot_x_;
        double rel_y = lookahead_point.y - robot_y_;
        
        // Rotar al marco del robot
        double robot_rel_x = rel_x * cos(robot_yaw_) + rel_y * sin(robot_yaw_);
        double robot_rel_y = -rel_x * sin(robot_yaw_) + rel_y * cos(robot_yaw_);
        
        // Calcular curvatura (L = distancia al punto lookahead)
        double L = std::sqrt(robot_rel_x*robot_rel_x + robot_rel_y*robot_rel_y);
        
        if (L > 0.001) {
            return 2.0 * robot_rel_y / (L * L);
        }
        
        return 0.0;
    }

    geometry_msgs::msg::Twist calculateControlCommands(double curvature, double distance_to_goal)
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        // Velocidad angular basada en curvatura: ω = v * κ
        double angular_vel = curvature * max_linear_vel_;
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        
        // Velocidad lineal 
        double linear_vel = max_linear_vel_;
        
        // Reducir velocidad si la curvatura es grande
        if (std::abs(curvature) > 0.5) {
            linear_vel *= (1.0 - std::abs(curvature) * 0.6);
        }
        
        // Reducir velocidad al acercarse al goal
        if (distance_to_goal < lookahead_distance_) {
            linear_vel *= (distance_to_goal / lookahead_distance_);
        }
        
        // Velocidad mínima
        linear_vel = std::max(0.1, linear_vel);

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        
        return cmd_vel;
    }

    void publishLookaheadMarker(const geometry_msgs::msg::Point& point)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        
        marker_pub_->publish(marker);
    }

    void updateRobotPose(double linear_vel, double angular_vel)
    {
        double dt = 0.05;

        robot_yaw_ += angular_vel * dt;
        robot_yaw_ = normalizeAngle(robot_yaw_);

        robot_x_ += linear_vel * cos(robot_yaw_) * dt;
        robot_y_ += linear_vel * sin(robot_yaw_) * dt;
    }

    void publishOdomAndTF()
    {
        auto now = this->now();

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.pose.pose.position.x = robot_x_;
        odom_msg.pose.pose.position.y = robot_y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = createQuaternionFromYaw(robot_yaw_);
        
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        
        odom_pub_->publish(odom_msg);

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
    geometry_msgs::msg::Pose current_goal_;
    std::vector<geometry_msgs::msg::Point> path_points_;
    bool has_goal_ = false;
    bool has_path_ = false;
    bool goal_reached_ = false;
    
    // Parámetros
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;

    // ROS2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
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