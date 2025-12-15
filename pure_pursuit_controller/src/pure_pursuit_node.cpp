#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <algorithm> // Necesario para std::clamp y std::min/std::max
#include <fstream>  // Necesario para escribir en ficheros
#include <iomanip>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // Parámetros
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.2);

        this->declare_parameter("lookahead_min", 1.0);      // delta_min
        this->declare_parameter("lookahead_max", 3.0);      // delta_max
        this->declare_parameter("lookahead_gamma", 0.8);    // gamma
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        delta_min_ = this->get_parameter("lookahead_min").as_double();
        delta_max_ = this->get_parameter("lookahead_max").as_double();
        gamma_ = this->get_parameter("lookahead_gamma").as_double();

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

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,  
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)); 

        // Publicadores
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Abrir archivo de log
        data_log_file_.open("robot_data_log.csv");
        
        if (data_log_file_.is_open()) {
            data_log_file_ << "time,robot_x,robot_y,robot_yaw,goal_x,goal_y,dist_error,linear_v,angular_w\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "No se pudo crear el archivo de log!");
        }
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized...");

        // Posición inicial del robot
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;
        robot_linear_vel_ = 0.0;

        // Timer para el control
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuitNode::controlLoop, this));
            
        // Timer para TF y odometría
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PurePursuitNode::publishOdomAndTF, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized - Continuous path following mode (Sequential)");
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

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        robot_linear_vel_ = msg->twist.twist.linear.x;
        
        // Convertir Cuaternión a Yaw usando Matrix3x3 (Método estándar TF2)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
            
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_yaw_);
        
        robot_yaw_ = normalizeAngle(robot_yaw_);
    }

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Guardar la ruta completa para Pure Pursuit
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose.position);
        }
        has_path_ = true;
        if (!path_points_.empty()) {
            double min_dist_sq = std::numeric_limits<double>::max();
            size_t best_idx = 0;
            
            // Búsqueda global rápida para resincronizar el índice
            for (size_t i = 0; i < path_points_.size(); ++i) {
                double dx = path_points_[i].x - robot_x_;
                double dy = path_points_[i].y - robot_y_;
                double dist_sq = dx*dx + dy*dy;
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    best_idx = i;
                }
            }
            current_path_index_ = best_idx;
        } else {
             current_path_index_ = 0;
        }

        RCLCPP_INFO(this->get_logger(), "Path updated. Resumed at index %zu", current_path_index_);
    }

    void updateCurrentPathIndex()
    {
        if (path_points_.empty()) return;

        double min_dist_sq = std::numeric_limits<double>::max();
        // Buscamos solo hacia adelante desde el índice actual para eficiencia
        size_t search_limit = std::min(path_points_.size(), current_path_index_ + 200); 

        for (size_t i = current_path_index_; i < search_limit; ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                current_path_index_ = i;
            }
        }
    }

    double getCrossTrackError()
    {
        if (path_points_.empty()) return 0.0;

        double min_dist = std::numeric_limits<double>::max();
        
        size_t start_idx = (current_path_index_ > 20) ? current_path_index_ - 20 : 0;
        size_t end_idx = std::min(path_points_.size(), current_path_index_ + 100);

        for (size_t i = start_idx; i < end_idx; ++i) {
            double dist = std::hypot(path_points_[i].x - robot_x_, path_points_[i].y - robot_y_);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        return min_dist;
    }

    double calculateLookaheadDistance(double cross_track_error, double vx)
    {
        double vel_f = std::clamp(vx/10.0, 0.5, 2.0);
        
        return vel_f * (delta_max_ - delta_min_) * std::exp(-gamma_ * std::abs(cross_track_error)) + delta_min_;
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

        updateCurrentPathIndex();

        // Verificar si se alcanzó el goal final del SEGMENTO (el waypoint original)
        double dx = current_goal_.position.x - robot_x_;
        double dy = current_goal_.position.y - robot_y_;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);

        if (distance_to_goal < goal_tolerance_) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "Final segment goal reached! Distance: %.3f", distance_to_goal);
                goal_reached_ = true;
                
                auto goal_reached_msg = std_msgs::msg::Bool();
                goal_reached_msg.data = true;
                goal_reached_pub_->publish(goal_reached_msg);
                
                // Detener el robot brevemente
                //geometry_msgs::msg::Twist cmd_vel;
                //cmd_vel.linear.x = 0.0;
                //cmd_vel.angular.z = 0.0;
                //cmd_vel_pub_->publish(cmd_vel);
            }
            //return;
        }
        
        // PURE PURSUIT CON SEGUIMIENTO DE TRAYECTORIA CONTINUA SECUENCIAL
        
        double cte = getCrossTrackError();
        double lookahead_dist = calculateLookaheadDistance(cte, std::abs(robot_linear_vel_));

        geometry_msgs::msg::Point lookahead_point;
        bool lookahead_found = findLookaheadPoint(lookahead_dist,lookahead_point);
        
        if (!lookahead_found) {
            lookahead_point = current_goal_.position;
        }

        publishLookaheadMarker(lookahead_point);

        RCLCPP_INFO(this->get_logger(), "L: %.3f | CTE: %.3f | Idx: %zu", lookahead_dist, cte, current_path_index_);

        double target_angle = std::atan2(lookahead_point.y - robot_y_, lookahead_point.x - robot_x_);
        double alpha = normalizeAngle(target_angle - robot_yaw_);

        double linear_vel = max_linear_vel_;

        if (std::abs(cte) > 0.05){
            linear_vel *= 0.6;
        }

        if (distance_to_goal < lookahead_dist){
            linear_vel *= (distance_to_goal / lookahead_dist);
        }

        linear_vel = std::max(0.1, linear_vel);

        double angular_vel = (2.0 * linear_vel * std::sin(alpha)) / lookahead_dist;
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd_vel);

        // 5. Actualizar posición del robot (simulación)
        // updateRobotPose(cmd_vel.linear.x, cmd_vel.angular.z);
        
        double curvature = calculateCurvature(lookahead_point);

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "PurePursuit: Path Index: %zu/%zu, Curvature: %.3f, Lin: %.2f, Ang: %.2f, Dist2Goal: %.3f",
                    current_path_index_, path_points_.size(), curvature, cmd_vel.linear.x, cmd_vel.angular.z, distance_to_goal);

        // 6. Registrar datos en el archivo de log
        if (data_log_file_.is_open()) {
            double current_time = this->now().seconds();
            
            data_log_file_ << std::fixed << std::setprecision(9)
                           << current_time << ","             // Tiempo
                           << robot_x_ << ","                 // Robot X
                           << robot_y_ << ","                 // Robot Y
                           << robot_yaw_ << ","               // Robot Yaw
                           << current_goal_.position.x << "," // Goal X actual
                           << current_goal_.position.y << "," // Goal Y actual
                           << lookahead_dist << ","
                           << cte << ","    
                           << current_path_index_ << ","
                           << distance_to_goal << ","         // Distancia al objetivo
                           << cmd_vel.linear.x << ","         // Velocidad Lineal enviada
                           << cmd_vel.angular.z << "\n";      // Velocidad Angular enviada
        }
    }

    bool findLookaheadPoint(double dynamic_lookahead_dist ,geometry_msgs::msg::Point& lookahead_point)
   {
        for (size_t i = current_path_index_; i < path_points_.size(); ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance >= dynamic_lookahead_dist) {
                lookahead_point = path_points_[i];
                return true;
            }
        }
        
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
            // Curvatura = 2 * y_rel_robot / L^2
            return 2.0 * robot_rel_y / (L * L);
        }
        
        return 0.0;
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
        
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        
        marker_pub_->publish(marker);
    }
/*
    void updateRobotPose(double linear_vel, double angular_vel)
    {
        double dt = 0.05;

        robot_yaw_ += angular_vel * dt;
        robot_yaw_ = normalizeAngle(robot_yaw_);

        robot_x_ += linear_vel * cos(robot_yaw_) * dt;
        robot_y_ += linear_vel * sin(robot_yaw_) * dt;
    }
*/
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
    double robot_x_, robot_y_, robot_yaw_, robot_linear_vel_;
    geometry_msgs::msg::Pose current_goal_;
    std::vector<geometry_msgs::msg::Point> path_points_;
    bool has_goal_ = false;
    bool has_path_ = false;
    bool goal_reached_ = false;
    size_t current_path_index_ = 0; // ÍNDICE SECUENCIAL para /waypoints_path
    std::ofstream data_log_file_;
    
    // Parámetros
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;

    double delta_min_;
    double delta_max_;
    double gamma_;

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}