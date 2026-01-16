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
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <vector>
#include <algorithm> 
#include <fstream> 
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // --- PARÁMETROS DE NAVEGACIÓN ---
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.2);
        this->declare_parameter("lookahead_min", 1.0);      
        this->declare_parameter("lookahead_max", 3.0);      
        this->declare_parameter("lookahead_gamma", 0.8);    
        this->declare_parameter("selected_route", 1);
        
        // --- PARÁMETROS DE EVASIÓN ---
        this->declare_parameter("bubble_base_radius", 0.5);
        this->declare_parameter("critical_distance", 0.25);
        this->declare_parameter("detour_offset", 1.0); 
        this->declare_parameter("rejoin_distance", 2.0); 

        // Lectura de parámetros
        selected_route_ = this->get_parameter("selected_route").as_int();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        bubble_base_radius_ = this->get_parameter("bubble_base_radius").as_double();
        critical_distance_ = this->get_parameter("critical_distance").as_double();
        detour_offset_ = this->get_parameter("detour_offset").as_double();
        rejoin_distance_ = this->get_parameter("rejoin_distance").as_double();

        delta_min_ = this->get_parameter("lookahead_min").as_double();
        delta_max_ = this->get_parameter("lookahead_max").as_double();
        gamma_ = this->get_parameter("lookahead_gamma").as_double();

        // --- CONFIGURACIÓN DE PUBLICADORES Y SUSCRIPTORES ---
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        bubble_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/bubble_viz", 10);
        robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);
        
        // Publicador MarkerArray para la ruta modificada
        detour_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detour_path", 10);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PurePursuitNode::goalCallback, this, std::placeholders::_1));
            
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoints_path", 10,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,  
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)); 

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&PurePursuitNode::scanCallback, this, std::placeholders::_1));

        // --- CONFIGURACIÓN DE LOGS ---
        std::string base_name;
        switch(selected_route_) {
            case 1: base_name = "ruta_defecto"; break;
            case 2: base_name = "ruta_recta"; break;
            case 3: base_name = "ruta_zigzag"; break;
            case 4: base_name = "ruta_ocho"; break;
            case 5: base_name = "ruta_espiral"; break;
            case 6: base_name = "ruta_cur_amplias"; break;
            case 7: base_name = "ruta_cur_cerradas"; break;
            case 8: base_name = "ruta_obstaculo"; break;
            default: base_name = "ruta_desconocida"; break;
        }

        auto now = std::chrono::system_clock::now();
        std::time_t in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "_%Y-%m-%d_%H-%M-%S");

        std::string final_filename = base_name + ss.str() + ".csv";
        data_log_file_.open(final_filename);

        if (data_log_file_.is_open()) {
            data_log_file_ << "time,robot_x,robot_y,robot_yaw,goal_x,goal_y,lookahead,dist_error,path_index,dist_goal,linear_v,angular_w,curvature\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "No se pudo crear el archivo de log: %s", final_filename.c_str());
        }

        robot_x_ = 0.0; robot_y_ = 0.0; robot_yaw_ = 0.0; robot_linear_vel_ = 0.0;
        current_vel_cmd_ = 0.0;

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuitNode::controlLoop, this));
            
        last_detour_time_ = this->now();
    }

private:
    enum class AvoidanceState { 
        NORMAL,           
        OBSTACLE_DETECTED, 
        EMERGENCY         
    };

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_goal_ = msg->pose;
        has_goal_ = true;
        goal_reached_ = false;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_linear_vel_ = msg->twist.twist.linear.x;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_yaw_);
        robot_yaw_ = normalizeAngle(robot_yaw_);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
    }

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if ((this->now() - last_detour_time_).seconds() > 3.0) {
            path_points_.clear();
            for (const auto& pose : msg->poses) {
                path_points_.push_back(pose.position);
            }
            has_path_ = true;
            current_path_index_ = 0;
            findClosestIndex();
        }
    }
    
    void findClosestIndex() {
        if (path_points_.empty()) return;
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t best_idx = 0;
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
    }

    void updateCurrentPathIndex()
    {
        if (path_points_.empty()) return;

        double min_dist_sq = std::numeric_limits<double>::max();
        size_t best_idx = current_path_index_;
        size_t search_limit = std::min(path_points_.size(), current_path_index_ + 50); 

        for (size_t i = current_path_index_; i < search_limit; ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }
        if (best_idx > current_path_index_) current_path_index_ = best_idx;
    }

    double getCrossTrackError() {
        if (path_points_.empty()) return 0.0;
        double min_dist = 1000.0;
        size_t end_idx = std::min(path_points_.size(), current_path_index_ + 20);
        for (size_t i = current_path_index_; i < end_idx; ++i) {
            double d = std::hypot(path_points_[i].x - robot_x_, path_points_[i].y - robot_y_);
            if (d < min_dist) min_dist = d;
        }
        return min_dist;
    }

    double calculateLookaheadDistance(double cross_track_error, double vx) {
        double vel_f = std::clamp(vx/5.0, 0.05, 0.2)*5.0;
        return vel_f * (delta_max_ - delta_min_) * std::exp(-gamma_ * std::abs(cross_track_error)) + delta_min_;
    }

    void updateAvoidanceState() {
        if (!last_scan_) return;

        double min_dist_in_fov = 999.0;
        bool obstacle_in_bubble = false;
        double fov = 60.0 * M_PI / 180.0;

        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double r = last_scan_->ranges[i];
            if (std::isnan(r) || std::isinf(r) || r < 0.1) continue;

            double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);
            
            if (std::abs(angle) > fov/2.0) continue;

            if (r < min_dist_in_fov) min_dist_in_fov = r;
            if (r < bubble_base_radius_) obstacle_in_bubble = true;
        }

        if (min_dist_in_fov < critical_distance_) {
            avoidance_state_ = AvoidanceState::EMERGENCY;
        } else if (obstacle_in_bubble) {
            avoidance_state_ = AvoidanceState::OBSTACLE_DETECTED;
        } else {
            avoidance_state_ = AvoidanceState::NORMAL;
        }
    }

    void generateDetourPath() {
        if ((this->now() - last_detour_time_).seconds() < 2.0) return;
        if (!last_scan_) return;

        double left_obst_weight = 0;
        double right_obst_weight = 0;
        double fov = 90.0 * M_PI / 180.0;

        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double r = last_scan_->ranges[i];
            if (r > bubble_base_radius_ * 1.5) continue;
            
            double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);
            if (angle > 0 && angle < fov/2) left_obst_weight += (1.0/r);
            if (angle < 0 && angle > -fov/2) right_obst_weight += (1.0/r);
        }

        double side_sign = (left_obst_weight > right_obst_weight) ? -1.0 : 1.0; 

        double heading_x = std::cos(robot_yaw_);
        double heading_y = std::sin(robot_yaw_);
        
        double perp_x = -heading_y * side_sign;
        double perp_y = heading_x * side_sign;

        double forward_offset = 0.5;
        geometry_msgs::msg::Point p_start;
        p_start.x = robot_x_; 
        p_start.y = robot_y_;

        geometry_msgs::msg::Point p_apex;
        p_apex.x = robot_x_ + (heading_x * forward_offset) + (perp_x * detour_offset_);
        p_apex.y = robot_y_ + (heading_y * forward_offset) + (perp_y * detour_offset_);

        size_t rejoin_idx = current_path_index_;
        bool found_rejoin = false;
        double cumulative_dist = 0;

        for (size_t i = current_path_index_; i < path_points_.size() - 1; ++i) {
            double dx = path_points_[i+1].x - path_points_[i].x;
            double dy = path_points_[i+1].y - path_points_[i].y;
            cumulative_dist += std::hypot(dx, dy);
            
            if (cumulative_dist > rejoin_distance_) {
                rejoin_idx = i + 1;
                found_rejoin = true;
                break;
            }
        }
        
        if (!found_rejoin) rejoin_idx = path_points_.size() - 1;

        geometry_msgs::msg::Point p_rejoin = path_points_[rejoin_idx];

        std::vector<geometry_msgs::msg::Point> detour_points;
        int steps = 15;

        for (int i = 1; i <= steps; ++i) {
            double t = (double)i / (double)steps;
            double u = 1 - t;
            
            geometry_msgs::msg::Point p;
            p.x = (u*u * p_start.x) + (2*u*t * p_apex.x) + (t*t * p_rejoin.x);
            p.y = (u*u * p_start.y) + (2*u*t * p_apex.y) + (t*t * p_rejoin.y);
            p.z = 0.0;
            detour_points.push_back(p);
        }

        if (rejoin_idx > current_path_index_) {
            auto it_start = path_points_.begin() + current_path_index_ + 1;
            auto it_end = path_points_.begin() + rejoin_idx;
            
            if (it_start < path_points_.end() && it_end < path_points_.end() && it_start < it_end) {
                path_points_.erase(it_start, it_end);
                path_points_.insert(path_points_.begin() + current_path_index_ + 1, 
                                  detour_points.begin(), detour_points.end());
                                  
                last_detour_time_ = this->now();
                publishDetourPath();
            }
        }
    }

    void publishDetourPath() {
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "detour_line";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.05; 
        line_strip.color.r = 0.0; line_strip.color.g = 1.0; line_strip.color.b = 1.0; line_strip.color.a = 1.0; 

        visualization_msgs::msg::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = this->now();
        points.ns = "detour_points";
        points.id = 1;
        points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        points.action = visualization_msgs::msg::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.1; points.scale.y = 0.1; points.scale.z = 0.1;
        points.color.r = 1.0; points.color.g = 0.0; points.color.b = 1.0; points.color.a = 1.0; 

        for(const auto& p : path_points_) {
            line_strip.points.push_back(p);
            points.points.push_back(p);
        }

        marker_array.markers.push_back(line_strip);
        marker_array.markers.push_back(points);

        detour_path_pub_->publish(marker_array);
    }

    void controlLoop()
    {
        publishRobotMarker();

        if (!has_goal_ || !has_path_ || path_points_.empty()) {
            stopRobot();
            return;
        }

        updateCurrentPathIndex();
        updateAvoidanceState();

        if (avoidance_state_ == AvoidanceState::EMERGENCY) {
            stopRobot();
            return;
        }

        if (avoidance_state_ == AvoidanceState::OBSTACLE_DETECTED) {
            generateDetourPath();
        }

        double cte = getCrossTrackError();
        double lookahead_dist = calculateLookaheadDistance(cte, std::abs(robot_linear_vel_));
        
        geometry_msgs::msg::Point lookahead_point;
        bool found = findLookaheadPoint(lookahead_dist, lookahead_point);
        if (!found) lookahead_point = path_points_.back();

        publishLookaheadMarker(lookahead_point);

        double curv = calculateCurvature(lookahead_point);
        double target_angle = std::atan2(lookahead_point.y - robot_y_, lookahead_point.x - robot_x_);
        double alpha = normalizeAngle(target_angle - robot_yaw_);

        double cmd_curva = std::clamp(1.0 - std::abs(curv)/5.0, 0.1, 1.0);
        current_vel_cmd_ = max_linear_vel_ * cmd_curva;
        
        if (avoidance_state_ == AvoidanceState::OBSTACLE_DETECTED) {
            current_vel_cmd_ *= 0.6;
        }

        double angular_vel = (2.0 * current_vel_cmd_ * std::sin(alpha)) / lookahead_dist;
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

        // --- CAMBIO CLAVE AQUÍ ---
        // Ignoramos 'current_goal_' para la parada. Calculamos distancia al ÚLTIMO punto de la ruta.
        double dist_to_end = 999.9;
        if (!path_points_.empty()) {
             auto last_p = path_points_.back();
             dist_to_end = std::hypot(last_p.x - robot_x_, last_p.y - robot_y_);
        }

        // --- GUARDADO DE LOGS ---
        if (data_log_file_.is_open()) {
            double current_time = this->now().seconds();
            data_log_file_ << std::fixed << std::setprecision(9)
                           << current_time << ","             
                           << robot_x_ << ","                 
                           << robot_y_ << ","                 
                           << robot_yaw_ << ","               
                           << current_goal_.position.x << "," 
                           << current_goal_.position.y << "," 
                           << lookahead_dist << ","   
                           << cte << ","              
                           << current_path_index_ << "," 
                           << dist_to_end << ","      // Guardamos la distancia real al final, no al waypoint intermedio
                           << current_vel_cmd_ << "," 
                           << angular_vel << ","      
                           << curv << "\n";           
        }

        // Condición de parada basada en el FINAL DE LA RUTA
        if (dist_to_end < goal_tolerance_) {
            if (!goal_reached_) {
                goal_reached_ = true;
                std_msgs::msg::Bool msg; msg.data = true;
                goal_reached_pub_->publish(msg);
                stopRobot();
            }
            return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = current_vel_cmd_;
        cmd.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd);

        double current_radius = bubble_base_radius_;
        publishBubbleViz(current_radius);
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0; cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
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
        return false;
    }

    double calculateCurvature(const geometry_msgs::msg::Point& lookahead_point)
    {
        double rel_x = lookahead_point.x - robot_x_;
        double rel_y = lookahead_point.y - robot_y_;
        double robot_rel_x = rel_x * cos(robot_yaw_) + rel_y * sin(robot_yaw_);
        double robot_rel_y = -rel_x * sin(robot_yaw_) + rel_y * cos(robot_yaw_);
        double L = std::sqrt(robot_rel_x*robot_rel_x + robot_rel_y*robot_rel_y);
        if (L > 0.001) return 2.0 * robot_rel_y / (L * L);
        return 0.0;
    }

    void publishBubbleViz(double radius) {
        auto m = visualization_msgs::msg::Marker();
        m.header.frame_id = "map"; // Cambiar a "map"
        m.header.stamp = now();
        // ¡ESTO ES LO QUE FALTABA!
        m.pose.position.x = robot_x_;
        m.pose.position.y = robot_y_;
        m.pose.position.z = 0.0; // Elevarlo un poco para que se vea
        m.pose.orientation.w = 1.0;
        m.ns = "bubble"; 
        m.id = 1; 
        m.type = visualization_msgs::msg::Marker::SPHERE; // Tipo 6 es mejor que 3 (SPHERE)
        m.action = 0;
        m.scale.x = radius * 2.0; 
        m.scale.y = radius * 2.0; 
        m.scale.z = 0.05; // Más alto para que se vea mejor

        if (avoidance_state_ == AvoidanceState::NORMAL) { 
            m.color.r = 0.0;
            m.color.g = 1.0; 
            m.color.b = 1.0; 
            m.color.a = 0.3; 
        }


        else if (avoidance_state_ == AvoidanceState::OBSTACLE_DETECTED) { 
            m.color.r = 1.0; 
            m.color.g = 0.65; 
            m.color.b = 0.0;
            m.color.a = 0.5; 
        }
        else { 
            m.color.r = 1.0; 
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 0.7; 
        }

        m.lifetime = rclcpp::Duration::from_seconds(0);
        bubble_viz_pub_->publish(m);
        }

    void publishLookaheadMarker(const geometry_msgs::msg::Point& point) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now(); marker.header.frame_id = "map";
        marker.ns = "pure_pursuit"; marker.id = 0; marker.type = 2; marker.action = 0;
        marker.pose.position = point; marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
        marker.color.r = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
        marker_pub_->publish(marker);
    }
    
    void publishRobotMarker() {
         auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.ns = "robot_viz";
        marker.id = 1; 
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = robot_x_;
        marker.pose.position.y = robot_y_;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.3; 
        marker.scale.y = 0.3; 
        marker.scale.z = 0.3; 
        
        marker.color.r = 0.0; 
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0; 
        
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        
        robot_marker_pub_->publish(marker);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Variables
    double robot_x_, robot_y_, robot_yaw_, robot_linear_vel_;
    geometry_msgs::msg::Pose current_goal_;
    std::vector<geometry_msgs::msg::Point> path_points_;
    bool has_goal_ = false; bool has_path_ = false; bool goal_reached_ = false;
    size_t current_path_index_ = 0;
    
    std::ofstream data_log_file_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    
    // Params
    double lookahead_distance_, max_linear_vel_, max_angular_vel_, goal_tolerance_;
    double delta_min_, delta_max_, gamma_;
    double bubble_base_radius_, critical_distance_;
    double detour_offset_, rejoin_distance_;
    int selected_route_;
    
    double current_vel_cmd_;
    rclcpp::Time last_detour_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_, bubble_viz_pub_, robot_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detour_path_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    AvoidanceState avoidance_state_ = AvoidanceState::NORMAL;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}