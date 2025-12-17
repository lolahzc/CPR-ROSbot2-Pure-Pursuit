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
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.2);
        this->declare_parameter("lookahead_min", 1.0);      
        this->declare_parameter("lookahead_max", 3.0);      
        this->declare_parameter("lookahead_gamma", 0.8);    
        this->declare_parameter("selected_route", 1);

        selected_route_ = this->get_parameter("selected_route").as_int();
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        delta_min_ = this->get_parameter("lookahead_min").as_double();
        delta_max_ = this->get_parameter("lookahead_max").as_double();
        gamma_ = this->get_parameter("lookahead_gamma").as_double();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PurePursuitNode::goalCallback, this, std::placeholders::_1));
            
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoints_path", 10,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,  
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)); 

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);

        std::string base_name;

        switch(selected_route_) {
            case 1: base_name = "ruta_defecto"; break;
            case 2: base_name = "ruta_recta"; break;
            case 3: base_name = "ruta_zigzag"; break;
            case 4: base_name = "ruta_ocho"; break;
            case 5: base_name = "ruta_espiral"; break;
            case 6: base_name = "ruta_cur_amplias"; break;
            case 7: base_name = "ruta_cur_cerradas"; break;
            default: base_name = "ruta_desconocida"; break;
        }

        auto now = std::chrono::system_clock::now();
        std::time_t in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "_%Y-%m-%d_%H-%M-%S");

        std::string final_filename = base_name + ss.str() + ".csv";

        data_log_file_.open(final_filename);

        if (data_log_file_.is_open()) {
            data_log_file_ << "time,robot_x,robot_y,robot_yaw,goal_x,goal_y,dist_error,linear_v,angular_w\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "No se pudo crear el archivo de log: %s", final_filename.c_str());
        }

        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;
        robot_linear_vel_ = 0.0;


        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuitNode::controlLoop, this));

    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_goal_ = msg->pose;
        has_goal_ = true;
        goal_reached_ = false;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        robot_linear_vel_ = msg->twist.twist.linear.x;
        
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
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose.position);
        }
        has_path_ = true;
        if (!path_points_.empty()) {
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
        } else {
             current_path_index_ = 0;
        }

    }

    void updateCurrentPathIndex()
    {
        if (path_points_.empty()) return;

        double min_dist_sq = std::numeric_limits<double>::max();
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

        publishRobotMarker();

        if (!has_goal_ || !has_path_ || path_points_.empty()) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        updateCurrentPathIndex();

        double dx = current_goal_.position.x - robot_x_;
        double dy = current_goal_.position.y - robot_y_;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);

        if (distance_to_goal < goal_tolerance_) {
            if (!goal_reached_) {
                goal_reached_ = true;
                
                auto goal_reached_msg = std_msgs::msg::Bool();
                goal_reached_msg.data = true;
                goal_reached_pub_->publish(goal_reached_msg);
                
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd_vel);
            }
            return;
        }
        
        double cte = getCrossTrackError();
        double lookahead_dist = calculateLookaheadDistance(cte, std::abs(robot_linear_vel_));

        geometry_msgs::msg::Point lookahead_point;
        bool lookahead_found = findLookaheadPoint(lookahead_dist,lookahead_point);
        
        if (!lookahead_found) {
            lookahead_point = current_goal_.position;
        }

        publishLookaheadMarker(lookahead_point);

        double target_angle = std::atan2(lookahead_point.y - robot_y_, lookahead_point.x - robot_x_);
        double alpha = normalizeAngle(target_angle - robot_yaw_);

        double linear_vel = max_linear_vel_;

        if (std::abs(cte) > 0.04){
            linear_vel *= 0.3;
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
                           << distance_to_goal << ","         
                           << cmd_vel.linear.x << ","         
                           << cmd_vel.angular.z << "\n";      
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
        double rel_x = lookahead_point.x - robot_x_;
        double rel_y = lookahead_point.y - robot_y_;
        
        double robot_rel_x = rel_x * cos(robot_yaw_) + rel_y * sin(robot_yaw_);
        double robot_rel_y = -rel_x * sin(robot_yaw_) + rel_y * cos(robot_yaw_);
        
        double L = std::sqrt(robot_rel_x*robot_rel_x + robot_rel_y*robot_rel_y);
        
        if (L > 0.001) {
            return 2.0 * robot_rel_y / (L * L);
        }
        
        return 0.0;
    }

    void publishLookaheadMarker(const geometry_msgs::msg::Point& point)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
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

    void publishRobotMarker()
    {
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

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double robot_x_, robot_y_, robot_yaw_, robot_linear_vel_;
    geometry_msgs::msg::Pose current_goal_;
    std::vector<geometry_msgs::msg::Point> path_points_;
    bool has_goal_ = false;
    bool has_path_ = false;
    bool goal_reached_ = false;
    size_t current_path_index_ = 0;
    std::ofstream data_log_file_;
    
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;

    double delta_min_;
    double delta_max_;
    double gamma_;

    int selected_route_;

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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}