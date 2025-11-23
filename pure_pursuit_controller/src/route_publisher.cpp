#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

class RoutePublisher : public rclcpp::Node
{
public:
    RoutePublisher() : Node("route_publisher")
    {
        // Publishers - usando PoseStamped para /goal_pose
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);
        
        // Subscriber para goal reached
        goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&RoutePublisher::goalReachedCallback, this, std::placeholders::_1));

        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Parámetros
        this->declare_parameter("loop_route", false);
        this->declare_parameter("interpolation_points_per_segment", 10);
        
        loop_route_ = this->get_parameter("loop_route").as_bool();
        interpolation_points_ = this->get_parameter("interpolation_points_per_segment").as_int();
        
        // Definir waypoints de la ruta original
        defineRoute();
        
        // Generar ruta interpolada
        generateInterpolatedRoute();
        
        // Timer para publicar TF estático del mapa
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoutePublisher::publishStaticTF, this));

        // Timer para publicar el primer waypoint con retraso (IMPORTANTE: mantener este retraso)
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),  // 5s de retraso para esperar a que otros nodos estén listos
            [this]() {
                // Publicar el primer waypoint después del retraso
                current_waypoint_ = 0;
                publishCurrentWaypoint();
                publishWaypointsMarkers();
                
                // Cancelar este timer después de ejecutarse una vez
                start_timer_->cancel();
                
                RCLCPP_INFO(this->get_logger(), "First waypoint published after delay");
                RCLCPP_INFO(this->get_logger(), "Original waypoints: %zu, Interpolated waypoints: %zu", 
                           original_waypoints_.size(), interpolated_waypoints_.size());
            });

        RCLCPP_INFO(this->get_logger(), "Route publisher started with %zu original waypoints", original_waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Generated %zu interpolated waypoints", interpolated_waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Publishing to /goal_pose (PoseStamped)");
        RCLCPP_INFO(this->get_logger(), "Waiting for goal reached signal to advance to next waypoint");
        RCLCPP_INFO(this->get_logger(), "First waypoint will be published in 500ms...");
    }

private:
    void defineRoute()
    {
        // Mantener tus waypoints originales
        original_waypoints_ = {
            {1.0, 0.0, 0.0},
            {2.0, 0.0, 0.0}, 
            {2.5, -0.5, 0.0},
            {3.0, -1.0, 0.0},
            {3.0, -2.0, 0.0},
            {2.5, -2.5, 0.0},
            {2.0, -2.5, 0.0}
        };
    }

    void generateInterpolatedRoute()
    {
        interpolated_waypoints_.clear();
        
        // Si hay menos de 2 waypoints, no podemos interpolar
        if (original_waypoints_.size() < 2) {
            interpolated_waypoints_ = original_waypoints_;
            return;
        }

        // Para cada segmento entre waypoints originales
        for (size_t i = 0; i < original_waypoints_.size() - 1; ++i) {
            const auto& start = original_waypoints_[i];
            const auto& end = original_waypoints_[i + 1];
            
            // Interpolación lineal entre waypoints consecutivos
            for (int j = 0; j < interpolation_points_; ++j) {
                double t = static_cast<double>(j) / interpolation_points_;
                
                std::array<double, 3> interpolated_point;
                interpolated_point[0] = start[0] + t * (end[0] - start[0]);
                interpolated_point[1] = start[1] + t * (end[1] - start[1]);
                interpolated_point[2] = start[2] + t * (end[2] - start[2]);
                
                interpolated_waypoints_.push_back(interpolated_point);
            }
        }
        
        // Añadir el último waypoint
        interpolated_waypoints_.push_back(original_waypoints_.back());
        
        // Si loop_route está activado, conectar el último waypoint con el primero
        if (loop_route_ && original_waypoints_.size() >= 2) {
            const auto& start = original_waypoints_.back();
            const auto& end = original_waypoints_.front();
            
            for (int j = 1; j <= interpolation_points_; ++j) {
                double t = static_cast<double>(j) / interpolation_points_;
                
                std::array<double, 3> interpolated_point;
                interpolated_point[0] = start[0] + t * (end[0] - start[0]);
                interpolated_point[1] = start[1] + t * (end[1] - start[1]);
                interpolated_point[2] = start[2] + t * (end[2] - start[2]);
                
                interpolated_waypoints_.push_back(interpolated_point);
            }
        }
    }

    void goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Goal reached signal received! Advancing to next waypoint...");
            advanceToNextWaypoint();
        }
    }

    void publishCurrentWaypoint()
    {
        // Publicar goal point como PoseStamped
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        
        const auto& wp = interpolated_waypoints_[current_waypoint_];
        goal_msg.pose.position.x = wp[0];
        goal_msg.pose.position.y = wp[1];
        goal_msg.pose.position.z = wp[2];
        goal_msg.pose.orientation.w = 1.0;  // Orientación neutral
        
        goal_pub_->publish(goal_msg);
        
        // Publicar ruta completa como PoseArray
        publishWaypointsPath();
        
        // Publicar marcadores actualizados
        publishWaypointsMarkers();
        
        RCLCPP_INFO(this->get_logger(), "Publicado waypoint %zu/%zu: (%.2f, %.2f)", 
                   current_waypoint_ + 1, interpolated_waypoints_.size(), wp[0], wp[1]);
    }

    void publishWaypointsPath()
    {
        auto path_msg = geometry_msgs::msg::PoseArray();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        // Publicar waypoints interpolados
        for (const auto& wp : interpolated_waypoints_) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = wp[0];
            pose.position.y = wp[1];
            pose.position.z = wp[2];
            pose.orientation.w = 1.0;
            
            path_msg.poses.push_back(pose);
        }
        
        path_pub_->publish(path_msg);
    }

    void publishWaypointsMarkers()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Limpiar marcadores anteriores
        auto clear_marker = visualization_msgs::msg::Marker();
        clear_marker.header.stamp = this->now();
        clear_marker.header.frame_id = "map";
        clear_marker.ns = "waypoints";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // Marcador para la ruta interpolada (línea verde)
        auto interpolated_line_marker = visualization_msgs::msg::Marker();
        interpolated_line_marker.header.stamp = this->now();
        interpolated_line_marker.header.frame_id = "map";
        interpolated_line_marker.ns = "interpolated_route";
        interpolated_line_marker.id = 0;
        interpolated_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        interpolated_line_marker.action = visualization_msgs::msg::Marker::ADD;
        interpolated_line_marker.scale.x = 0.05;
        interpolated_line_marker.color.r = 0.0;
        interpolated_line_marker.color.g = 1.0;
        interpolated_line_marker.color.b = 0.0;
        interpolated_line_marker.color.a = 0.8;
        
        for (const auto& wp : interpolated_waypoints_) {
            geometry_msgs::msg::Point point;
            point.x = wp[0];
            point.y = wp[1];
            point.z = wp[2];
            interpolated_line_marker.points.push_back(point);
        }
        marker_array.markers.push_back(interpolated_line_marker);
        
        // Marcador para waypoints originales (línea roja discontinua)
        auto original_line_marker = visualization_msgs::msg::Marker();
        original_line_marker.header.stamp = this->now();
        original_line_marker.header.frame_id = "map";
        original_line_marker.ns = "original_route";
        original_line_marker.id = 1;
        original_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        original_line_marker.action = visualization_msgs::msg::Marker::ADD;
        original_line_marker.scale.x = 0.03;
        original_line_marker.color.r = 1.0;
        original_line_marker.color.g = 0.0;
        original_line_marker.color.b = 0.0;
        original_line_marker.color.a = 0.6;
        
        for (const auto& wp : original_waypoints_) {
            geometry_msgs::msg::Point point;
            point.x = wp[0];
            point.y = wp[1];
            point.z = wp[2];
            original_line_marker.points.push_back(point);
        }
        marker_array.markers.push_back(original_line_marker);
        
        // Marcadores para waypoints originales (esferas grandes)
        for (size_t i = 0; i < original_waypoints_.size(); ++i) {
            auto sphere_marker = visualization_msgs::msg::Marker();
            sphere_marker.header.stamp = this->now();
            sphere_marker.header.frame_id = "map";
            sphere_marker.ns = "original_waypoints";
            sphere_marker.id = i + 10;  // IDs altos para waypoints originales
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            
            sphere_marker.pose.position.x = original_waypoints_[i][0];
            sphere_marker.pose.position.y = original_waypoints_[i][1];
            sphere_marker.pose.position.z = original_waypoints_[i][2];
            sphere_marker.pose.orientation.w = 1.0;
            
            sphere_marker.scale.x = 0.2;
            sphere_marker.scale.y = 0.2;
            sphere_marker.scale.z = 0.2;
            sphere_marker.color.r = 1.0;
            sphere_marker.color.g = 0.0;
            sphere_marker.color.b = 0.0;
            sphere_marker.color.a = 1.0;
            
            marker_array.markers.push_back(sphere_marker);
        }
        
        // Marcadores para waypoints interpolados actuales (esferas pequeñas)
        for (size_t i = 0; i < interpolated_waypoints_.size(); ++i) {
            auto sphere_marker = visualization_msgs::msg::Marker();
            sphere_marker.header.stamp = this->now();
            sphere_marker.header.frame_id = "map";
            sphere_marker.ns = "interpolated_waypoints";
            sphere_marker.id = i;
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            
            sphere_marker.pose.position.x = interpolated_waypoints_[i][0];
            sphere_marker.pose.position.y = interpolated_waypoints_[i][1];
            sphere_marker.pose.position.z = interpolated_waypoints_[i][2];
            sphere_marker.pose.orientation.w = 1.0;
            
            sphere_marker.scale.x = 0.1;
            sphere_marker.scale.y = 0.1;
            sphere_marker.scale.z = 0.1;
            
            if (i == current_waypoint_) {
                sphere_marker.color.r = 1.0;
                sphere_marker.color.g = 1.0;
                sphere_marker.color.b = 0.0;  // Amarillo para waypoint actual
            } else if (i < current_waypoint_) {
                sphere_marker.color.r = 0.5;
                sphere_marker.color.g = 0.5;
                sphere_marker.color.b = 0.5;  // Gris para waypoints pasados
            } else {
                sphere_marker.color.r = 0.0;
                sphere_marker.color.g = 0.0;
                sphere_marker.color.b = 1.0;  // Azul para waypoints futuros
            }
            sphere_marker.color.a = 0.7;
            
            marker_array.markers.push_back(sphere_marker);
        }
        
        marker_pub_->publish(marker_array);
    }

    void publishStaticTF()
    {
        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(transform);
    }

    void advanceToNextWaypoint()
    {
        current_waypoint_++;
        
        if (current_waypoint_ >= interpolated_waypoints_.size()) {
            if (loop_route_) {
                RCLCPP_INFO(this->get_logger(), "Ruta completada! Reiniciando...");
                current_waypoint_ = 0;
            } else {
                RCLCPP_INFO(this->get_logger(), "Ruta completada! Finalizando.");
                return;
            }
        }
        
        publishCurrentWaypoint();
    }

    // Variables
    std::vector<std::array<double, 3>> original_waypoints_;
    std::vector<std::array<double, 3>> interpolated_waypoints_;
    size_t current_waypoint_ = 0;
    bool loop_route_;
    int interpolation_points_;
    
    // ROS2
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr start_timer_;  // Timer para el retraso inicial (MANTENIDO)
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}