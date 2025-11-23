#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <array>
#include <cmath>

class RoutePublisher : public rclcpp::Node
{
public:
    RoutePublisher() : Node("route_publisher")
    {
        // Publishers
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_pose", 10);
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);
        
        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Parámetros
        this->declare_parameter("waypoint_duration", 4.0);
        this->declare_parameter("loop_route", true);
        
        waypoint_duration_ = this->get_parameter("waypoint_duration").as_double();
        loop_route_ = this->get_parameter("loop_route").as_bool();
        
        // Definir waypoints de la ruta
        defineRoute();
        
        // Publicar el primer waypoint inmediatamente
        publishCurrentWaypoint();
        
        // Publicar marcadores
        publishWaypointsMarkers();
        
        // Timer para cambiar de waypoint automáticamente
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(waypoint_duration_),
            std::bind(&RoutePublisher::advanceToNextWaypoint, this));
            
        // Timer para publicar TF estático del mapa
        tf_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RoutePublisher::publishStaticTF, this));

        RCLCPP_INFO(this->get_logger(), "Route publisher started with %zu waypoints", waypoints_.size());
    }

private:
    void defineRoute()
    {
        waypoints_ = {
            {1.0, 0.0, 0.0}, //x+ para arriba e y- para derecha, incrementales 
            {2.0, 0.0, 0.0}, 
            {2.5, -0.5, 0.0},
            {3.0, -1.0, 0.0},
            {3.0, -2.0, 0.0},
            {2.5, -2.5, 0.0},
            {2.0, -2.5, 0.0}
        };
    }

    void publishCurrentWaypoint()
    {
        // Publicar goal point
        auto goal_msg = geometry_msgs::msg::PointStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        
        const auto& wp = waypoints_[current_waypoint_];
        goal_msg.point.x = wp[0];
        goal_msg.point.y = wp[1];
        goal_msg.point.z = wp[2];
        
        goal_pub_->publish(goal_msg);
        
        // Publicar ruta completa como PoseArray
        publishWaypointsPath();
        
        // Publicar marcadores actualizados
        publishWaypointsMarkers();
        
        RCLCPP_INFO(this->get_logger(), "Publicado waypoint %zu/%zu: (%.1f, %.1f)", 
                   current_waypoint_ + 1, waypoints_.size(), wp[0], wp[1]);
    }

    void publishWaypointsPath()
    {
        auto path_msg = geometry_msgs::msg::PoseArray();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = wp[0];
            pose.position.y = wp[1];
            pose.position.z = wp[2];
            pose.orientation.w = 1.0;  // Orientación neutral
            
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
        
        // Marcador para la ruta completa (línea)
        auto line_marker = visualization_msgs::msg::Marker();
        line_marker.header.stamp = this->now();
        line_marker.header.frame_id = "map";
        line_marker.ns = "waypoints";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.1;  // Grosor de la línea
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        
        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::Point point;
            point.x = wp[0];
            point.y = wp[1];
            point.z = wp[2];
            line_marker.points.push_back(point);
        }
        marker_array.markers.push_back(line_marker);
        
        // Marcadores para cada waypoint individual (esferas)
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            auto sphere_marker = visualization_msgs::msg::Marker();
            sphere_marker.header.stamp = this->now();
            sphere_marker.header.frame_id = "map";
            sphere_marker.ns = "waypoints";
            sphere_marker.id = i + 1;  // ID único para cada waypoint
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            
            sphere_marker.pose.position.x = waypoints_[i][0];
            sphere_marker.pose.position.y = waypoints_[i][1];
            sphere_marker.pose.position.z = waypoints_[i][2];
            sphere_marker.pose.orientation.w = 1.0;
            
            sphere_marker.scale.x = 0.3;
            sphere_marker.scale.y = 0.3;
            sphere_marker.scale.z = 0.3;
            
            // Color diferente para el waypoint actual
            if (i == current_waypoint_) {
                sphere_marker.color.r = 1.0;  // Rojo para waypoint actual
                sphere_marker.color.g = 0.0;
                sphere_marker.color.b = 0.0;
            } else {
                sphere_marker.color.r = 0.0;  // Azul para otros waypoints
                sphere_marker.color.g = 0.0;
                sphere_marker.color.b = 1.0;
            }
            sphere_marker.color.a = 1.0;
            
            marker_array.markers.push_back(sphere_marker);
        }
        
        marker_pub_->publish(marker_array);
    }

    void publishStaticTF()
    {
        // Publicar transformación estática del mapa
        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        
        // Transformación identidad
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
        
        if (current_waypoint_ >= waypoints_.size()) {
            if (loop_route_) {
                RCLCPP_INFO(this->get_logger(), "Ruta completada! Reiniciando...");
                current_waypoint_ = 0;
            } else {
                RCLCPP_INFO(this->get_logger(), "Ruta completada! Finalizando.");
                timer_->cancel();
                tf_timer_->cancel();
                return;
            }
        }
        
        publishCurrentWaypoint();
    }

    // Variables
    std::vector<std::array<double, 3>> waypoints_;
    size_t current_waypoint_ = 0;
    double waypoint_duration_;
    bool loop_route_;
    
    // ROS2
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}