#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vector>
#include <array>

class RoutePublisher : public rclcpp::Node
{
public:
    RoutePublisher() : Node("route_publisher")
    {
        // Publisher
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_pose", 10);
        
        // Parámetros
        this->declare_parameter("waypoint_duration", 4.0);  // Segundos en cada waypoint
        this->declare_parameter("loop_route", true);
        
        waypoint_duration_ = this->get_parameter("waypoint_duration").as_double();
        loop_route_ = this->get_parameter("loop_route").as_bool();
        
        // Definir waypoints de la ruta
        defineRoute();
        
        // Publicar el primer waypoint inmediatamente
        publishCurrentWaypoint();
        
        // Timer para cambiar de waypoint automáticamente
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(waypoint_duration_),
            std::bind(&RoutePublisher::advanceToNextWaypoint, this));
        
        RCLCPP_INFO(this->get_logger(), "Route publisher started with %zu waypoints", waypoints_.size());
    }

private:
    void defineRoute()
    {
        waypoints_ = {
            {1.0, 0.0, 0.0}, //x+ para arriba e y- para derecha, 
            {3.0, 0.0, 0.0},       
            {3.0, -2.0, 0.0},
            {2.0, -2.0, 0.0},            
            {2.0, -3.0, 0.0}
        };
    }

    void publishCurrentWaypoint()
    {
        auto goal_msg = geometry_msgs::msg::PointStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        
        const auto& wp = waypoints_[current_waypoint_];
        goal_msg.point.x = wp[0];
        goal_msg.point.y = wp[1];
        goal_msg.point.z = wp[2];
        
        goal_pub_->publish(goal_msg);
        
        RCLCPP_INFO(this->get_logger(), "Publicado waypoint %zu/%zu: (%.1f, %.1f)", 
                   current_waypoint_ + 1, waypoints_.size(), wp[0], wp[1]);
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
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}