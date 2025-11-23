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
        
        // Generar ruta interpolada con splines cúbicos
        generateSplineRoute();
        
        // Timer para publicar TF estático del mapa
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoutePublisher::publishStaticTF, this));

        // Timer para publicar el primer waypoint con retraso (MANTENIDO)
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),  // 5s de retraso
            [this]() {
                // Publicar el primer waypoint después del retraso
                current_waypoint_ = 0;
                publishCurrentWaypoint();
                publishWaypointsMarkers();
                
                // Cancelar este timer después de ejecutarse una vez
                start_timer_->cancel();
                
                RCLCPP_INFO(this->get_logger(), "First waypoint published after delay");
                RCLCPP_INFO(this->get_logger(), "Original waypoints: %zu, Spline waypoints: %zu", 
                           original_waypoints_.size(), spline_waypoints_.size());
            });

        RCLCPP_INFO(this->get_logger(), "Route publisher started with %zu original waypoints", original_waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Generated %zu spline waypoints", spline_waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Publishing to /goal_pose (PoseStamped)");
        RCLCPP_INFO(this->get_logger(), "Waiting for goal reached signal to advance to next waypoint");
        RCLCPP_INFO(this->get_logger(), "First waypoint will be published in 5s...");
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

    // Implementación de splines cúbicos naturales
    void generateSplineRoute()
    {
        spline_waypoints_.clear();
        
        if (original_waypoints_.size() < 2) {
            spline_waypoints_ = original_waypoints_;
            return;
        }

        // Extraer coordenadas x e y
        std::vector<double> x_vals, y_vals;
        for (const auto& wp : original_waypoints_) {
            x_vals.push_back(wp[0]);
            y_vals.push_back(wp[1]);
        }

        // Calcular splines cúbicos naturales para x e y
        std::vector<double> x_spline = computeNaturalCubicSpline(x_vals);
        std::vector<double> y_spline = computeNaturalCubicSpline(y_vals);

        // Generar puntos interpolados usando el parámetro t
        int total_points = (original_waypoints_.size() - 1) * interpolation_points_;
        for (int i = 0; i <= total_points; ++i) {
            double t = static_cast<double>(i) / total_points;
            
            // Encontrar el segmento correspondiente
            int segment = static_cast<int>(t * (original_waypoints_.size() - 1));
            segment = std::min(segment, static_cast<int>(original_waypoints_.size() - 2));
            
            double local_t = t * (original_waypoints_.size() - 1) - segment;
            local_t = std::clamp(local_t, 0.0, 1.0);
            
            // Evaluar spline cúbico
            double x = evaluateCubicSpline(x_vals, x_spline, segment, local_t);
            double y = evaluateCubicSpline(y_vals, y_spline, segment, local_t);
            
            spline_waypoints_.push_back({x, y, 0.0});
        }

        // Asegurar que pasamos exactamente por los waypoints originales
        for (size_t i = 0; i < original_waypoints_.size(); ++i) {
            double t = static_cast<double>(i) / (original_waypoints_.size() - 1);
            int index = static_cast<int>(t * total_points);
            if (index < spline_waypoints_.size()) {
                spline_waypoints_[index] = original_waypoints_[i];
            }
        }
    }

    std::vector<double> computeNaturalCubicSpline(const std::vector<double>& points)
    {
        int n = points.size() - 1;
        std::vector<double> gamma(n + 1);
        std::vector<double> delta(n + 1);
        std::vector<double> D(n + 1);
        
        gamma[0] = 0.5;
        for (int i = 1; i < n; ++i) {
            gamma[i] = 1.0 / (4.0 - gamma[i - 1]);
        }
        gamma[n] = 1.0 / (2.0 - gamma[n - 1]);
        
        delta[0] = 3.0 * (points[1] - points[0]) * gamma[0];
        for (int i = 1; i < n; ++i) {
            delta[i] = (3.0 * (points[i + 1] - points[i - 1]) - delta[i - 1]) * gamma[i];
        }
        delta[n] = (3.0 * (points[n] - points[n - 1]) - delta[n - 1]) * gamma[n];
        
        D[n] = delta[n];
        for (int i = n - 1; i >= 0; --i) {
            D[i] = delta[i] - gamma[i] * D[i + 1];
        }
        
        return D;
    }

    double evaluateCubicSpline(const std::vector<double>& points, const std::vector<double>& D, int i, double t)
    {
        double a = points[i];
        double b = D[i];
        double c = 3.0 * (points[i + 1] - points[i]) - 2.0 * D[i] - D[i + 1];
        double d = 2.0 * (points[i] - points[i + 1]) + D[i] + D[i + 1];
        
        return a + b * t + c * t * t + d * t * t * t;
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
        
        const auto& wp = spline_waypoints_[current_waypoint_];
        goal_msg.pose.position.x = wp[0];
        goal_msg.pose.position.y = wp[1];
        goal_msg.pose.position.z = wp[2];
        
        // Calcular orientación hacia el siguiente waypoint para curvas más suaves
        if (current_waypoint_ < spline_waypoints_.size() - 1) {
            const auto& next_wp = spline_waypoints_[current_waypoint_ + 1];
            double yaw = std::atan2(next_wp[1] - wp[1], next_wp[0] - wp[0]);
            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = std::sin(yaw / 2.0);
            goal_msg.pose.orientation.w = std::cos(yaw / 2.0);
        } else {
            goal_msg.pose.orientation.w = 1.0;  // Orientación neutral para el último punto
        }
        
        goal_pub_->publish(goal_msg);
        
        // Publicar ruta completa como PoseArray
        publishWaypointsPath();
        
        // Publicar marcadores actualizados
        publishWaypointsMarkers();
        
        RCLCPP_INFO(this->get_logger(), "Publicado waypoint %zu/%zu: (%.2f, %.2f)", 
                   current_waypoint_ + 1, spline_waypoints_.size(), wp[0], wp[1]);
    }

    void publishWaypointsPath()
    {
        auto path_msg = geometry_msgs::msg::PoseArray();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        // Publicar waypoints de spline
        for (const auto& wp : spline_waypoints_) {
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
        
        // Marcador para la ruta spline (línea verde suave)
        auto spline_line_marker = visualization_msgs::msg::Marker();
        spline_line_marker.header.stamp = this->now();
        spline_line_marker.header.frame_id = "map";
        spline_line_marker.ns = "spline_route";
        spline_line_marker.id = 0;
        spline_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        spline_line_marker.action = visualization_msgs::msg::Marker::ADD;
        spline_line_marker.scale.x = 0.08;
        spline_line_marker.color.r = 0.0;
        spline_line_marker.color.g = 1.0;
        spline_line_marker.color.b = 0.0;
        spline_line_marker.color.a = 1.0;
        
        for (const auto& wp : spline_waypoints_) {
            geometry_msgs::msg::Point point;
            point.x = wp[0];
            point.y = wp[1];
            point.z = wp[2];
            spline_line_marker.points.push_back(point);
        }
        marker_array.markers.push_back(spline_line_marker);
        
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
        
        // Marcadores para waypoints originales (esferas grandes rojas)
        for (size_t i = 0; i < original_waypoints_.size(); ++i) {
            auto sphere_marker = visualization_msgs::msg::Marker();
            sphere_marker.header.stamp = this->now();
            sphere_marker.header.frame_id = "map";
            sphere_marker.ns = "original_waypoints";
            sphere_marker.id = i + 10;
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            
            sphere_marker.pose.position.x = original_waypoints_[i][0];
            sphere_marker.pose.position.y = original_waypoints_[i][1];
            sphere_marker.pose.position.z = original_waypoints_[i][2];
            sphere_marker.pose.orientation.w = 1.0;
            
            sphere_marker.scale.x = 0.25;
            sphere_marker.scale.y = 0.25;
            sphere_marker.scale.z = 0.25;
            sphere_marker.color.r = 1.0;
            sphere_marker.color.g = 0.0;
            sphere_marker.color.b = 0.0;
            sphere_marker.color.a = 1.0;
            
            marker_array.markers.push_back(sphere_marker);
        }
        
        // Marcadores para waypoints spline actuales (esferas pequeñas coloreadas)
        for (size_t i = 0; i < spline_waypoints_.size(); ++i) {
            auto sphere_marker = visualization_msgs::msg::Marker();
            sphere_marker.header.stamp = this->now();
            sphere_marker.header.frame_id = "map";
            sphere_marker.ns = "spline_waypoints";
            sphere_marker.id = i;
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            
            sphere_marker.pose.position.x = spline_waypoints_[i][0];
            sphere_marker.pose.position.y = spline_waypoints_[i][1];
            sphere_marker.pose.position.z = spline_waypoints_[i][2];
            sphere_marker.pose.orientation.w = 1.0;
            
            sphere_marker.scale.x = 0.08;
            sphere_marker.scale.y = 0.08;
            sphere_marker.scale.z = 0.08;
            
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
                sphere_marker.color.g = 0.7;
                sphere_marker.color.b = 1.0;  // Azul claro para waypoints futuros
            }
            sphere_marker.color.a = 0.8;
            
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
        
        if (current_waypoint_ >= spline_waypoints_.size()) {
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
    std::vector<std::array<double, 3>> spline_waypoints_;
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