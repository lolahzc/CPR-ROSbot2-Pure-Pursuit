#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp" 
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <array>

class GridPlanner : public rclcpp::Node {
public:
    GridPlanner() : Node("grid_planner") {
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&GridPlanner::mapCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&GridPlanner::odomCallback, this, std::placeholders::_1));
            
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&GridPlanner::goalCallback, this, std::placeholders::_1));

        this->declare_parameter("interpolation_points_per_segment", 100); 
        this->declare_parameter("knot_distance", 2.0);

        interpolation_points_ = this->get_parameter("interpolation_points_per_segment").as_int();
        knot_distance_ = this->get_parameter("knot_distance").as_double();

        RCLCPP_INFO(this->get_logger(), "GridPlanner (Estilo RoutePublisher) Listo.");
    }

private:
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;
    double robot_x = 0.0, robot_y = 0.0;
    
    int interpolation_points_;
    double knot_distance_;

    std::vector<std::array<double, 3>> original_waypoints_; 
    std::vector<std::array<double, 3>> spline_waypoints_;   

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        map_received_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "¡Esperando mapa!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Calculando ruta a (%.2f, %.2f)...", msg->pose.position.x, msg->pose.position.y);
        processRoute(msg->pose.position.x, msg->pose.position.y);
    }

    void processRoute(double target_x, double target_y) {
        std::vector<std::pair<int, int>> raw_indices = computeBFS(target_x, target_y);
        
        if (raw_indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No se encontró camino BFS.");
            return;
        }

        extractKnots(raw_indices);

        if (original_waypoints_.size() < 2) {
             RCLCPP_WARN(this->get_logger(), "Ruta demasiado corta para spline.");
             return;
        }

        generateSplineRoute();

        publishWaypointsPath();

        publishWaypointsMarkers();
        
        RCLCPP_INFO(this->get_logger(), "Ruta publicada: %lu puntos originales -> %lu puntos interpolados.", 
            original_waypoints_.size(), spline_waypoints_.size());
    }

    void extractKnots(const std::vector<std::pair<int, int>>& raw_indices) {
        original_waypoints_.clear();
        
        std::vector<std::pair<int, int>> forward_path = raw_indices;
        std::reverse(forward_path.begin(), forward_path.end());

        double res = current_map_.info.resolution;
        double ox = current_map_.info.origin.position.x;
        double oy = current_map_.info.origin.position.y;

        original_waypoints_.push_back({robot_x, robot_y, 0.0});

        double last_x = robot_x;
        double last_y = robot_y;

        for (size_t i = 1; i < forward_path.size(); ++i) {
            double wx = forward_path[i].first * res + ox;
            double wy = forward_path[i].second * res + oy;

            double dist = std::hypot(wx - last_x, wy - last_y);

            if (dist >= knot_distance_) {
                original_waypoints_.push_back({wx, wy, 0.0});
                last_x = wx;
                last_y = wy;
            }
        }
        
        double goal_wx = forward_path.back().first * res + ox;
        double goal_wy = forward_path.back().second * res + oy;
        
        if (std::hypot(goal_wx - last_x, goal_wy - last_y) > 0.1) {
             original_waypoints_.push_back({goal_wx, goal_wy, 0.0});
        }
    }

    void generateSplineRoute()
    {
        spline_waypoints_.clear();
        
        if (original_waypoints_.size() < 2) {
            spline_waypoints_ = original_waypoints_;
            return;
        }

        std::vector<double> x_vals, y_vals;
        for (const auto& wp : original_waypoints_) {
            x_vals.push_back(wp[0]);
            y_vals.push_back(wp[1]);
        }

        std::vector<double> x_spline = computeNaturalCubicSpline(x_vals);
        std::vector<double> y_spline = computeNaturalCubicSpline(y_vals);

        int total_points = (original_waypoints_.size() - 1) * interpolation_points_;
        for (int i = 0; i <= total_points; ++i) {
            double t = static_cast<double>(i) / total_points;
            
            int segment = static_cast<int>(t * (original_waypoints_.size() - 1));
            segment = std::min(segment, static_cast<int>(original_waypoints_.size() - 2));
            
            double local_t = t * (original_waypoints_.size() - 1) - segment;
            local_t = std::clamp(local_t, 0.0, 1.0);
            
            double x = evaluateCubicSpline(x_vals, x_spline, segment, local_t);
            double y = evaluateCubicSpline(y_vals, y_spline, segment, local_t);
            
            spline_waypoints_.push_back({x, y, 0.0});
        }
    }

    std::vector<double> computeNaturalCubicSpline(const std::vector<double>& points)
    {
        int n = points.size() - 1;
        std::vector<double> gamma(n + 1), delta(n + 1), D(n + 1);
        
        gamma[0] = 0.5;
        for (int i = 1; i < n; ++i) gamma[i] = 1.0 / (4.0 - gamma[i - 1]);
        gamma[n] = 1.0 / (2.0 - gamma[n - 1]);
        
        delta[0] = 3.0 * (points[1] - points[0]) * gamma[0];
        for (int i = 1; i < n; ++i) delta[i] = (3.0 * (points[i + 1] - points[i - 1]) - delta[i - 1]) * gamma[i];
        delta[n] = (3.0 * (points[n] - points[n - 1]) - delta[n - 1]) * gamma[n];
        
        D[n] = delta[n];
        for (int i = n - 1; i >= 0; --i) D[i] = delta[i] - gamma[i] * D[i + 1];
        
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

    void publishWaypointsPath()
    {
        auto path_msg = geometry_msgs::msg::PoseArray();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
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
        
        auto clear_marker = visualization_msgs::msg::Marker();
        clear_marker.header.stamp = this->now();
        clear_marker.header.frame_id = "map";
        clear_marker.ns = "waypoints";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
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
        
        marker_pub_->publish(marker_array);
    }

    std::vector<std::pair<int, int>> computeBFS(double target_x, double target_y) {
        double res = current_map_.info.resolution;
        double ox = current_map_.info.origin.position.x;
        double oy = current_map_.info.origin.position.y;
        int w = current_map_.info.width;
        int h = current_map_.info.height;

        int start_x = (int)((robot_x - ox) / res);
        int start_y = (int)((robot_y - oy) / res);
        int goal_x = (int)((target_x - ox) / res);
        int goal_y = (int)((target_y - oy) / res);

        int start_idx = start_y * w + start_x;
        int goal_idx = goal_y * w + goal_x;

        if (!isValid(start_x, start_y, w, h) || !isValid(goal_x, goal_y, w, h)) {
            RCLCPP_ERROR(this->get_logger(), "Inicio o Fin fuera de límites");
            return {};
        }

        std::vector<int> came_from(w * h, -1);
        std::queue<int> frontier;
        std::vector<bool> visited(w * h, false);

        frontier.push(start_idx);
        visited[start_idx] = true;
        
        int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
        int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};
        bool found = false;

        while (!frontier.empty()) {
            int current = frontier.front();
            frontier.pop();

            if (current == goal_idx) { found = true; break; }

            int cx = current % w;
            int cy = current / w;

            for (int i = 0; i < 8; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];
                int n_idx = ny * w + nx;

                if (isValid(nx, ny, w, h)) {
                    if (!visited[n_idx] && current_map_.data[n_idx] < 50 && current_map_.data[n_idx] != -1) {
                        visited[n_idx] = true;
                        frontier.push(n_idx);
                        came_from[n_idx] = current;
                    }
                }
            }
        }

        if (!found) return {};

        std::vector<std::pair<int, int>> path;
        int curr = goal_idx;
        while (curr != start_idx) {
            path.push_back({curr % w, curr / w});
            curr = came_from[curr];
        }
        return path; 
    }

    bool isValid(int x, int y, int w, int h) {
        return (x >= 0 && x < w && y >= 0 && y < h);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridPlanner>());
    rclcpp::shutdown();
    return 0;
}