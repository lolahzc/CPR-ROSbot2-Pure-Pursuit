#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int32.hpp>
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
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);
        
        goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&RoutePublisher::goalReachedCallback, this, std::placeholders::_1));
           
        route_change_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/change_route", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg){
                changeRoute(msg->data);
            });

       
        this->declare_parameter("selected_route", 1);
		selected_route_ = this->get_parameter("selected_route").as_int();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        this->declare_parameter("loop_route", false);
        this->declare_parameter("interpolation_points_per_segment", 10);
        
        loop_route_ = this->get_parameter("loop_route").as_bool();
        interpolation_points_ = this->get_parameter("interpolation_points_per_segment").as_int();
        
        defineRoute();
        
        generateSplineRoute();
        
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoutePublisher::publishStaticTF, this));

        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000), 
            [this]() {
                current_segment_index_ = 0; 
                route_finished_ = false; 
                
                publishWaypointsPath();
                publishWaypointsMarkers();
                
                start_timer_->cancel();
            });
    }

private:

	int selected_route_;
    size_t current_segment_index_ = 0;
    bool route_finished_ = false;

    void defineRoute()
    {
    
     switch (selected_route_) {
        case 1: 
            original_waypoints_ = {
                {0.0,0.0,0.0},
                {1.0,1.0,0.0},
                {2.0,-0.5,0.0},
                {3.0,-2.0,0.0},
                {2.0,-3.0,0.0},
                {1.0,-4.0,0.0},
                {2.0,-4.5,0.0},
                {4.0,-4.5,0.0},
                {5.0,-2.5,0.0}
            };
            break;

        case 2: 
            original_waypoints_ = {
                {0.0,  0.0, 0.0},
                {1.0, 0.0, 0.0},
                {2.0, 0.0, 0.0},
                {3.0, 0.0, 0.0},
                {4.0, 0.0, 0.0},
                {5.0, 0.0, 0.0}
            };
            break;
        case 3:
       		original_waypoints_={
                {0.0, 0.0, 0.0},
       			{1.0, 0.0, 0.0},    
	   		    {1.0, -2.0, 0.0},   
	     		{3.0, -2.0, 0.0},   
	     		{3.0, -4.0, 0.0},  
	     		{3.0, -5.0, 0.0},   
    			{3.0, -6.0, 0.0}    
	     	};
	     	break;
        case 4:
            original_waypoints_ = {
                {0.0,  0.0, 0.0},  
                {1.5,  1.5, 0.0},
                {3.0,  1.0, 0.0},
                {4.0, -0.5, 0.0},
                {3.0, -2.0, 0.0},
                {1.5, -1.5, 0.0},
                {0.0,  0.0, 0.0}, 
                {-1.5, 1.5, 0.0},
                {-3.0, 1.0, 0.0},
                {-4.0, -0.5, 0.0},
                {-3.0, -2.0, 0.0},
                {-1.5, -1.5, 0.0},
                {0.0,  0.0, 0.0}   
            };
            break;
            break;
	    case 5: 
            original_waypoints_ = {
                {0.0, 0.0, 0.0},
                {1.0, 0.0, 0.0},
                {1.5, 1.0, 0.0},
                {1.0, 2.0, 0.0},
                {0.0, 2.5, 0.0},
                {-1.5, 2.0, 0.0},
                {-2.5, 1.0, 0.0},
                {-3.0, -0.5, 0.0},
                {-2.5, -2.0, 0.0},
                {-1.5, -3.0, 0.0},
                {0.0, -3.5, 0.0},
                {1.5, -3.0, 0.0},
                {3.0, -2.0, 0.0},
                {3.5, -0.5, 0.0},
                {3.0, 1.0, 0.0}
            };
            break; 
            
        case 6: 
            original_waypoints_ = {
                {0.0, 0.0, 0.0}, 
                {1.0, 1.0, 0.0},
                {2.0, -0.5, 0.0},
                {3.0, 1.5, 0.0},
                {2.0, 3.0, 0.0},
                {0.5, 2.0, 0.0},
                {-1.0, 3.5, 0.0},
                {-2.0, 1.0, 0.0},
                {-1.0, -1.0, 0.0},
                {0.5, -2.5, 0.0},
                {2.0, -1.0, 0.0},
                {3.0, -3.0, 0.0},
                {1.5, -4.0, 0.0},
                {0.0, -3.0, 0.0},
                {-1.5, -4.0, 0.0},
                {-3.0, -2.0, 0.0}
	        };
            break;   
            
        case 7: 
            original_waypoints_ = {
                {0.0, 0.0, 0.0},
                {0.6, 0.2, 0.0},   
                {0.0, 0.6, 0.0},   
                {0.2, 0.9, 0.0},   
                {-0.2, 0.95, 0.0},
                {-0.6, 1.2, 0.0},  
                {0.0, 1.5, 0.0},  
                {0.8, 1.65, 0.0},  
                {0.0, 2.0, 0.0},   
                {1.0, 2.4, 0.0},   
                {-0.6, 3.0, 0.0},   
            };
            break;   
            
        case 8: 
   		    original_waypoints_ = {
                {-0.5, 0.0, 0.0},
                {0.0, -5.0, 0.0},
                {-3.0, -5.0, 0.0},
                {-6.5,-5.0, 0.0},
                {-8.0,-3.5, 0.0},
                {-6.0,-1.5, 0.0}
            };  
            break;    

        default:
            selected_route_ = 1;
            defineRoute(); 
            break;
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

        for (size_t i = 0; i < original_waypoints_.size(); ++i) {
            double t = static_cast<double>(i) / (original_waypoints_.size() - 1);
            size_t index = static_cast<size_t>(t * total_points); 
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
        if (route_finished_) {
            return;
        }

        if (msg->data) {
            if (loop_route_) {
                publishWaypointsPath(); 
            } else {
                route_finished_ = true; 
            }
        }
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
        
        if (current_segment_index_ < original_waypoints_.size()) {
            auto goal_marker = visualization_msgs::msg::Marker();
            goal_marker.header.stamp = this->now();
            goal_marker.header.frame_id = "map";
            goal_marker.ns = "current_goal";
            goal_marker.id = 99;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            
            goal_marker.pose.position.x = original_waypoints_[current_segment_index_][0];
            goal_marker.pose.position.y = original_waypoints_[current_segment_index_][1];
            goal_marker.pose.position.z = original_waypoints_[current_segment_index_][2];
            
            goal_marker.scale.x = 0.35;
            goal_marker.scale.y = 0.35;
            goal_marker.scale.z = 0.35;
            goal_marker.color.r = 1.0;
            goal_marker.color.g = 1.0;
            goal_marker.color.b = 0.0; 
            goal_marker.color.a = 0.8;
            
            marker_array.markers.push_back(goal_marker);
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
    
    void changeRoute(int new_route)
    {
        selected_route_ = new_route;
        defineRoute();
        generateSplineRoute();
        current_segment_index_ = 0; 
        route_finished_ = false;
	
	    visualization_msgs::msg::MarkerArray marker_array;
        auto clear_marker = visualization_msgs::msg::Marker();
        clear_marker.header.stamp = this->now();
        clear_marker.header.frame_id = "map";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL; 
        marker_array.markers.push_back(clear_marker);
        marker_pub_->publish(marker_array);	
        
        publishWaypointsPath();
        publishWaypointsMarkers();
    }

    std::vector<std::array<double, 3>> original_waypoints_;
    std::vector<std::array<double, 3>> spline_waypoints_;
    bool loop_route_;
    int interpolation_points_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr route_change_sub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr start_timer_;
};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
