#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class SimpleMapperTF : public rclcpp::Node {
public:
    SimpleMapperTF() : Node("simple_mapper_tf") {
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_filtered", qos, std::bind(&SimpleMapperTF::scanCallback, this, std::placeholders::_1));

        width_ = 800;    
        height_ = 800;   
        resolution_ = 0.05; 
        origin_x_ = -20.0;
        origin_y_ = -20.0;

        map_data_.resize(width_ * height_, -1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&SimpleMapperTF::publishMap, this));

        RCLCPP_INFO(this->get_logger(), "Mapper TF Iniciado. Esperando scan...");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::vector<int8_t> map_data_;
    int width_, height_;
    double resolution_, origin_x_, origin_y_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {

            transform_stamped = tf_buffer_->lookupTransform(
                "odom", msg->header.frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "No se pudo transformar scan: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PointStamped sensor_origin_local, sensor_origin_global;
        sensor_origin_local.header.frame_id = msg->header.frame_id;
        sensor_origin_local.point.x = 0.0;
        sensor_origin_local.point.y = 0.0;
        sensor_origin_local.point.z = 0.0;
        tf2::doTransform(sensor_origin_local, sensor_origin_global, transform_stamped);

        int rob_grid_x = (int)((sensor_origin_global.point.x - origin_x_) / resolution_);
        int rob_grid_y = (int)((sensor_origin_global.point.y - origin_y_) / resolution_);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];

            if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            geometry_msgs::msg::PointStamped p_local, p_global;
            p_local.header.frame_id = msg->header.frame_id;
            p_local.point.x = r * cos(angle);
            p_local.point.y = r * sin(angle);
            p_local.point.z = 0.0;

            tf2::doTransform(p_local, p_global, transform_stamped);

            int hit_grid_x = (int)((p_global.point.x - origin_x_) / resolution_);
            int hit_grid_y = (int)((p_global.point.y - origin_y_) / resolution_);

            bresenhamLine(rob_grid_x, rob_grid_y, hit_grid_x, hit_grid_y);

            if (isValid(hit_grid_x, hit_grid_y)) {
                map_data_[hit_grid_y * width_ + hit_grid_x] = 100; 
            }
        }
    }

    void publishMap() {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map"; 
        
        map_msg.info.resolution = resolution_;
        map_msg.info.width = width_;
        map_msg.info.height = height_;
        map_msg.info.origin.position.x = origin_x_;
        map_msg.info.origin.position.y = origin_y_;
        map_msg.info.origin.orientation.w = 1.0;
        
        map_msg.data = map_data_;
        map_pub_->publish(map_msg);
    }

    void bresenhamLine(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            if (x0 == x1 && y0 == y1) break;
            if (isValid(x0, y0)) {
                int idx = y0 * width_ + x0;
                if (map_data_[idx] != 100) map_data_[idx] = 0; 
            }
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    bool isValid(int x, int y) {
        return (x >= 0 && x < width_ && y >= 0 && y < height_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMapperTF>());
    rclcpp::shutdown();
    return 0;
}