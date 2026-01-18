#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class SimpleMapperTF : public rclcpp::Node {
public:
    SimpleMapperTF() : Node("simple_mapper_tf") {
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local();
        map_qos.reliable();
        
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_filtered", qos, std::bind(&SimpleMapperTF::scanCallback, this, std::placeholders::_1));

        resolution_ = 0.05; 
        width_ = 800;       
        height_ = 800;   
        origin_x_ = -20.0;
        origin_y_ = -20.0;

        grid_counters_.resize(width_ * height_, 0);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SimpleMapperTF::updateLoop, this));

    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::vector<int> grid_counters_;
    int width_, height_;
    double resolution_, origin_x_, origin_y_;

    const int VAL_HIT = 100;    
    const int VAL_MISS = 50;    
    const int DECAY_RATE = 2;   

    const int MAX_VAL = 100;
    const int MIN_VAL = 0;
    const int OCCUPANCY_THRESHOLD = 50; 

    const double MAX_MAPPING_RANGE = 4.5; 

    inline void worldToMap(double wx, double wy, int& mx, int& my) {
        mx = std::floor((wx - origin_x_) / resolution_);
        my = std::floor((wy - origin_y_) / resolution_);
    }

    inline bool isValid(int x, int y) {
        return (x >= 0 && x < width_ && y >= 0 && y < height_);
    }

    inline void updateCell(int x, int y, int amount) {
        if (!isValid(x, y)) return;
        int idx = y * width_ + x;
        int val = grid_counters_[idx] + amount;
        if (val > MAX_VAL) val = MAX_VAL;
        if (val < MIN_VAL) val = MIN_VAL;
        grid_counters_[idx] = val;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform(
                "odom", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        } catch (tf2::TransformException &ex) {
            return;
        }

        tf2::Transform transform;
        tf2::fromMsg(tf_stamped.transform, transform);

        tf2::Vector3 origin_global = transform * tf2::Vector3(0,0,0);
        int rob_grid_x, rob_grid_y;
        worldToMap(origin_global.x(), origin_global.y(), rob_grid_x, rob_grid_y);

        double angle_min = msg->angle_min;
        double angle_inc = msg->angle_increment;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];

            if (std::isinf(r) || std::isnan(r)) {
                continue; 
            }

            if (r > MAX_MAPPING_RANGE) {
                continue;
            }
            
            if (r < msg->range_min) {
                continue;
            }

            double angle = angle_min + i * angle_inc;
            tf2::Vector3 point_local(r * cos(angle), r * sin(angle), 0.0);
            tf2::Vector3 point_global = transform * point_local;

            int hit_grid_x, hit_grid_y;
            worldToMap(point_global.x(), point_global.y(), hit_grid_x, hit_grid_y);

            bresenhamUpdate(rob_grid_x, rob_grid_y, hit_grid_x, hit_grid_y, true);
        }
    }

    void bresenhamUpdate(int x0, int y0, int x1, int y1, bool is_hit) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            if (x0 == x1 && y0 == y1) {
                if (is_hit) updateCell(x0, y0, VAL_HIT); 
                break;
            }
            
            updateCell(x0, y0, -VAL_MISS); 
            
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void updateLoop() {
        applyDecay();
        publishMap();
    }

    void applyDecay() {
        for (auto &val : grid_counters_) {
            if (val > 0) val -= DECAY_RATE; 
        }
    }

    void publishMap() {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "odom";
        
        map_msg.info.resolution = resolution_;
        map_msg.info.width = width_;
        map_msg.info.height = height_;
        map_msg.info.origin.position.x = origin_x_;
        map_msg.info.origin.position.y = origin_y_;
        map_msg.info.origin.orientation.w = 1.0;
        
        map_msg.data.resize(width_ * height_);

        for (size_t i = 0; i < grid_counters_.size(); ++i) {
            if (grid_counters_[i] > OCCUPANCY_THRESHOLD) {
                map_msg.data[i] = 100; 
            } else {
                map_msg.data[i] = 0;   
            }
        }
        map_pub_->publish(map_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMapperTF>());
    rclcpp::shutdown();
    return 0;
}