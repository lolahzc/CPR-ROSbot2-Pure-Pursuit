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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <vector>
#include <algorithm> 
#include <fstream> 
#include <iomanip>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // --- PARÁMETROS ORIGINALES ---
        this->declare_parameter("lookahead_distance", 1.5);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.1);
        
        // --- PARÁMETROS DE LA BURBUJA SENSIBLE (NUEVOS) ---
        // Radio base: Zona de seguridad mínima cuando el robot está quieto
        this->declare_parameter("bubble_base_radius", 0.30);
        // Factor de velocidad: Cuánto crece la burbuja por cada m/s (R = Base + Factor * Vel)
        this->declare_parameter("bubble_speed_factor", 1.0);
        // Ganancia repulsiva: Cuánto gira el robot para evitar el obstáculo
        this->declare_parameter("repulsive_gain", 2.0);
        // Distancia crítica: Si algo entra aquí, paramos (EMERGENCY)
        this->declare_parameter("critical_distance", 0.20);
        // Campo de visión de la burbuja (para no asustarse por cosas detrás)
        this->declare_parameter("bubble_fov_degrees", 190.0);
        
        // Lectura de parámetros
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        
        bubble_base_radius_ = this->get_parameter("bubble_base_radius").as_double();
        bubble_speed_factor_ = this->get_parameter("bubble_speed_factor").as_double();
        repulsive_gain_ = this->get_parameter("repulsive_gain").as_double();
        critical_distance_ = this->get_parameter("critical_distance").as_double();
        scan_fov_rad_ = (this->get_parameter("bubble_fov_degrees").as_double()) * M_PI / 180.0;

        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publicadores
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // [NUEVO] Visualización de la burbuja en RViz
        bubble_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/bubble_viz", 10);

        // Subscriptores
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

        // Logs
        data_log_file_.open("robot_data_log.csv");
        if (data_log_file_.is_open()) {
            data_log_file_ << "time,robot_x,robot_y,robot_yaw,state,bubble_radius\n";
        }

        robot_x_ = 0.0; robot_y_ = 0.0; robot_yaw_ = 0.0;
        robot_linear_vel_ = 0.0;

        // Timers
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuitNode::controlLoop, this));
            
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PurePursuitNode::publishOdomAndTF, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit + Sensitive Bubble Node Initialized.");
    }

private:
    // --- MÁQUINA DE ESTADOS ACTUALIZADA ---
    enum class AvoidanceState { 
        NORMAL,           // Pure Pursuit sin obstáculos
        BUBBLE_AVOIDANCE, // Obstáculo detectado en la burbuja (Evasión suave)
        EMERGENCY         // Obstáculo crítico (Parada)
    };

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_goal_ = msg->pose;
        has_goal_ = true;
        goal_reached_ = false;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_linear_vel_ = msg->twist.twist.linear.x; // Guardamos velocidad para la burbuja

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_yaw_);
        robot_yaw_ = normalizeAngle(robot_yaw_);
    }

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose.position);
        }
        has_path_ = true;
        current_path_index_ = 0;
    }

    // --- CALLBACK DEL LIDAR (TAL CUAL TU CÓDIGO 1) ---
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
        // Aquí no procesamos lógica compleja, lo hacemos en el controlLoop para
        // tener acceso al estado y modificar la burbuja.
    }

    // --- LÓGICA DE CONTROL PRINCIPAL ---
    void controlLoop()
    {
        if (!has_goal_ || !has_path_ || path_points_.empty()) {
            stopRobot();
            return;
        }

        // 1. Chequeo de Goal
        double dx = current_goal_.position.x - robot_x_;
        double dy = current_goal_.position.y - robot_y_;
        if (std::sqrt(dx*dx + dy*dy) < goal_tolerance_) {
            if (!goal_reached_) {
                goal_reached_ = true;
                goal_reached_pub_->publish(std_msgs::msg::Bool().set__data(true));
                stopRobot();
            }
            return;
        }

        // 2. Calcular Comando Pure Pursuit Base
        geometry_msgs::msg::Point lookahead;
        bool found = findSequentialLookaheadPoint(lookahead);
        if(!found) lookahead = current_goal_.position;
        publishLookaheadMarker(lookahead);

        double curvature = calculateCurvature(lookahead);
        auto pp_cmd = calculatePurePursuitCommand(curvature);

        // 3. ACTUALIZAR ESTADO (Burbuja Sensible)
        updateAvoidanceState();

        // 4. EJECUTAR ACCIÓN SEGÚN ESTADO
        geometry_msgs::msg::Twist final_cmd;

        switch (avoidance_state_) {
            case AvoidanceState::NORMAL:
                // Si estamos normal, pasamos el comando de Pure Pursuit tal cual
                final_cmd = pp_cmd;
                break;

            case AvoidanceState::BUBBLE_AVOIDANCE:
                // Aplicamos la evasión de la burbuja
                // Calculamos fuerza repulsiva y modificamos el comando PP
                final_cmd = applyBubbleRepulsion(pp_cmd);
                break;

            case AvoidanceState::EMERGENCY:
                // Parada de seguridad
                final_cmd.linear.x = 0.0;
                final_cmd.angular.z = 0.0;
                break;
        }

        // Publicar comando final
        cmd_vel_pub_->publish(final_cmd);
        
        // Visualizar burbuja actual
        double current_radius = bubble_base_radius_ + (bubble_speed_factor_ * std::abs(robot_linear_vel_));
        publishBubbleViz(current_radius);
    }

    // --- MÉTODOS DE LA BURBUJA SENSIBLE ---
    
    void updateAvoidanceState()
    {
        if (!last_scan_) return;

        double current_radius = bubble_base_radius_ + (bubble_speed_factor_ * std::abs(robot_linear_vel_));
        double min_dist_in_fov = 999.0;
        bool obstacle_in_bubble = false;
        double half_fov = scan_fov_rad_ / 2.0;

        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double r = last_scan_->ranges[i];
            if (std::isnan(r) || std::isinf(r) || r < 0.1) continue;

            // TU CÁLCULO DE ÁNGULO (CÓDIGO 1) + PI
            double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);

            // Solo miramos en el FOV frontal
            if (std::abs(angle) > half_fov) continue;

            if (r < min_dist_in_fov) min_dist_in_fov = r;

            if (r < current_radius) {
                obstacle_in_bubble = true;
            }
        }

        // Lógica de transición de estados
        if (min_dist_in_fov < critical_distance_) {
            avoidance_state_ = AvoidanceState::EMERGENCY;
        } else if (obstacle_in_bubble) {
            avoidance_state_ = AvoidanceState::BUBBLE_AVOIDANCE;
        } else {
            avoidance_state_ = AvoidanceState::NORMAL;
        }
    }

    geometry_msgs::msg::Twist applyBubbleRepulsion(geometry_msgs::msg::Twist pp_cmd)
    {
        geometry_msgs::msg::Twist mod_cmd = pp_cmd;
        
        if (!last_scan_) return mod_cmd;

        double current_radius = bubble_base_radius_ + (bubble_speed_factor_ * std::abs(robot_linear_vel_));
        
        double repulsion_x = 0.0;
        double repulsion_y = 0.0;
        double total_weight = 0.0; 

        // Calcular vector repulsivo
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double r = last_scan_->ranges[i];

            // Filtro básico de validez y chasis
            if (std::isnan(r) || std::isinf(r) || r < 0.1) continue;

            // Si está dentro de la burbuja
            if (r < current_radius) {
                
                // TU CÁLCULO DE ÁNGULO (Correcto con + PI)
                double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);

                // --- PONDERACIÓN CUADRÁTICA (Inverse Distance Weighting) ---
                // Damos muchísima más fuerza a los objetos cercanos que a los lejanos.
                // Fórmula: (1 - (dist / radio))^3
                double weight = (current_radius - r) / current_radius;
                weight = weight * weight * weight; // Al cubo para ser muy agresivo con lo cercano
                
                // Acumulamos el vector opuesto ponderado
                repulsion_x -= std::cos(angle) * weight;
                repulsion_y -= std::sin(angle) * weight;
                
                total_weight += weight;
            }
        }    

        // Si hay obstáculos significativos (peso acumulado > 0)
        if (total_weight > 0.001) {
            // Normalizamos el vector repulsivo por el peso total
            repulsion_x /= total_weight;
            repulsion_y /= total_weight;

            // Calcular ángulo de evasión resultante
            double avoidance_angle = std::atan2(repulsion_y, repulsion_x);

            // --- FUSIÓN DINÁMICA: Pure Pursuit + Repulsión ---
            
            // 1. Ganancia dinámica: 
            // Si total_weight es bajo (pocos obstáculos/lejos), giramos poco.
            // Si total_weight es alto (obstáculos cerca/densos), giramos mucho (hasta un tope de 4.0).
            double dynamic_gain = std::min(repulsive_gain_ + total_weight, 4.0);
            
            // Mezcla: 40% Pure Pursuit, 60% Evasión (ajustado por la ganancia)
            mod_cmd.angular.z = (pp_cmd.angular.z * 0.4) + (avoidance_angle * dynamic_gain * 0.6);
            
            // 2. Frenado inteligente:
            // Cuanto mayor sea la densidad de obstáculos (total_weight), más frenamos.
            // clamp asegura que no bajemos del 10% de la velocidad original.
            double slow_down_factor = std::clamp(1.0 - total_weight, 0.1, 1.0);
            mod_cmd.linear.x = pp_cmd.linear.x * slow_down_factor;
        }

        // Saturación final de seguridad para no exceder límites del robot
        mod_cmd.angular.z = std::clamp(mod_cmd.angular.z, -max_angular_vel_, max_angular_vel_);
        
        return mod_cmd;
    }

    // --- MÉTODOS PURE PURSUIT (CÓDIGO 1) ---
    bool findSequentialLookaheadPoint(geometry_msgs::msg::Point& lookahead_point)
    {
        if (current_path_index_ >= path_points_.size()) return false;
        
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t closest_index = current_path_index_;
        size_t search_limit = std::min(path_points_.size(), current_path_index_ + 100); 

        for (size_t i = current_path_index_; i < search_limit; ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double d2 = dx*dx + dy*dy;
            if (d2 < min_dist_sq) { min_dist_sq = d2; closest_index = i; }
        }
        current_path_index_ = closest_index;

        for (size_t i = current_path_index_; i < path_points_.size(); ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            if (std::sqrt(dx*dx + dy*dy) >= lookahead_distance_) {
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

    double calculateCurvature(const geometry_msgs::msg::Point& pt) {
        double rx = pt.x - robot_x_;
        double ry = pt.y - robot_y_;
        double r_ry = -rx * sin(robot_yaw_) + ry * cos(robot_yaw_); // Y relativo robot
        double L2 = rx*rx + ry*ry;
        return (L2 > 0.001) ? (2.0 * r_ry / L2) : 0.0;
    }

    geometry_msgs::msg::Twist calculatePurePursuitCommand(double curvature) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = max_linear_vel_;
        
        // Reducción en curvas
        if (std::abs(curvature) > 0.5) cmd.linear.x *= 0.6;
        
        cmd.angular.z = curvature * cmd.linear.x;
        cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_vel_, max_angular_vel_);
        return cmd;
    }

    // --- UTILIDADES ---
    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd_vel_pub_->publish(cmd);
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

void publishLookaheadMarker(const geometry_msgs::msg::Point& pt) {
       auto m = visualization_msgs::msg::Marker();
       m.header.frame_id = "odom"; 
       m.header.stamp = now();
       m.id = 0; 
       m.type = 2; // SPHERE
       m.action = 0; // ADD
       m.pose.position = pt; 
       m.pose.orientation.w = 1.0;
       
       m.scale.x = 0.2; 
       m.scale.y = 0.2; 
       m.scale.z = 0.2;
       
       // --- COLOR MORADO ---
       m.color.r = 1.0; // Rojo
       m.color.g = 0.0; // Nada de Verde
       m.color.b = 1.0; // Azul (ESTO FALTABA)
       m.color.a = 1.0; // Alpha (Opacidad)
       
       marker_pub_->publish(m);
   }

    void publishBubbleViz(double radius) {
        auto m = visualization_msgs::msg::Marker();
        m.header.frame_id = "base_link"; m.header.stamp = now();
        m.ns = "bubble"; m.id = 1; m.type = 3; m.action = 0;
        m.scale.x = radius * 2.0; m.scale.y = radius * 2.0; m.scale.z = 0.05;
        
        // Color según estado
        if (avoidance_state_ == AvoidanceState::NORMAL) { m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.2; }
        else if (avoidance_state_ == AvoidanceState::BUBBLE_AVOIDANCE) { m.color.r = 1.0; m.color.g = 1.0; m.color.a = 0.4; }
        else { m.color.r = 1.0; m.color.a = 0.6; }
        
        bubble_viz_pub_->publish(m);
    }
    
    void publishOdomAndTF() {
        // Tu función original de Odom+TF tal cual
        auto now = this->now();
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now; odom_msg.header.frame_id = "odom"; odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = robot_x_; odom_msg.pose.pose.position.y = robot_y_;
        odom_msg.pose.pose.orientation = createQuaternionFromYaw(robot_yaw_);
        odom_pub_->publish(odom_msg);

        auto t = geometry_msgs::msg::TransformStamped();
        t.header.stamp = now; t.header.frame_id = "odom"; t.child_frame_id = "base_link";
        t.transform.translation.x = robot_x_; t.transform.translation.y = robot_y_;
        t.transform.rotation = createQuaternionFromYaw(robot_yaw_);
        tf_broadcaster_->sendTransform(t);
    }

    geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw) {
        tf2::Quaternion q; q.setRPY(0, 0, yaw); return tf2::toMsg(q);
    }

    // Variables
    double robot_x_, robot_y_, robot_yaw_, robot_linear_vel_;
    geometry_msgs::msg::Pose current_goal_;
    std::vector<geometry_msgs::msg::Point> path_points_;
    bool has_goal_ = false; bool has_path_ = false; bool goal_reached_ = false;
    size_t current_path_index_ = 0;
    std::ofstream data_log_file_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    
    AvoidanceState avoidance_state_ = AvoidanceState::NORMAL;

    // Params
    double lookahead_distance_, max_linear_vel_, max_angular_vel_, goal_tolerance_;
    double bubble_base_radius_, bubble_speed_factor_, repulsive_gain_, critical_distance_, scan_fov_rad_;

    // ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_, bubble_viz_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr control_timer_, tf_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
