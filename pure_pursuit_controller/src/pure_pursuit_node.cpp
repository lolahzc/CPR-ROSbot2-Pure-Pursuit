 /**
 * @file pure_pursuit_node.cpp
 * @brief Implementación de Pure Pursuit Controller con evasión dinámica de obstáculos
 * @details Este nodo implementa un controlador Pure Pursuit avanzado para navegación
 * autónoma de robots móviles en ROS 2. Incluye capacidades de:
 * - Seguimiento de trayectorias con lookahead distance adaptativo
 * - Velocidad variable según curvatura
 * - Evasión reactiva de obstáculos mediante curvas de Bézier
 * - Sistema de tres estados de seguridad
 * - Logging automático de datos de navegación
 * @author Pedro Cabello Pulido | Gabriela Cano Azuaga  | Lola Hernández Canizares | 
Almudena Jin | Lucía Pérez Guerrero 
 */
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
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <vector>
#include <algorithm> 
#include <fstream> 
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>

/**
 * @class PurePursuitNode
 * @brief Nodo ROS 2 para control de navegación mediante Pure Pursuit con evasión de obstáculos
 * 
 * Esta clase implementa un sistema completo de navegación autónoma que combina:
 * - Algoritmo Pure Pursuit para seguimiento de trayectorias
 * - Sistema de evasión de obstáculos basado en sensor láser
 * - Generación dinámica de rutas de desvío mediante curvas de Bézier
 * - Ajuste adaptativo de velocidad y lookahead distance
 */
class PurePursuitNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor del nodo Pure Pursuit
     * 
     * Inicializa todos los parámetros, publicadores, suscriptores y temporizadores.
     * También configura el sistema de logging automático con timestamp.
     */
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // --- PARÁMETROS DE NAVEGACIÓN ---
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.2);
        this->declare_parameter("lookahead_min", 1.0);      
        this->declare_parameter("lookahead_max", 3.0);      
        this->declare_parameter("lookahead_gamma", 0.8);    
        this->declare_parameter("selected_route", 1);
        
        // --- PARÁMETROS DE EVASIÓN ---
        this->declare_parameter("bubble_base_radius", 0.5);
        this->declare_parameter("critical_distance", 0.25);
        this->declare_parameter("detour_offset", 1.0);
        this->declare_parameter("rejoin_distance", 2.0);
        this->declare_parameter("forward_offset", 0.5);

        // Lectura de parámetros
        selected_route_ = this->get_parameter("selected_route").as_int();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        bubble_base_radius_ = this->get_parameter("bubble_base_radius").as_double();
        critical_distance_ = this->get_parameter("critical_distance").as_double();
        detour_offset_ = this->get_parameter("detour_offset").as_double();
        rejoin_distance_ = this->get_parameter("rejoin_distance").as_double();
        forward_offset_ = this->get_parameter("forward_offset").as_double();

        delta_min_ = this->get_parameter("lookahead_min").as_double();
        delta_max_ = this->get_parameter("lookahead_max").as_double();
        gamma_ = this->get_parameter("lookahead_gamma").as_double();

        // --- CONFIGURACIÓN DE PUBLICADORES Y SUSCRIPTORES ---
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        bubble_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/bubble_viz", 10);
        robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);
        
        detour_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detour_path", 10);

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

        // --- CONFIGURACIÓN DE LOGS ---
        std::string base_name;
        switch(selected_route_) {
            case 1: base_name = "ruta_defecto"; break;
            case 2: base_name = "ruta_recta"; break;
            case 3: base_name = "ruta_zigzag"; break;
            case 4: base_name = "ruta_ocho"; break;
            case 5: base_name = "ruta_espiral"; break;
            case 6: base_name = "ruta_cur_amplias"; break;
            case 7: base_name = "ruta_cur_cerradas"; break;
            case 8: base_name = "ruta_obstaculo"; break;
            default: base_name = "ruta_desconocida"; break;
        }

        auto now = std::chrono::system_clock::now();
        std::time_t in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "_%Y-%m-%d_%H-%M-%S");

        std::string final_filename = base_name + ss.str() + ".csv";
        data_log_file_.open(final_filename);

        if (data_log_file_.is_open()) {
            data_log_file_ << "time,robot_x,robot_y,robot_yaw,goal_x,goal_y,lookahead,dist_error,path_index,dist_goal,linear_v,angular_w,curvature\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "No se pudo crear el archivo de log: %s", final_filename.c_str());
        }

        robot_x_ = 0.0; robot_y_ = 0.0; robot_yaw_ = 0.0; robot_linear_vel_ = 0.0;
        current_vel_cmd_ = 0.0;

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuitNode::controlLoop, this));
            
        last_detour_time_ = this->now();
    }

private:
    /**
     * @enum AvoidanceState
     * @brief Estados del sistema de evasión de obstáculos
     */
    enum class AvoidanceState { 
        NORMAL,            ///< Sin obstáculos, navegación normal (>0.5m)
        OBSTACLE_DETECTED, ///< Obstáculo en burbuja, genera desvío (0.25-0.5m)
        EMERGENCY         ///< Colisión inminente, parada inmediata (<0.25m)
    };

    /**
     * @brief Callback para recibir el objetivo de navegación
     * @param msg Pose del objetivo en el marco de referencia global
     */
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_goal_ = msg->pose;
        has_goal_ = true;
        goal_reached_ = false;
    }

    /**
     * @brief Callback para recibir la odometría filtrada del robot
     * @param msg Mensaje de odometría con pose y velocidad
     * 
     * Actualiza la posición (x,y), orientación (yaw) y velocidad lineal del robot
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_linear_vel_ = msg->twist.twist.linear.x;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_yaw_);
        robot_yaw_ = normalizeAngle(robot_yaw_);
    }

    /**
     * @brief Callback para recibir datos del sensor láser
     * @param msg Escaneo láser con distancias y ángulos
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
    }

    /**
     * @brief Callback para recibir la trayectoria a seguir
     * @param msg Array de poses que conforman el camino
     * 
     * Solo actualiza la trayectoria si han pasado >3 segundos desde el último desvío
     * para evitar conflictos entre rutas planificadas y desvíos reactivos
     */
    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if ((this->now() - last_detour_time_).seconds() > 3.0) {
            path_points_.clear();
            for (const auto& pose : msg->poses) {
                path_points_.push_back(pose.position);
            }
            has_path_ = true;
            current_path_index_ = 0;
            findClosestIndex();
        }
    }
    
    /**
     * @brief Encuentra el índice del punto más cercano en la trayectoria
     * 
     * Busca en toda la trayectoria el waypoint más próximo a la posición actual
     * del robot para inicializar correctamente el índice de seguimiento
     */
    void findClosestIndex() {
        if (path_points_.empty()) return;
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
    }

    /**
     * @brief Actualiza el índice actual en la trayectoria durante navegación
     * 
     * Busca hacia adelante (hasta 50 puntos) para encontrar el waypoint más cercano,
     * asegurando progreso continuo sin retrocesos
     */
    void updateCurrentPathIndex()
    {
        if (path_points_.empty()) return;

        double min_dist_sq = std::numeric_limits<double>::max();
        size_t best_idx = current_path_index_;
        size_t search_limit = std::min(path_points_.size(), current_path_index_ + 50); 

        for (size_t i = current_path_index_; i < search_limit; ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }
        if (best_idx > current_path_index_) current_path_index_ = best_idx;
    }

    /**
     * @brief Calcula el error de seguimiento lateral (Cross-Track Error)
     * @return Distancia mínima perpendicular a la trayectoria (metros)
     * 
     * Busca entre los próximos 20 waypoints la distancia mínima al robot
     */
    double getCrossTrackError() {
        if (path_points_.empty()) return 0.0;
        double min_dist = 1000.0;
        size_t end_idx = std::min(path_points_.size(), current_path_index_ + 20);
        for (size_t i = current_path_index_; i < end_idx; ++i) {
            double d = std::hypot(path_points_[i].x - robot_x_, path_points_[i].y - robot_y_);
            if (d < min_dist) min_dist = d;
        }
        return min_dist;
    }

    /**
     * @brief Calcula la distancia lookahead adaptativa
     * @param cross_track_error Error lateral actual (m)
     * @param vx Velocidad lineal del robot (m/s)
     * @return Lookahead distance calculado (m)
     * 
     * Fórmula: LHD = v_factor × (δ_max - δ_min) × e^(-γ×|CTE|) + δ_min
     * - Mayor velocidad → Mayor LHD (más anticipación)
     * - Mayor error → Menor LHD (más corrección agresiva)
     */
    double calculateLookaheadDistance(double cross_track_error, double vx) {
        double vel_f = std::clamp(vx/5.0, 0.05, 0.2)*5.0;
        return vel_f * (delta_max_ - delta_min_) * std::exp(-gamma_ * std::abs(cross_track_error)) + delta_min_;
    }

    /**
     * @brief Actualiza el estado del sistema de evasión de obstáculos
     * 
     * Escanea el láser en un FOV de 60° frontal (±30°) y determina:
     * - EMERGENCY: si hay obstáculo < 0.25m
     * - OBSTACLE_DETECTED: si hay obstáculo < 0.5m
     * - NORMAL: si todos los obstáculos > 0.5m
     */
    void updateAvoidanceState() {
        if (!last_scan_) return;

        double min_dist_in_fov = 999.0;
        bool obstacle_in_bubble = false;
        double fov = 60.0 * M_PI / 180.0;

        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double r = last_scan_->ranges[i];
            if (std::isnan(r) || std::isinf(r) || r < 0.1) continue;

            double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);
            
            if (std::abs(angle) > fov/2.0) continue;

            if (r < min_dist_in_fov) min_dist_in_fov = r;
            if (r < bubble_base_radius_) obstacle_in_bubble = true;
        }

        if (min_dist_in_fov < critical_distance_) {
            avoidance_state_ = AvoidanceState::EMERGENCY;
        } else if (obstacle_in_bubble) {
            avoidance_state_ = AvoidanceState::OBSTACLE_DETECTED;
        } else {
            avoidance_state_ = AvoidanceState::NORMAL;
        }
    }

    /**
     * @brief Genera una ruta de desvío para evadir obstáculos
     * 
     * Algoritmo:
     * 1. Escanea 190° (±95°) para analizar entorno completo
     * 2. Calcula pesos por hemisferio: peso = 1/distancia
     * 3. Decide dirección: hacia lado con menor peso total
     * 4. Calcula 3 puntos clave:
     *    - P_inicio: posición actual
     *    - P_ápex: 0.5m adelante + 1.0m lateral
     *    - P_reincorporación: sobre ruta a 2.0m adelante
     * 5. Genera curva de Bézier cuadrática con 15 puntos
     * 6. Reemplaza segmento de ruta original por desvío
     * 
     * @note Solo se ejecuta si han pasado >2s desde último desvío
     */
    void generateDetourPath() {
        if ((this->now() - last_detour_time_).seconds() < 2.0) return;
        if (!last_scan_) return;

        double left_obst_weight = 0;
        double right_obst_weight = 0;
        double fov = 190.0 * M_PI / 180.0;

        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double r = last_scan_->ranges[i];
            if (r > bubble_base_radius_ * 1.5) continue;
            
            double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);
            if (angle > 0 && angle < fov/2) left_obst_weight += (1.0/r);
            if (angle < 0 && angle > -fov/2) right_obst_weight += (1.0/r);
        }

        double side_sign = (left_obst_weight > right_obst_weight) ? -1.0 : 1.0; 

        double heading_x = std::cos(robot_yaw_);
        double heading_y = std::sin(robot_yaw_);
        
        double perp_x = -heading_y * side_sign;
        double perp_y = heading_x * side_sign;

        geometry_msgs::msg::Point p_start;
        p_start.x = robot_x_; 
        p_start.y = robot_y_;

        geometry_msgs::msg::Point p_apex;
        p_apex.x = robot_x_ + (heading_x * forward_offset_) + (perp_x * detour_offset_);
        p_apex.y = robot_y_ + (heading_y * forward_offset_) + (perp_y * detour_offset_);

        size_t rejoin_idx = current_path_index_;
        bool found_rejoin = false;
        double cumulative_dist = 0;

        for (size_t i = current_path_index_; i < path_points_.size() - 1; ++i) {
            double dx = path_points_[i+1].x - path_points_[i].x;
            double dy = path_points_[i+1].y - path_points_[i].y;
            cumulative_dist += std::hypot(dx, dy);
            
            if (cumulative_dist > rejoin_distance_) {
                rejoin_idx = i + 1;
                found_rejoin = true;
                break;
            }
        }
        
        if (!found_rejoin) rejoin_idx = path_points_.size() - 1;

        geometry_msgs::msg::Point p_rejoin = path_points_[rejoin_idx];

        std::vector<geometry_msgs::msg::Point> detour_points;
        int steps = 15;

        for (int i = 1; i <= steps; ++i) {
            double t = (double)i / (double)steps;
            double u = 1 - t;
            
            geometry_msgs::msg::Point p;
            p.x = (u*u * p_start.x) + (2*u*t * p_apex.x) + (t*t * p_rejoin.x);
            p.y = (u*u * p_start.y) + (2*u*t * p_apex.y) + (t*t * p_rejoin.y);
            p.z = 0.0;
            detour_points.push_back(p);
        }

        if (rejoin_idx > current_path_index_) {
            auto it_start = path_points_.begin() + current_path_index_ + 1;
            auto it_end = path_points_.begin() + rejoin_idx;
            
            if (it_start < path_points_.end() && it_end < path_points_.end() && it_start < it_end) {
                path_points_.erase(it_start, it_end);
                path_points_.insert(path_points_.begin() + current_path_index_ + 1, 
                                  detour_points.begin(), detour_points.end());
                                  
                last_detour_time_ = this->now();
                publishDetourPath();
            }
        }
    }

    /**
     * @brief Publica visualización de la trayectoria modificada en RViz
     * 
     * Genera dos marcadores:
     * - LINE_STRIP: línea continua cian mostrando la ruta completa
     * - SPHERE_LIST: esferas magenta en cada waypoint
     */
    void publishDetourPath() {
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "detour_line";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.05; 
        line_strip.color.r = 0.0; line_strip.color.g = 1.0; line_strip.color.b = 1.0; line_strip.color.a = 1.0; 

        visualization_msgs::msg::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = this->now();
        points.ns = "detour_points";
        points.id = 1;
        points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        points.action = visualization_msgs::msg::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.1; points.scale.y = 0.1; points.scale.z = 0.1;
        points.color.r = 1.0; points.color.g = 0.0; points.color.b = 1.0; points.color.a = 1.0; 

        for(const auto& p : path_points_) {
            line_strip.points.push_back(p);
            points.points.push_back(p);
        }

        marker_array.markers.push_back(line_strip);
        marker_array.markers.push_back(points);

        detour_path_pub_->publish(marker_array);
    }

    /**
     * @brief Bucle principal de control (ejecutado a 20Hz - cada 50ms)
     * 
     * Secuencia de ejecución:
     * 1. Actualizar índice en trayectoria
     * 2. Evaluar estado de evasión (láser 60°)
     * 3. Si EMERGENCY → parar
     * 4. Si OBSTACLE → generar desvío
     * 5. Calcular LHD adaptativo
     * 6. Encontrar punto lookahead
     * 7. Calcular curvatura y velocidades
     * 8. Publicar comandos y visualizaciones
     * 9. Guardar datos en CSV
     */
    void controlLoop()
    {
        publishRobotMarker();

        if (!has_goal_ || !has_path_ || path_points_.empty()) {
            stopRobot();
            return;
        }

        updateCurrentPathIndex();
        updateAvoidanceState();

        if (avoidance_state_ == AvoidanceState::EMERGENCY) {
            stopRobot();
            return;
        }

        if (avoidance_state_ == AvoidanceState::OBSTACLE_DETECTED) {
            generateDetourPath();
        }

        double cte = getCrossTrackError();
        double lookahead_dist = calculateLookaheadDistance(cte, std::abs(robot_linear_vel_));
        
        geometry_msgs::msg::Point lookahead_point;
        bool found = findLookaheadPoint(lookahead_dist, lookahead_point);
        if (!found) lookahead_point = path_points_.back();

        publishLookaheadMarker(lookahead_point);

        double curv = calculateCurvature(lookahead_point);
        double target_angle = std::atan2(lookahead_point.y - robot_y_, lookahead_point.x - robot_x_);
        double alpha = normalizeAngle(target_angle - robot_yaw_);

        double cmd_curva = std::clamp(1.0 - std::abs(curv)/5.0, 0.1, 1.0);
        current_vel_cmd_ = max_linear_vel_ * cmd_curva;
        
        if (avoidance_state_ == AvoidanceState::OBSTACLE_DETECTED) {
            current_vel_cmd_ *= 0.6;
        }

        double angular_vel = (2.0 * current_vel_cmd_ * std::sin(alpha)) / lookahead_dist;
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

        double dist_to_end = 999.9;
        if (!path_points_.empty()) {
             auto last_p = path_points_.back();
             dist_to_end = std::hypot(last_p.x - robot_x_, last_p.y - robot_y_);
        }

        // Logging de datos
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
                           << dist_to_end << ","
                           << current_vel_cmd_ << "," 
                           << angular_vel << ","      
                           << curv << "\n";           
        }

        if (dist_to_end < goal_tolerance_) {
            if (!goal_reached_) {
                goal_reached_ = true;
                std_msgs::msg::Bool msg; msg.data = true;
                goal_reached_pub_->publish(msg);
                stopRobot();
            }
            return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = current_vel_cmd_;
        cmd.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd);

        double current_radius = bubble_base_radius_;
        publishBubbleViz(current_radius);
    }

    /**
     * @brief Detiene el robot completamente
     * 
     * Publica velocidad lineal y angular = 0
     */
    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0; cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    /**
     * @brief Busca el punto lookahead en la trayectoria
     * @param dynamic_lookahead_dist Distancia lookahead calculada dinámicamente
     * @param lookahead_point Punto encontrado (salida)
     * @return true si encontró punto, false si llegó al final
     * 
     * Busca hacia adelante el primer waypoint que esté a una distancia
     * mayor o igual al lookahead distance calculado
     */
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
        return false;
    }

    /**
     * @brief Calcula la curvatura instantánea hacia el punto lookahead
     * @param lookahead_point Punto objetivo en coordenadas globales
     * @return Curvatura (1/m) - positiva=izquierda, negativa=derecha
     * 
     * Fórmula en sistema de coordenadas del robot:
     * κ = 2 × y_rel / L²
     * donde L es la distancia al punto lookahead
     */
    double calculateCurvature(const geometry_msgs::msg::Point& lookahead_point)
    {
        double rel_x = lookahead_point.x - robot_x_;
        double rel_y = lookahead_point.y - robot_y_;
        double robot_rel_x = rel_x * cos(robot_yaw_) + rel_y * sin(robot_yaw_);
        double robot_rel_y = -rel_x * sin(robot_yaw_) + rel_y * cos(robot_yaw_);
        double L = std::sqrt(robot_rel_x*robot_rel_x + robot_rel_y*robot_rel_y);
        if (L > 0.001) return 2.0 * robot_rel_y / (L * L);
        return 0.0;
    }

    /**
     * @brief Publica marcador visual de la burbuja de seguridad en RViz
     * @param radius Radio de la burbuja (metros)
     * 
     * Colores según estado:
     * - NORMAL: Cian translúcido (α=0.3)
     * - OBSTACLE_DETECTED: Naranja semi-opaco (α=0.5)
     * - EMERGENCY: Rojo opaco (α=0.7)
     */
    void publishBubbleViz(double radius) {
        auto m = visualization_msgs::msg::Marker();
        m.header.frame_id = "map";
        m.header.stamp = now();
        m.pose.position.x = robot_x_;
        m.pose.position.y = robot_y_;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.ns = "bubble"; 
        m.id = 1; 
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = 0;
        m.scale.x = radius * 2.0; 
        m.scale.y = radius * 2.0; 
        m.scale.z = 0.05;

        if (avoidance_state_ == AvoidanceState::NORMAL) { 
            m.color.r = 0.0;
            m.color.g = 1.0; 
            m.color.b = 1.0; 
            m.color.a = 0.3; 
        }
        else if (avoidance_state_ == AvoidanceState::OBSTACLE_DETECTED) { 
            m.color.r = 1.0; 
            m.color.g = 0.65; 
            m.color.b = 0.0;
            m.color.a = 0.5; 
        }
        else { 
            m.color.r = 1.0; 
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 0.7; 
        }

        m.lifetime = rclcpp::Duration::from_seconds(0);
        bubble_viz_pub_->publish(m);
    }

    /**
     * @brief Publica marcador visual del punto lookahead en RViz
     * @param point Punto lookahead en coordenadas globales
     * 
     * Marcador: esfera magenta de 0.2m de diámetro
     */
    void publishLookaheadMarker(const geometry_msgs::msg::Point& point) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now(); marker.header.frame_id = "map";
        marker.ns = "pure_pursuit"; marker.id = 0; marker.type = 2; marker.action = 0;
        marker.pose.position = point; marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2;
        marker.color.r = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
        marker_pub_->publish(marker);
    }
    
    /**
     * @brief Publica marcador visual de la posición del robot en RViz
     * 
     * Marcador: esfera azul de 0.3m de diámetro en la posición actual
     */
    void publishRobotMarker() {
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

    /**
     * @brief Normaliza un ángulo al rango [-π, π]
     * @param angle Ángulo en radianes (cualquier valor)
     * @return Ángulo normalizado en [-π, π]
     */
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // ========== VARIABLES MIEMBRO ==========
    
    /// @name Estado del Robot
    /// @{
    double robot_x_;           ///< Posición X en marco global (m)
    double robot_y_;           ///< Posición Y en marco global (m)
    double robot_yaw_;         ///< Orientación en marco global (rad)
    double robot_linear_vel_;  ///< Velocidad lineal actual (m/s)
    /// @}
    
    /// @name Objetivos y Trayectoria
    /// @{
    geometry_msgs::msg::Pose current_goal_;              ///< Objetivo final de navegación
    std::vector<geometry_msgs::msg::Point> path_points_; ///< Waypoints de la trayectoria
    bool has_goal_;                                      ///< Flag: objetivo recibido
    bool has_path_;                                      ///< Flag: trayectoria recibida
    bool goal_reached_;                                  ///< Flag: objetivo alcanzado
    size_t current_path_index_;                          ///< Índice actual en la trayectoria
    /// @}
    
    /// @name Sistema de Logs y Sensores
    /// @{
    std::ofstream data_log_file_;                           ///< Archivo CSV de logging
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;      ///< Último escaneo láser recibido
    /// @}
    
    /// @name Parámetros de Navegación
    /// @{
    double lookahead_distance_;  ///< LHD base (no usado con adaptativo)
    double max_linear_vel_;      ///< Velocidad lineal máxima (m/s)
    double max_angular_vel_;     ///< Velocidad angular máxima (rad/s)
    double goal_tolerance_;      ///< Tolerancia de llegada (m)
    double delta_min_;           ///< LHD mínimo para adaptativo (m)
    double delta_max_;           ///< LHD máximo para adaptativo (m)
    double gamma_;               ///< Factor de decaimiento exponencial
    /// @}
    
    /// @name Parámetros de Evasión
    /// @{
    double bubble_base_radius_;  ///< Radio de burbuja de seguridad (m)
    double critical_distance_;   ///< Distancia de emergencia (m)
    double detour_offset_;       ///< Desplazamiento lateral del desvío (m)
    double rejoin_distance_;     ///< Distancia de reincorporación (m)
    double forward_offset_;      ///< Desplazamiento frontal del ápex (m)
    /// @}
    
    /// @name Control y Estado
    /// @{
    int selected_route_;                      ///< ID de ruta seleccionada (1-8)
    double current_vel_cmd_;                  ///< Última velocidad comandada (m/s)
    rclcpp::Time last_detour_time_;           ///< Timestamp del último desvío
    AvoidanceState avoidance_state_;          ///< Estado actual de evasión
    /// @}

    /// @name Publicadores ROS 2
    /// @{
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bubble_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detour_path_pub_;
    /// @}

    /// @name Suscriptores ROS 2
    /// @{
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    /// @}
    
    /// @name Utilidades
    /// @{
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  ///< Broadcaster TF2
    rclcpp::TimerBase::SharedPtr control_timer_;                      ///< Timer del bucle control
    /// @}
};

/**
 * @brief Función principal - Punto de entrada del programa
 * @param argc Número de argumentos
 * @param argv Array de argumentos
 * @return 0 si terminó correctamente
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
