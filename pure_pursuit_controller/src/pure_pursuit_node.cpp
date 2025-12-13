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
#include <algorithm> // Necesario para std::clamp y std::min/std::max
#include <fstream>  // Necesario para escribir en ficheros
#include <iomanip>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node")
    {
        // Parámetros
        this->declare_parameter("lookahead_distance", 1.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 0.5);
        this->declare_parameter("goal_tolerance", 0.1);
        
        this->declare_parameter("obstacle_distance_threshold", 0.8);
        this->declare_parameter("scan_fov_degrees", 50.0);
        this->declare_parameter("min_free_distance", 1.2);
        this->declare_parameter("search_angular_vel", 0.45);
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        obstacle_distance_threshold_ = this->get_parameter("obstacle_distance_threshold").as_double();
        scan_fov_rad_ = (this->get_parameter("scan_fov_degrees").as_double()) * M_PI / 180.0;
        min_free_distance_ = this->get_parameter("min_free_distance").as_double();
        search_angular_vel_ = this->get_parameter("search_angular_vel").as_double();

        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publicadores
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriptores
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PurePursuitNode::goalCallback, this, std::placeholders::_1));
            
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoints_path", 10,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,  // Cambio 1: Nombre del topic (asegúrate de que está bien escrito 'filtered')
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)); // Cambio 2: Usar la función correcta

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&PurePursuitNode::scanCallback, this, std::placeholders::_1));

        // Abrir archivo de log
        data_log_file_.open("robot_data_log.csv");
        
        if (data_log_file_.is_open()) {
            data_log_file_ << "time,robot_x,robot_y,robot_yaw,goal_x,goal_y,dist_error,linear_v,angular_w,avoidance_state\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "No se pudo crear el archivo de log!");
        }
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized...");

        // Posición inicial del robot
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;

        // Timer para el control
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuitNode::controlLoop, this));
            
        // Timer para TF y odometría
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PurePursuitNode::publishOdomAndTF, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized - Continuous path following mode (Sequential)");
        RCLCPP_INFO(this->get_logger(), "Lookahead distance: %.2f", lookahead_distance_);
    }

private:
    // Estados de la máquina de evasión de obstáculos
    enum class AvoidanceState { 
        NORMAL,           // Seguimiento normal de trayectoria
        STOPPED,          // Robot detenido al detectar obstáculo
        SEARCHING_LEFT,   // Buscando espacio libre girando a la izquierda
        SEARCHING_RIGHT   // Buscando espacio libre girando a la derecha
    };

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_goal_ = msg->pose;
        has_goal_ = true;
        goal_reached_ = false;
        
        RCLCPP_DEBUG(this->get_logger(), "Goal updated: (%.2f, %.2f)", 
                    current_goal_.position.x, current_goal_.position.y);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        
        // Convertir Cuaternión a Yaw usando Matrix3x3 (Método estándar TF2)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
            
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_yaw_);
        
        robot_yaw_ = normalizeAngle(robot_yaw_);
    }

    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Guardar la ruta completa para Pure Pursuit
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose.position);
        }
        has_path_ = true;
        current_path_index_ = 0; // REINICIAR ÍNDICE AL RECIBIR NUEVA RUTA
        
        RCLCPP_DEBUG(this->get_logger(), "Received path with %zu points", path_points_.size());
    }

    // Callback del LIDAR - Procesa las lecturas para detectar obstáculos frontales
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;

        double min_dist = std::numeric_limits<double>::max();
        bool found = false;

        // Rango mínimo válido del sensor (ignora lecturas muy cercanas que pueden ser ruido)
        const double MIN_SCAN_RANGE = 0.18;
        double half_fov = scan_fov_rad_ / 2.0;

        // Recorrer todas las lecturas del LIDAR
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double r = msg->ranges[i];
            // Filtrar lecturas inválidas
            if (std::isnan(r) || std::isinf(r) || r < MIN_SCAN_RANGE) continue;

            // Calcular el ángulo de esta lectura respecto al frente del robot
            // Se suma M_PI porque el LIDAR puede tener un sistema de referencia diferente
            double angle = normalizeAngle(msg->angle_min + i * msg->angle_increment + M_PI);

            // Solo considerar lecturas dentro del campo de visión frontal (FOV)
            if (std::abs(angle) < half_fov)
            {
                if (r < min_dist)
                {
                    min_dist = r;
                    found = true;
                }
            }
        }

        // Marcar obstáculo detectado si la distancia mínima es menor al umbral
        obstacle_detected_ = found && (min_dist < obstacle_distance_threshold_);
        min_dist_frontal_ = found ? min_dist : 5.0;
    }

    // Verifica si hay suficiente espacio libre frontal para avanzar con seguridad
    bool checkFreeSpace()
    {
        if (!last_scan_) return false;

        double half_fov = scan_fov_rad_ / 2.0;

        // Revisar todas las lecturas en el FOV frontal
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i)
        {
            double r = last_scan_->ranges[i];
            if (r < 0.18 || std::isnan(r) || std::isinf(r)) continue;

            double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);

            // Si alguna lectura frontal está demasiado cerca, NO hay espacio libre
            if (std::abs(angle) < half_fov)
            {
                if (r < min_free_distance_) return false;
            }
        }
        return true; // Todas las lecturas frontales están a distancia segura
    }

    // Determina hacia qué lado (izquierda o derecha) hay más espacio libre
    // Retorna: 1 para izquierda, -1 para derecha
    int findGapSide()
    {
        if (!last_scan_) return 1;

        double sum_left = 0, sum_right = 0;

        // Sumar las distancias de cada lado
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i)
        {
            double angle = normalizeAngle(last_scan_->angle_min + i * last_scan_->angle_increment + M_PI);
            double r = last_scan_->ranges[i];
            if (std::isnan(r) || r < 0.18) continue;
            
            // Ángulos positivos = izquierda, negativos = derecha
            if (angle > 0) sum_left += r;
            else           sum_right += r;
        }
        
        // Girar hacia donde hay más espacio acumulado
        return (sum_left > sum_right) ? 1 : -1;
    }

    // CLAVE: Encuentra el punto más cercano del path HACIA ADELANTE después de evadir
    // Esto evita que el robot intente volver atrás en la trayectoria
    size_t findClosestIndexForward()
    {
        double best_dist = std::numeric_limits<double>::max();
        size_t best_idx = current_path_index_;

        // Buscar SOLO hacia adelante desde el índice actual
        for (size_t i = current_path_index_; i < path_points_.size(); ++i)
        {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double d2 = dx * dx + dy * dy;

            if (d2 < best_dist)
            {
                best_dist = d2;
                best_idx = i;
            }
        }
        return best_idx;
    }

    void controlLoop()
    {
        if (!has_goal_ || !has_path_ || path_points_.empty()) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        // Verificar si se alcanzó el goal final del SEGMENTO (el waypoint original)
        double dx = current_goal_.position.x - robot_x_;
        double dy = current_goal_.position.y - robot_y_;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);

        if (distance_to_goal < goal_tolerance_) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "Final segment goal reached! Distance: %.3f", distance_to_goal);
                goal_reached_ = true;
                
                auto goal_reached_msg = std_msgs::msg::Bool();
                goal_reached_msg.data = true;
                goal_reached_pub_->publish(goal_reached_msg);
                
                // Detener el robot brevemente
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd_vel);
            }
            // Mantenemos las velocidades en cero y esperamos el nuevo goal de route_publisher
            return;
        }
        
        // Transición a evasión de obstáculos
        if (avoidance_state_ == AvoidanceState::NORMAL && obstacle_detected_)
        {
            avoidance_state_ = AvoidanceState::STOPPED;
            RCLCPP_WARN(this->get_logger(), "Obstáculo detectado → STOPPED");
        }

        geometry_msgs::msg::Twist cmd_vel;

        // ======================================================================
        // MÁQUINA DE ESTADOS PARA EVASIÓN DE OBSTÁCULOS
        // ======================================================================
        switch (avoidance_state_)
        {
            case AvoidanceState::NORMAL:
            {
                // PURE PURSUIT CON SEGUIMIENTO DE TRAYECTORIA CONTINUA SECUENCIAL
                
                // 1. Encontrar el punto lookahead secuencial
                geometry_msgs::msg::Point lookahead_point;
                bool lookahead_found = findSequentialLookaheadPoint(lookahead_point);
                
                if (!lookahead_found) {
                    // Si no encontramos punto lookahead (estamos cerca del final de la ruta interpolada), 
                    // apuntamos al goal actual para la convergencia final.
                    lookahead_point = current_goal_.position;
                }

                // 2. Publicar marcador del punto lookahead
                publishLookaheadMarker(lookahead_point);
                
                // 3. Calcular curvatura usando Pure Pursuit
                double curvature = calculateCurvature(lookahead_point);
                
                // 4. Calcular velocidades de control
                cmd_vel = calculateControlCommands(curvature, distance_to_goal);
                
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "PurePursuit: Path Index: %zu/%zu, Curvature: %.3f, Lin: %.2f, Ang: %.2f, Dist2Goal: %.3f",
                            current_path_index_, path_points_.size(), curvature, cmd_vel.linear.x, cmd_vel.angular.z, distance_to_goal);
                break;
            }

            case AvoidanceState::STOPPED:
            {
                // Detener el robot y decidir hacia qué lado buscar espacio
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                avoidance_state_ = (findGapSide() > 0) ?
                    AvoidanceState::SEARCHING_LEFT :
                    AvoidanceState::SEARCHING_RIGHT;
                RCLCPP_INFO(this->get_logger(), "Iniciando búsqueda: %s", 
                           avoidance_state_ == AvoidanceState::SEARCHING_LEFT ? "IZQUIERDA" : "DERECHA");
                break;
            }

            case AvoidanceState::SEARCHING_LEFT:
            case AvoidanceState::SEARCHING_RIGHT:
            {
                // Girar en el sitio buscando espacio libre
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = (avoidance_state_ == AvoidanceState::SEARCHING_LEFT ?
                                     search_angular_vel_ : -search_angular_vel_);

                // Si encontramos espacio libre, volver al modo normal
                if (!obstacle_detected_ && checkFreeSpace())
                {
                    RCLCPP_INFO(this->get_logger(), "Espacio libre detectado → reenganchar camino");
                    current_path_index_ = findClosestIndexForward();
                    avoidance_state_ = AvoidanceState::NORMAL;
                }
                break;
            }
        }

        cmd_vel_pub_->publish(cmd_vel);

        // 5. Actualizar posición del robot (simulación)
        // updateRobotPose(cmd_vel.linear.x, cmd_vel.angular.z);

        // 6. Registrar datos en el archivo de log
        if (data_log_file_.is_open()) {
            double current_time = this->now().seconds();
            
            data_log_file_ << std::fixed << std::setprecision(9)
                           << current_time << ","             // Tiempo
                           << robot_x_ << ","                 // Robot X
                           << robot_y_ << ","                 // Robot Y
                           << robot_yaw_ << ","               // Robot Yaw
                           << current_goal_.position.x << "," // Goal X actual
                           << current_goal_.position.y << "," // Goal Y actual
                           << distance_to_goal << ","         // Distancia al objetivo
                           << cmd_vel.linear.x << ","         // Velocidad Lineal enviada
                           << cmd_vel.angular.z << ","        // Velocidad Angular enviada
                           << static_cast<int>(avoidance_state_) << "\n"; // Estado de evasión
        }
    }

    bool findSequentialLookaheadPoint(geometry_msgs::msg::Point& lookahead_point)
    {
        if (current_path_index_ >= path_points_.size()) {
            return false;
        }
        
        // 1. Encontrar el punto más cercano *a partir* del índice actual para actualizar el progreso.
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t closest_index = current_path_index_;
        // Limitar la búsqueda hacia adelante para eficiencia y estabilidad, buscando en el remanente de la ruta
        size_t search_limit = std::min(path_points_.size(), current_path_index_ + 100); 

        // Buscar el punto más cercano en un rango limitado hacia adelante.
        for (size_t i = current_path_index_; i < search_limit; ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_index = i;
            }
        }
        
        // Actualizar el índice de inicio para la próxima búsqueda.
        current_path_index_ = closest_index;

        // 2. Buscar el punto lookahead (el que está a una distancia L)
        // La búsqueda comienza desde el punto más cercano encontrado.
        for (size_t i = current_path_index_; i < path_points_.size(); ++i) {
            double dx = path_points_[i].x - robot_x_;
            double dy = path_points_[i].y - robot_y_;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance >= lookahead_distance_) {
                lookahead_point = path_points_[i];
                return true;
            }
        }
        
        // Si no encontramos un punto con la distancia lookahead, usamos el punto final del path
        if (!path_points_.empty()) {
            lookahead_point = path_points_.back();
            return true;
        }
        
        return false;
    }

    double calculateCurvature(const geometry_msgs::msg::Point& lookahead_point)
    {
        // Transformar punto lookahead al marco del robot
        double rel_x = lookahead_point.x - robot_x_;
        double rel_y = lookahead_point.y - robot_y_;
        
        // Rotar al marco del robot
        double robot_rel_x = rel_x * cos(robot_yaw_) + rel_y * sin(robot_yaw_);
        double robot_rel_y = -rel_x * sin(robot_yaw_) + rel_y * cos(robot_yaw_);
        
        // Calcular curvatura (L = distancia al punto lookahead)
        double L = std::sqrt(robot_rel_x*robot_rel_x + robot_rel_y*robot_rel_y);
        
        if (L > 0.001) {
            // Curvatura = 2 * y_rel_robot / L^2
            return 2.0 * robot_rel_y / (L * L);
        }
        
        return 0.0;
    }

    geometry_msgs::msg::Twist calculateControlCommands(double curvature, double distance_to_goal)
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        // Velocidad angular basada en curvatura: ω = v * κ
        double angular_vel = curvature * max_linear_vel_;
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        
        // Velocidad lineal 
        double linear_vel = max_linear_vel_;
        
        // Reducir velocidad si la curvatura es grande
        if (std::abs(curvature) > 0.5) {
            linear_vel *= (1.0 - std::abs(curvature) * 0.6);
        }
        
        // Reducir velocidad al acercarse al goal (Waypoint Original)
        if (distance_to_goal < lookahead_distance_ * 2.0) {
            linear_vel *= std::max(0.0, distance_to_goal / (lookahead_distance_ * 2.0));
        }
        
        // Velocidad mínima (para evitar detenerse totalmente a menos que se llegue al goal)
        linear_vel = std::max(0.1, linear_vel);

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        
        return cmd_vel;
    }

    void publishLookaheadMarker(const geometry_msgs::msg::Point& point)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        
        marker_pub_->publish(marker);
    }
/*
    void updateRobotPose(double linear_vel, double angular_vel)
    {
        double dt = 0.05;

        robot_yaw_ += angular_vel * dt;
        robot_yaw_ = normalizeAngle(robot_yaw_);

        robot_x_ += linear_vel * cos(robot_yaw_) * dt;
        robot_y_ += linear_vel * sin(robot_yaw_) * dt;
    }
*/
    void publishOdomAndTF()
    {
        auto now = this->now();

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.pose.pose.position.x = robot_x_;
        odom_msg.pose.pose.position.y = robot_y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = createQuaternionFromYaw(robot_yaw_);
        
        odom_pub_->publish(odom_msg);

        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = now;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = robot_x_;
        transform.transform.translation.y = robot_y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = createQuaternionFromYaw(robot_yaw_);
        
        tf_broadcaster_->sendTransform(transform);
    }

    geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Variables
    double robot_x_, robot_y_, robot_yaw_;
    double min_dist_frontal_{5.0};
    geometry_msgs::msg::Pose current_goal_;
    std::vector<geometry_msgs::msg::Point> path_points_;
    bool has_goal_ = false;
    bool has_path_ = false;
    bool goal_reached_ = false;
    bool obstacle_detected_ = false;
    size_t current_path_index_ = 0; // ÍNDICE SECUENCIAL para /waypoints_path
    std::ofstream data_log_file_;
    
    // Variables de evasión de obstáculos
    AvoidanceState avoidance_state_{AvoidanceState::NORMAL};        // Estado actual de la FSM
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;               // Última lectura del LIDAR
    
    // Parámetros
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;
    double obstacle_distance_threshold_;  // Distancia mínima para considerar obstáculo
    double scan_fov_rad_;                 // Campo de visión del LIDAR en radianes
    double min_free_distance_;            // Distancia mínima libre requerida para avanzar
    double search_angular_vel_;           // Velocidad angular al buscar espacio libre

    // ROS2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
