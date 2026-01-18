 /**
 * @file route_publisher.cpp
 * @brief Nodo para la publicación de rutas y generación de trayectorias suavizadas (Splines).
 * @author Pedro Cabello Pulido | Gabriela Cano Azuaga  | Lola Hernández Canizares | 
Almudena Jin | Lucía Pérez Guerrero 
 */

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

/**
 * @class RoutePublisher
 * @brief Nodo ROS2 que gestiona la publicación de rutas predefinidas con interpolación suave
 * 
 * Esta clase implementa un nodo de ROS2 que:
 * - Gestiona múltiples rutas predefinidas (8 rutas diferentes)
 * - Interpola waypoints mediante splines cúbicos naturales para generar trayectorias suaves
 * - Publica objetivos (goals) secuenciales para navegación
 * - Visualiza las rutas en RViz mediante markers
 * - Permite cambio dinámico de rutas
 * - Mantiene transformaciones TF entre frames map y odom
 */

class RoutePublisher : public rclcpp::Node
{
public:    
    /**
     * @brief Constructor de la clase RoutePublisher
     * 
     * Inicializa todos los publicadores, suscriptores, parámetros y timers necesarios.
     * Configura la ruta inicial según el parámetro 'selected_route' y comienza la publicación
     * de waypoints tras un delay de 5 segundos.
     */

    RoutePublisher() : Node("route_publisher")
    {
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
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
                publishCurrentWaypoint();
                publishWaypointsMarkers();
                
                start_timer_->cancel();
            });
    }

private:

    /** @brief Índice de la ruta seleccionada actualmente (1-8) */
    int selected_route_;
    /** @brief Índice del waypoint actual en la secuencia de navegación */
    size_t current_segment_index_ = 0;
	
    /**
     * @brief Define la ruta según el número de ruta seleccionado
     * 
     * Carga los waypoints originales correspondientes a la ruta seleccionada.
     * Incluye 8 rutas predefinidas con diferentes características:
     * - Ruta 1: Trayectoria con curvas suaves
     * - Ruta 2: Línea recta
     * - Ruta 3: Trayectoria en escalera
     * - Ruta 4: Patrón en forma de ocho
     * - Ruta 5: Trayectoria compleja con múltiples giros
     * - Ruta 6: Ruta irregular con cambios de dirección
     * - Ruta 7: Trayectoria con oscilaciones
     * - Ruta 8: Ruta para evasión de obstáculos
     * 
     * Si se proporciona un número de ruta inválido, se selecciona la ruta 1 por defecto.
     */
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

    /**
     * @brief Genera una ruta interpolada mediante splines cúbicos naturales
     * 
     * Toma los waypoints originales y genera una trayectoria suave mediante
     * interpolación con splines cúbicos. El número de puntos interpolados por
     * segmento está definido por el parámetro 'interpolation_points_per_segment'.
     * 
     * Los waypoints originales se preservan en sus posiciones exactas dentro
     * de la trayectoria interpolada.
     */
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

    /**
     * @brief Calcula los coeficientes de un spline cúbico natural
     * 
     * @param points Vector de valores de coordenadas (x o y) de los waypoints
     * @return Vector de derivadas en cada punto (coeficientes D del spline)
     * 
     * Implementa el algoritmo de splines cúbicos naturales usando el método
     * de Thomas para resolver el sistema tridiagonal. Las condiciones de frontera
     * naturales asumen segunda derivada cero en los extremos.
     */
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
    /**
     * @brief Evalúa el valor del spline cúbico en un punto específico
     * 
     * @param points Vector de valores originales de los waypoints
     * @param D Vector de coeficientes calculados por computeNaturalCubicSpline
     * @param i Índice del segmento a evaluar
     * @param t Parámetro de interpolación local [0,1] dentro del segmento
     * @return Valor interpolado en la posición t del segmento i
     * 
     * Utiliza la forma polinómica del spline cúbico: a + bt + ct² + dt³
     */
    double evaluateCubicSpline(const std::vector<double>& points, const std::vector<double>& D, int i, double t)
    {
        double a = points[i];
        double b = D[i];
        double c = 3.0 * (points[i + 1] - points[i]) - 2.0 * D[i] - D[i + 1];
        double d = 2.0 * (points[i] - points[i + 1]) + D[i] + D[i + 1];
        
        return a + b * t + c * t * t + d * t * t * t;
    }

    /**
     * @brief Callback para el topic /goal_reached
     * 
     * @param msg Mensaje booleano indicando si se alcanzó el objetivo
     * 
     * Cuando se recibe confirmación de que se alcanzó el objetivo actual,
     * avanza automáticamente al siguiente waypoint en la secuencia.
     */

    void goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            advanceToNextWaypoint();
        }
    }
    /**
     * @brief Publica el waypoint actual como objetivo de navegación
     * 
     * Publica un mensaje PoseStamped con la posición y orientación del waypoint
     * actual en el topic /goal_pose. La orientación se calcula como el ángulo
     * hacia el siguiente waypoint. También actualiza la visualización de la ruta
     * y los markers.
     */
    void publishCurrentWaypoint()
    {
        if (current_segment_index_ >= original_waypoints_.size()) {
            return;
        }

        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        
        const auto& wp = original_waypoints_[current_segment_index_]; 
        goal_msg.pose.position.x = wp[0];
        goal_msg.pose.position.y = wp[1];
        goal_msg.pose.position.z = wp[2];
        
        if (current_segment_index_ < original_waypoints_.size() - 1) {
            const auto& next_wp = original_waypoints_[current_segment_index_ + 1];
            double yaw = std::atan2(next_wp[1] - wp[1], next_wp[0] - wp[0]);
            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = std::sin(yaw / 2.0);
            goal_msg.pose.orientation.w = std::cos(yaw / 2.0);
        } else {
            goal_msg.pose.orientation.w = 1.0; 
        }
        
        goal_pub_->publish(goal_msg);
        
        publishWaypointsPath();
        
        publishWaypointsMarkers();
        
    }

    /**
     * @brief Publica la trayectoria completa interpolada
     * 
     * Publica un mensaje PoseArray con todos los puntos de la trayectoria
     * interpolada mediante splines en el topic /waypoints_path. Esta trayectoria
     * puede ser visualizada en RViz.
     */
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
    /**
     * @brief Publica markers de visualización para RViz
     * 
     * Crea y publica un conjunto de markers que incluyen:
     * - Línea verde representando la trayectoria interpolada (spline)
     * - Línea roja semi-transparente representando los waypoints originales
     * - Esferas rojas en cada waypoint original
     * - Esfera amarilla destacando el objetivo actual
     * 
     * Los markers se publican en el topic /waypoints_markers
     */
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

    /**
     * @brief Publica la transformación TF estática entre map y odom
     * 
     * Mantiene una transformación identidad entre los frames "map" y "odom".
     * Se ejecuta periódicamente cada 50ms mediante un timer.
     */
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

    /**
     * @brief Avanza al siguiente waypoint en la secuencia
     * 
     * Incrementa el índice del waypoint actual y publica el nuevo objetivo.
     * Si se ha alcanzado el final de la ruta:
     * - Si loop_route_ está activado, reinicia desde el principio
     * - Si no, detiene la publicación de nuevos objetivos
     */
    void advanceToNextWaypoint()
    {
        current_segment_index_++; 
        
        if (current_segment_index_ >= original_waypoints_.size()) {
            if (loop_route_) {
                current_segment_index_ = 0;
            } else {
                return;
            }
        }
        
        publishCurrentWaypoint();
    }

    /**
     * @brief Cambia dinámicamente la ruta activa
     * 
     * @param new_route Número de la nueva ruta a cargar (1-8)
     * 
     * Limpia los markers anteriores, carga la nueva ruta, regenera la interpolación
     * y reinicia la navegación desde el primer waypoint de la nueva ruta.
     */

    void changeRoute(int new_route)
    {
        selected_route_ = new_route;

        defineRoute();
        generateSplineRoute();
        current_segment_index_ = 0; 
	
	    visualization_msgs::msg::MarkerArray marker_array;
        auto clear_marker = visualization_msgs::msg::Marker();
        clear_marker.header.stamp = this->now();
        clear_marker.header.frame_id = "map";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL; 
        marker_array.markers.push_back(clear_marker);
        marker_pub_->publish(marker_array);	
        
        publishCurrentWaypoint();
        publishWaypointsMarkers();
    }

    /** @brief Waypoints originales de la ruta seleccionada */
    std::vector<std::array<double, 3>> original_waypoints_;
    /** @brief Waypoints interpolados mediante splines cúbicos */
    std::vector<std::array<double, 3>> spline_waypoints_;
    /** @brief Flag para activar el modo de bucle continuo de la ruta */
    bool loop_route_;
    /** @brief Número de puntos de interpolación por segmento de waypoints */
        int interpolation_points_;
    /** @brief Publicador del objetivo actual (/goal_pose) */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    /** @brief Publicador del path completo de waypoints interpolados (/waypoints_path) */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    /** @brief Publicador de markers de visualización para RViz (/waypoints_markers) */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    /** @brief Suscriptor para recibir confirmación de objetivo alcanzado (/goal_reached) */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    /** @brief Suscriptor para recibir comandos de cambio de ruta (/change_route) */
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr route_change_sub_;
    /** @brief Broadcaster de transformaciones TF2 (map <-> odom) */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /** @brief Timer para publicación periódica de transformación TF estática (50ms) */
    rclcpp::TimerBase::SharedPtr tf_timer_;
    /** @brief Timer de inicio único para delay de 5 segundos antes de comenzar la ruta */
    rclcpp::TimerBase::SharedPtr start_timer_;
};

/**
 * @brief Función principal del programa
 * 
 * @param argc Número de argumentos de línea de comandos
 * @param argv Array de argumentos de línea de comandos
 * @return int Código de retorno (0 si es exitoso)
 * 
 * Inicializa el contexto de ROS2, crea una instancia del nodo RoutePublisher,
 * mantiene el nodo en ejecución (spin) hasta que se reciba una señal de apagado,
 * y finalmente realiza la limpieza del contexto de ROS2.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
