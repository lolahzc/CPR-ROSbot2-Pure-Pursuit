#!/bin/bash


RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'


function finish_experiment() {
    echo -e "\n${BLUE}==================================================${NC}"
    echo -e "${YELLOW}[STOP] Deteniendo simulación y controlador...${NC}"
    

    kill $SIM_PID 2>/dev/null
    kill $PP_PID 2>/dev/null
    
    echo -e "${GREEN}[INFO] Entorno ROS cerrado.${NC}"
    echo -e "${CYAN}[ANALISIS] Lanzando Octave para graficar resultados...${NC}"
    echo -e "${BLUE}==================================================${NC}"

    octave analyzer_automatico.m

    exit 0
}


trap finish_experiment INT

ROUTE_ID=${1:-1}

echo -e "${BLUE}==================================================${NC}"
echo -e "${BLUE}   SISTEMA DE EXPERIMENTACIÓN ROSBOT XL${NC}"
echo -e "${BLUE}   Ruta Activa: ${CYAN}$ROUTE_ID${NC}"
echo -e "${BLUE}==================================================${NC}"


if [ ! -d "install" ]; then
    echo -e "${RED}[ERROR] Ejecuta desde la carpeta 'ppcpr'.${NC}"
    exit 1
fi

source /opt/ros/humble/setup.bash > /dev/null 2>&1
source install/setup.bash > /dev/null 2>&1


echo -e "${GREEN}[INFO] Iniciando Gazebo (Silencioso)...${NC}"
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot > /dev/null 2>&1 &
SIM_PID=$! 

echo -e "${GREEN}[INFO] Iniciando Pure Pursuit (Ruta $ROUTE_ID)...${NC}"
ros2 launch pure_pursuit_controller pure_pursuit.launch.py selected_route:=$ROUTE_ID > /dev/null 2>&1 &
PP_PID=$! 

echo -e "${BLUE}--------------------------------------------------${NC}"
echo -e "${GREEN}[RUNNING] Sistema corriendo.${NC}"
echo -e "${CYAN}[AUTO] Al pulsar Ctrl+C se abrirán las gráficas automáticamente.${NC}"
echo -e "${BLUE}--------------------------------------------------${NC}"

wait
