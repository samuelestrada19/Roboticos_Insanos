#!/usr/bin/env python3
# Indica que este archivo debe ejecutarse utilizando Python 3

# Importa la clase necesaria para crear una descripción de lanzamiento (launch)
from launch import LaunchDescription
# Importa la acción Node para ejecutar nodos de ROS 2
from launch_ros.actions import Node
# Permite obtener la ruta del directorio share de un paquete ROS 2
from ament_index_python import get_package_share_directory
# Permite ejecutar comandos desde el archivo launch
from launch.substitutions import Command
# Permite definir parámetros indicando explícitamente su tipo
from launch_ros.parameter_descriptions import ParameterValue

# Función principal que ROS 2 ejecuta al lanzar este archivo
def generate_launch_description():

  # Obtiene la ruta del directorio share del paquete robot_description
  description_path = get_package_share_directory(
    "robot_description")

  # Construye la ruta completa al archivo URDF del robot
  urdf_path = description_path + "/urdf/robotinsano.urdf"

  # Construye la ruta completa al archivo de configuración de RViz
  rviz_path = description_path + "/rviz/rviz.conf.rviz" 

  # Ejecuta el comando xacro sobre el archivo URDF
  # para generar la descripción del robot
  urdf_xacro  = Command(["xacro ", urdf_path])

  # Crea el parámetro robot_description que contendrá
  # el modelo completo del robot en formato XML
  urdf_param = {"robot_description": 
                ParameterValue(urdf_xacro, value_type=str)}
  
  # =========================
  # Definición de nodos
  # =========================

  # Nodo RViz:
  # Inicia RViz utilizando el archivo de configuración especificado
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_path])

  # Nodo robot_state_publisher:
  # Publica las transformaciones TF del robot a partir
  # del modelo almacenado en robot_description
  robot_description_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[urdf_param])

  # Nodo de cinemática:
  # Ejecuta el programa trajectory_publisher del paquete
  # robot_kinematics. Normalmente se encarga de calcular
  # y publicar las posiciones de las articulaciones.
  kinematics_node = Node(
    package="robot_kinematics",
    executable="trajectory_publisher")

  # Nodo de hardware:
  # Ejecuta la interfaz con el hardware físico del robot.
  # Generalmente recibe comandos y los envía a motores,
  # sensores o controladores reales.
  hardware_node = Node(
    package="robot_hardware",
    executable="robot_hardware")

  # Crea la descripción de lanzamiento agregando todos
  # los nodos que se ejecutarán al iniciar el sistema
  launch_description = LaunchDescription([
    rviz_node, robot_description_node, 
    kinematics_node, hardware_node])

  # Devuelve la descripción de lanzamiento a ROS 2
  return launch_description