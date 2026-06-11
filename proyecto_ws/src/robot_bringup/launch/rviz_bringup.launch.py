#!/usr/bin/env python3
# Indica que este script debe ejecutarse con Python 3

# Importa la clase principal para crear una descripción de lanzamiento (launch)
from launch import LaunchDescription
# Importa la acción Node para ejecutar nodos de ROS 2
from launch_ros.actions import Node
# Permite obtener la ruta de instalación de un paquete ROS 2
from ament_index_python import get_package_share_directory
# Permite ejecutar comandos desde el archivo launch
from launch.substitutions import Command
# Permite definir parámetros con un tipo de dato específico
from launch_ros.parameter_descriptions import ParameterValue

# Función principal que ROS 2 ejecuta al lanzar este archivo
def generate_launch_description():

  # Obtiene la ruta del directorio share del paquete robot_description
  description_path = get_package_share_directory(
    "robot_description")

  # Construye la ruta completa del archivo URDF
  urdf_path = description_path + "/urdf/robotRRR.urdf"

  # Construye la ruta completa de la configuración de RViz
  rviz_path = description_path + "/rviz/rviz.conf.rviz"

  # Ejecuta el comando xacro sobre el archivo URDF
  # Esto permite procesar archivos xacro o URDF mediante el procesador xacro
  urdf_xacro  = Command(["xacro ", urdf_path])

  # Crea el parámetro robot_description que contendrá
  # el modelo completo del robot en formato XML
  urdf_param = {"robot_description": 
                ParameterValue(urdf_xacro, value_type=str)}
  
  # =========================
  # Definición de nodos
  # =========================
  
  # Nodo RViz:
  # Abre RViz utilizando el archivo de configuración especificado
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_path]
  )

  # Nodo robot_state_publisher:
  # Publica las transformaciones TF del robot a partir del URDF
  # almacenado en el parámetro robot_description
  robot_description_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[urdf_param]
  )

  # Nodo joint_state_publisher_gui:
  # Muestra una interfaz gráfica con deslizadores para mover
  # las articulaciones del robot y publicar sus estados
  joint_publisher_node = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui"
  )

  # Crea la descripción de lanzamiento agregando los nodos
  # que se ejecutarán al iniciar el launch
  launch_description = LaunchDescription([
    rviz_node, robot_description_node, 
    joint_publisher_node
  ])

  # Devuelve la descripción de lanzamiento a ROS 2
  return launch_description