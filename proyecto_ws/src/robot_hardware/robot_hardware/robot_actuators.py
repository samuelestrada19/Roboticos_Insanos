#!/usr/bin/env python3
# Indica que este archivo debe ejecutarse utilizando Python 3

# Biblioteca principal de ROS 2 para Python
import rclpy
# Clase base para crear nodos ROS 2
from rclpy.node import Node
# Tipo de mensaje utilizado para publicar y recibir estados de articulaciones
from sensor_msgs.msg import JointState

# Clase que representa el nodo encargado de la comunicación con el hardware
class NodoHardware(Node):

  def __init__(self):

    # Inicializa el nodo con el nombre "nodo_hardware"
    super().__init__("nodo_hardware")

    # Publicador para enviar el estado actual de las juntas
    # Publica mensajes JointState en el tópico /joint_states
    self.js_pub = self.create_publisher(
      JointState,"/joint_states",10)

    # Suscriptor para recibir las posiciones deseadas de las juntas
    # Escucha mensajes JointState en el tópico /joint_states_goals
    self.j_goal_sub = self.create_subscription(
      JointState, "/joint_states_goals",
      self.goal_callback, 10)

    # Mensaje que almacenará el estado actual de las juntas
    self.js_state = JointState()

    # Nombres de las articulaciones del robot
    self.js_state.name = ["shoulder_joint",
                          "arm_joint",
                          "forearm_joint"]

    # Posiciones iniciales de las articulaciones
    self.js_state.position = [0.1, 0.1, 0.1]

    # Mensaje que almacenará las posiciones objetivo recibidas
    self.js_goal = JointState()

    # Nombres de las articulaciones para el mensaje objetivo
    self.js_goal.name = ["shoulder_joint",
                          "arm_joint",
                          "forearm_joint"]

    # Posiciones objetivo iniciales
    self.js_goal.position = [0.1, 0.1, 0.1]

    # Temporizador que ejecuta hw_callback cada 0.01 segundos (100 Hz)
    # Se utiliza para simular la comunicación periódica con el hardware
    self.create_timer(0.01, self.hw_callback)

  # ==================================================
  # Callback que se ejecuta cuando llega un mensaje
  # con posiciones deseadas de las articulaciones
  # ==================================================
  def goal_callback(self, msg:JointState):

    # Lee la posición deseada y la enviaría al hardware
    # Por ahora únicamente la guarda en una variable
    self.js_goal = msg

  # ==================================================
  # Callback ejecutado por el temporizador
  # Se encarga de leer la posición actual del hardware
  # y publicarla en ROS 2
  # ==================================================
  def hw_callback(self):

    # Obtiene el valor actual del hardware
    # Actualmente se simula copiando la posición deseada
    self.js_state.position = self.js_goal.position

    # Actualiza la marca de tiempo del mensaje
    self.js_state.header.stamp = self.get_clock().now().to_msg()

    # Publica el estado actual de las articulaciones
    self.js_pub.publish(self.js_state)

# ==================================================
# Función principal del programa
# ==================================================
def main():

  try:
    # Inicializa el sistema ROS 2
    rclpy.init()
    # Crea una instancia del nodo de hardware
    nodo_hardware = NodoHardware()
    # Mantiene el nodo en ejecución procesando callbacks
    rclpy.spin(nodo_hardware)
    # Cierra ROS 2 cuando termina la ejecución
    rclpy.shutdown()
  # Captura la interrupción por teclado (Ctrl+C)
  except KeyboardInterrupt as e:

    # Muestra el mensaje de interrupción
    print(e)