#!/usr/bin/env python3

# Biblioteca principal de ROS 2 para Python
import rclpy
# Clase base para crear nodos ROS 2
from rclpy.node import Node
# Modelo cinemático del robot
from robot_kinematics.kinematics import Robot
# Mensaje para recibir posiciones objetivo del efector final
from geometry_msgs.msg import Twist
# Mensaje para publicar estados articulares
from sensor_msgs.msg import JointState

# Nodo encargado de generar trayectorias y publicar
# las posiciones articulares correspondientes
class PublicadorTrayectoria(Node):

  def __init__(self):
    # Inicializar nodo ROS 2
    super().__init__("nodo_publicador")

    # Instanciar modelo cinemático del robot
    self.robot = Robot()

    # Suscriptor para recibir posiciones objetivo
    self.sub_twist = self.create_subscription(
      Twist,
      "/goals_twist",
      self.twist_callback, 1) 

    # Publicador de estados articulares
    self.js_pub = self.create_publisher(
      JointState,
      "/joint_states", 1)

    # Bandera para evitar procesar varios movimientos simultáneamente
    self.is_moving = False

    # Mensaje JointState reutilizable
    self.joint_state_msg = JointState()

    # Nombres de las articulaciones
    self.joint_state_msg.name = [
      "shoulder_joint",
      "arm_joint",
      "forearm_joint"]

  # Procesa una nueva posición objetivo del efector final
  def twist_callback(self, msg:Twist):

    # Ignorar nuevas órdenes mientras el robot está en movimiento
    if self.is_moving:
      return

    self.is_moving = True

    # Mostrar posición recibida
    self.get_logger().info(
      "Posición recibida: {}".format(str(msg.linear)))

    # Generar trayectoria desde la configuración inicial
    self.robot.def_tray(
      th_i=(0.1, 0.1, 0.1),
      xi_f=(
        msg.linear.x,
        msg.linear.z,
        msg.angular.y))

    # Mostrar posición final del efector final
    self.get_logger().info(
      "Posición final EF: {}".format(
        self.robot.xi_m[:, self.robot.muestras - 1]))

    # Mostrar posición final de las articulaciones
    self.get_logger().info(
      "Posición final juntas: {}".format(
        self.robot.th_m[:, self.robot.muestras - 1]))

    # Graficar trayectoria cartesiana
    self.robot.imp_tray()

    # Graficar trayectoria articular
    self.robot.imp_junt()

    # Actualizar marca de tiempo
    self.joint_state_msg.header.stamp = (
      self.get_clock().now().to_msg())

    # Asignar posición final de las articulaciones
    self.joint_state_msg.position = [
      float(self.robot.th_m[0, self.robot.muestras - 1]),
      float(self.robot.th_m[1, self.robot.muestras - 1]),
      float(self.robot.th_m[2, self.robot.muestras - 1])]

    # Publicar estado articular
    self.js_pub.publish(self.joint_state_msg)

    # Habilitar recepción de una nueva orden
    self.is_moving = False

# Punto de entrada del programa
def main():

  try:
    # Inicializar ROS 2
    rclpy.init()
    # Crear nodo publicador
    publicador = PublicadorTrayectoria()
    # Mantener nodo en ejecución
    rclpy.spin(publicador)
    # Finalizar ROS 2
    rclpy.shutdown()

  except KeyboardInterrupt as e:

    # Manejar interrupción por teclado
    print(e)

# Ejecutar únicamente cuando el archivo se corre directamente
if __name__ == "__main__":
  main()