#!/usr/bin/env python3

import rclpy                                        # Biblioteca principal de ROS 2
from rclpy.node import Node                         # Clase base para crear nodos ROS 2
from robot_kinematics.kinematics import Robot       # Modelo cinemático del robot
from geometry_msgs.msg import Twist, PointStamped   # Mensajes para recibir posiciones objetivo
from sensor_msgs.msg import JointState              # Mensaje para publicar y recibir estados articulares

# Nodo encargado de generar trayectorias y publicarlas
class PublicadorTrayectoria(Node):

  def __init__(self):

    # Inicializar nodo
    super().__init__("nodo_publicador")

    # Crear modelo del robot
    self.robot = Robot((0.15, 0.3, 0.45))

    # Recibir objetivos mediante Twist
    self.sub_twist = self.create_subscription(
      Twist,
      "/goals_twist",
      self.twist_callback,
      1
    )

    # Recibir objetivos mediante PointStamped
    self.sub_ps = self.create_subscription(
      PointStamped,
      "/clicked_point",
      self.ps_callback,
      1
    )

    # Publicar posiciones articulares deseadas
    self.js_pub = self.create_publisher(
      JointState,
      "/joint_states_goals",
      1
    )

    # Leer estado actual de las articulaciones
    self.js_sub = self.create_subscription(
      JointState,
      "/joint_states",
      self.js_callback,
      10
    )

    # Evita ejecutar varias trayectorias simultáneamente
    self.is_moving = False

    # Mensaje JointState reutilizable
    self.joint_state_msg = JointState()

    # Nombres de las articulaciones
    self.joint_state_msg.name = [
      "base_joint",
      "shoulder_joint",
      "arm_joint",
      "forearm_joint"
    ]

  # Procesa objetivos recibidos por /goals_twist
  def twist_callback(self, msg:Twist):

    if self.is_moving:
      return

    self.is_moving = True

    self.get_logger().info(
      "Posición recibida: {}".format(str(msg.linear))
    )

    # Generar trayectoria desde la posición actual
    self.robot.def_tray(
      th_i=(
        self.js_current.position[0],
        self.js_current.position[1],
        self.js_current.position[2]
      ),
      xi_f=(
        msg.linear.x,
        msg.linear.y,
        msg.linear.z
      )
    )

    self.get_logger().info(
      "Posición final EF: {}".format(
        self.robot.xi_m[:, self.robot.muestras - 1]
      )
    )

    self.get_logger().info(
      "Posición final juntas: {}".format(
        self.robot.th_m[:, self.robot.muestras - 1]
      )
    )

    # Mostrar gráficas de la trayectoria
    self.robot.imp_tray()
    self.robot.imp_junt()
    self.robot.imp_vel()
    self.robot.imp_ace()

    # Iniciar publicación de la trayectoria articular
    self.current_pos = 0

    self.timer_pub = self.create_timer(
      self.robot.dt,
      self.timer_pub_callback
    )

  # Procesa objetivos recibidos por /clicked_point
  def ps_callback(self, msg:PointStamped):

    if self.is_moving:
      return

    self.is_moving = True

    self.get_logger().info(
      "Posición recibida: {}".format(str(msg.point))
    )

    # Generar trayectoria desde la posición actual
    self.robot.def_tray(
      th_i=(
        self.js_current.position[0],
        self.js_current.position[1],
        self.js_current.position[2]
      ),
      xi_f=(
        msg.point.x,
        msg.point.y,
        msg.point.z
      )
    )

    self.get_logger().info(
      "Posición final EF: {}".format(
        self.robot.xi_m[:, self.robot.muestras - 1]
      )
    )

    self.get_logger().info(
      "Posición final juntas: {}".format(
        self.robot.th_m[:, self.robot.muestras - 1]
      )
    )

    # Mostrar gráficas de la trayectoria
    self.robot.imp_tray()
    self.robot.imp_junt()
    self.robot.imp_vel()
    self.robot.imp_ace()

    # Iniciar publicación de la trayectoria articular
    self.current_pos = 0

    self.timer_pub = self.create_timer(
      self.robot.dt,
      self.timer_pub_callback
    )

  # Publica punto por punto la trayectoria calculada
  def timer_pub_callback(self):

    # Actualizar marca de tiempo
    self.joint_state_msg.header.stamp = (
      self.get_clock().now().to_msg()
    )

    # Posición articular correspondiente al instante actual
    self.joint_state_msg.position = [
      float(self.robot.th_m[0, self.current_pos]),
      float(self.robot.th_m[1, self.current_pos]),
      float(self.robot.th_m[2, self.current_pos])
    ]

    # Publicar estado deseado de las juntas
    self.js_pub.publish(self.joint_state_msg)

    # Avanzar al siguiente punto de la trayectoria
    self.current_pos += 1

    # Finalizar cuando se alcance el último punto
    if self.current_pos == (self.robot.muestras - 1):
      self.is_moving = False
      self.timer_pub.destroy()

  # Guarda el estado actual de las articulaciones
  def js_callback(self, msg:JointState):
    self.js_current = msg


# Punto de entrada del programa
def main():
  try:
    rclpy.init()
    publicador = PublicadorTrayectoria()
    rclpy.spin(publicador)
    rclpy.shutdown()
  except KeyboardInterrupt as e:

    print(e)

# Ejecutar únicamente cuando se invoque directamente
if __name__ == "__main__":
  main()