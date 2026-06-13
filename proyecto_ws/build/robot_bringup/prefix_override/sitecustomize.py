import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samuel_arteaga/Desktop/Roboticos_Insanos/proyecto_ws/install/robot_bringup'
