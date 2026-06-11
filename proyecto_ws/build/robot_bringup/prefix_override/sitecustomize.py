import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/davicito/Roboticos_Insanos/proyecto_ws/install/robot_bringup'
