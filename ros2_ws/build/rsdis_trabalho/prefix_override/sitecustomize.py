import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/claudio/rsdis/RSDIS_TRABALHO-FINAL/ros2_ws/install/rsdis_trabalho'
