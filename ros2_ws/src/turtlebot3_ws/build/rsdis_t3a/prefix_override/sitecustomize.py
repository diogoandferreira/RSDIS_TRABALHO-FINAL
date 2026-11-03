import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/claudio/RSDIS/turtlebot3_ws/install/rsdis_t3a'
