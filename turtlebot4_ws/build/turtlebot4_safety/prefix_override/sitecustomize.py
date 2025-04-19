import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eason-pan/Documents/Gits/5335-final-project/turtlebot4_ws/install/turtlebot4_safety'
