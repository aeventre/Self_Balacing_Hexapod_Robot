import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/spiderbot/SpiderBot/ros2_ws/install/i2c_comms'
