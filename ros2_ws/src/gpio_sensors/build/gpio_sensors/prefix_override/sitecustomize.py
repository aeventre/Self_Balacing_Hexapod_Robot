import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/spiderbot/SpiderBot/ros2_ws/src/gpio_sensors/install/gpio_sensors'
