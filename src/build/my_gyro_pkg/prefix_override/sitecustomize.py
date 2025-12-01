import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thejas/high_tide_ws/src/install/my_gyro_pkg'
