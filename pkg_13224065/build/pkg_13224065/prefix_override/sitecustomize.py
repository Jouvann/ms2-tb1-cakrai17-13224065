import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jouvan/ros2_ws/src/ms2-tb1-cakrai17-13224065/pkg_13224065/install/pkg_13224065'
