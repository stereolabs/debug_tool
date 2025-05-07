import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/Documents/dev/internal_ros_ws/src/sync_data/install/debug_tool'
