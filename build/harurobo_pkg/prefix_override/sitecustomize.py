import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/altair/.local/share/Trash/files/ros2_haru/install/harurobo_pkg'
