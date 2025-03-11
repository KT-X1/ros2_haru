#!/bin/bash
#sudo modprobe slcan
#sudo modprobe can
#sudo modprobe can_raw

#sudo slcand -o -c -s8 /dev/ttyACM1 can0
#sudo ip link set can0 up

colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

# planning_nodeを新しいタブで開始
gnome-terminal --tab --title="planning_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run harurobo_pkg planning_node; exec bash"

# web_socket_nodeを新しいタブで開始
gnome-terminal --tab --title="web_socket_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run harurobo_pkg web_socket_node; exec bash"

# serial_send_nodeを新しいタブで開始
gnome-terminal --tab --title="serial_send_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run harurobo_pkg serial_send_node; exec bash"

# serial_receive_nodeを新しいタブで開始
gnome-terminal --tab --title="serial_receive_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run harurobo_pkg serial_receive_node; exec bash"