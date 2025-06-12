
cd /mnt/3TB_indego/shuo/ws_ros2jazzy-main_lab/ros2_ws


source install/setup.bash

export PYTHONPATH=/mnt/3TB_indego/shuo/ws_ros2jazzy-main_lab/py3.12/lib/python3.12/site-packages:$PYTHONPATH
export PYTHONPATH=/mnt/3TB_indego/shuo/ws_ros2jazzy-main_lab/openpi/src:$PYTHONPATH
export PYTHONPATH=/mnt/3TB_indego/shuo/ws_ros2jazzy-main_lab/openpi/packages/openpi-client/src:$PYTHONPATH

ros2 run pi0_ur5_grasp pi0grasp
