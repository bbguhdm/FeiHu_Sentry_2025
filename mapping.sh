cd /home/feihu/shaobing/FeiHu_Sentry_2025
source install/setup.bash
source /opt/ros/humble/setup.bash


commands=(
"gnome-terminal -- bash -c ' ros2 launch livox_ros_driver2 msg_MID360_launch.py   ; exec bash;'"
"sleep 2.8"
"gnome-terminal -- bash -c ' ros2 launch rm_bringup mapping.launch.py  ; exec bash;'"
)

cd /home/feihu/shaobing/FeiHu_Sentry_2025
for command in "${commands[@]}"; do
  
  eval "$command"
done





                                                                                                                  





