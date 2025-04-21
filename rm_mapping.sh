cd /home/feihu/shaobing/FeiHu_Sentry_2025
source install/setup.bash
source /opt/ros/humble/setup.bash


commands=(
"gnome-terminal -- bash -c ' ros2 launch livox_ros_driver2 msg_MID360_launch.py   ; exec bash;'"
"sleep 2.8"
"gnome-terminal -- bash -c ' ros2 launch rm_bringup rm_mapping.launch.py  ; exec bash;'"
"sleep 60"
"gnome-terminal -- bash -c 'ros2 service call /sc_liorf_localization/save_map sc_liorf_localization/srv/SaveMap \"{resolution: 0.2, destination: /RMUL/}\"; exec bash'"
)

cd /home/feihu/shaobing/FeiHu_Sentry_2025
for command in "${commands[@]}"; do
  
  eval "$command"
done





                                                                                                                  





