# The lidar needs to be activated
cd amr_reflective_strip_recognition
colcon build
source install/setup.bash 
ros2 launch amr_reflective_strip_recognition yl_launch.py

