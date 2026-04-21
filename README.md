# xarm_ws

xarm_1s 로봇 작업 과정

# 실행방법

# 시뮬레이션 (기존과 동일)
ros2 launch xarm_1s_moveit_config demo.launch.py use_mock:=true

# 실제 로봇
ros2 launch xarm_1s_moveit_config demo.launch.py use_mock:=false

# 다른 터미널에서 (둘 다 동일)
ros2 launch mtc pick_place_demo.launch.py