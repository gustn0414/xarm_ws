# xarm_ws

xarm_1s 로봇 작업 워크스페이스

## 패키지 구성

```
src/
├── xarm_1s_description    # URDF / xacro / 메쉬
├── xarm_1s_hardware       # ros2_control HW interface (USB HID)
├── xarm_1s_moveit_config  # MoveIt2 설정 (SRDF, kinematics, controllers...)
├── xarm_1s_bringup        # ★ 통합 launch 파일 (메인 진입점)
└── mtc                    # MoveIt Task Constructor pick&place 노드
```

## 빌드

```bash
cd ~/xarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행 방법

모든 진입점은 `xarm_1s_bringup` 패키지에 모여 있습니다.

### 시뮬레이션 (mock 하드웨어)

```bash
ros2 launch xarm_1s_bringup demo.launch.py use_mock:=true
```

### 실제 로봇 (USB HID)

`/dev/hidraw0` 접근 권한이 필요합니다 (임시: `sudo chmod 666 /dev/hidraw0`).

```bash
ros2 launch xarm_1s_bringup demo.launch.py use_mock:=false
```

### MTC pick&place 데모

위 demo.launch.py 가 실행 중인 상태에서, 다른 터미널에서:

```bash
ros2 launch xarm_1s_bringup pick_place_demo.launch.py
```

## 호환성 (이전 명령어)

기존 명령어도 그대로 동작합니다 (내부적으로 bringup 을 include 하는 wrapper):

```bash
ros2 launch xarm_1s_moveit_config demo.launch.py use_mock:=true
ros2 launch mtc                   pick_place_demo.launch.py
```
