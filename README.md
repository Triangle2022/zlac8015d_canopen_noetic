# zlac8015d_canopen_noetic

ZLAC8015D 2채널 모터 드라이버를 USB-to-CAN으로 제어하기 위한 ROS Noetic 패키지입니다.

현재 노드는 Linux SocketCAN 인터페이스(`can0`)를 사용합니다. 즉 USB-to-CAN 장치가 Ubuntu에서 `can0`, `can1` 같은 CAN 네트워크 인터페이스로 잡혀야 합니다.

## 현재 구조

- 입력: `/cmd_vel`
- 출력 제어: CANopen RPDO로 좌/우 모터 목표 RPM 전송
- 피드백 publish: 좌/우 엔코더 카운트와 라디안 각도
- 대상 드라이버: ZLAC8015D 2모터 드라이버

2개 모터 제어는 다음처럼 설정되어 있습니다.

```text
CAN ID: 0x300 + node_id
node_id = 1 이면 0x301

Data[0..1] = left target rpm, int16 little-endian
Data[2..3] = right target rpm, int16 little-endian
```

EDS 기준으로 좌/우 속도와 엔코더는 다음 오브젝트를 사용합니다.

```text
0x60FF sub1 = Left Motor Target_velocity
0x60FF sub2 = Right Motor Target_velocity
0x6064 sub1 = Left Motor Position_actual_value
0x6064 sub2 = Right Motor Position_actual_value
```

## CAN ID

`config/zlac8015d.yaml`의 `node_id`가 `1`일 때 기준입니다.

```text
NMT Start
CAN ID: 0x000
Data: 01 01

SDO Request
CAN ID: 0x601

SDO Response
CAN ID: 0x581

Target Velocity RPDO
CAN ID: 0x301
Data: left_rpm int16 + right_rpm int16
```

`node_id`를 `2`로 바꾸면 SDO/RPDO CAN ID도 자동으로 `0x602`, `0x582`, `0x302`처럼 바뀝니다.

## 파라미터

파라미터 파일은 [config/zlac8015d.yaml](config/zlac8015d.yaml)입니다.

```yaml
can_interface: can0
node_id: 1

wheel_radius: 0.08
wheel_separation: 0.38

rpm_limit: 1000
command_timeout: 0.5

publish_encoder: true
encoder_poll_rate: 20.0
encoder_counts_per_rev: 4096.0

auto_enable: true
configure_velocity_mode_on_start: true
configure_rpdo_on_start: true
```

각 파라미터 의미는 다음과 같습니다.

| 파라미터 | 의미 |
| --- | --- |
| `can_interface` | SocketCAN 인터페이스 이름입니다. 보통 `can0`입니다. |
| `node_id` | ZLAC8015D CANopen Node ID입니다. 기본값은 `1`입니다. |
| `wheel_radius` | 바퀴 반지름입니다. `/cmd_vel`을 RPM으로 변환할 때 사용합니다. 단위는 meter입니다. |
| `wheel_separation` | 좌/우 바퀴 중심 간 거리입니다. 회전 명령을 좌/우 속도로 나눌 때 사용합니다. 단위는 meter입니다. |
| `rpm_limit` | 목표 RPM 제한값입니다. EDS상 `60FF` 범위가 `-1000..1000`이라 기본값은 `1000`입니다. |
| `command_timeout` | `/cmd_vel`이 이 시간 동안 안 들어오면 목표 RPM을 0으로 보냅니다. 단위는 second입니다. |
| `publish_encoder` | `true`이면 엔코더 값을 주기적으로 읽어서 publish합니다. |
| `encoder_poll_rate` | 엔코더 SDO read 주기입니다. 기본 `20.0 Hz`입니다. |
| `encoder_counts_per_rev` | 1회전당 엔코더 카운트입니다. 현재 사용하는 값 기준으로 `4096.0`입니다. |
| `auto_enable` | 시작 시 자동으로 드라이버 enable sequence를 보냅니다. |
| `configure_velocity_mode_on_start` | 시작 시 velocity mode 관련 SDO 설정을 보냅니다. |
| `configure_rpdo_on_start` | 시작 시 목표속도 RPDO 매핑을 설정합니다. |

엔코더 각도 변환은 다음 식을 사용합니다.

```text
angle_rad = encoder_count / encoder_counts_per_rev * 2*pi
```

현재 설정은 `encoder_counts_per_rev: 4096.0`입니다.

## 추가로 필요할 수 있는 파라미터

실제 로봇에 올리면 좌/우 모터 장착 방향이나 엔코더 증가 방향이 반대로 나올 수 있습니다. 그 경우 아래 파라미터를 코드에 추가하는 것이 좋습니다.

```yaml
left_motor_inverted: false
right_motor_inverted: false
left_encoder_inverted: false
right_encoder_inverted: false
gear_ratio: 1.0
```

현재 코드에는 아직 이 반전/감속비 파라미터가 적용되어 있지 않습니다. 실제 테스트에서 한쪽 바퀴가 반대로 돌거나 엔코더 부호가 반대로 나오면 추가하면 됩니다.

## Published Topics

```text
/zlac8015d/target_rpm
type: std_msgs/Int16MultiArray
data[0] = left target rpm
data[1] = right target rpm
```

```text
/zlac8015d/encoder_counts
type: std_msgs/Int32MultiArray
data[0] = left encoder count
data[1] = right encoder count
```

```text
/zlac8015d/encoder_angle
type: std_msgs/Float64MultiArray
data[0] = left encoder angle, rad
data[1] = right encoder angle, rad
```

## Subscribed Topics

```text
/cmd_vel
type: geometry_msgs/Twist
linear.x  = 전진/후진 속도, m/s
angular.z = 회전 속도, rad/s
```

## Services

```text
/zlac8015d_canopen_node/enable
type: std_srvs/Trigger
```

```text
/zlac8015d_canopen_node/disable
type: std_srvs/Trigger
```

## 빌드 방법

ROS Noetic이 설치된 Ubuntu PC에서 catkin workspace에 패키지를 넣고 빌드합니다.

```bash
mkdir -p ~/catkin_ws/src
cp -r zlac8015d_canopen_noetic ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

이미 catkin workspace가 있다면 `src` 폴더 안에 패키지만 복사하고 `catkin_make`를 실행하면 됩니다.

## CAN 인터페이스 실행

USB-to-CAN 장치가 SocketCAN으로 잡히는지 확인합니다.

```bash
ip link
```

예를 들어 `can0`가 보이면 bitrate를 설정하고 인터페이스를 올립니다.

500 kbit/s 예시:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

드라이버 설정과 같은 bitrate를 사용해야 합니다. EDS에는 `50`, `125`, `250`, `500`, `1000 kbit/s`가 지원되는 것으로 되어 있습니다.

CAN 송수신 확인용:

```bash
candump can0
```

다른 터미널에서 프레임을 수동 전송할 수도 있습니다.

```bash
cansend can0 000#0101
```

## 노드 실행 방법

패키지 실행:

```bash
roslaunch zlac8015d_canopen_noetic zlac8015d_canopen.launch
```

다른 설정 파일을 사용하려면:

```bash
roslaunch zlac8015d_canopen_noetic zlac8015d_canopen.launch \
  config:=/home/user/my_zlac8015d.yaml
```

속도 명령 테스트:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}' -r 10
```

정지 명령:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' -1
```

엔코더 카운트 확인:

```bash
rostopic echo /zlac8015d/encoder_counts
```

엔코더 각도 확인:

```bash
rostopic echo /zlac8015d/encoder_angle
```

목표 RPM 확인:

```bash
rostopic echo /zlac8015d/target_rpm
```

드라이버 disable:

```bash
rosservice call /zlac8015d_canopen_node/disable
```

드라이버 enable:

```bash
rosservice call /zlac8015d_canopen_node/enable
```

## 확인 순서

1. USB-to-CAN 장치가 `can0`로 잡히는지 확인합니다.
2. ZLAC8015D의 Node ID와 bitrate를 확인합니다.
3. `config/zlac8015d.yaml`의 `node_id`, `can_interface`, `wheel_radius`, `wheel_separation`을 실제 값으로 맞춥니다.
4. `can0`를 올립니다.
5. `roslaunch`로 노드를 실행합니다.
6. `/zlac8015d/encoder_counts`가 들어오는지 확인합니다.
7. 낮은 속도의 `/cmd_vel`을 보내 좌/우 바퀴 방향을 확인합니다.

## C++와 Python 선택

메인 ROS 드라이버는 C++가 좋습니다.

- `/cmd_vel`을 계속 받아 CAN으로 안정적으로 보내야 합니다.
- Linux SocketCAN API와 바로 붙기 좋습니다.
- 장시간 실행되는 로봇 베이스 노드에 적합합니다.

Python은 초기 테스트, SDO read/write 실험, 파라미터 튜닝 스크립트에 좋습니다.


