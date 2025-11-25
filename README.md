# neat_movement

ROS1 Melodic용 2WD 로봇 단순 주행 패키지입니다. 목표 지점을 받으면 회전 후 직진/후진하며, LiDAR 기반 감속/정지 영역으로 장애물을 회피합니다.

## 기능

- **커스텀 액션 서버**: `SimpleMove.action` - 목표 위치 + 전진/후진 플래그
- **회전 후 주행**: 목표 방향으로 회전(후진 시 180° 반대) 후 직진 또는 후진
- **LiDAR 기반 안전 제어**: 
  - 감속 영역(기본 2.0m): 속도 비율 감소
  - 정지 영역(기본 1.0m): 속도 0
- **Dynamic Reconfigure**: 속도, 가감속, 감속/정지 거리, PID 게인 등 실시간 조정 가능
- **로봇 사양 기준 기본값**: 1.8m × 1.8m, 200kg 로봇에 최적화된 파라미터

## 빌드

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

## 실행

```bash
roslaunch neat_movement simple_move_server.launch
```

런치 파일 인자:
- `odom_topic`: 오도메트리 토픽 (기본: `/odom`)
- `scan_topic`: 라이다 스캔 토픽 (기본: `/scan`)
- `cmd_vel_topic`: 속도 명령 토픽 (기본: `/cmd_vel`)
- `base_frame`: 베이스 프레임 (기본: `base_link`)

## Dynamic Reconfigure 파라미터

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

주요 파라미터:
- 속도 제한: `max_linear_vel_forward` (0.8 m/s), `max_linear_vel_backward` (0.5 m/s), `max_angular_vel` (0.8 rad/s)
- 가감속: `max_linear_accel` (0.4 m/s²), `max_linear_decel` (0.6 m/s²)
- 안전 영역: `decel_zone_distance` (2.0m), `stop_zone_distance` (1.0m)
- 제어: `rotation_kp`, `rotation_kd`, `rotation_tolerance`, `xy_goal_tolerance`

## 액션 테스트

```bash
# actionlib 클라이언트
rosrun actionlib_tools axclient.py /simple_move

# 또는 rostopic으로 goal 전송
rostopic pub /simple_move/goal neat_movement/SimpleMoveActionGoal "goal: {target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}, drive_forward: true}"
```

## 노드 구조

- **simple_move_server**: 액션 서버 노드
  - 구독: `odom_topic`, `scan_topic`
  - 발행: `cmd_vel_topic`
  - 액션: `/simple_move`

