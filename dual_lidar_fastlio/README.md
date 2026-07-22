# dual_lidar_fastlio

## 개요

두 대의 Ouster OS1-32 LiDAR와 FAST-LIO2를 동시에 운용하기 위한 ROS 2 런치 패키지입니다.

각 LiDAR에 대해 독립적인 `ouster_ros` 드라이버와 `fast_lio` 인스턴스를 기동하며, 두 SLAM 결과를 `/Odometry1`, `/Odometry2`로 분리 출력합니다.  
`config/components.yaml`로 각 컴포넌트를 개별 활성화/비활성화할 수 있으며, 런치 인자로 실시간 override도 가능합니다.

```
Sensor 1 (192.168.10.11) ──► /ouster1/points ──► [fast_lio 1] ──► /Odometry1  ──► [RViz2 #1]
                              /imu/data ──────────────────────┘   TF: camera_init_1 → body_1
                                                                    (외부 IMU)

Sensor 2 (192.168.20.11) ──► /ouster2/points ──► [fast_lio 2] ──► /Odometry2  ──► [RViz2 #2]
                              /ouster2/imu ────────────────────┘   TF: camera_init_2 → body_2
                                                                    (Ouster 내장 IMU)
```

---

## 개발 사양

| 항목 | 내용 |
|------|------|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| LiDAR | Ouster OS1-32 × 2대 |
| Sensor 1 IP | 192.168.10.11 |
| Sensor 2 IP | 192.168.20.11 |
| IMU (Sensor 1) | 외부 IMU — `/imu/data` |
| IMU (Sensor 2) | Ouster 내장 IMU — `/ouster2/imu` |
| 의존 패키지 | `ouster_ros` 0.13.15, `fast_lio` (FAST_LIO_ROS2) |

### 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ouster1/points` | `sensor_msgs/PointCloud2` | Sensor 1 포인트 클라우드 |
| `/ouster2/points` | `sensor_msgs/PointCloud2` | Sensor 2 포인트 클라우드 |
| `/ouster2/imu` | `sensor_msgs/Imu` | Sensor 2 내장 IMU |
| `/imu/data` | `sensor_msgs/Imu` | 외부 IMU (Sensor 1용) |
| `/Odometry1` | `nav_msgs/Odometry` | Sensor 1 FAST-LIO2 출력 |
| `/Odometry2` | `nav_msgs/Odometry` | Sensor 2 FAST-LIO2 출력 |

### UDP 포트 할당

| | lidar_port | imu_port |
|-|------------|----------|
| Sensor 1 | 7502 | 7503 |
| Sensor 2 | 7512 | 7513 |

---

## 파일 구성

```
dual_lidar_fastlio/
├── config/
│   ├── components.yaml   # 컴포넌트 활성화/비활성화 설정
│   ├── ouster64_1.yaml   # FAST-LIO2 인스턴스 1 파라미터
│   └── ouster64_2.yaml   # FAST-LIO2 인스턴스 2 파라미터
├── launch/
│   └── dual_fastlio.launch.py
└── rviz/
    ├── fastlio_1.rviz    # RViz2 설정 (Sensor 1)
    └── fastlio_2.rviz    # RViz2 설정 (Sensor 2)
```

---

## 빌드 방법

### 1. 사전 요구사항

`ouster_ros`와 `fast_lio` 패키지가 같은 워크스페이스에 있어야 합니다.

```
ros2_ws/src/
├── ouster-ros/
├── FAST_LIO_ROS2/
└── dual_lidar_fastlio/   ← 본 패키지
```

### 2. 빌드

```bash
cd ~/ros2_ws

# fast_lio도 함께 빌드 (TF 프레임 파라미터화 패치 포함)
colcon build --packages-select fast_lio dual_lidar_fastlio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

> **참고:** `FAST_LIO_ROS2/src/laserMapping.cpp`에 `common.map_frame` / `common.body_frame` 파라미터가 추가되었습니다.  
> 두 인스턴스의 TF 프레임 충돌을 방지하기 위한 패치이며, 기본값(`camera_init`, `body`)은 기존 동작과 동일합니다.

---

## 사용법

### 기본 실행 (전체 컴포넌트)

```bash
ros2 launch dual_lidar_fastlio dual_fastlio.launch.py
```

### 컴포넌트 활성화/비활성화

#### 방법 1 — `config/components.yaml` 수정 (영구 설정)

```yaml
# config/components.yaml
ouster_1:        true
ouster_2:        true
fast_lio_1:      true
fast_lio_2:      false  # ← FAST-LIO2 인스턴스 2 비활성화
fast_lio_1_rviz: true
fast_lio_2_rviz: false  # ← RViz2 인스턴스 2 비활성화
```

수정 후 재빌드 없이 바로 반영됩니다 (`install/` 심볼릭 링크 사용 시).

#### 방법 2 — 런치 인자 (일회성 override)

YAML 설정을 무시하고 CLI에서 개별 제어할 수 있습니다.

```bash
# Sensor 2 드라이버와 관련 컴포넌트 모두 비활성화
ros2 launch dual_lidar_fastlio dual_fastlio.launch.py \
  ouster_2:=false fast_lio_2:=false fast_lio_2_rviz:=false
```

### 런치 인자 목록

| 인자 | 기본값(YAML) | 설명 |
|------|-------------|------|
| `ouster_1` | `true` | Ouster 드라이버 1 활성화 |
| `ouster_2` | `true` | Ouster 드라이버 2 활성화 |
| `fast_lio_1` | `true` | FAST-LIO2 인스턴스 1 활성화 |
| `fast_lio_2` | `true` | FAST-LIO2 인스턴스 2 활성화 |
| `fast_lio_1_rviz` | `true` | RViz2 (인스턴스 1) 활성화 |
| `fast_lio_2_rviz` | `true` | RViz2 (인스턴스 2) 활성화 |
| `lidar_mode` | `1024x10` | Ouster 해상도/주파수 |
| `use_sim_time` | `false` | 시뮬레이션 클럭 사용 여부 |

### Extrinsic 캘리브레이션 수정

```yaml
# config/ouster64_1.yaml 또는 ouster64_2.yaml
mapping:
    extrinsic_T: [ tx, ty, tz ]       # LiDAR → IMU 평행이동 (미터)
    extrinsic_R: [ r00, r01, r02,     # LiDAR → IMU 회전행렬
                   r10, r11, r12,
                   r20, r21, r22 ]
```
