# lidar_calib

항만 AGV(폭 3 m × 길이 15 m)에 장착된 두 대의 **Ouster OS1-32** LiDAR 간  
extrinsic calibration 패키지 (ROS2 Humble, C++17, PCL).

---

## LiDAR 배치 (탑뷰, 차량 중심 기준)

```
         ↑ 전방 (+Y)

  ┌───────────────────────────────┐  ─┐
  │                               │   │
  │  (-1.454, +7.26)              │   │
  │  ★ LiDAR2 (sim_lidar2)        │   │
  │                               │  15 m
  │                               │   │
  │                               │   │
  │              LiDAR1 ★         │   │
  │              (+1.454, -7.26)  │   │
  └───────────────────────────────┘  ─┘
  ├────────── 3 m ─────────────────┤

  두 LiDAR 간 간격
    ΔX = -2.908 m  (좌→우)
    ΔY = +14.52 m  (후→전)
    직선 거리 ≈ 14.82 m
```

---

## 캘리브레이션 방법 비교

| | **방법 A — 큐브 기반** ★ | **방법 B — 구(Sphere) 기반** | **방법 C — 다중 yaw ICP** |
|---|---|---|---|
| 초기값 필요 | **불필요** | **불필요** | 필요 (init_x, init_y) |
| 타깃 | 1×1×2 m 큐브 3개 | 구 3개 이상 | 없음 (환경 전체 사용) |
| 알고리즘 | Plane RANSAC → Kabsch SVD → ICP 정제 | Sphere RANSAC → Kabsch SVD | 다중 yaw 탐색 ICP |
| Isaac Sim | **권장** | 구 오브젝트 필요 | 가능 |
| 실차 | 가능 | **권장** | 보조 수단 |

> **Isaac Sim 기본 권장: 방법 A** — 큐브 3개를 비직선 배치하면 초기값 없이 정확한 transform을 계산합니다.

---

## 빌드

```bash
# 의존성
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-eigen \
  ros-humble-message-filters \
  libpcl-dev

# 빌드
cd ~/ros2_ws
colcon build --packages-select lidar_calib --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 방법 A — 큐브 기반 캘리브레이션 (Isaac Sim 권장)

### 알고리즘 흐름

```
raw cloud (각 LiDAR)
    ↓
Euclidean clustering → BBox 검사 (1×1×2 m ± 0.4 m)
    ↓
Plane RANSAC → face_center - 0.5 m × 법선 = 큐브 기하 중심
    ↓
matchAndSolve (순열 전수탐색 + Kabsch SVD)  ← 초기값 불필요
    ↓
ICP 정제 (coarse → fine, 전체 voxelized cloud)
    ↓
YAML 저장 + TF 브로드캐스트
```

### 1단계 — Isaac Sim에서 큐브 배치

> 큐브 크기: **1 m × 1 m × 2 m** (높이 2 m 방향이 Z축)

코드는 큐브의 **위치를 사전에 알지 않습니다.** 포인트 클라우드에서 `1×1×2 m` bbox에 해당하는 클러스터를 자동으로 찾습니다. 따라서 아래 조건만 만족하면 어디든 배치 가능합니다.

**배치 조건 (필수):**

| 조건 | 기준 | 이유 |
|------|------|------|
| 두 LiDAR 공통 시야 안에 배치 | 양쪽에서 클러스터 검출 가능해야 함 | 대응점 없으면 Kabsch 불가 |
| 3개 비직선 배치 | 삼각형 면적 > 5 m² 권장 | SVD conditioning |
| 주변에 동일 크기(1×1×2 m) 물체 없음 | — | 오검출 방지 |
| 수직으로 세워서 배치 | Z축이 높이 2 m | bbox 높이 검사 기준 |

**시뮬레이션 참조 배치 (docs 기준):**

```
         ↑ 전방 (+Y)
                              ■ object3 (0, 15)
  ★ LiDAR2 (-1.454, 7.26)
                                ■ object2 (5, 10)
         [  AGV 차체  ]
    ■ object1 (-5, -10)
  ★ LiDAR1 (1.454, -7.26)
```

| 오브젝트 | 참조 위치 (차량 중심 기준) | LiDAR1 거리 | LiDAR2 거리 |
|----------|--------------------------|-------------|-------------|
| object1 | (-5, -10, 1) m | 7.0 m | 17.6 m |
| object2 | ( 5,  10, 1) m | 17.6 m | 7.0 m |
| object3 | ( 0,  15, 1) m | 22.3 m | 7.9 m |

삼각형 면적 = **75 m²**, object3의 직선 이탈 = **6.7 m** (SVD conditioning 양호)

### 2단계 — 노드 실행

```bash
# Isaac Sim (general 모드)
ros2 launch lidar_calib lidar_calib.launch.py mode:=general

# 실차 Ouster (ouster 모드)
ros2 launch lidar_calib lidar_calib.launch.py mode:=ouster
```

### 3단계 — RViz2로 큐브 포인트 확인 (선택)

```bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/lidar_calib/rviz/lidar_calib.rviz
```

각 LiDAR의 포인트 클라우드에서 세 큐브가 모두 확인되는지 검토합니다.  
원거리 큐브(LiDAR1 기준 object3 = 22.3 m)는 포인트 수가 적을 수 있으나 정상입니다.

### 4단계 — 큐브 캘리브레이션 실행

```bash
ros2 service call /lidar_calib/cube_calibration std_srvs/srv/Trigger {}
```

**정상 로그 예시:**

```
=== 큐브 기반 캘리브레이션 시작 ===
  cloud1=XXXXX pts  cloud2=XXXXX pts
  기대 큐브 크기: 1.0×1.0×2.0 m  허용오차 ±0.4 m
[LiDAR1] 큐브 검출 중...
  클러스터 N개 검출
    큐브 검출: center=(-5.032, -10.015, 1.001)  bbox=(0.98, 0.95, 1.99)  inlier=87%  pts=124
    큐브 검출: center=(4.971,   9.988, 1.002)   bbox=(0.97, 0.94, 1.98)  inlier=82%  pts=28
    큐브 검출: center=(-0.008, 14.992, 1.003)   bbox=(0.96, 0.93, 1.97)  inlier=74%  pts=18
[LiDAR2] 큐브 검출 중...
  ...
검출 결과: LiDAR1=3개  LiDAR2=3개
  큐브 삼각형 면적 (LiDAR1 기준): 75.0 m²
Kabsch SVD 시작 (3개 큐브 중심)...
  Kabsch 결과: x=-14.523 y=-2.907 z=0.001 yaw=0.000 rad  잔차=0.0152 m/큐브
ICP 정제 시작 (초기값 = Kabsch 결과)...
  coarse ICP score=0.0421
  fine   ICP score=0.0089
큐브 캘리브레이션 성공  ICP_score=0.0089  x=-14.523 y=-2.907 z=0.001 ...
```

**경고 발생 시 대처:**

| 경고 메시지 | 의미 | 대처 |
|-------------|------|------|
| `plane fit 불안정 → 무게중심 fallback` | 원거리 큐브 포인트 부족 | Isaac Sim 정지 상태에서 재시도 또는 `cube_min_cluster_size` 낮춤 |
| `삼각형 면적 < 5 m²` | 큐브 거의 직선 배치 | 큐브 위치 재조정 |
| `Kabsch 잔차 > 1.0 m` | 큐브 검출 오류 또는 매칭 실패 | `cube_size_tolerance` 조정 후 재시도 |
| `ICP fitness > threshold` | 정제 실패 | Kabsch 결과는 저장됨, `fit_threshold` 조정 |

### 5단계 — 결과 저장

```bash
ros2 service call /lidar_calib/save_calibration std_srvs/srv/Trigger {}
```

기본 저장 경로: `/tmp/lidar_calib_result.yaml`

### 6단계 — RViz2에서 정렬 결과 확인

| 토픽 | 내용 |
|------|------|
| `/lidar_calib/aligned_cloud` | LiDAR2 클라우드를 LiDAR1 프레임으로 변환한 결과 |
| `/lidar_calib/merged_cloud` | 두 클라우드 합본 |

세 큐브 영역에서 두 클라우드가 자연스럽게 겹치면 성공입니다.

---

## 방법 B — 구(Sphere) 기반 캘리브레이션

실차 환경에서 구 타겟(반지름 0.3~0.5 m)을 사용하는 경우.

### 절차 요약

1. 구 3개 이상을 두 LiDAR의 공통 시야 내 **비직선** 배치
2. 노드 실행: `ros2 launch lidar_calib lidar_calib.launch.py mode:=ouster`
3. `sphere_radius` 파라미터를 실제 구 크기에 맞춤
4. 서비스 호출: `ros2 service call /lidar_calib/sphere_calibration std_srvs/srv/Trigger {}`
5. 결과 저장: `ros2 service call /lidar_calib/save_calibration std_srvs/srv/Trigger {}`

---

## 방법 C — 다중 yaw ICP 캘리브레이션

타깃 없이 환경 전체 포인트 클라우드로 ICP를 수행합니다.  
초기 위치(init_x, init_y)를 수동 지정해야 합니다.

### 절차 요약

1. `config/params_general.yaml` 에서 초기값 확인:

```yaml
# 현재 설정값 (Isaac Sim 기준)
init_x:  -14.52   # LiDAR1 프레임 기준 LiDAR2의 X 위치 [m]
init_y:   -2.908  # LiDAR1 프레임 기준 LiDAR2의 Y 위치 [m]
yaw_candidates: 8  # 45° 간격으로 전방향 탐색
```

2. 서비스 호출: `ros2 service call /lidar_calib/run_calibration std_srvs/srv/Trigger {}`

---

## 저장 결과 파일 형식

```yaml
# LiDAR Extrinsic Calibration Result
# parent: sim_lidar  child: sim_lidar2
# residual/score: 0.00890000

lidar_extrinsic:
  parent_frame: "sim_lidar"
  child_frame:  "sim_lidar2"
  translation:
    x: -14.52300000
    y:  -2.90800000
    z:   0.00100000
  rotation_rpy:  # radians, ZYX
    roll:   0.00000000
    pitch:  0.00000000
    yaw:    0.00000000
  rotation_quaternion:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  matrix4x4:  # row-major
    - [1.0, 0.0, 0.0, -14.523]
    - [0.0, 1.0, 0.0,  -2.908]
    - [0.0, 0.0, 1.0,   0.001]
    - [0.0, 0.0, 0.0,   1.000]
```

---

## 파라미터 레퍼런스

### 공통

| 파라미터 | general 기본값 | ouster 기본값 | 설명 |
|----------|---------------|--------------|------|
| `lidar_mode` | `general` | `ouster` | 모드 선택 |
| `lidar1_topic` | `/point_cloud` | `/ouster1/points` | LiDAR1 토픽 |
| `lidar2_topic` | `/point_cloud2` | `/ouster2/points` | LiDAR2 토픽 |
| `lidar1_frame` | `sim_lidar` | `os_sensor` | LiDAR1 프레임 ID |
| `lidar2_frame` | `sim_lidar2` | `os_sensor2` | LiDAR2 프레임 ID |
| `sync_slop_sec` | `0.05` s | `0.01` s | ApproximateTime 허용 시간차 |
| `output_path` | `/tmp/lidar_calib_result.yaml` | 동일 | 결과 저장 경로 |

### 전처리

| 파라미터 | general 기본값 | 설명 |
|----------|---------------|------|
| `voxel_size` | `0.1` m | ICP용 다운샘플 크기 |
| `min_z / max_z` | `-2.0 / 5.0` m | 높이 필터 |
| `min_range` | `1.0` m | 자기반사 제거 최소 거리 |
| `max_range` | `60.0` m | 유효 최대 거리 |

### 큐브 캘리브레이션 (방법 A)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `cube_size_xy` | `1.0` m | 큐브 수평 변 길이 |
| `cube_size_z` | `2.0` m | 큐브 높이 |
| `cube_size_tolerance` | `0.4` m | bbox 크기 허용 오차 |
| `cube_cluster_tolerance` | `0.25` m | 유클리드 클러스터링 거리 |
| `cube_min_cluster_size` | `8` pts | 클러스터 최소 포인트 수 (원거리 22 m 고려) |
| `cube_max_cluster_size` | `2000` pts | 클러스터 최대 포인트 수 |
| `cube_plane_inlier_min` | `0.4` | plane RANSAC 최소 inlier 비율 |

### 구 캘리브레이션 (방법 B)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `sphere_radius` | `0.5` m | 사용 구의 반지름 |
| `sphere_radius_tolerance` | `0.1` m | RANSAC 반지름 허용 오차 |
| `sphere_cluster_tolerance` | `0.4` m | 클러스터링 거리 임계값 |
| `sphere_min_cluster_size` | `15` pts | 구로 인정하는 최소 포인트 수 |
| `sphere_max_cluster_size` | `3000` pts | 최대 포인트 수 |

### 다중 yaw ICP (방법 C)

| 파라미터 | general 기본값 | 설명 |
|----------|---------------|------|
| `init_x / init_y / init_z` | `-14.52 / -2.908 / 0.0` | 초기 위치 추정값 [m] |
| `yaw_candidates` | `8` | yaw 탐색 방향 수 (360°/N 간격) |
| `search_correspondence_distance` | `3.0` m | 1차(coarse) ICP 대응 거리 |
| `refine_correspondence_distance` | `1.0` m | 2차(fine) ICP 대응 거리 |
| `max_iterations` | `100` | ICP 최대 반복 횟수 |
| `fitness_score_threshold` | `0.5` | 성공 판정 최대 score |

---

## 서비스 목록

| 서비스 | 설명 |
|--------|------|
| `/lidar_calib/cube_calibration` | **큐브 기반 Kabsch SVD + ICP (초기값 불필요) ★** |
| `/lidar_calib/sphere_calibration` | 구 기반 Kabsch SVD (초기값 불필요) |
| `/lidar_calib/run_calibration` | 다중 yaw ICP (init_x/y 필요) |
| `/lidar_calib/save_calibration` | 현재 transform을 YAML로 저장 |

---

## 트러블슈팅

| 증상 | 원인 | 조치 |
|------|------|------|
| `큐브 검출 부족 (N<3)` | bbox 크기 불일치 | `cube_size_tolerance` 를 0.5~0.6으로 확대 |
| `큐브 검출 부족 (N<3)` | 클러스터 포인트 부족 | `cube_min_cluster_size` 를 5로 낮춤 |
| `plane fit 불안정` 경고 반복 | 원거리 큐브 포인트 극소 | 큐브를 LiDAR에 더 가까운 위치로 이동 |
| `삼각형 면적 < 5 m²` | 큐브 3개 직선에 가까운 배치 | object3을 object1-object2 선에서 수직 방향으로 이동 |
| `Kabsch 잔차 > 1.0 m` | 검출 오류 또는 잘못된 큐브 매칭 | `cube_cluster_tolerance` 조정 또는 큐브 배치 재확인 |
| `ICP fitness > threshold` | ICP 정제 실패 | `fitness_score_threshold` 를 1.0으로 완화하거나 `search_correspondence_distance` 를 5.0으로 확대 |
| 구 검출 0개 | 반지름 불일치 | `sphere_radius` 를 실제 구 크기에 맞춤 |
| ICP 수렴 실패 (방법 C) | init_x/y 오차 과대 | `search_correspondence_distance` 를 5.0으로 확대 |
| 포인트 클라우드 수신 없음 | 토픽 불일치 | `ros2 topic list` 로 실제 토픽명 확인 |
| TF 브로드캐스트 안 됨 | 캘리브레이션 미완료 | 서비스 호출 후 성공 메시지 확인 |

---

## 패키지 구조

```
lidar_calib/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── params_general.yaml    # Isaac Sim / 일반 PointCloud2
│   ├── params_ouster.yaml     # 실차 Ouster OS1-32
│   └── params.yaml            # 레거시
├── launch/
│   └── lidar_calib.launch.py
├── rviz/
│   └── lidar_calib.rviz
├── include/lidar_calib/
│   └── lidar_calibration_node.hpp
├── src/
│   └── lidar_calibration_node.cpp
└── docs/
    ├── lidar_calibration_requirement.pdf
    └── lidar_calibration_requirement2.pdf
```
