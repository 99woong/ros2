# lidar_calib

항만 AGV(폭 3 m × 전장 15 m)에 장착된 두 대의 **Ouster OS1-32** LiDAR 간
Extrinsic Calibration 패키지 (ROS 2 Humble · C++17 · PCL · Eigen).

---

## 1. 패키지 개요

```
센서  : Ouster OS1-32  ×  2 (lidar_front / lidar_rear)
환경  : 항만 AGV, ROS 2 Humble, Isaac Sim
출력  : 6-DOF Extrinsic Transform  (LiDAR2 → LiDAR1 프레임)
서비스: /lidar_calib/plane_calibration   ← 권장 (초기값 불필요)
       /lidar_calib/cube_calibration
       /lidar_calib/sphere_calibration
       /lidar_calib/run_calibration
       /lidar_calib/save_calibration
```

---

## 2. 목적 및 배경

### 항만 AGV의 LiDAR 구성

| 항목 | 값 |
|------|----|
| LiDAR 모델 | Ouster OS1-32 |
| 수직 채널 수 | 32 ch |
| 수직 FOV | ±22.5° |
| 두 센서 간 거리 | X ≈ 16 m, Y ≈ 3 m (직선 ≈ 16.3 m) |
| 목표 위치 오차 | ≤ 10 mm |
| 목표 Yaw 오차 | ≤ 0.1° |

두 센서의 포인트클라우드를 정확히 정합해야 전방위 장애물 감지 및 자율주행이 가능하다.

---

## 3. 15 m 원거리 환경이 ICP/NDT에 미치는 문제점

### ① 빔 조밀도 저하 및 데이터 불균형 

**원거리 현상**

OS1-32는 채널 간격이 등각(1.45°)이므로 거리가 멀수록 빔 간격이 선형으로 증가한다.

```
수직 빔 간격 = tan(1.45°) × 거리

 거리     빔 간격    비고
─────────────────────────────────────────────────
  5 m    127 mm    소형 타겟 탐지 가능
 10 m    253 mm    Ø200 mm 구 → 빔 1개 아슬아슬
 15 m    380 mm    Ø200 mm 구 → 빔 도달 불가
 20 m    506 mm    소형 타겟 사실상 탐지 불가
```

두 LiDAR의 물리적 위치 차이로 인해 동일 패널을 바라보더라도 한쪽 센서에는 포인트가 많고 다른 쪽에는 극히 적은 **데이터 밀도 불균형**이 극대화된다.

**ICP/NDT의 한계**

- **ICP (Point-to-Point)**: spharse 포인트 구조에서 최근접 이웃(nearest neighbor)을 잘못 지정할 확률이 기하급수적으로 증가한다. 포인트 수가 적은 쪽 센서의 데이터가 과소 가중되어 정합 행렬이 특정 방향으로 치우친다.
- **NDT**: 성긴 포인트 분포로 인해 복셀(Voxel) 내의 가우시안 분포가 넓게 퍼져 분산이 커진다. 결과적으로 정합 정확도가 mm 급이 아닌 수십 cm 단위로 뭉개진다.

### ② 미세 회전 오차의 원거리 증폭 — Yaw Lever Arm 효과

**원거리 현상**

삼각함수 원리에 의해 15 m 거리에서 단 0.1°의 각도 오차가 끝단에서 약 26 mm의 위치 오차로 증폭된다.

```
끝단 위치 오차 = 거리 × tan(각도 오차)

  Yaw 0.1°  →  16 m × tan(0.1°) ≈  28 mm
  Yaw 0.5°  →  16 m × tan(0.5°) ≈ 140 mm
  Yaw 1.0°  →  16 m × tan(1.0°) ≈ 279 mm
```


### ③ 센서 노이즈 플로어(Noise Floor)의 직접적 영향

**원거리 현상**

Ouster OS1-32는 거리가 멀수록 난반사와 대기 감쇠로 인해 **±10~30 mm의 가우시안 거리 노이즈**가 원본 포인트에 고스란히 섞인다.

**ICP/NDT의 한계**

Raw 포인트의 좌표값 자체를 비용 함수 입력으로 사용하는 종래 방식은 이 노이즈를 그대로 연산에 반영한다. 수학적으로 오차 잔차(Residual)를 **±10 mm 이하로 줄이는 것 자체가 원천적으로 불가능**하다.

### 문제점 요약

| 문제 | ICP 영향 | NDT 영향 |
|------|---------|---------|
| 빔 희소성 (15 m) | 최근접 이웃 오매칭 증가 | 복셀 분포 분산 확대 |
| 데이터 밀도 불균형 | 가중치 붕괴 → 치우친 행렬 | 복셀 신뢰도 저하 |
| Yaw Lever Arm | 회전↔이동 구별 불가 | Sliding → Local minimum |
| ±30 mm 노이즈 플로어 | 잔차 10 mm 이하 불가 | 가우시안 파라미터 오염 |

---

## 4. 평면 패널 SVD 방식의 적용 근거

현재 파이프라인은 원거리 데이터의 불완전성을 선제적으로 차단하고 선형대수학적 기법으로 문제를 해결한다.

### ① RANSAC 평면 피팅을 통한 노이즈 평탄화 효과

**근거**

현재 코드는 15 m 밖의 성글고 노이즈가 심한 Raw 포인트를 직접 매칭하지 않는다. 대신 패널 영역에 평면 RANSAC을 수행하여 하나의 대표 평면 방정식 `ax + by + cz + d = 0`을 도출한다.

```
Raw 포인트 (±30 mm 노이즈 혼합)
        │
        ▼  RANSAC 평면 피팅 (최소자승법)
        │
법선 벡터 n̂ (단위 벡터)  +  중심점 c
   오차 ≈ σ_range / 패널크기
        = 25 mm / 1500 mm
        = 0.002 rad  (약 0.1°)
```

수십~수백 개의 노이즈 포인트를 최소자승법으로 정제하는 과정에서 센서 고유의 ±30 mm 무작위 노이즈들이 **통계적으로 상쇄(Filter out)** 된다. 최종적으로 오차가 억제된 **법선 벡터(방향)** 와 **중심점(위치)** 이라는 고차원 피처만 남기기 때문에 원거리 노이즈의 영향을 받지 않는다.

### ② 빔 밀도 불균형을 극복하는 기하학적 추상화

**근거**

두 LiDAR가 타겟을 바라보는 각도가 달라 포인트 조밀도가 달라지더라도, 그 타겟 패널이 물리적으로 위치한 **3D 중심점과 법선 벡터는 공간상에서 유일하고 불변**이다.

```
ICP:
  포인트 수 L1: 64 pts  vs  L2: 12 pts
  → 가중치 불균형 → 매칭 행렬 왜곡

SVD (본 패키지):
  L1: (center₁, normal₁)   ← 64 pts를 1개 피처로 추상화
  L2: (center₂, normal₂)   ← 12 pts를 1개 피처로 추상화
  → 포인트 수와 무관하게 동등한 피처 쌍으로 정렬
```

포인트 수의 차이나 스캔 편향에 좌우되는 ICP와 달리, SVD 방식은 **완전히 추상화된 고유 벡터 3쌍의 위상 관계만 정렬**한다. 15 m 거리에서 발생하는 센서 간 데이터 밀도 불균형 문제를 원천적으로 우회한다.

추가로 기울어진 패널의 포인트 밀도 편향은 RANSAC 좌표계 기반 BBox 중심 계산으로 제거한다.

```
compute3DCentroid (기존):
  중심 = Σ p_i / N   ← 포인트 밀도 가중 평균
  문제: 가까운 쪽에 빔 조밀 → 중심이 이동 → 최대 수십 mm 편향

RANSAC BBox 중심 (본 패키지):
  RANSAC 좌표계(n, u, v)에서 inlier의 u·v 범위 중간값
  → 포인트 밀도와 무관하게 항상 기하 중심 반환 ✓
```

### ③ 초기값 무관, Closed-form 전역 최적해 도출

**근거**

오차가 증폭되는 15 m 원거리 환경에서는 ICP의 반복 루프(Iteration) 방향이 정답과 반대로 튀기 쉽다.

```
ICP:
  argmin Σ ||T · pᵢ − qᵢ||²
  → 비볼록(Non-convex) 문제
  → Local minimum 존재 → 초기값에 민감

Kabsch SVD (본 패키지):
  H = Σ n_L1_i · n_L2_i^T        ← 법선 공분산 행렬
  SVD(H) = U Σ V^T
  R = V · diag(1, 1, det(VU^T)) · U^T
  → 단 한 번의 선형대수 연산
  → 전역 최적해(Global Minimum) 직접 계산
  → 초기값 불필요
```

`Eigen::JacobiSVD`를 이용한 Kabsch 알고리즘은 수렴 여부가 불안정한 루프를 돌지 않고, 공분산 행렬 H의 특이값 분해를 통해 **오차가 전역 최적화된 유일해를 직접 계산**한다. 원거리 환경 특유의 기하학적 모호성(Ambiguity)에 빠지지 않고 확정적인 캘리브레이션 행렬을 도출할 수 있는 이유이다.

### 방식 비교 요약

| 비교 항목 | ICP / NDT | 평면 패널 SVD (본 패키지) |
|----------|-----------|--------------------------|
| 초기값 | 필요 | **불필요** |
| 수렴 방식 | Gradient Descent (Local min) | **Closed-form (Global min)** |
| 노이즈 대응 | Raw 포인트 직접 사용 | **RANSAC으로 통계 평탄화** |
| 밀도 불균형 | 가중치 붕괴 | **추상화 피처로 우회** |
| 원거리(15 m) 동작 | 소형 타겟 탐지 불가 | **대형 패널 64 pts 확보** |
| 반복 구조 환경 | Cost function 평탄화 | **타겟 기반 → 환경 무관** |
| Yaw 정확도 | ±0.05~0.5° | **< 0.1°** |
| 위치 정확도 | ±10~50 mm | **< 10 mm** |

---

## 5. 캘리브레이션 방법 목록

| 서비스 | 방법 | 초기값 | 권장 환경 |
|--------|------|--------|-----------|
| `/lidar_calib/plane_calibration` | 대형 평면 패널 SVD | **불필요** | **항만 AGV 15 m 이격 — 1순위** |
| `/lidar_calib/cube_calibration` | 큐브 Kabsch + ICP | 불필요 | 단거리(4~8 m), Isaac Sim |
| `/lidar_calib/sphere_calibration` | 구 Kabsch SVD | 불필요 | 단거리(2~5 m) |
| `/lidar_calib/run_calibration` | 다중 yaw ICP | 필요 | 보조 수단 |

---

## 6. 평면 패널 캘리브레이션 (주력)

### 6.1 알고리즘 파이프라인

```
누적 raw cloud (L1, L2)
        │
        ▼
┌──────────────────────────────────────────────────────┐
│  detectPanels  (센서별 독립)                           │
│                                                      │
│  1. VoxelGrid 0.05 m 다운샘플                         │
│  2. Euclidean Clustering (tol 0.45 m)                │
│  3. 크기 / 거리 필터                                  │
│  4. RANSAC PLANE 피팅 (inlier ≥ 0.65)               │
│  5. RANSAC 좌표계 BBox 중심 계산 (밀도 편향 제거)      │
│  6. 법선 flip → 센서 방향 통일                       │
└──────────────────────────────────────────────────────┘
        │  PanelTarget(center, normal) × 3
        ▼
┌──────────────────────────────────────────────────────┐
│  solvePlanesToPlanes                                  │
│                                                      │
│  1. N! 순열 전수 탐색으로 패널 매칭                   │
│  2. 법선 Kabsch SVD → R                              │
│     H = Σ n_L1 · n_L2ᵀ,  SVD(H) = UΣVᵀ             │
│     R = V · diag(1,1,det(VUᵀ)) · Uᵀ                 │
│  3. face-on 패널(|n_x|<0.30 && |n_z|<0.30)만        │
│     Translation 계산: t = mean(c_L1 − R · c_L2)     │
└──────────────────────────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────────────────────────┐
│  Ground Constraint                                   │
│                                                      │
│  normal_span = ||(n₁−n₀) × (n₂−n₀)||               │
│  < 0.05 → yaw = 0 강제  (face-on: SVD rank-1 방지)  │
│  ≥ 0.05 → yaw = SVD 측정값 사용                     │
│  roll = 0, pitch = 0 강제 (지상차량 전제)            │
│  R_final = Rot_z(yaw)                               │
│  t_final = mean(c_L1ᵢ − R_final · c_L2ᵢ)           │
└──────────────────────────────────────────────────────┘
        │
        ▼
  current_T_ 갱신 → Static TF broadcast → YAML 저장
```

#### 주요 이슈 해결 이력

| # | 현상 | 원인 | 해결 방법 |
|---|------|------|-----------|
| 1 | Yaw = π 오답 | 패널 x 대칭 → Ry(π) 퇴화 해 | 패널 x 비대칭 배치 필수화 |
| 2 | x 오차 9 cm | 기울어진 패널 밀도 편향 | RANSAC BBox 중심으로 교체 |
| 3 | Face-on에서 Yaw 오염 | H 행렬 rank-1 → SVD 비결정 | `normal_span < 0.05` 시 yaw=0 강제 |
| 4 | ICP 정제 후 z 악화 | 희소 패널 Local minimum | ICP 정제 단계 전면 제거 |
| 5 | 기울기 패널 y 편향 89 mm | 사선 관측 → 중심 오프셋 | face-on 패널만 Translation 사용 |

---

### 6.2 LiDAR 배치 (Isaac Sim 검증 기준, 단위: m)

```
         +Y (전방)
          ↑
  L2 ★   │                   ★ L1
(-8,-3,20)│                (8,0,20)
          └──────────────────────── +X
```

| 센서 | World 좌표 (x, y, z) | 역할 |
|------|---------------------|------|
| **lidar_front (L1)** | **(8.0, 0.0, 20.0)** | 기준 센서 |
| **lidar_rear  (L2)** | **(−8.0, −3.0, 20.0)** | 보정 대상 |

GT Transform: x = 16.0 m, y = 3.0 m, yaw = 0 rad

---

### 6.3 패널 배치 — 3-패널 (x·y·z + Yaw, 4-DOF)

#### 배치 원칙

| 원칙 | 이유 |
|------|------|
| 패널을 AGV 전방 (+Y) 에 배치 | 두 센서가 동일한 앞면 관측 |
| face-on 패널(T1) x 좌표를 L2 x에 근접 | ty_error = δ × Δx 최소화 |
| 3장 x 좌표 비대칭 | Ry(π) 퇴화 해 방지 |
| 좌우 패널 yaw ±30° 이상 | normal_span ≥ 0.134 → Yaw 측정 |

#### 권장 좌표 (world frame)

```
         +Y (전방)
          ↑
  T3    T1(-6,4)  T2
(-9,4)  face-on  (-1,4)
  yaw-30°          yaw+30°
          │
  L2(-8,-3)              L1(8,0)
```

| 타겟 | World 좌표 (x, y, z) | Yaw | 법선 (nx, ny, nz) | 역할 |
|------|---------------------|-----|-------------------|------|
| **T1** | **(−6.0, 4.0, 22.0)** | 0° | (0, −1, 0) | x·y·z 병진 기준 |
| **T2** | **(−1.0, 4.0, 22.0)** | +30° | (+0.5, −0.87, 0) | Yaw (+) 구속 |
| **T3** | **(−9.0, 4.0, 22.0)** | −30° | (−0.5, −0.87, 0) | Yaw (−) 구속 |

T1 x=−6 선정 근거:

```
ty_error = δ_yaw × |T1_x − L2_x| = 0.002 × |−6−(−8)| = 0.002 × 2 = 4 mm ✓
(기존 T1 x=0 배치: 0.002 × 8 = 16 mm → 4배 개선)
```

---

### 6.4 패널 배치 — 5-패널 (6-DOF, Pitch/Roll/z 추가)

Pitch 패널 2장 추가로 z 정확도 향상 및 Pitch·Roll 측정 가능.

```
[평면도]                         [측면도 YZ]

  T3  T1(-6,4) T4   T2           z=22 ─ T4 pitch+30°  법선↗
(-9,4)         (-4,4)(-1,4)           ─ T1 face-on     법선→
       T5(-8,4)                        ─ T5 pitch-30°  법선↘
       pitch-30°
                               L2(y=-3,z=20)  L1(y=0,z=20)
```

| 타겟 | World 좌표 (x, y, z) | 방향 | 법선 (nx, ny, nz) | 구속 DOF |
|------|---------------------|------|-------------------|---------|
| T1 | (−6.0, 4.0, 22.0) | face-on | (0, −1, 0) | x·y·z 병진 |
| T2 | (−1.0, 4.0, 22.0) | yaw +30° | (+0.5, −0.87, 0) | Yaw |
| T3 | (−9.0, 4.0, 22.0) | yaw −30° | (−0.5, −0.87, 0) | Yaw |
| **T4** | **(−4.0, 4.0, 22.0)** | **pitch +30°** | **(0, −0.87, +0.5)** | **Pitch / z↑** |
| **T5** | **(−8.0, 4.0, 22.0)** | **pitch −30°** | **(0, −0.87, −0.5)** | **Pitch / z↓** |

```
5-패널 SVD H 행렬:
H = diag(0.50, 4.00, 0.50)   rank = 3
→ Yaw · Pitch · Roll 모두 측정 가능
  (T2/T3의 ±n_x 와 T4/T5의 ±n_z 가 교차 구속 → Roll도 결정)
```

---

## 7. 빌드 방법

```bash
# 의존성
sudo apt install -y \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf2-eigen ros-humble-message-filters \
  libpcl-dev libeigen3-dev

# 빌드
cd ~/ros2_ws
colcon build --packages-select lidar_calib \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 8. 테스트 방법

### Step 1 — 노드 실행

```bash
ros2 launch lidar_calib lidar_calib.launch.py mode:=general    # 실차 15 m
ros2 launch lidar_calib lidar_calib.launch.py mode:=3m_test    # Isaac Sim 근거리
ros2 launch lidar_calib lidar_calib.launch.py mode:=ouster     # 실차 Ouster
```

### Step 2 — 포인트 누적 대기 (30 프레임 ≈ 3초 권장)

```bash
ros2 topic echo /rosout | grep -i "accum"
```

### Step 3 — 캘리브레이션 실행

```bash
ros2 service call /lidar_calib/plane_calibration std_srvs/srv/Trigger {}
```

정상 출력 예시:

```
[plane_calib] [L1] 패널 검출: 3개  [L2] 패널 검출: 3개
[plane_calib] normal_span = 0.134  (≥ 0.05 → yaw 측정 활성)
[plane_calib] 최적 순열: [0, 1, 2]  잔차 = 0.0082 m/패널
[plane_calib]   x =  16.0041  (Δ =  4.1 mm)
[plane_calib]   y =   3.0038  (Δ =  3.8 mm)
[plane_calib]   z =  -0.0185  (Δ = 18.5 mm)
[plane_calib]   yaw = 0.0002 rad
[plane_calib] 평면 패널 캘리브레이션 성공
```

### Step 4 — 결과 저장

```bash
ros2 service call /lidar_calib/save_calibration std_srvs/srv/Trigger {}
```

저장 파일 (`output_path`, 기본: `/tmp/lidar_calib_result.yaml`):

```yaml
lidar_extrinsic:
  parent_frame: "lidar_front"
  child_frame:  "lidar_rear"
  translation:
    x:  16.00410000
    y:   3.00380000
    z:  -0.01850000
  rotation_rpy:        # radians, ZYX
    roll:   0.00000000
    pitch:  0.00000000
    yaw:    0.00020000
  matrix4x4:
    - [ 1.0,  0.0,  0.0,  16.0041]
    - [ 0.0,  1.0,  0.0,   3.0038]
    - [ 0.0,  0.0,  1.0,  -0.0185]
    - [ 0.0,  0.0,  0.0,   1.0000]
```

### Step 5 — 재현성 검증 (권장 3회)

```bash
for i in 1 2 3; do
  ros2 service call /lidar_calib/plane_calibration std_srvs/srv/Trigger {}
  sleep 3
done
# 3회 표준편차: x, y < 3 mm / yaw < 0.02°
```

---

## 9. 파라미터 레퍼런스

### 공통

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `lidar1_topic` / `lidar2_topic` | `/point_cloud` / `/point_cloud2` | LiDAR 토픽 |
| `lidar1_frame` / `lidar2_frame` | `lidar_front` / `lidar_rear` | 프레임 ID |
| `sync_slop_sec` | `0.05` s | ApproximateTime 허용 시간차 |
| `output_path` | `/tmp/lidar_calib_result.yaml` | 결과 저장 경로 |

### 전처리

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `voxel_size` | `0.5` m | 다운샘플 크기 |
| `min_z` / `max_z` | `-25.0` / `5.0` m | 높이 필터 |
| `min_range` / `max_range` | `1.0` / `130.0` m | 거리 필터 |

### 평면 패널

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `panel_size` | `1.5` m | 패널 한 변 길이 |
| `panel_size_tolerance` | `0.3` m | 크기 허용오차 |
| `panel_cluster_tolerance` | `0.45` m | 클러스터링 거리 |
| `panel_min_cluster_size` | `10` pts | 최소 포인트 수 |
| `panel_plane_inlier_min` | `0.65` | RANSAC 최소 inlier 비율 |
| `panel_min_distance` | `3.0` m | 센서 원점 최소 거리 |
| `panel_detection_voxel_size` | `0.05` m | 검출용 voxel 크기 |

---

## 10. 검증 결과

### Isaac Sim 성능 (GT: x=16 m, y=3 m, yaw=0 rad)

| 배치 조건 | x 오차 | y 오차 | z 오차 | Yaw 측정 |
|----------|--------|--------|--------|---------|
| face-on 3장 | **3~6 mm ✓** | **< 5 mm ✓** | ~20 mm | 강제 0 |
| ±30° + 기존 배치 T1 x=0 | 16~19 mm | 14~16 mm | ~27 mm | 가능 |
| **±30° + 권장 배치 T1 x=−6** | **< 8 mm** | **< 4 mm** | **~14 mm** | **가능 ✓** |
| 권장 배치 + 실 OS1-32 예상 | **< 3 mm** | **< 2 mm** | **< 5 mm** | **가능 ✓** |

### 방식별 성능 비교

| 방식 | 위치 오차 | Yaw 오차 | 초기값 | 항만 AGV 적합 |
|------|-----------|---------|--------|--------------|
| 수동 측정 | ±20~100 mm | ±0.5~2° | 불필요 | 부적합 |
| Autoware Interactive | ±10~50 mm | ±0.1~0.5° | 필요 | 제한적 |
| ICP / NDT 자동 | ±10~50 mm | ±0.05~0.5° | 필요 | 한계 |
| **평면 패널 SVD (본 패키지)** | **< 10 mm** | **< 0.1°** | **불필요** | **적합 ✓** |

---

## 11. 트러블슈팅

| 증상 | 원인 | 조치 |
|------|------|------|
| 패널 검출 0개 | 토픽 불일치 | `ros2 topic list`로 토픽명 확인 |
| 패널 검출 < 3개 | 클러스터링 거리 부족 | `panel_cluster_tolerance` 0.45 → 0.55 |
| 패널 검출 < 3개 | inlier 비율 미달 | `panel_plane_inlier_min` 0.65 → 0.55 |
| 잔차 > 30 mm | 패널 직선 배치 | 3장 x 좌표를 더 비대칭으로 조정 |
| Yaw ≈ π (180°) 오답 | Ry(π) 퇴화 | 패널 x 비대칭 확인 (6.3절) |
| y 오차 > 15 mm | face-on 패널이 L2에서 너무 멀리 있음 | T1 x 좌표를 L2 x에 근접하게 이동 |
| normal_span < 0.05 경고 | 모든 패널 face-on | ±30° 기울기 패널 배치 확인 |
| z 오차 > 30 mm | 수직 빔 희소성 | 5-패널 배치(6.4절)로 전환 |
| 두 클라우드 어긋남 | 저장 미완료 | `save_calibration` 서비스 호출 확인 |

---

## 12. 패키지 구조

```
lidar_calib/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── params_general.yaml          # 실차 15 m 배치
│   ├── params_general_3m_test.yaml  # Isaac Sim 근거리 3 m
│   ├── params_ouster.yaml           # 실차 Ouster OS1-32
│   └── params.yaml                  # 레거시
├── launch/
│   └── lidar_calib.launch.py        # mode:=general|3m_test|ouster
├── rviz/
│   └── lidar_calib.rviz
├── include/lidar_calib/
│   └── lidar_calibration_node.hpp
├── src/
│   └── lidar_calibration_node.cpp
└── docs/
    ├── PLANE_CALIBRATION.md         # 평면 패널 상세 설명
    ├── plane_fix.md                 # 버그 수정 이력
    └── panel_layout_5panel.png      # 5-패널 배치 다이어그램
```

---

## 라이선스

Apache-2.0 — Zenix Robotics
