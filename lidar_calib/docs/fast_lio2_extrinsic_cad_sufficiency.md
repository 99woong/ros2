# FAST-LIO2 환경에서 CAD 기반 Extrinsic이 충분한 이유

> 작성 일자: 2026-06-08  
> 대상 시스템: 항만 AGV 전방·후방 OS1-32 + IMU 각각 FAST-LIO2 + RTK-GNSS 헤딩 → EKF

---

## 1. 시스템 구성 요약

```
[전방 LiDAR + IMU] ──FAST-LIO2──→ LIO odometry 1 ─┐
                                                     ├──→ EKF ──→ 최종 pose
[후방 LiDAR + IMU] ──FAST-LIO2──→ LIO odometry 2 ─┤
                                                     │
[RTK dual GNSS] ──────────────→ 위치 + 헤딩 ────────┘
```

- 두 FAST-LIO2는 **각각 독립적으로** LIO를 수행 (클라우드 공유 없음)
- EKF는 두 LIO 출력 + RTK를 **센서 융합**
- **전후방 LiDAR 간 extrinsic calibration은 직접 사용되지 않음**
  → 각 LIO의 내부 `T_lidar_imu` (LiDAR-IMU extrinsic)만 필요

---

## 2. 실제로 필요한 Extrinsic 종류

| Extrinsic | 용도 | 필요 정확도 |
|-----------|------|-------------|
| `T_lidar_imu` (전방) | FAST-LIO2 내부 point-to-plane 변환 | ±수 mm / ±0.1° |
| `T_lidar_imu` (후방) | FAST-LIO2 내부 point-to-plane 변환 | ±수 mm / ±0.1° |
| `T_lidar1_lidar2` | **이 시스템에서는 직접 사용 안 함** | — |

→ **전후방 LiDAR 간 상대 extrinsic(T_lidar1_lidar2)** 은 EKF 구성에 불필요.  
→ 각 FAST-LIO2에서 필요한 것은 자신의 `T_lidar_imu`뿐.

---

## 3. CAD 데이터가 충분한 근거

### 3-1. iEKF의 수렴 특성 — Extrinsic 오차 자동 흡수

FAST-LIO2의 iEKF(iterated Extended Kalman Filter)는 매 스캔마다 **수백~수천 개의 point-to-plane 잔차**로 pose를 반복 업데이트한다.

```
잔차 정의:
  r_i = nᵢᵀ × (T_world_body × T_body_lidar_CAD × p_lidar_i  −  q_i)
         ↑ IMU 적분 예측    ↑ CAD extrinsic (고정)            ↑ 맵의 plane 위 점
```

`T_body_lidar_CAD`에 소량의 오차 ε 가 있으면:

```
T_body_lidar_real = T_body_lidar_CAD · ΔT(ε)

오차 ε 효과:
  p_world(CAD)  =  T_world_body  ×  T_body_lidar_CAD  ×  p_lidar
                ≠  p_world(real) =  T_world_body_real  ×  T_body_lidar_real  ×  p_lidar

차이:  ΔT_world = T_world_body_real × T_world_body⁻¹
```

iEKF 업데이트 단계:

```
K = P · Hᵀ · (H · P · Hᵀ + R)⁻¹   (칼만 게인)
x_new = x + K · r                    (상태 갱신)
```

- **P**(예측 공분산)가 충분히 크면 K가 커져서 잔차 r을 강하게 흡수
- iEKF는 매 반복에서 T_world_body를 조금씩 수정해 r → 0 으로 수렴
- 결과: **CAD extrinsic 오차 ε가 T_world_body의 미세 조정으로 보상됨**

### 3-2. 항만 개방 환경의 특성 — 잔차 품질이 충분히 풍부

| 환경 조건 | 항만 개방 환경 |
|-----------|---------------|
| 평탄한 노면 | 항시 존재 → 수백 개의 지면 point-to-plane 잔차 |
| 부두 벽면·컨테이너 | 크고 평탄한 수직면 → 강한 수렴 신호 |
| RTK 보조 | 절대 위치 제공 → LIO 드리프트 주기적 리셋 |
| 저속 AGV | IMU 노이즈 및 bias 발산 느림 |

→ 잔차가 풍부하고 안정적 → CAD extrinsic 오차가 수 mm 수준이면 1~2 스캔 내 흡수

### 3-3. 실측 근거

이전 테스트에서 LiDAR-IMU extrinsic을 CAD 값으로 입력했을 때 FAST-LIO2 잔차가 정상 수준(수 cm 이하)으로 수렴하는 것을 확인. 이는 iEKF의 댐핑 특성이 실제로 작동하고 있음을 의미한다.

---

## 4. FAST-LIO2 iEKF 스캔 잔차란?

### 4-1. 한 줄 정의

```
잔차 = "현재 pose 추정으로 월드 변환한 LiDAR 포인트"와
        "로컬 맵의 가장 가까운 평면(plane)" 사이의 수직 거리
```

### 4-2. 수식

각 LiDAR 포인트 pᵢ (LiDAR 좌표계) 에 대해:

```
p_world_i = T_world_imu  ×  T_imu_lidar  ×  p_i
            └── iEKF 현재 추정 pose ──┘

이웃 포인트 k-NN 탐색 → SVD plane 피팅 → (nⱼ, qⱼ) 획득

잔차:  rᵢ = nⱼᵀ × (p_world_i − qⱼ)   [단위: m, 스칼라]
```

```
      로컬 맵의 plane
      ────────────────────  ← qⱼ (면의 점), nⱼ (법선벡터)
            ↑ rᵢ
            │
            ● p_world_i     ← 현재 pose로 변환된 측정 포인트
```

### 4-3. iEKF 반복 순서

```
① IMU 적분 → 예측 pose T̂

② 반복 (k = 0, 1, 2 … 수렴까지)

   a) 현재 Tₖ 로 전체 스캔 포인트 변환
   b) 각 포인트에 대해 ikd-tree에서 5-NN 탐색
   c) SVD로 plane (nⱼ, qⱼ) 피팅 (조건 나쁘면 skip)
   d) 잔차 rᵢ = nⱼᵀ × (p_world_i − qⱼ)  계산 (수백~수천 개)
   e) 야코비안 H = ∂rᵢ/∂x 계산
   f) 칼만 업데이트: Tₖ₊₁ = Tₖ + K · r
   
③ |Tₖ₊₁ − Tₖ| < ε → 수렴 → 출력
```

### 4-4. Point-to-Point 대신 Point-to-Plane을 쓰는 이유

```
Point-to-point  rᵢ = p_world_i − p_map_i    (3D 벡터)
  → 면의 접선 방향 오차도 잔차에 포함 → 수렴 불안정, 느림

Point-to-plane  rᵢ = nⱼᵀ × (p_world_i − qⱼ)  (스칼라 1개)
  → 법선 방향 오차만 측정 → 정보가 집중된 방향만 보정
  → 수렴 빠름, 점 수 대비 효율 높음
  → 면에 평행한 슬라이딩은 잔차 0 → 퇴화 방향은 IMU가 보완
```

---

## 5. Extrinsic 오차 흡수 전 과정 요약

```
CAD T_lidar_imu 입력 (오차 ε ≈ 수 mm, 수 0.1°)
         │
         ▼
① 첫 스캔: 잔차 rᵢ 비제로 (ε 만큼 틀린 위치로 변환)
         │
         ▼
② iEKF 업데이트: T_world_body를 rᵢ→0 방향으로 미세 조정
         │   (K 게인 × 잔차 = 수 mm 단위 pose 보정)
         │
         ▼
③ 2~3 스캔 후: 잔차 r ≈ 0 으로 수렴
         │   (T_world_body가 ε를 암묵적으로 흡수)
         │
         ▼
④ 이후: RTK 위치+헤딩이 주기적으로 T_world_body를 앵커
         → 장거리 드리프트 억제
         │
         ▼
최종: 실용 정확도 수 cm 이내 유지
     (항만 AGV 자율주행에 충분)
```

---

## 6. 결론

| 항목 | 판단 | 근거 |
|------|------|------|
| `T_lidar_imu` CAD 입력 | **충분** | iEKF가 수 스캔 내에 오차 흡수 |
| `T_lidar1_lidar2` 정밀 캘리브레이션 | **불필요** | 이 아키텍처에서 두 LIO는 독립적으로 동작 |
| 항만 환경 적합성 | **유리** | 개방 환경 → 각 LIO의 커버리지 독립 확보 |
| RTK 역할 | **드리프트 앵커** | 두 LIO 출력 + RTK를 EKF 융합 → 전역 일관성 |

**CAD 데이터로 extrinsic을 입력하고, FAST-LIO2의 iEKF 수렴에 맡기는 것이 이 시스템에서 실용적으로 올바른 선택이다.**
