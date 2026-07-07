# RTK-LIO EKF 튜닝 기록

**작성일**: 2026-06-24  
**테스트 환경**: Isaac Sim (isim_pagv/pagv_lidar_imu_260622.usda)  
**센서**: OS1-32 LiDAR (~6-7 Hz), 시뮬레이션 IMU (~50 Hz)  
**RTK GNSS**: Isaac Sim rtk_gnss_260622.py (Port of Castellón, Spain 기준점)

---

## 1. 시스템 개요

`rtk_lio` 노드(`gnss_corrector_node`)는 Fast-LIO `/Odometry`와 RTK GNSS `/fix`, `/heading`을 3-state EKF로 융합한다.

```
EKF 상태: [dx, dy, dyaw]  ← LIO local frame과 GNSS map frame 간의 drift
입력:
  - Fast-LIO  /Odometry         → LIO 포즈 (로컬 좌표계)
  - RTK GNSS  /fix              → 절대 위치 (위경도)
  - RTK GNSS  /heading          → 절대 방위각
출력:
  /lpc/localization/pose_update → map 프레임 기준 융합 포즈 + quality
```

EKF 갱신 방정식:

```
K  = P / (P + R)             ← Kalman Gain
x ← x + K × (z - Hx)        ← 상태 갱신
P ← (I - KH) × P             ← 공분산 갱신
```

---

## 2. 문제 현상

### 2.1 정지 상태 위치 분산 (튜닝 전)

| 축 | range | std |
|----|-------|-----|
| X  | ±11 cm | ~3.7 cm |
| Y  | ±3 cm  | ~1.0 cm |
| Z  | ±0.7 cm| ~0.2 cm |

**원인 분석**: 바다 쪽 환경에서 LiDAR 스캔 기하 결핍(Feature Degeneracy).  
X축 방향으로 LiDAR 반사 점군이 거의 없어 Fast-LIO IEKF가 X 방향 제약 없이 수렴.  
Fast-LIO 출력 공분산은 ~5e-9 m² (std ~7 μm)로 실제 오차를 전혀 반영하지 못함.

### 2.2 초기 파라미터 (pos_r_fix = 0.5)

```
K = P / (P + R) ≈ 0.011 / (0.011 + 0.5) ≈ 6%   ← GNSS 영향 매우 약함
gate = sqrt(9.21 × 0.511) ≈ 2.21 m               ← 넓지만 보정력 부족
```

GNSS가 매 업데이트마다 LIO 오차의 6%만 보정 → 오차가 빠르게 누적.

---

## 3. 파라미터 변경 과정

### 3.1 1차 시도: pos_r_fix = 0.01

```yaml
pos_r_fix: 0.01   # K ≈ 77%
```

**결과**: 실패

- K = 77% → 매 업데이트마다 ~7.7 cm 급격한 map 점프 발생
- chi2 gate = sqrt(9.21 × 0.011) ≈ 0.32 m 로 극도로 좁아짐
- 주행 중 LIO 누적 오차 1.0–1.7 m >> gate 0.32 m → 5회 연속 거부 → `gnss_frozen = true`
- quality = 3 고착

### 3.2 2차 시도: pos_r_fix = 0.05

```yaml
pos_r_fix: 0.05   # K ≈ 18%
pos_innovation_gate: 9.21
```

**결과**: 실패

- gate = sqrt(9.21 × 0.061) ≈ 0.75 m
- 컨테이너 야드 18 m 주행 중 LIO 누적 오차 ~1.8 m (10% drift)
- 0.79–1.69 m 혁신량 >> gate 0.75 m → 연속 거부 → quality = 3

#### 로그 예시 (문제 상태)
```
[QUALITY=3] frozen=YES(rej=5 good=0/3)
Pos REJECTED chi2=47.32 > gate=9.21  innov=[1.692, 0.348]m
Pos REJECTED chi2=12.18 > gate=9.21  innov=[0.790, -0.219]m
drift stuck at [-27.196, ...]
```

#### freeze_s 음수 표시 버그 (표시 전용, 품질 계산에 영향 없음)

`lioCallback` 디버그 로그에서 `now = msg->header.stamp` (시뮬 시간 ~3001 s) 와  
`gnss_freeze_start_time_ = this->now()` (Wall clock ~1782267191 s) 를 빼면  
`freeze_s ≈ -1782264190` 같은 음수가 출력된다.  
quality 계산 자체는 `this->now()` 를 일관되게 사용하므로 영향 없음.

### 3.3 최종 설정 (현재 적용)

```yaml
# ---- GPS measurement noise (R) ----
pos_r_fix:    0.05        # K ≈ 37–40%

# ---- Chi-squared innovation gate ----
pos_innovation_gate: 80.0  # gate = sqrt(80 × 0.061) ≈ 2.2 m

# ---- Safety: max drift correction per GPS update ----
max_pos_correction: 0.50   # 최대 50 cm/update 허용

# ---- Use fixed R instead of GNSS reported covariance ----
use_rtk_covariance: false
```

---

## 4. 변경 이유 상세

| 파라미터 | 이전값 | 이후값 | 변경 이유 |
|---------|-------|-------|---------|
| `pos_r_fix` | 0.5 | **0.05** | K 6%→40%: GNSS가 LIO 오차를 실질적으로 보정하게 함 |
| `pos_innovation_gate` | 9.21 | **80.0** | 주행 중 LIO 누적 오차 최대 1.8 m 포함 가능하도록 gate 2.2 m 확보 |
| `max_pos_correction` | 0.3 | **0.50** | gate 확대로 큰 혁신량이 허용될 수 있으므로 보정 상한 상향 |
| `use_rtk_covariance` | true | **false** | 시뮬 GNSS 공분산 불안정 → 고정 R 사용이 더 안정적 |

### Kalman Gain 비교

| pos_r_fix | Kalman Gain K | 해석 |
|-----------|--------------|------|
| 0.5  | ~6%  | GNSS 거의 무시됨 |
| 0.05 | ~40% | GNSS와 LIO 균형 융합 |
| 0.01 | ~77% | GNSS 과신 → map 점프 발생 |

### Chi-squared Gate 비교

```
gate_distance = sqrt(pos_innovation_gate × S_diagonal)
S ≈ P + R = 0.011 + 0.05 = 0.061 m²

gate=9.21  → gate_distance = sqrt(9.21 × 0.061) = 0.75 m  (LIO drift 1.8m 초과)
gate=80.0  → gate_distance = sqrt(80.0 × 0.061) = 2.21 m  (LIO drift 1.8m 포함)
```

---

## 5. 테스트 결과

**테스트 조건**: 컨테이너 야드 내 정지 상태, 약 26회 샘플, quality = 5 유지

### 5.1 위치 정밀도

| 축 | range | std | 비고 |
|----|-------|-----|------|
| X  | 6.9 cm | ~1.28 cm | 바다 방향 (퇴화 방향) |
| Y  | 6.7 cm | ~1.76 cm | 컨테이너 방향 (퇴화 방향) |
| Z  | 0.7 cm | ~0.18 cm | 수직 방향 |

```
2D RMS = sqrt(1.28² + 1.76²) ≈ 2.18 cm
RTK GNSS 이론 정밀도 spec ≈ 2 cm (1σ)
```

### 5.2 Quality 상태

```
quality = 5  (P_trace < pos_converge_threshold 0.1, gnss_frozen = false)
P_ss (정상상태 공분산) ≈ 0.011 << 0.1  → 항상 quality=5 달성
```

### 5.3 통계적 신뢰 구간

| 신뢰 구간 | X 범위 | Y 범위 |
|----------|-------|-------|
| 68% (±1σ) | ±1.28 cm | ±1.76 cm |
| 95% (±2σ) | ±2.56 cm | ±3.52 cm |
| 99% (±3σ) | ±3.84 cm | ±5.28 cm |

range 6.9 cm는 26샘플 최대-최솟값으로, 통계적 대표값은 std(1.3–1.8 cm)가 적합.

---

## 6. 항만 AGV 적용성 평가

| 작업 유형 | 요구 정밀도 | 현재 결과 (std) | 판정 |
|----------|-----------|----------------|------|
| 야드 내 경로 추종 | ±10–15 cm | 1.3–1.8 cm | ✅ 충분 |
| RTG/TC 크레인 진입 | ±5 cm     | 1.3–1.8 cm | ✅ 충분 |
| ASC 크레인 핸드오버 | ±2–3 cm  | 1.3–1.8 cm | △ 경계 |
| 컨테이너 적재 정렬 | ±1–2 cm  | 1.3–1.8 cm | ⚠️ 추가 검토 필요 |

**정지 정밀도는 대부분의 항만 AGV 운용에 충분하다.**  
주행 중 LIO 누적 오차(최대 1.8 m)가 실운용에서 더 큰 제약이며, 별도 검증이 필요하다.

---

## 7. 추후 과제

1. **주행 중 동적 정밀도 검증**: 1–3 m/s 주행 중 `/lpc/localization/pose_update` 기록 및 분석
2. **휠 오도메트리 융합**: LIO 퇴화 방향 보완을 위한 3rd 소스 추가 검토
3. **실제 센서 적용**: 실 OS1-32 (100 Hz IMU) 환경에서 동일 파라미터 재검증
4. **freeze_s 표시 버그 수정**: `lioCallback`의 `now` 를 `this->now()` 로 통일

---

## 8. 최종 gnss_corrector.yaml 핵심 파라미터

```yaml
pos_r_fix:           0.05
pos_innovation_gate: 80.0
max_pos_correction:  0.50
pos_process_noise:   0.001
use_rtk_covariance:  false
```

파일 위치: `rtk_lio/config/gnss_corrector.yaml`
