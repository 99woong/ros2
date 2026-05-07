# 🚀 AGV GNSS-LIO Fusion Localization Project

## 1. 프로젝트 개요
*   **목적**: 15톤 대형 AGV의 주행 안정성 확보를 위한 LIO 드리프트 GPS 보정
*   **핵심 문제**: LIO(빨강)와 GPS(노랑) 데이터는 정상이나, EKF 퓨전 결과(파랑)에서 헤딩 불일치 및 진동 발생
*   **환경**: ROS2 Humble, `robot_localization` (EKF), `navsat_transform_node`

## 2. 센서 데이터 사양
*   **LIO (Odom0)**: `/Odometry` (30Hz) - 현재 상대 위치 및 헤딩의 기준
*   **GNSS (Odom1)**: `/fix` (1Hz) -> `navsat_transform`을 통해 투영된 위치[cite: 1]
*   **GPS Heading (IMU0)**: `/heading` (1Hz) -> `/gps/imu`로 변환된 절대 헤딩[cite: 1]

## 3. 주요 이슈 및 분석 로그

### 이슈 1: EKF 헤딩의 90도 직교 현상 (Orthogonal Mismatch)
*   **현상**: `image_94e20f.png`와 같이 파란색(EKF) 헤딩이 빨간색(LIO)과 약 90도 틀어진 채 주행.[cite: 1]


## 4. 현재 설정 (Configuration)
### `ekf.yaml`
```yaml
world_frame: odom
odom0: /Odometry
odom0_config: [true, true, true, false, false, true, ...]

odom1: /odometry/gps
odom1_config: [true, true, false, false, false, false, ...] # Yaw 차단 시도 중[cite: 1]
odom1_differential: true
