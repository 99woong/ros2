# Dual LiDAR PTP 동기화 설정 (nuvo-9531 ↔ OS1-32 x2)

- 작성일: 2026-07-22
- 대상 장비: nuvo-9531 (PTP grandmaster), OS1-32 lidar x2
- 목적: 기존 lidar 1대만 PTP 동기화하던 구성을 `enp3s0`, `enp4s0` 두 인터페이스에 연결된 lidar 2대 동시 동기화로 확장

---

## 1. 하드웨어 구성 확인

`enp3s0`, `enp4s0`가 서로 다른 PTP Hardware Clock(PHC)을 사용하는지 확인. 같은 PHC를 공유한다면 별도 처리가 필요 없지만, 다르면 두 클럭을 서로 맞춰주는 설정이 추가로 필요함.

```bash
ethtool -T enp3s0   # enp3s0 인터페이스의 타임스탬핑 기능 및 PHC 번호 확인 -> PTP Hardware Clock: 1
ethtool -T enp4s0   # enp4s0 인터페이스의 타임스탬핑 기능 및 PHC 번호 확인 -> PTP Hardware Clock: 2
```

**결과: `enp3s0` = PHC1(`/dev/ptp1`), `enp4s0` = PHC2(`/dev/ptp2`) — 서로 다른 물리 클럭.**
→ 두 lidar가 진짜로 같은 시간 기준을 갖게 하려면 `ptp4l`이 PHC1↔PHC2를 내부적으로 맞춰줘야 함 (아래 3번 항목).

---

## 2. ptp4l 실행 방식 — 프로세스 1개, 인터페이스 2개

### 잘못된 방법 (사용하지 말 것)

```bash
sudo ptp4l -i enp3s0 -m -H -f /etc/linuxptp/ptp4l.conf &   # lidar1용 마스터
sudo ptp4l -i enp4s0 -m -H -f /etc/linuxptp/ptp4l.conf &   # lidar2용 마스터 (별도 프로세스)
```

이렇게 프로세스를 2개로 나누면:
- 두 프로세스가 같은 config의 `uds_address`(관리용 소켓 경로)를 그대로 사용해서 소켓 바인딩이 충돌함
- PHC1과 PHC2가 완전히 독립적으로 free-run 하게 되어, lidar1과 lidar2가 서로 다른 시간 기준을 갖게 될 위험이 있음 (개별 lidar는 "락"이 걸려도 서로는 어긋날 수 있음)

### 올바른 방법 — 하나의 ptp4l 프로세스에 `-i`를 두 번 지정

```bash
ptp4l -i enp3s0 -i enp4s0 -m -H -f /etc/linuxptp/ptp4l.conf &
# -i enp3s0        : lidar1이 연결된 인터페이스 (port 1로 등록됨)
# -i enp4s0        : lidar2가 연결된 인터페이스 (port 2로 등록됨), -i를 반복 지정하면 한 프로세스가 여러 포트를 동시에 관리
# -m               : 로그를 syslog 대신 stdout으로 출력 (터미널에서 바로 확인용)
# -H               : 하드웨어 타임스탬핑 강제 사용 (config의 time_stamping hardware와 동일한 의미, 명시적으로 재확인)
# -f <config>      : 아래 3번 항목에서 수정한 ptp4l.conf 경로 지정
PTP_PID=$!
```

인터페이스를 하나의 프로세스에 함께 등록하면 ptp4l이 내부적으로 하나의 클럭 서보를 공유하면서 두 포트(PHC)를 같은 기준으로 관리하며, `uds_address` 소켓도 하나만 생성되므로 충돌이 없음.

---

## 3. `/etc/linuxptp/ptp4l.conf` 수정 사항

### 3-1. `boundary_clock_jbod` — 서로 다른 PHC를 소프트웨어로 동기화

```
- boundary_clock_jbod 0
+ boundary_clock_jbod 1
```

- `0`(기본값): 여러 포트의 PHC가 원래 같은 칩에서 나와 하드웨어적으로 cross-timestamping을 지원한다고 가정 (지금 하드웨어는 해당 안 됨)
- `1`: PHC들이 서로 무관한 독립 하드웨어(Just a Bunch Of Devices)임을 명시 → ptp4l이 시스템 클럭을 매개로 PHC1과 PHC2를 소프트웨어적으로 맞춰줌
- 실행 로그에 `port 2: just a bunch of devices` 가 출력되면 이 모드가 정상 적용된 것

> 참고: 소프트웨어 기반 동기화라 완전한 하드웨어 등급(수십 ns급) 정밀도를 **보장하지는 않음.** 두 PHC가 진짜 같은 오실레이터를 공유하는 하드웨어(동일 칩 멀티포트 NIC)이거나 외부 공통 기준(GNSS/PPS 분배)을 쓰는 경우보다는 이론상 정밀도가 낮을 수 있음. 다만 실측 결과 두 lidar 모두 `master_offset` 200ns 내외로 나와 dual-lidar 용도로는 충분한 수준임을 확인함 (7번 검증 항목 참고). 만약 나중에 이보다 훨씬 타이트한(<100ns급) 보장이 필요해지면 하드웨어 교체(공유 PHC NIC) 또는 외부 공통 기준 분배가 필요함.

### 3-2. `uds_address` — 관리용 소켓 경로를 non-root 쓰기 가능한 위치로 변경

```
- uds_address        /var/run/ptp4l
+ uds_address        /run/ptp4l/ptp4l
```

`/var/run`(`/run`)은 `root:root 0755`라 일반 계정으로는 그 안에 소켓 파일을 생성할 수 없어서, 별도로 쓰기 권한이 열린 하위 디렉터리(`/run/ptp4l/`)를 만들고 그 안을 가리키도록 변경 (생성 방법은 6번 항목 참고).

---

## 4. sudo 없이 실행하기 위한 권한(capability) 부여

`ptp4l`은 원래 다음 이유로 root 권한이 필요함: 시스템 클럭 조정(`CAP_SYS_TIME`), raw/PTP 소켓 및 네트워크 제어(`CAP_NET_ADMIN`, `CAP_NET_RAW`), 1024 미만 privileged 포트(UDP 319/320) 바인딩(`CAP_NET_BIND_SERVICE`). `sudo` 없이도 되도록, 매번 비밀번호를 묻는 대신 바이너리에 필요한 capability만 직접 부여함 (`sudo` 전체를 열어주는 것보다 안전).

```bash
sudo setcap cap_net_admin,cap_net_raw,cap_net_bind_service,cap_sys_time+ep /usr/sbin/ptp4l
# cap_net_admin         : 네트워크 인터페이스/소켓 설정 제어 권한
# cap_net_raw           : PTP raw 소켓 사용 권한
# cap_net_bind_service  : UDP 319/320 (PTP 이벤트/제너럴 포트, 1024 미만) 바인딩 권한
# cap_sys_time          : 시스템/하드웨어 클럭(PHC) 시간 조정 권한
# +ep                   : Effective + Permitted 플래그로 부여 (실행 시 항상 활성화)

getcap /usr/sbin/ptp4l   # capability가 실제로 파일에 부여됐는지 확인
```

> 주의: `linuxptp` 패키지가 업데이트(`apt upgrade`)되면 바이너리가 교체되면서 이 capability가 초기화되므로, 업데이트 후에는 `setcap` 명령을 다시 실행해야 함.

---

## 5. `/dev/ptp*` 디바이스 접근 권한 — 전용 그룹 생성

capability만으로는 부족함. `/dev/ptp1`, `/dev/ptp2`는 일반 리눅스 파일 권한(DAC)으로 `root:root 0660`처럼 막혀 있어서, capability와 별개로 그룹 권한을 열어줘야 함.

```bash
sudo groupadd -f ptp        # ptp 전용 그룹 생성 (-f: 이미 있으면 에러 없이 넘어감)
sudo usermod -aG ptp zenix  # 현재 계정을 ptp 그룹에 추가 (-aG: 기존 그룹 유지하면서 추가)

echo 'KERNEL=="ptp[0-9]*", GROUP="ptp", MODE="0660"' | sudo tee /etc/udev/rules.d/99-ptp.rules
# KERNEL=="ptp[0-9]*" : /dev/ptp0, /dev/ptp1, ... 이름 패턴에 매칭되는 디바이스에 규칙 적용
# GROUP="ptp"         : 디바이스 파일의 소유 그룹을 ptp로 지정
# MODE="0660"         : 소유자/그룹만 읽기·쓰기 가능하도록 권한 설정
# 위 내용을 root 권한으로 /etc/udev/rules.d/99-ptp.rules 파일에 저장 (재부팅해도 유지됨)

sudo udevadm control --reload-rules   # udev 규칙을 다시 읽어들여 방금 추가한 규칙 반영
sudo udevadm trigger                  # 현재 연결된 디바이스에 대해 규칙을 즉시 재적용
```

**그룹 추가는 재로그인해야 세션에 반영됨.** 재로그인 없이 바로 테스트하려면:

```bash
newgrp ptp   # 현재 쉘에서 바로 ptp 그룹 권한이 적용된 서브쉘 실행 (영구 적용은 재로그인/재부팅 필요)
groups       # ptp 그룹이 목록에 보이는지 확인
ls -l /dev/ptp*   # crw-rw---- root ptp 로 바뀌었는지 확인
```

---

## 6. `uds_address`용 디렉터리 생성

`/run`은 tmpfs라 재부팅 시 초기화되므로, 부팅마다 자동으로 재생성되도록 `systemd-tmpfiles` 규칙으로 등록.

```bash
echo 'd /run/ptp4l 0775 root ptp -' | sudo tee /etc/tmpfiles.d/ptp4l.conf
# d            : 디렉터리 생성 타입
# /run/ptp4l   : 생성할 경로 (ptp4l.conf의 uds_address가 가리키는 위치)
# 0775         : 소유자/그룹 읽기·쓰기·실행, 그 외 읽기·실행 권한
# root ptp     : 소유자 root, 소유 그룹 ptp
# -            : 자동 삭제(age) 설정 없음 (수동 삭제 전까지 유지)

sudo systemd-tmpfiles --create /etc/tmpfiles.d/ptp4l.conf   # 규칙 파일 기준으로 디렉터리 즉시 생성
ls -ld /run/ptp4l   # root:ptp 0775로 생성됐는지 확인
```

---

## 7. 최종 실행 및 검증

### 실행

```bash
ptp4l -i enp3s0 -i enp4s0 -m -H -f /etc/linuxptp/ptp4l.conf &
# sudo 없이 실행 (4, 5, 6번 권한 설정 완료 후)
PTP_PID=$!
```

정상 로그:
```
ptp4l[...]: selected /dev/ptp1 as PTP clock
ptp4l[...]: port 2: just a bunch of devices          # boundary_clock_jbod 1이 정상 적용됨
ptp4l[...]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[...]: port 2: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[...]: port 1: assuming the grand master role
ptp4l[...]: port 2: assuming the grand master role
```

### lidar 측에서 실제 동기화 여부 검증

각 OS1-32의 HTTP API(`/api/v1/time/ptp` 등, 스크립트/alias로 `ptp4_1`, `ptp4_2` 등록해둔 상태)로 확인.

```bash
ptp4_1   # lidar1의 PTP 상태 조회 (내부적으로 lidar1 HTTP API에 curl 요청)
ptp4_2   # lidar2의 PTP 상태 조회
```

**검증 결과 (2026-07-22 기준):**

| 항목 | lidar1 | lidar2 |
|---|---|---|
| `port_state` | SLAVE | SLAVE |
| `gm_present` | true | true |
| `grandmaster_identity` | `78d004.fffe.384cf2` | `78d004.fffe.384cf2` |
| `master_offset` | 208 ns | 185 ns |

두 lidar가 **동일한 grandmaster**(nuvo-9531 자신)를 기준으로 잡고 있고, offset도 200ns 내외로 서브-마이크로초 수준 → dual-lidar PTP 동기화 정상 동작 확인.

---

## 8. 남은 이슈 (기능에는 영향 없음)

`ptp4l` 실행 시 `uds: bind failed: No such file or directory` 로그가 남아있음. 이건 로컬 관리 도구(`pmc`)용 유닉스 소켓 바인딩 실패로, **네트워크로 나가는 실제 PTP 동기화(위 7번 검증 결과)와는 무관**하며 lidar 동기화 자체는 정상 동작함.

- 디렉터리(`/run/ptp4l`, root:ptp 0775)는 존재함을 확인했는데도 실패가 재현됨
- 의심되는 원인: `ptp4l.conf`의 `uds_address` 값 뒤에 인라인 주석(`# [old] /var/run/ptp4l`)을 같은 줄에 붙여놨는데, 이 주석이 파서에서 값의 일부로 잘못 처리됐을 가능성이 있음 → 아직 확정 원인은 아님, 주석을 별도 줄로 분리해서 재시도 필요
- 추후 `pmc -u -b 0 'GET TIME_STATUS_NP'` 같은 로컬 디버깅이 필요해지면 이 부분부터 재점검할 것

---

## 9. 최종 자동화 스크립트 예시

```bash
#!/bin/bash
set -e   # 주의: 백그라운드(&)로 실행되는 명령은 fork에만 성공하면 즉시 0을 반환하므로,
         # ptp4l이 뒤에서 죽어도 set -e는 이를 감지하지 못하고 스크립트가 계속 진행됨 (아래 "개선 제안" 참고)

echo "=== 1. Start PTP master (2 lidar, single process) ==="
ptp4l -i enp3s0 -i enp4s0 -m -H -f /etc/linuxptp/ptp4l.conf &
# sudo 불필요 (setcap + ptp 그룹 설정 완료 상태 기준)
PTP_PID=$!

sleep 3   # ptp4l이 MASTER 상태로 전이될 시간 확보

echo "=== 2. Start Ouster ==="
ros2 launch dual_lidar_fastlio dual_fastlio.launch.py &

sleep 3

echo "=== 3. Start IMU driver ==="
ros2 launch microstrain_inertial_driver microstrain_launch.py &

sleep 2

echo "=== 4. Send time sync (5 times only) ==="
ros2 run time_reference_sender time_reference_sender &

sleep 3

echo "=== 5. Start rosbag (필요 시 주석 해제) ==="
#ros2 bag record /imu/data /imu/data_raw /ouster/imu /ouster/points

wait   # 모든 백그라운드 프로세스가 종료될 때까지 대기 (Ctrl+C로 전체 종료)
```

### 개선 제안 (논의는 했으나 아직 스크립트에 미반영)

```bash
# ptp4l이 정말 떴는지 명시적으로 확인 (set -e가 못 잡는 백그라운드 실패를 대신 검출)
sleep 3
if ! pgrep -f "ptp4l -i enp3s0" > /dev/null; then
    echo "PTP4l 실행 실패!" >&2
    exit 1
fi

# 스크립트 종료(Ctrl+C 포함) 시 백그라운드 프로세스 정리
trap 'kill $(jobs -p) 2>/dev/null' EXIT INT TERM
```
