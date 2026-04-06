# GPZDA Sender (ROS2)

Low-latency GPZDA sender for PPS-based time synchronization (e.g., Microstrain 3DM-GV7).

---

## Features

- PPS 기준 선행 전송 (configurable)
- Low-latency UART (raw mode)
- Real-time thread (SCHED_FIFO)
- μs-level timing (busy-wait)

---

## Build

```bash
cd ~/ros2_ws/src
git clone <your_repo> gpzda_sender

cd ~/ros2_ws
colcon build --packages-select gpzda_sender

source install/setup.bash