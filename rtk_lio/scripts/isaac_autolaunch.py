"""
Isaac Sim 자동 실행 스크립트 (isaac_autolaunch.py)

사용법:
  Isaac Sim > Window > Script Editor > Open 으로 이 파일을 열고 Run(F5)

동작:
  1. Play 감지 즉시 시뮬레이션을 일시정지
  2. ros2 launch rtk_lio fusion.launch.py 를 백그라운드로 실행
  3. WAIT_S 초 대기 후 시뮬레이션 재개 (센서 발행 시작)
  4. Stop 감지 시 ROS2 프로세스 전체 종료, 다음 Play 에 재실행
"""

import subprocess
import asyncio
import os
import signal

import omni.timeline
import carb

# ─── 설정 ───────────────────────────────────────────────────────────────────
WAIT_S = 7          # ROS2 노드 초기화 대기 시간 (초) — fusion.launch 내부가 5s이므로 여유 포함
ROS_DISTRO    = "humble"
WORKSPACE_DIR = "/home/zenix/ros2_ws"

LAUNCH_CMD = (
    f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
    f"source {WORKSPACE_DIR}/install/setup.bash && "
    "ros2 launch rtk_lio fusion.launch.py"
)
# ────────────────────────────────────────────────────────────────────────────

_proc  = None   # ROS2 프로세스 핸들
_armed = True   # Play 이벤트 발화 플래그


async def _resume_after(seconds: float):
    """WAIT_S 초 후 시뮬레이션을 재개한다."""
    await asyncio.sleep(seconds)
    omni.timeline.get_timeline_interface().play()
    carb.log_warn(f"[autolaunch] 시뮬레이션 재개 — ROS2 준비 완료 ({seconds}s)")


def _kill_ros2():
    """ROS2 론치 프로세스 그룹 전체를 종료한다."""
    global _proc
    if _proc is not None and _proc.poll() is None:
        try:
            os.killpg(os.getpgid(_proc.pid), signal.SIGTERM)
            carb.log_warn("[autolaunch] ROS2 프로세스 종료 완료")
        except ProcessLookupError:
            pass
    _proc = None


def _on_timeline_event(event):
    global _proc, _armed

    # ── Stop: ROS2 종료, 재실행 대기 ──────────────────────────────────────
    if event.type == int(omni.timeline.TimelineEventType.STOP):
        _kill_ros2()
        _armed = True
        carb.log_warn("[autolaunch] Stop 감지 — 다음 Play 시 ROS2 재실행 준비")
        return

    # ── Play: 이미 발화했으면 무시 ────────────────────────────────────────
    if event.type != int(omni.timeline.TimelineEventType.PLAY):
        return
    if not _armed:
        return
    _armed = False

    # Play 감지 즉시 일시정지 (센서가 발행되기 전에 막음)
    omni.timeline.get_timeline_interface().pause()
    carb.log_warn(f"[autolaunch] Play 감지 → ros2 launch 시작 ({WAIT_S}s 후 재개)")

    # 새 프로세스 그룹으로 실행 (stop 시 killpg 로 하위 프로세스까지 정리)
    _proc = subprocess.Popen(
        ["bash", "-c", LAUNCH_CMD],
        start_new_session=True,
    )

    # WAIT_S 초 후 재개 예약
    asyncio.ensure_future(_resume_after(WAIT_S))


# ── 훅 등록 ──────────────────────────────────────────────────────────────────
_sub = (
    omni.timeline.get_timeline_interface()
    .get_timeline_event_stream()
    .create_subscription_to_pop(_on_timeline_event)
)
carb.log_warn(
    "[autolaunch] 훅 등록 완료 — "
    "Play 시 ROS2 자동 실행, Stop 시 자동 종료"
)
