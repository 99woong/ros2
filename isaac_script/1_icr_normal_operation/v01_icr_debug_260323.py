import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.utils.extensions import enable_extension
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import omni.kit.app
import carb.input
import omni.appwindow
import math
#from omni.isaac.debug_draw import _debug_draw
from pxr import Gf

# 설정값
MAX_VEL_DEG = 300.0

# 확장 기능 활성화
enable_extension("omni.isaac.ros2_bridge")
#enable_extension("omni.isaac.debug_draw")

# --- 수정 포인트 1: 확장 기능 강제 로드 및 임포트 방식 변경 ---
enable_extension("omni.isaac.debug_draw")
try:
    from omni.isaac.debug_draw import _debug_draw
except ImportError:
    # 5.1 버전에서 경로가 다를 경우를 대비한 대체 임포트
    import omni.isaac.debug_draw as debug_draw
    _debug_draw = debug_draw.acquire_debug_draw_interface()


class AGVAllWheelVisualizer(Node):
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        super().__init__('agv_all_wheel_visualizer')
        
        # 시각화 인터페이스
        self.draw = _debug_draw.acquire_debug_draw_interface()
        
        self.cmd_vel = 0.0
        self.cmd_steer = 0.0
        
        # 8륜 좌표 설정
        self.wheel_coords = {
            "w1_l": (4.65, 1.2665), "w1_r": (4.65, -1.2665),
            "w2_l": (3.0, 1.2665),  "w2_r": (3.0, -1.2665),
            "w3_l": (-3.0, 1.2665), "w3_r": (-3.0, -1.2665),
            "w4_l": (-4.65, 1.2665), "w4_r": (-4.65, -1.2665)
        }
        
        self.max_steer_deg = 30.0
        self.min_radius = 9.4
        
        self.drive_sub = self.create_subscription(Float64, '/cmd_vel_linear', self.drive_cb, 10)
        self.steer_sub = self.create_subscription(Float64, '/cmd_steer_angle', self.steer_cb, 10)
        
        self._update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self.on_update)
        self.get_logger().info("🚀 All-Wheel ICR Visualizer Active")

    def drive_cb(self, msg): self.cmd_vel = msg.data
    def steer_cb(self, msg): self.cmd_steer = msg.data

    def calculate_ackermann(self, radius, velocity):
        steer_dict = {}
        vel_dict = {}
        for name, (x, y) in self.wheel_coords.items():
            if radius > 0:
                angle = math.degrees(math.atan2(x, abs(radius) - y))
            else: 
                angle = math.degrees(math.atan2(x, abs(radius) + y))
                
            steer_dict[name] = angle if radius > 0 else -angle
            vel_dict[name] = velocity
        return steer_dict, vel_dict
  

    def on_update(self, e):
        appw = omni.appwindow.get_default_app_window()
        kb = appw.get_keyboard()
        ii = carb.input.acquire_input_interface()
        
        target_steers = {name: self.cmd_steer for name in self.wheel_coords}
        target_vels = {name: self.cmd_vel for name in self.wheel_coords}
        
        radius, vel = 0, 0
        if ii.get_keyboard_value(kb, carb.input.KeyboardInput.W): vel = MAX_VEL_DEG
        elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.S): vel = -MAX_VEL_DEG
        
        if ii.get_keyboard_value(kb, carb.input.KeyboardInput.Q): radius, vel = self.min_radius, MAX_VEL_DEG
        elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.E): radius, vel = -self.min_radius, MAX_VEL_DEG
        elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.Z): radius, vel = self.min_radius, -MAX_VEL_DEG
        elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.C): radius, vel = -self.min_radius, -MAX_VEL_DEG
        elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.A):
            for n in target_steers: target_steers[n] = self.max_steer_deg
        elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.D):
            for n in target_steers: target_steers[n] = -self.max_steer_deg        
        if radius != 0:
            target_steers, target_vels = self.calculate_ackermann(radius, vel)
        elif vel != 0:
            for n in target_vels: target_vels[n] = vel
            
        stage = stage_utils.get_current_stage()
        self.draw.clear_lines()

        for name, (x_loc, y_loc) in self.wheel_coords.items():
            num = name[1] # '1', '2', '3', '4'
            side = name[3] # 'l', 'r'
            d_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/joints/w{num}_{side}_drive_joint"
            s_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/joints/w{num}_{side}_steer_joint"
            l_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/{name}_steer_link"
#            d_path = f"/World/agv_precision_alignment/joints/w{num}_{side}_drive_joint"
#            s_path = f"/World/agv_precision_alignment/joints/w{num}_{side}_steer_joint"
#            l_path = f"/World/agv_precision_alignment/{name}_steer_link"
            
            d_prim = stage.GetPrimAtPath(d_path)
            s_prim = stage.GetPrimAtPath(s_path)
            l_prim = stage.GetPrimAtPath(l_path)
            
            # 1. 물리 제어 적용
            if d_prim.IsValid():
                d_prim.GetAttribute("drive:angular:physics:stiffness").Set(0)
                d_prim.GetAttribute("drive:angular:physics:damping").Set(1e6)            
                d_prim.GetAttribute("drive:angular:physics:targetVelocity").Set(target_vels[name])
            if s_prim.IsValid():
                s_prim.GetAttribute("drive:angular:physics:targetPosition").Set(target_steers[name])
#                s_prim.GetAttribute("drive:angular:physics:stiffness").Set(1e9) 
                s_prim.GetAttribute("drive:angular:physics:stiffness").Set(1e9)
                s_prim.GetAttribute("drive:angular:physics:damping").Set(1e7)                
                # s_prim.GetAttribute("drive:angular:physics:targetPosition").Set(target_steers[name])

            # 2. 실시간 법선 시각화 (모든 바퀴)
            if l_prim.IsValid():
                world_matrix = omni.usd.get_world_transform_matrix(l_prim)
                wheel_pos = world_matrix.ExtractTranslation()
                wheel_rot = world_matrix.ExtractRotation()
                
                # URDF 기준 바퀴 전방 벡터 (Local Y)
                local_forward = Gf.Vec3d(0, 1, 0)
                world_forward = wheel_rot.TransformDir(local_forward)
                world_forward.Normalize()

                # 조향 방향에 수직인 법선 벡터 계산 (ICR 방향)
                world_normal = Gf.Vec3d(-world_forward[1], world_forward[0], 0.0)
                world_normal.Normalize()

                # 양방향 선 그리기 좌표 계산
                start = wheel_pos + Gf.Vec3d(0, 0, 0.2)
                end_pos = start + world_normal * 20.0
                end_neg = start - world_normal * 20.0
                
                # 시각화: 노란색(안쪽)과 빨간색(바깥쪽)으로 구분하여 투사
                self.draw.draw_lines([tuple(start)], [tuple(end_pos)], [(1, 1, 0, 1)], [2.0])
                self.draw.draw_lines([tuple(start)], [tuple(end_neg)], [(1, 0, 0, 1)], [2.0])

def run_node():
    node = AGVAllWheelVisualizer()
    rclpy.spin(node)

threading.Thread(target=run_node, daemon=True).start()
