import glob
import os
import sys
import random
import time
import pygame
import numpy as np
import math
from collections import deque

# 將 CARLA PythonAPI 加入 sys.path
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# ====================================================
# 攝影機管理類別：生成攝影機、轉換影像為 pygame surface
# ====================================================
class CameraManager:
    def __init__(self, parent_actor, cam_res=(640, 480), transform=carla.Transform(carla.Location(x=1.6, z=1.2))):
        self.surface = None
        self.camera = None
        self.sensor_specs = transform
        self.parent_actor = parent_actor
        self.cam_res = cam_res

        self.sensor_type = 'sensor.camera.rgb'
        self.world = self.parent_actor.get_world()
        bp_library = self.world.get_blueprint_library()
        self.camera_bp = bp_library.find(self.sensor_type)
        self.camera_bp.set_attribute('image_size_x', str(self.cam_res[0]))
        self.camera_bp.set_attribute('image_size_y', str(self.cam_res[1]))

    # 建立攝影機 Sensor 並 attach 到 actor
    def set_sensor(self, attach_to=None):
        if attach_to is None:
            attach_to = self.parent_actor
        self.camera = self.world.spawn_actor(
            self.camera_bp,
            self.sensor_specs,
            attach_to=attach_to,
            attachment_type=carla.AttachmentType.Rigid
        )
        self.camera.listen(lambda image: self.process_image(image))

    # 將影像轉換成 pygame surface
    def process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]   # 取 RGB
        array = array[:, :, ::-1] # BGR -> RGB
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    # 銷毀攝影機
    def destroy(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()

# ====================================================
# HUD 顯示類別：顯示前後車速度、位置、感測器資訊
# ====================================================
class HUD:
    def __init__(self, width, height, cam_res, front_vehicle_id, rear_vehicle_id):
        self.dim = (width, height)
        self.cam_res = cam_res
        self.front_vehicle_id = front_vehicle_id
        self.rear_vehicle_id = rear_vehicle_id
        self.font = pygame.font.Font(pygame.font.get_default_font(), 14)
        self.is_visible = True  # P 鍵切換 HUD 顯示
        self.info = {
            'front_vehicle': {
                'location': None, 'speed': 0.0, 'throttle': 0.0, 'steer': 0.0, 'brake': 0.0,
                'gnss': None, 'accel': None, 'gyro': None
            },
            'rear_vehicle': {
                'location': None, 'speed': 0.0, 'throttle': 0.0, 'steer': 0.0, 'brake': 0.0,
                'gnss': None, 'accel': None, 'gyro': None
            }
        }
    
    def toggle_visibility(self):
        self.is_visible = not self.is_visible

    # GNSS callback
    def on_gnss_data(self, data, vehicle_key):
        self.info[vehicle_key]['gnss'] = (data.latitude, data.longitude)

    # IMU callback
    def on_imu_data(self, data, vehicle_key):
        self.info[vehicle_key]['accel'] = (data.accelerometer.x, data.accelerometer.y, data.accelerometer.z)
        self.info[vehicle_key]['gyro'] = (data.gyroscope.x, data.gyroscope.y, data.gyroscope.z)

    # 更新車輛基本數據 (速度、位置、控制)
    def update_vehicle_data(self, front_vehicle, rear_vehicle):
        # 前車
        front_speed_kmh = np.sqrt(front_vehicle.get_velocity().x**2 + front_vehicle.get_velocity().y**2) * 3.6
        self.info['front_vehicle']['speed'] = front_speed_kmh
        self.info['front_vehicle']['location'] = front_vehicle.get_location()
        front_control = front_vehicle.get_control()
        self.info['front_vehicle']['throttle'] = front_control.throttle
        self.info['front_vehicle']['steer'] = front_control.steer
        self.info['front_vehicle']['brake'] = front_control.brake

        # 後車
        rear_speed_kmh = np.sqrt(rear_vehicle.get_velocity().x**2 + rear_vehicle.get_velocity().y**2) * 3.6
        self.info['rear_vehicle']['speed'] = rear_speed_kmh
        self.info['rear_vehicle']['location'] = rear_vehicle.get_location()
        rear_control = rear_vehicle.get_control()
        self.info['rear_vehicle']['throttle'] = rear_control.throttle
        self.info['rear_vehicle']['steer'] = rear_control.steer
        self.info['rear_vehicle']['brake'] = rear_control.brake

    # 繪製 HUD
    def render(self, display):
        if not self.is_visible:
            return

        # 前車資訊
        front_info_surface = pygame.Surface(self.cam_res)
        front_info_surface.set_colorkey(pygame.Color(0, 0, 0))
        front_info_text = [
            f"Front Car (ID: {self.front_vehicle_id}):",
            f"  Speed: {self.info['front_vehicle']['speed']:.2f} km/h",
            f"  Loc: ({self.info['front_vehicle']['location'].x:.2f}, {self.info['front_vehicle']['location'].y:.2f})",
            f"  GNSS: Lat:{self.info['front_vehicle']['gnss'][0]:.6f} Long:{self.info['front_vehicle']['gnss'][1]:.6f}" if self.info['front_vehicle']['gnss'] else "  GNSS: N/A",
            f"  Accel: ({self.info['front_vehicle']['accel'][0]:.2f}, {self.info['front_vehicle']['accel'][1]:.2f})" if self.info['front_vehicle']['accel'] else "  Accel: N/A",
            f"  Gyro: ({self.info['front_vehicle']['gyro'][0]:.2f}, {self.info['front_vehicle']['gyro'][1]:.2f})" if self.info['front_vehicle']['gyro'] else "  Gyro: N/A",
            f"  Control: T:{self.info['front_vehicle']['throttle']:.2f} S:{self.info['front_vehicle']['steer']:.2f} B:{self.info['front_vehicle']['brake']:.2f}",
        ]
        y_offset = 10
        for text in front_info_text:
            text_surface = self.font.render(text, True, (255, 255, 255))
            front_info_surface.blit(text_surface, (10, y_offset))
            y_offset += 20
        display.blit(front_info_surface, (0, 0))

        # 後車資訊
        rear_info_surface = pygame.Surface(self.cam_res)
        rear_info_surface.set_colorkey(pygame.Color(0, 0, 0))
        rear_info_text = [
            f"Rear Car (ID: {self.rear_vehicle_id}):",
            f"  Speed: {self.info['rear_vehicle']['speed']:.2f} km/h",
            f"  Loc: ({self.info['rear_vehicle']['location'].x:.2f}, {self.info['rear_vehicle']['location'].y:.2f})",
            f"  GNSS: Lat:{self.info['rear_vehicle']['gnss'][0]:.6f} Long:{self.info['rear_vehicle']['gnss'][1]:.6f}" if self.info['rear_vehicle']['gnss'] else "  GNSS: N/A",
            f"  Accel: ({self.info['rear_vehicle']['accel'][0]:.2f}, {self.info['rear_vehicle']['accel'][1]:.2f})" if self.info['rear_vehicle']['accel'] else "  Accel: N/A",
            f"  Gyro: ({self.info['rear_vehicle']['gyro'][0]:.2f}, {self.info['rear_vehicle']['gyro'][1]:.2f})" if self.info['rear_vehicle']['gyro'] else "  Gyro: N/A",
            f"  Control: T:{self.info['rear_vehicle']['throttle']:.2f} S:{self.info['rear_vehicle']['steer']:.2f} B:{self.info['rear_vehicle']['brake']:.2f}",
        ]
        y_offset = 10
        for text in rear_info_text:
            text_surface = self.font.render(text, True, (255, 255, 255))
            rear_info_surface.blit(text_surface, (10, y_offset))
            y_offset += 20
        display.blit(rear_info_surface, (self.cam_res[0], 0))


# ====================================================
# 主模擬函式
# ====================================================
def run_simulation():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    pygame.init()

    # 跟車控制器參數
    kp_dist, ki_dist, kd_dist = 0.4, 0.03, 0.1  # PID for distance
    integral_error_dist = 0.0
    last_error_dist = 0.0
    kp_speed = 0.8  # P controller for speed

    # 初始化變數
    front_vehicle = rear_vehicle = None
    front_cam = rear_cam = top_view_cam = None
    gnss_front_sensor = imu_front_sensor = None
    gnss_rear_sensor = imu_rear_sensor = None
    front_vehicle_trajectory = deque(maxlen=200)  # 儲存前車軌跡

    try:
        # pygame 視窗設定
        cam_res = (640, 480)
        display_width = cam_res[0] * 3
        display_height = cam_res[1]
        display = pygame.display.set_mode((display_width, display_height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("CARLA Dual-Vehicle Simulation")

        # 車輛藍圖
        blueprint_library = world.get_blueprint_library()
        front_vehicle_bp = random.choice(blueprint_library.filter('audi') or blueprint_library.filter('vehicle'))
        rear_vehicle_bp = random.choice(blueprint_library.filter('model3') or blueprint_library.filter('vehicle'))

        # 前車 spawn
        spawn_points = world.get_map().get_spawn_points()
        front_vehicle_spawn_point = random.choice(spawn_points)
        front_vehicle = world.spawn_actor(front_vehicle_bp, front_vehicle_spawn_point)
        print('Front vehicle spawned:', front_vehicle_bp.id)

        # 根據前車長度決定初始距離
        front_vehicle_length = front_vehicle.bounding_box.extent.x * 2 
        initial_distance = front_vehicle_length * 2  
        min_distance_stopped = front_vehicle_length * 0.5
        time_headway = 1.5   

        # 後車 spawn
        spawn_location = front_vehicle_spawn_point.location - front_vehicle_spawn_point.get_forward_vector() * initial_distance
        spawn_transform = carla.Transform(spawn_location, front_vehicle_spawn_point.rotation)
        rear_vehicle = world.spawn_actor(rear_vehicle_bp, spawn_transform)
        print('Rear vehicle spawned:', rear_vehicle_bp.id)

        rear_vehicle_length = rear_vehicle.bounding_box.extent.x * 2

        # 啟動前車自動駕駛
        front_vehicle.set_autopilot(True)

        # HUD 初始化
        hud = HUD(display_width, display_height, cam_res, front_vehicle.id, rear_vehicle.id)

        # === 建立 Sensors ===
        gnss_front_sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.gnss'), carla.Transform(), attach_to=front_vehicle)
        gnss_front_sensor.listen(lambda data: hud.on_gnss_data(data, 'front_vehicle'))
        imu_front_sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.imu'), carla.Transform(), attach_to=front_vehicle)
        imu_front_sensor.listen(lambda data: hud.on_imu_data(data, 'front_vehicle'))

        gnss_rear_sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.gnss'), carla.Transform(), attach_to=rear_vehicle)
        gnss_rear_sensor.listen(lambda data: hud.on_gnss_data(data, 'rear_vehicle'))
        imu_rear_sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.imu'), carla.Transform(), attach_to=rear_vehicle)
        imu_rear_sensor.listen(lambda data: hud.on_imu_data(data, 'rear_vehicle'))

        # === 建立 Cameras ===
        front_cam = CameraManager(front_vehicle, cam_res)
        rear_cam = CameraManager(rear_vehicle, cam_res)
        top_view_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=50.0), carla.Rotation(pitch=-90.0))
        top_view_cam = CameraManager(rear_vehicle, cam_res, transform=top_view_transform)
        front_cam.set_sensor()
        rear_cam.set_sensor()
        top_view_cam.set_sensor(attach_to=None)

        # === 主迴圈 ===
        clock = pygame.time.Clock()
        running = True
        while running:
            clock.tick_busy_loop(60)  # 固定 60 FPS
            
            # 鍵盤事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT: running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE: running = False
                    elif event.key == pygame.K_p: hud.toggle_visibility()

            # 更新前車軌跡
            front_vehicle_trajectory.append(front_vehicle.get_transform().location)
            front_transform = front_vehicle.get_transform()
            rear_transform = rear_vehicle.get_transform()
            front_speed = np.linalg.norm([front_vehicle.get_velocity().x, front_vehicle.get_velocity().y])
            rear_speed = np.linalg.norm([rear_vehicle.get_velocity().x, rear_vehicle.get_velocity().y])

            # ========== 後車跟車控制 ==========
            rear_throttle = rear_brake = 0.0
            distance = front_transform.location.distance(rear_transform.location)

            if front_speed < 0.1 and distance < min_distance_stopped:
                # 前車停下來 -> 後車全煞車
                rear_brake = 1.0
            else:
                # 計算期望距離 (動態 headway)
                target_distance = max(rear_vehicle_length * 2.0, rear_speed * time_headway)
                distance_error = distance - target_distance
                speed_error = front_speed - rear_speed

                # PID for distance
                integral_error_dist += distance_error
                integral_error_dist = np.clip(integral_error_dist, -100, 100)
                derivative_error_dist = distance_error - last_error_dist
                last_error_dist = distance_error

                dist_control_signal = kp_dist * distance_error + ki_dist * integral_error_dist + kd_dist * derivative_error_dist
                speed_control_signal = kp_speed * speed_error
                combined_control = dist_control_signal + speed_control_signal

                # 控制輸出
                if combined_control > 0:
                    rear_throttle = np.clip(0.5 + combined_control, 0.0, 1.0)
                else:
                    rear_brake = np.clip(-combined_control, 0.0, 1.0)

            # ========== 後車轉向控制 (Pure Pursuit) ==========
            rear_steer = 0.0
            if rear_speed > 0.5 and len(front_vehicle_trajectory) > 5:
                lookahead_point = None
                lookahead_distance = max(rear_vehicle_length * 1.5, rear_speed * 1.0)
                for point in reversed(front_vehicle_trajectory):
                    if rear_transform.location.distance(point) > lookahead_distance:
                        lookahead_point = point
                        break
                if lookahead_point:
                    rear_to_lookahead_vec = lookahead_point - rear_transform.location
                    rear_heading = rear_transform.get_forward_vector()
                    angle_rad = math.atan2(rear_to_lookahead_vec.y, rear_to_lookahead_vec.x) - math.atan2(rear_heading.y, rear_heading.x)
                    if angle_rad > math.pi: angle_rad -= 2 * math.pi
                    elif angle_rad < -math.pi: angle_rad += 2 * math.pi
                    L = 2.9
                    rear_steer = np.clip(math.atan2(2 * L * math.sin(angle_rad), lookahead_distance), -1.0, 1.0)

            # 套用控制
            rear_vehicle.apply_control(carla.VehicleControl(throttle=rear_throttle, brake=rear_brake, steer=rear_steer))

            # 更新 HUD
            hud.update_vehicle_data(front_vehicle, rear_vehicle)

            # 顯示三個攝影機畫面
            if front_cam.surface and rear_cam.surface and top_view_cam.surface:
                display.blit(front_cam.surface, (0, 0))
                display.blit(rear_cam.surface, (cam_res[0], 0))
                display.blit(top_view_cam.surface, (cam_res[0] * 2, 0))
            
            hud.render(display)
            pygame.display.flip()

    finally:
        # 程式結束時清理 actors
        print('Destroying actors...')
        if front_cam: front_cam.destroy()
        if rear_cam: rear_cam.destroy()
        if top_view_cam: top_view_cam.destroy()
        if gnss_front_sensor: gnss_front_sensor.destroy()
        if imu_front_sensor: imu_front_sensor.destroy()
        if gnss_rear_sensor: gnss_rear_sensor.destroy()
        if imu_rear_sensor: imu_rear_sensor.destroy()
        if front_vehicle and front_vehicle.is_alive: front_vehicle.destroy()
       

