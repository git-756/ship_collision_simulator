import pybullet as p
import pybullet_data
import time
import numpy as np

class ShipSimulation:
    def __init__(self):
        # 物理クライアントを初期化
        self.client = p.connect(p.DIRECT)
        
        # 物理データのパスを追加
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)

        # パラメータ設定
        self.gravity = -9.81
        self.time_step = 1.0 / 240.0
        self.boat_mass = 80  # kg
        self.boat_initial_velocity = [3, 0, 0]  # m/s
        self.water_density = 997 # kg/m^3 (水の密度)
        
        # 物理ワールドの設定
        p.setGravity(0, 0, self.gravity, physicsClientId=self.client)
        p.setTimeStep(self.time_step, physicsClientId=self.client)
        # 接触ソルバーの反復回数を増やすことで、Stiffness/Dampingの計算精度を上げる
        p.setPhysicsEngineParameter(numSolverIterations=150, physicsClientId=self.client)

        # オブジェクトIDと状態変数を初期化
        self.plane_id = None
        self.boat_id = None
        self.wall_id = None
        self.collided = False
        
        # 衝突エネルギー計算用
        self.last_velocity = np.array([0.0, 0.0, 0.0])
        self.lost_kinetic_energy = None
        
        # 推進力と抵抗に関するパラメータ
        self.thrust_force = 0.0
        self.frontal_area = 0.0
        self.drag_coefficient = 0.8 # 抵抗係数

        # ワールドの作成
        self.create_world()

    def create_world(self):
        # 地面（海面）を作成
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)

        # 船の形状とプロパティ
        boat_half_extents = [1.0, 0.4, 0.2] 
        boat_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=boat_half_extents, physicsClientId=self.client)
        boat_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=boat_half_extents, rgbaColor=[0.8, 0.2, 0.2, 1.0], physicsClientId=self.client)
        
        boat_start_pos = [-5, 0, boat_half_extents[2]]
        self.boat_id = p.createMultiBody(baseMass=self.boat_mass,
                                         baseCollisionShapeIndex=boat_shape_id,
                                         baseVisualShapeIndex=boat_visual_id,
                                         basePosition=boat_start_pos,
                                         physicsClientId=self.client)
        
        p.resetBaseVelocity(self.boat_id, linearVelocity=self.boat_initial_velocity, physicsClientId=self.client)
        self.last_velocity = np.array(self.boat_initial_velocity)

        # 壁の形状とプロパティ
        wall_half_extents = [0.1, 2, 1]
        wall_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half_extents, physicsClientId=self.client)
        wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_half_extents, rgbaColor=[0.5, 0.5, 0.5, 1.0], physicsClientId=self.client)
        
        wall_start_pos = [0, 0, wall_half_extents[2]]
        self.wall_id = p.createMultiBody(baseMass=0,
                                         baseCollisionShapeIndex=wall_shape_id,
                                         baseVisualShapeIndex=wall_visual_id,
                                         basePosition=wall_start_pos,
                                         physicsClientId=self.client)

        # 衝突プロパティの設定（レクタングラーダンパーをモデル化）
        p.changeDynamics(self.boat_id, -1, 
                         lateralFriction=0.3, 
                         restitution=0.01, # 反発はほぼ無くし、Stiffness/Dampingで制御
                         contactStiffness=20000, # [調整パラメータ] スプリングの硬さ
                         contactDamping=1000)    # [調整パラメータ] ダンパーの粘り気
                         
        p.changeDynamics(self.wall_id, -1, 
                         lateralFriction=0.5, 
                         restitution=0.8,
                         contactStiffness=100000, # 壁は非常に硬く設定
                         contactDamping=1000)
        
        # 浮力が重力と釣り合うように設定
        self.buoyancy_force = self.boat_mass * abs(self.gravity)
        
        # 抵抗計算に使うパラメータを設定
        width = boat_half_extents[1] * 2
        submerged_height = boat_half_extents[2] 
        self.frontal_area = width * submerged_height

        # 3m/sで進む際の水の抵抗と釣り合う推力を計算
        target_velocity = 3.0
        drag_force_at_target_speed = 0.5 * self.water_density * (target_velocity**2) * self.drag_coefficient * self.frontal_area
        
        self.thrust_force = drag_force_at_target_speed
        print(f"INFO: 3m/sを維持するための推力を {self.thrust_force:.2f} N に設定しました。")

    def step(self):
        vel, _ = p.getBaseVelocity(self.boat_id, physicsClientId=self.client)
        self.last_velocity = np.array(vel)
        self.apply_ocean_effects()
        p.stepSimulation(physicsClientId=self.client)
        self.check_collision()

    def apply_ocean_effects(self):
        pos, _ = p.getBasePositionAndOrientation(self.boat_id, physicsClientId=self.client)
        vel, _ = p.getBaseVelocity(self.boat_id, physicsClientId=self.client)
        p.applyExternalForce(self.boat_id, -1, [0, 0, self.buoyancy_force], pos, p.WORLD_FRAME, physicsClientId=self.client)
        p.applyExternalForce(self.boat_id, -1, [self.thrust_force, 0, 0], pos, p.WORLD_FRAME, physicsClientId=self.client)
        drag_force_magnitude = 0.5 * self.water_density * (vel[0]**2) * self.drag_coefficient * self.frontal_area
        drag_force_direction = -1.0 if vel[0] > 0 else 1.0
        drag_force = [drag_force_magnitude * drag_force_direction, 0, 0]
        p.applyExternalForce(self.boat_id, -1, drag_force, pos, p.WORLD_FRAME, physicsClientId=self.client)

    def check_collision(self):
        if not self.collided:
            contact_points = p.getContactPoints(bodyA=self.boat_id, bodyB=self.wall_id, physicsClientId=self.client)
            if contact_points:
                total_impulse = 0
                for point in contact_points:
                    total_impulse += point[9]
                
                if total_impulse > 0:
                    print(f"衝突を検出！ 合計インパルス: {total_impulse:.4f} Ns")
                    vel_after, _ = p.getBaseVelocity(self.boat_id, physicsClientId=self.client)
                    vel_after = np.array(vel_after)
                    speed_before_sq = np.dot(self.last_velocity, self.last_velocity)
                    speed_after_sq = np.dot(vel_after, vel_after)
                    energy_before = 0.5 * self.boat_mass * speed_before_sq
                    energy_after = 0.5 * self.boat_mass * speed_after_sq
                    self.lost_kinetic_energy = energy_before - energy_after
                    print(f"  - 衝突直前の運動エネルギー: {energy_before:.2f} J")
                    print(f"  - 衝突直後の運動エネルギー: {energy_after:.2f} J")
                    print(f"  - 衝突で失われたエネルギー: {self.lost_kinetic_energy:.2f} J (ジュール)")
                    self.collided = True

    def get_camera_image(self, width=640, height=480):
        # --- ▼▼▼ ここが修正箇所です ▼▼▼ ---
        boat_pos, _ = p.getBasePositionAndOrientation(self.boat_id, physicsClientId=self.client)
        # --- ▲▲▲ ここが修正箇所です ▲▲▲ ---
        view_matrix = p.computeViewMatrix(cameraEyePosition=[boat_pos[0] - 5, 5, 3], cameraTargetPosition=boat_pos, cameraUpVector=[0, 0, 1], physicsClientId=self.client)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=width / height, nearVal=0.1, farVal=100, physicsClientId=self.client)
        _, _, rgba_img, _, _ = p.getCameraImage(width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL, physicsClientId=self.client)
        return rgba_img

    def close(self):
        p.disconnect(physicsClientId=self.client)