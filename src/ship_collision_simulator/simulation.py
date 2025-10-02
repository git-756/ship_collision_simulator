import pybullet as p
import pybullet_data
import time

class ShipSimulation:
    def __init__(self):
        # 物理クライアントを初期化 (GUIモード)
        # p.DIRECTだと計算のみ、p.GUIだとウィンドウ表示
        self.client = p.connect(p.DIRECT)
        
        # 物理データ（地面など）のパスを追加
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)

        # パラメータ設定
        self.gravity = -9.81
        self.time_step = 1.0 / 240.0
        self.boat_mass = 80  # kg
        self.boat_initial_velocity = [3, 0, 0]  # m/s
        self.rubber_thickness = 0.005 # 5mm
        self.water_density = 997 # kg/m^3 (水の密度)
        
        # 物理ワールドの設定
        p.setGravity(0, 0, self.gravity, physicsClientId=self.client)
        p.setTimeStep(self.time_step, physicsClientId=self.client)

        # オブジェクトIDを格納する変数を初期化
        self.plane_id = None
        self.boat_id = None
        self.wall_id = None
        self.collided = False
        
        # ワールドの作成
        self.create_world()

    def create_world(self):
        # 地面（海面）を作成
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)

        # 船の形状とプロパティ
        # 船のサイズ (長さ, 幅, 高さ)
        boat_half_extents = [1.0, 0.4, 0.2] 
        boat_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=boat_half_extents, physicsClientId=self.client)
        boat_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=boat_half_extents, rgbaColor=[0.8, 0.2, 0.2, 1.0], physicsClientId=self.client)
        
        # 船の初期位置
        boat_start_pos = [-5, 0, boat_half_extents[2]]
        self.boat_id = p.createMultiBody(baseMass=self.boat_mass,
                                         baseCollisionShapeIndex=boat_shape_id,
                                         baseVisualShapeIndex=boat_visual_id,
                                         basePosition=boat_start_pos,
                                         physicsClientId=self.client)
        
        # 船に初期速度を与える
        p.resetBaseVelocity(self.boat_id, linearVelocity=self.boat_initial_velocity, physicsClientId=self.client)

        # 壁の形状とプロパティ
        wall_half_extents = [0.1, 2, 1]
        wall_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half_extents, physicsClientId=self.client)
        wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_half_extents, rgbaColor=[0.5, 0.5, 0.5, 1.0], physicsClientId=self.client)
        
        # 壁の位置（船の進行方向）
        wall_start_pos = [0, 0, wall_half_extents[2]]
        self.wall_id = p.createMultiBody(baseMass=0,  # 質量0で固定オブジェクトになる
                                         baseCollisionShapeIndex=wall_shape_id,
                                         baseVisualShapeIndex=wall_visual_id,
                                         basePosition=wall_start_pos,
                                         physicsClientId=self.client)

        # 衝突プロパティの設定（ゴムの表現）
        # restitution: 反発係数 (0に近いほど反発しない)
        # lateralFriction: 横摩擦係数
        p.changeDynamics(self.boat_id, -1, lateralFriction=0.3, restitution=0.1, physicsClientId=self.client)
        p.changeDynamics(self.wall_id, -1, lateralFriction=0.5, restitution=0.8, physicsClientId=self.client)
        
        # 船の水中体積を計算 (簡易的に全体の半分とする)
        self.boat_volume_submerged = (boat_half_extents[0]*2) * (boat_half_extents[1]*2) * (boat_half_extents[2])
        self.buoyancy_force = self.water_density * abs(self.gravity) * self.boat_volume_submerged

    def step(self):
        # 海の影響を適用
        self.apply_ocean_effects()
        
        # 物理シミュレーションを1ステップ進める
        p.stepSimulation(physicsClientId=self.client)
        
        # 衝突の検出と衝撃の計算
        self.check_collision()

    def apply_ocean_effects(self):
        pos, _ = p.getBasePositionAndOrientation(self.boat_id, physicsClientId=self.client)
        vel, _ = p.getBaseVelocity(self.boat_id, physicsClientId=self.client)

        # 1. 浮力 (常に上向きに作用)
        p.applyExternalForce(self.boat_id, -1, [0, 0, self.buoyancy_force], pos, p.WORLD_FRAME, physicsClientId=self.client)
        
        # 2. 水の抵抗 (速度の2乗に比例するとして簡易計算)
        # C_d: 抵抗係数 (形状による), A: 前方投影面積
        C_d = 0.8
        A = (0.4 * 2) * 0.2 
        drag_force_magnitude = 0.5 * self.water_density * (vel[0]**2) * C_d * A
        drag_force = [-drag_force_magnitude if vel[0] > 0 else drag_force_magnitude, 0, 0]
        p.applyExternalForce(self.boat_id, -1, drag_force, pos, p.WORLD_FRAME, physicsClientId=self.client)


    def check_collision(self):
        if not self.collided:
            contact_points = p.getContactPoints(bodyA=self.boat_id, bodyB=self.wall_id, physicsClientId=self.client)
            if contact_points:
                total_impulse = 0
                for point in contact_points:
                    # normalForceは持続的な力, appliedImpulseは衝突の力積
                    total_impulse += point[9] # appliedImpulse
                
                if total_impulse > 0:
                    print(f"衝突を検出！ 合計インパルス: {total_impulse:.4f} Ns")
                    self.collided = True # 一度検出したら再度表示しない

    def get_camera_image(self, width=640, height=480):
        # 船の位置を取得してカメラのターゲットにする
        boat_pos, _ = p.getBasePositionAndOrientation(self.boat_id, physicsClientId=self.client)
        
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[boat_pos[0] - 5, 5, 3],
            cameraTargetPosition=boat_pos,
            cameraUpVector=[0, 0, 1],
            physicsClientId=self.client)
        
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=width / height,
            nearVal=0.1,
            farVal=100,
            physicsClientId=self.client)
        
        _, _, rgba_img, _, _ = p.getCameraImage(
            width, height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            physicsClientId=self.client)
        
        return rgba_img

    def close(self):
        p.disconnect(physicsClientId=self.client)