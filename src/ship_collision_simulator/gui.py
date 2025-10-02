import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton
from PySide6.QtCore import QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel
import numpy as np

from .simulation import ShipSimulation

class SimulationWidget(QLabel):
    def __init__(self, simulation: ShipSimulation, parent=None):
        super().__init__(parent)
        self.simulation = simulation
        self.setMinimumSize(640, 480)

    def update_simulation_image(self):
        width, height = self.width(), self.height()
        if width > 0 and height > 0:
            # PyBulletから画像データを取得
            rgba_img = self.simulation.get_camera_image(width, height)
            
            # NumPy配列に変換
            np_img = np.array(rgba_img, dtype=np.uint8).reshape((height, width, 4))
            
            # QImageを作成
            q_image = QImage(np_img.data, width, height, width * 4, QImage.Format.Format_RGBA8888)
            
            # QPixmapに変換してラベルにセット
            pixmap = QPixmap.fromImage(q_image)
            self.setPixmap(pixmap)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("船 衝突シミュレーター")
        self.setGeometry(100, 100, 700, 550)

        # シミュレーションインスタンスを作成
        self.simulation = ShipSimulation()

        # UIのセットアップ
        self.setup_ui()

        # タイマーの設定
        self.timer = QTimer(self)
        self.timer.setInterval(int(self.simulation.time_step * 1000)) # 物理ステップと同じ間隔
        self.timer.timeout.connect(self.update_simulation)
        
    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)

        # シミュレーション表示ウィジェット
        self.simulation_widget = SimulationWidget(self.simulation)
        layout.addWidget(self.simulation_widget)

        # 開始/停止ボタン
        self.start_button = QPushButton("シミュレーション開始")
        self.start_button.clicked.connect(self.toggle_simulation)
        layout.addWidget(self.start_button)
        
        self.is_running = False

    def toggle_simulation(self):
        if not self.is_running:
            self.timer.start()
            self.start_button.setText("シミュレーション停止")
            self.is_running = True
        else:
            self.timer.stop()
            self.start_button.setText("シミュレーション再開")
            self.is_running = False

    def update_simulation(self):
        self.simulation.step()
        self.simulation_widget.update_simulation_image()

    def closeEvent(self, event):
        self.simulation.close()
        event.accept()