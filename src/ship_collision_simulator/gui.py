import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QHBoxLayout
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap, QFont
from PySide6.QtWidgets import QLabel
import numpy as np

from .simulation import ShipSimulation

class SimulationWidget(QLabel):
    # (このクラスの中身は変更ありません)
    def __init__(self, simulation: ShipSimulation, parent=None):
        super().__init__(parent)
        self.simulation = simulation
        self.setMinimumSize(640, 480)

    def update_simulation_image(self):
        width, height = self.width(), self.height()
        if width > 0 and height > 0:
            rgba_img = self.simulation.get_camera_image(width, height)
            np_img = np.array(rgba_img, dtype=np.uint8).reshape((height, width, 4))
            q_image = QImage(np_img.data, width, height, width * 4, QImage.Format.Format_RGBA8888)
            pixmap = QPixmap.fromImage(q_image)
            self.setPixmap(pixmap)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("船 衝突シミュレーター")
        self.setGeometry(100, 100, 700, 600) # 高さを少し広げる

        self.simulation = ShipSimulation()
        self.setup_ui()

        self.timer = QTimer(self)
        self.timer.setInterval(int(self.simulation.time_step * 1000))
        self.timer.timeout.connect(self.update_simulation)
        
        # --- ▼▼▼ ここから追加 ▼▼▼ ---
        self.energy_displayed = False
        # --- ▲▲▲ ここまで追加 ▲▲▲ ---

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)

        self.simulation_widget = SimulationWidget(self.simulation)
        main_layout.addWidget(self.simulation_widget)
        
        # --- ▼▼▼ ここから変更 ▼▼▼ ---
        # 結果表示と操作ボタン用のレイアウト
        bottom_layout = QHBoxLayout()

        # エネルギー表示ラベル
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.energy_label = QLabel("衝突エネルギー損失: N/A")
        self.energy_label.setFont(font)
        self.energy_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        bottom_layout.addWidget(self.energy_label)

        # 開始/停止ボタン
        self.start_button = QPushButton("シミュレーション開始")
        self.start_button.clicked.connect(self.toggle_simulation)
        bottom_layout.addWidget(self.start_button)
        
        main_layout.addLayout(bottom_layout)
        # --- ▲▲▲ ここまで変更 ▲▲▲ ---
        
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
        
        # --- ▼▼▼ ここから追加 ▼▼▼ ---
        # エネルギーが計算され、まだ表示されていない場合にラベルを更新
        if not self.energy_displayed and self.simulation.lost_kinetic_energy is not None:
            energy_val = self.simulation.lost_kinetic_energy
            self.energy_label.setText(f"衝突エネルギー損失: {energy_val:.2f} J")
            self.energy_displayed = True
        # --- ▲▲▲ ここまで追加 ▲▲▲ ---

    def closeEvent(self, event):
        self.simulation.close()
        event.accept()