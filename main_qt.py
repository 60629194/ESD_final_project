import sys
import time
import re
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGridLayout, QFrame, QSizePolicy
)
from PyQt6.QtGui import QPainter, QColor, QFont, QPen, QBrush
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QRectF, QTimer

# ---------------------------------------------------------
# Serial Reader Thread
# ---------------------------------------------------------
class SerialThread(QThread):
    data_ready = pyqtSignal(dict)
    connection_status = pyqtSignal(str, bool)

    def __init__(self):
        super().__init__()
        self.running = False
        self.ser = None
        self.port_name = None

    def find_arduino_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.connection_status.emit(f"Scanning {port.device}...", False)
            try:
                # Open port
                s = serial.Serial(port.device, 115200, timeout=1.5)
                time.sleep(2)  # Wait for Arduino to reset/initialize
                
                # Send Handshake Ping
                s.write(b'P')
                s.flush()
                
                # Wait for Handshake Pong
                start_time = time.time()
                while time.time() - start_time < 2:
                    if s.in_waiting:
                        line = s.readline().decode('utf-8', errors='ignore').strip()
                        if "ARDUINO_DUE_HANDSHAKE" in line:
                            self.connection_status.emit(f"Connected to {port.device}", True)
                            self.port_name = port.device
                            return s
                s.close()
            except Exception as e:
                pass
        return None

    def run(self):
        self.running = True
        self.connection_status.emit("Searching for Arduino...", False)
        
        self.ser = self.find_arduino_port()
        if not self.ser:
            self.connection_status.emit("Arduino Due not found. Please connect and restart.", False)
            return

        # Parsing State
        in_grid = False
        grid_lines = []
        parsed_data = {
            "grid": [],
            "char": "-",
            "score": "0.0",
            "lat_c3": "-",
            "lat_node": "-",
            "lat_due": "-",
            "lat_total": "-"
        }

        while self.running and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    # 1. Grid Parsing
                    if "╔" in line and "28x28" in line:
                        pass # Header
                    elif "╠" in line:
                        in_grid = True
                        grid_lines = []
                    elif "╚" in line and in_grid:
                        in_grid = False
                        # Process grid
                        if len(grid_lines) == 28:
                            bool_grid = []
                            for r in grid_lines:
                                # Remove borders '║'
                                row_str = r.strip('║')
                                # Each block is 2 chars. '██' or '  '
                                row_bool = []
                                for i in range(0, min(len(row_str), 56), 2):
                                    chunk = row_str[i:i+2]
                                    row_bool.append(chunk == '██')
                                # Pad to 28 if needed
                                while len(row_bool) < 28:
                                    row_bool.append(False)
                                bool_grid.append(row_bool)
                            parsed_data["grid"] = bool_grid
                            self.data_ready.emit(parsed_data)
                    elif in_grid and line.startswith("║"):
                        grid_lines.append(line)

                    # 2. Extract Character and Confidence
                    elif "FINAL Predicted Character:" in line:
                        parts = line.split(":")
                        if len(parts) > 1:
                            parsed_data["char"] = parts[1].strip()
                    elif "FINAL Confidence Score:" in line:
                        parts = line.split(":")
                        if len(parts) > 1:
                            parsed_data["score"] = parts[1].strip()
                    
                    # 3. Extract Latencies
                    elif "1. ESP32-C3" in line:
                        m = re.search(r':\s*(\d+)\s*us', line)
                        if m: parsed_data["lat_c3"] = m.group(1)
                    elif "3. NodeMCU" in line:
                        m = re.search(r':\s*(\d+)\s*us', line)
                        if m: parsed_data["lat_node"] = m.group(1)
                    elif "4. Due" in line:
                        m = re.search(r':\s*(\d+)\s*us', line)
                        if m: parsed_data["lat_due"] = m.group(1)
                    elif "總耗時" in line:
                        m = re.search(r':\s*(\d+)\s*us', line)
                        if m: 
                            parsed_data["lat_total"] = m.group(1)
                            # Total latency usually signifies the end of a transmission block
                            self.data_ready.emit(parsed_data)

            except Exception as e:
                print(f"Serial read error: {e}")
                self.connection_status.emit("Connection lost.", False)
                break

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.wait()

# ---------------------------------------------------------
# UI Components
# ---------------------------------------------------------

class GridWidget(QWidget):
    """Custom widget to draw the 28x28 grid."""
    def __init__(self):
        super().__init__()
        self.grid_data = [[False]*28 for _ in range(28)]
        self.setMinimumSize(400, 400)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def update_grid(self, new_data):
        if new_data and len(new_data) == 28 and len(new_data[0]) == 28:
            self.grid_data = new_data
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Draw background
        painter.fillRect(self.rect(), QColor("#1E1E2E"))

        # Calculate cell size
        margin = 10
        w = self.width() - 2 * margin
        h = self.height() - 2 * margin
        cell_size = min(w, h) / 28.0
        
        start_x = (self.width() - (cell_size * 28)) / 2
        start_y = (self.height() - (cell_size * 28)) / 2

        # Draw cells
        ink_color = QColor("#89B4FA") # Soft blue/purple for modern look
        empty_color = QColor("#313244") # Dark grey for empty

        painter.setPen(Qt.PenStyle.NoPen)
        for y in range(28):
            for x in range(28):
                rect = QRectF(start_x + x * cell_size, start_y + y * cell_size, cell_size - 1, cell_size - 1)
                if self.grid_data[y][x]:
                    painter.setBrush(QBrush(ink_color))
                else:
                    painter.setBrush(QBrush(empty_color))
                painter.drawRoundedRect(rect, 2, 2)


class InfoCard(QFrame):
    """A styled frame for displaying metrics."""
    def __init__(self, title, initial_value=""):
        super().__init__()
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet("""
            InfoCard {
                background-color: #181825;
                border-radius: 12px;
                border: 1px solid #313244;
            }
        """)
        
        layout = QVBoxLayout(self)
        
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("color: #A6ADC8; font-size: 14px; font-weight: bold;")
        
        self.value_label = QLabel(initial_value)
        self.value_label.setStyleSheet("color: #CBA6F7; font-size: 24px; font-weight: bold;")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

    def set_value(self, val):
        self.value_label.setText(str(val))

    def set_value_color(self, color_hex):
        self.value_label.setStyleSheet(f"color: {color_hex}; font-size: 24px; font-weight: bold;")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stroke Recognition UI")
        self.resize(1000, 600)
        self.setStyleSheet("background-color: #11111B;") # Deep dark background

        # Main Widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # Left side - Grid
        left_layout = QVBoxLayout()
        self.grid_widget = GridWidget()
        
        grid_title = QLabel("28x28 Input Grid")
        grid_title.setStyleSheet("color: #CDD6F4; font-size: 18px; font-weight: bold;")
        grid_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        left_layout.addWidget(grid_title)
        left_layout.addWidget(self.grid_widget)
        
        # Right side - Metrics
        right_layout = QVBoxLayout()
        right_layout.setSpacing(15)

        # Connection Status
        self.status_label = QLabel("Initializing...")
        self.status_label.setStyleSheet("color: #F38BA8; font-size: 14px; font-weight: bold;")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_layout.addWidget(self.status_label)

        # Primary Metrics (Char & Score)
        primary_layout = QHBoxLayout()
        self.char_card = InfoCard("Predicted Character", "-")
        self.char_card.value_label.setStyleSheet("color: #A6E3A1; font-size: 64px; font-weight: bold;")
        
        self.score_card = InfoCard("Confidence Score", "0.0")
        
        primary_layout.addWidget(self.char_card)
        primary_layout.addWidget(self.score_card)
        right_layout.addLayout(primary_layout)

        # Latency Metrics
        lat_title = QLabel("Latency Breakdown (us)")
        lat_title.setStyleSheet("color: #CDD6F4; font-size: 16px; font-weight: bold; margin-top: 20px;")
        right_layout.addWidget(lat_title)

        grid_lat = QGridLayout()
        self.c3_card = InfoCard("ESP32-C3", "-")
        self.node_card = InfoCard("NodeMCU", "-")
        self.due_card = InfoCard("Arduino Due", "-")
        self.total_card = InfoCard("Total Latency", "-")
        self.total_card.set_value_color("#F9E2AF") # Yellowish for total
        
        grid_lat.addWidget(self.c3_card, 0, 0)
        grid_lat.addWidget(self.node_card, 0, 1)
        grid_lat.addWidget(self.due_card, 1, 0)
        grid_lat.addWidget(self.total_card, 1, 1)
        right_layout.addLayout(grid_lat)
        
        right_layout.addStretch() # Push everything up

        # Add left and right layouts to main
        main_layout.addLayout(left_layout, stretch=2) # Grid gets more space
        main_layout.addLayout(right_layout, stretch=1)

        # Start Thread
        self.serial_thread = SerialThread()
        self.serial_thread.connection_status.connect(self.update_status)
        self.serial_thread.data_ready.connect(self.update_ui)
        self.serial_thread.start()

    def update_status(self, msg, is_connected):
        self.status_label.setText(msg)
        if is_connected:
            self.status_label.setStyleSheet("color: #A6E3A1; font-size: 14px; font-weight: bold;")
        else:
            self.status_label.setStyleSheet("color: #F38BA8; font-size: 14px; font-weight: bold;")

    def update_ui(self, data):
        if "grid" in data and data["grid"]:
            self.grid_widget.update_grid(data["grid"])
        
        self.char_card.set_value(data.get("char", "-"))
        self.score_card.set_value(data.get("score", "0.0"))
        self.c3_card.set_value(data.get("lat_c3", "-"))
        self.node_card.set_value(data.get("lat_node", "-"))
        self.due_card.set_value(data.get("lat_due", "-"))
        self.total_card.set_value(data.get("lat_total", "-"))

    def closeEvent(self, event):
        self.serial_thread.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
