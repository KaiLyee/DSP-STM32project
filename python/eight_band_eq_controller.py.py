"""
Bluetooth Speaker DSP Control Panel - 8-Band Parametric EQ
===========================================================
64-bin FFT spectrum + 8-band EQ for STM32F407 DSP system.
"""

import sys
import numpy as np
import serial
import serial.tools.list_ports
import threading
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QComboBox, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QPalette
import pyqtgraph as pg

FS = 44100.0
PI = np.pi
BAND_FREQS = [63, 125, 250, 500, 1000, 2000, 4000, 8000]
BAND_LABELS = ["63", "125", "250", "500", "1k", "2k", "4k", "8k"]
BAND_Q = 0.7
DISPLAY_BINS = 64


def compute_peaking_eq_response(fc, gain_dB, Q, freqs):
    A = 10 ** (gain_dB / 40.0)
    w0 = 2 * PI * fc / FS
    alpha = np.sin(w0) / (2 * Q)
    a0 = 1 + alpha / A
    b0 = (1 + alpha * A) / a0
    b1 = (-2 * np.cos(w0)) / a0
    b2 = (1 - alpha * A) / a0
    a1 = (-2 * np.cos(w0)) / a0
    a2 = (1 - alpha / A) / a0
    z = np.exp(1j * 2 * PI * freqs / FS)
    H = (b0 + b1 * z**-1 + b2 * z**-2) / (1 + a1 * z**-1 + a2 * z**-2)
    return H


def total_eq_response(gains_dB):
    freqs = np.logspace(np.log10(20), np.log10(20000), 500)
    H_total = np.ones(len(freqs), dtype=complex)
    for i in range(8):
        if gains_dB[i] != 0:
            H_total *= compute_peaking_eq_response(BAND_FREQS[i], gains_dB[i], BAND_Q, freqs)
    return freqs, 20 * np.log10(np.abs(H_total) + 1e-12)


class SerialBridge(QObject):
    fft_received = pyqtSignal(list, list)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.connected = False
        self.running = False
        self.reader_thread = None
        self.lock = threading.Lock()

    def connect(self, port, baudrate=38400):
        try:
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = baudrate
            self.ser.timeout = 1
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            self.connected = True
            time.sleep(4)
            self.ser.reset_input_buffer()
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False

    def start_reader(self):
        self.running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def disconnect(self):
        self.running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def _reader_loop(self):
        buf = ""
        while self.running and self.connected:
            try:
                with self.lock:
                    waiting = self.ser.in_waiting
                    data = self.ser.read(waiting).decode(errors='ignore') if waiting else ""
                if data:
                    buf += data
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip()
                        if line.startswith('FFT:'):
                            self._parse_fft(line)
                else:
                    time.sleep(0.02)
            except:
                time.sleep(0.1)

    def _parse_fft(self, line):
        try:
            parts = line[4:].split(';')
            if len(parts) == 2:
                pre = [int(x) for x in parts[0].split(',') if x]
                post = [int(x) for x in parts[1].split(',') if x]
                if len(pre) == DISPLAY_BINS and len(post) == DISPLAY_BINS:
                    self.fft_received.emit(pre, post)
        except:
            pass

    def send_eq(self, band, gain):
        if not self.connected:
            return
        try:
            with self.lock:
                self.ser.write(f"EQ:{band},{gain}\n".encode())
        except:
            pass

    def send_ping(self):
        if not self.connected:
            return False
        for attempt in range(3):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b'PING\n')
                time.sleep(1 + attempt)
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                    if 'PONG' in data:
                        return True
            except:
                pass
        return False

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]


class DSPControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bridge = SerialBridge()
        self.bridge.fft_received.connect(self.on_fft_data)
        self.band_gains = [0] * 8
        self.fft_pre = np.zeros(DISPLAY_BINS)
        self.fft_post = np.zeros(DISPLAY_BINS)
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Bluetooth Speaker DSP Control - 8-Band EQ")
        self.setMinimumSize(1100, 750)

        self.setStyleSheet("""
            QMainWindow { background-color: #0a0a1a; }
            QLabel { color: #c0c0d0; }
            QGroupBox {
                color: #00d4aa; border: 1px solid #1a1a3e; border-radius: 8px;
                margin-top: 14px; padding-top: 20px; font-size: 13px; font-weight: bold;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 8px; }
            QPushButton {
                background-color: #12122a; color: #00d4aa; border: 1px solid #00d4aa;
                border-radius: 5px; padding: 6px 16px; font-size: 12px; font-weight: bold;
            }
            QPushButton:hover { background-color: #00d4aa; color: #0a0a1a; }
            QComboBox {
                background-color: #12122a; color: #c0c0d0; border: 1px solid #1a1a3e;
                border-radius: 4px; padding: 4px 8px; font-size: 12px;
            }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(4)
        main_layout.setContentsMargins(12, 6, 12, 6)

        title = QLabel("8-Band Parametric EQ")
        title.setStyleSheet("font-size: 22px; font-weight: bold; color: #00d4aa; font-family: Consolas, monospace;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Connection
        conn_layout = QHBoxLayout()
        conn_layout.setSpacing(6)
        self.port_combo = QComboBox()
        self.port_combo.addItems(SerialBridge.list_ports())
        self.port_combo.setFixedWidth(80)
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setFixedWidth(70)
        self.refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(self.refresh_btn)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setFixedWidth(90)
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 12px;")
        conn_layout.addWidget(self.status_label)
        conn_layout.addStretch()
        main_layout.addLayout(conn_layout)

        # Combined plot
        plot_group = QGroupBox("Spectrum & EQ Response")
        plot_layout = QVBoxLayout(plot_group)

        legend_layout = QHBoxLayout()
        legend_layout.addStretch()
        for text, color in [("Original", "#00d4aa"), ("Filtered", "#e74c3c"), ("EQ Curve", "#f1c40f")]:
            lbl = QLabel(f"  {text}")
            lbl.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 11px;")
            legend_layout.addWidget(lbl)
        legend_layout.addStretch()
        plot_layout.addLayout(legend_layout)

        pg.setConfigOptions(antialias=True)
        self.main_plot = pg.PlotWidget()
        self.main_plot.setBackground('#08081a')
        self.main_plot.showGrid(x=True, y=True, alpha=0.1)
        self.main_plot.setLabel('left', 'FFT (dB)')
        self.main_plot.setLabel('bottom', 'Frequency (Hz)')
        self.main_plot.setYRange(-10, 65)
        self.main_plot.setXRange(np.log10(50), np.log10(20000))

        freq_ticks = [63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000]
        self.main_plot.getAxis('bottom').setTicks([
            [(np.log10(f), f"{f}" if f < 1000 else f"{f//1000}k") for f in freq_ticks]
        ])
        axis_pen = pg.mkPen(color='#222240')
        for ax in ['left', 'bottom']:
            self.main_plot.getAxis(ax).setPen(axis_pen)
            self.main_plot.getAxis(ax).setTextPen(pg.mkPen(color='#666680'))

        # Log-spaced bin frequencies matching STM32 bin_map (512-point FFT, 64 bins)
        bin_map = [
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
            17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
            33, 34, 35, 36, 37, 38, 39, 42, 46, 51, 56, 62, 68, 75, 83, 91,
            100, 110, 121, 133, 147, 162, 178, 196, 215, 237, 261, 287, 316, 348, 383, 422
        ]
        self.bin_freqs = np.array(bin_map) * (FS / 1024.0)
        self.bin_log_freqs = np.log10(self.bin_freqs)

        self.curve_pre = self.main_plot.plot(
            pen=pg.mkPen(color='#00d4aa', width=1.5),
            fillLevel=-10, fillBrush=pg.mkBrush(0, 212, 170, 20)
        )
        self.curve_post = self.main_plot.plot(
            pen=pg.mkPen(color='#e74c3c', width=1.5),
            fillLevel=-10, fillBrush=pg.mkBrush(231, 76, 60, 20)
        )

        self.eq_curve = self.main_plot.plot(
            pen=pg.mkPen(color='#f1c40f', width=2.5, style=Qt.DashLine)
        )

        self.main_plot.addItem(pg.InfiniteLine(pos=30, angle=0,
            pen=pg.mkPen(color='#333355', width=1, style=Qt.DotLine)))

        self.band_dots = []
        for i in range(8):
            dot = pg.ScatterPlotItem(
                pos=[[np.log10(BAND_FREQS[i]), 30]],
                size=8, brush=pg.mkBrush('#f1c40f'), pen=pg.mkPen(None)
            )
            self.main_plot.addItem(dot)
            self.band_dots.append(dot)

        plot_layout.addWidget(self.main_plot)
        main_layout.addWidget(plot_group, stretch=3)

        # 8-Band EQ Sliders
        eq_group = QGroupBox("8-Band Equalizer")
        eq_main_layout = QHBoxLayout(eq_group)
        eq_main_layout.setSpacing(4)

        self.band_sliders = []
        self.band_value_labels = []

        for i in range(8):
            band_layout = QVBoxLayout()
            band_layout.setSpacing(2)

            if i == 0:
                top_label = QLabel("+12")
                top_label.setAlignment(Qt.AlignCenter)
                top_label.setStyleSheet("color: #444460; font-size: 9px;")
                band_layout.addWidget(top_label)
            else:
                band_layout.addSpacing(14)

            val_label = QLabel("0")
            val_label.setAlignment(Qt.AlignCenter)
            val_label.setStyleSheet("color: #666680; font-size: 13px; font-weight: bold; font-family: Consolas, monospace;")
            val_label.setFixedWidth(50)
            band_layout.addWidget(val_label)
            self.band_value_labels.append(val_label)

            slider = QSlider(Qt.Vertical)
            slider.setMinimum(-12)
            slider.setMaximum(12)
            slider.setValue(0)
            slider.setFixedHeight(180)
            slider.setStyleSheet("""
                QSlider::groove:vertical {
                    width: 4px; background: #12122a; border-radius: 2px;
                }
                QSlider::handle:vertical {
                    background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                        fx:0.5, fy:0.3, stop:0 #00ffcc, stop:1 #00aa88);
                    height: 18px; width: 18px; margin: 0 -7px; border-radius: 9px;
                }
                QSlider::sub-page:vertical { background: #12122a; border-radius: 2px; }
                QSlider::add-page:vertical { background: #0d3333; border-radius: 2px; }
            """)
            slider.valueChanged.connect(lambda val, idx=i: self.on_band_changed(idx, val))
            band_layout.addWidget(slider, alignment=Qt.AlignCenter)
            self.band_sliders.append(slider)

            freq_label = QLabel(BAND_LABELS[i])
            freq_label.setAlignment(Qt.AlignCenter)
            freq_label.setStyleSheet("color: #00d4aa; font-size: 12px; font-weight: bold;")
            band_layout.addWidget(freq_label)

            if i == 0:
                bot_label = QLabel("-12")
                bot_label.setAlignment(Qt.AlignCenter)
                bot_label.setStyleSheet("color: #444460; font-size: 9px;")
                band_layout.addWidget(bot_label)
            else:
                band_layout.addSpacing(14)

            eq_main_layout.addLayout(band_layout)

        eq_main_layout.addSpacing(12)

        preset_layout = QVBoxLayout()
        preset_layout.setSpacing(4)
        for name, gains in [
            ("FLAT", [0, 0, 0, 0, 0, 0, 0, 0]),
            ("BASS+", [8, 6, 3, 0, 0, 0, 0, 0]),
            ("TREBLE+", [0, 0, 0, 0, 0, 3, 6, 8]),
            ("V-SHAPE", [6, 4, 1, -3, -3, 1, 4, 6]),
            ("VOCAL", [-2, -1, 0, 4, 6, 4, 0, -2]),
            ("ROCK", [4, 3, -2, -1, 2, 4, 6, 4]),
        ]:
            btn = QPushButton(name)
            btn.setFixedWidth(80)
            btn.clicked.connect(lambda checked, g=gains: self.apply_preset(g))
            preset_layout.addWidget(btn)
        preset_layout.addStretch()
        eq_main_layout.addLayout(preset_layout)

        main_layout.addWidget(eq_group, stretch=2)
        self.update_eq_plot()

    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItems(SerialBridge.list_ports())

    def toggle_connection(self):
        if self.bridge.connected:
            self.bridge.disconnect()
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 12px;")
        else:
            port = self.port_combo.currentText()
            if not port:
                return
            self.status_label.setText("Connecting...")
            QApplication.processEvents()
            if self.bridge.connect(port):
                if self.bridge.send_ping():
                    self.bridge.start_reader()
                    self.connect_btn.setText("Disconnect")
                    self.status_label.setText(f"Connected ({port})")
                    self.status_label.setStyleSheet("color: #00d4aa; font-weight: bold; font-size: 12px;")
                    for i in range(8):
                        self.bridge.send_eq(i, self.band_gains[i])
                        time.sleep(0.1)
                else:
                    self.bridge.disconnect()
                    self.status_label.setText("PING failed")
                    self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 12px;")
            else:
                self.status_label.setText("Connection failed")
                self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 12px;")

    def on_band_changed(self, band, value):
        self.band_gains[band] = value
        sign = "+" if value > 0 else ""
        self.band_value_labels[band].setText(f"{sign}{value}")
        if value > 0:
            self.band_value_labels[band].setStyleSheet("color: #00d4aa; font-size: 13px; font-weight: bold; font-family: Consolas, monospace;")
        elif value < 0:
            self.band_value_labels[band].setStyleSheet("color: #e74c3c; font-size: 13px; font-weight: bold; font-family: Consolas, monospace;")
        else:
            self.band_value_labels[band].setStyleSheet("color: #666680; font-size: 13px; font-weight: bold; font-family: Consolas, monospace;")

        self.update_eq_plot()

        timer_name = f'_send_timer_{band}'
        if hasattr(self, timer_name):
            getattr(self, timer_name).stop()
        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(lambda b=band, g=value: self.bridge.send_eq(b, g))
        timer.start(150)
        setattr(self, timer_name, timer)

    def apply_preset(self, gains):
        for i in range(8):
            self.band_sliders[i].setValue(gains[i])

    def update_eq_plot(self):
        freqs, mag_dB = total_eq_response(self.band_gains)
        self.eq_curve.setData(np.log10(freqs), mag_dB + 30)
        for i in range(8):
            self.band_dots[i].setData(
                pos=[[np.log10(BAND_FREQS[i]), self.band_gains[i] + 30]]
            )

    def on_fft_data(self, pre, post):
        pre_arr = np.array(pre, dtype=float)
        post_arr = np.array(post, dtype=float)
        self.fft_pre = self.fft_pre * 0.3 + pre_arr * 0.7
        self.fft_post = self.fft_post * 0.3 + post_arr * 0.7
        pre_db = 20 * np.log10(self.fft_pre + 0.1)
        post_db = 20 * np.log10(self.fft_post + 0.1)
        self.curve_pre.setData(self.bin_log_freqs, pre_db)
        self.curve_post.setData(self.bin_log_freqs, post_db)

    def closeEvent(self, event):
        self.bridge.disconnect()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor('#0a0a1a'))
    palette.setColor(QPalette.WindowText, QColor('#c0c0d0'))
    palette.setColor(QPalette.Base, QColor('#12122a'))
    palette.setColor(QPalette.Text, QColor('#c0c0d0'))
    palette.setColor(QPalette.Button, QColor('#12122a'))
    palette.setColor(QPalette.ButtonText, QColor('#c0c0d0'))
    palette.setColor(QPalette.Highlight, QColor('#00d4aa'))
    app.setPalette(palette)
    window = DSPControlPanel()
    window.show()
    sys.exit(app.exec_())