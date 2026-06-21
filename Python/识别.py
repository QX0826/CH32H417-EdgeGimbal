import cv2
import mediapipe as mp
import numpy as np
import math
import serial
import threading
import time
import sys
import os
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, QGroupBox
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject

# ========== 信号类 ==========
class SerialSignal(QObject):
    received = pyqtSignal(str)

# ========== MediaPipe 手部检测 ==========
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=2)
mpDraw = mp.solutions.drawing_utils

def get_angle(v1, v2):
    angle = np.dot(v1, v2) / (np.sqrt(np.sum(v1 * v1)) * np.sqrt(np.sum(v2 * v2)))
    angle = np.degrees(np.arccos(np.clip(angle, -1.0, 1.0)))
    return angle

def get_str_guester(up_fingers, list_lms):
    if len(up_fingers) == 0:
        return "fist"
    elif len(up_fingers) == 1 and up_fingers[0] == 8:
        return "1"
    elif len(up_fingers) == 2 and up_fingers[0] == 8 and up_fingers[1] == 12:
        return "2"
    elif len(up_fingers) == 3 and up_fingers[0] == 4 and up_fingers[1] == 8 and up_fingers[2] == 12:
        tips = [4, 8, 12]
        tip_distances = []
        for i in range(len(tips)):
            for j in range(i + 1, len(tips)):
                dist = np.sqrt((list_lms[tips[i]][0] - list_lms[tips[j]][0]) ** 2 +
                               (list_lms[tips[i]][1] - list_lms[tips[j]][1]) ** 2)
                tip_distances.append(dist)
        avg_tip_dist = np.mean(tip_distances)
        return "7" if avg_tip_dist < 60 else "3"
    elif len(up_fingers) == 4:
        return "4"
    elif len(up_fingers) == 5:
        return "5"
    elif len(up_fingers) == 2 and up_fingers[0] == 4 and up_fingers[1] == 20:
        return "6"
    else:
        return " "

def send_gesture_to_mcu(gesture):
    """发送手势指令到MCU via COM22"""
    try:
        ser = serial.Serial('COM22', 9600, timeout=0.1)
        if gesture == "fist":
            ser.write("0\n".encode())
        else:
            ser.write(f"{gesture}\n".encode())
        ser.close()
    except Exception as e:
        print(f"[WARN] COM22 send error: {e}")

def send_face_coord_to_mcu(x, y):
    """发送人脸坐标到MCU via COM20"""
    try:
        ser = serial.Serial('COM20', 115200, timeout=0.1)
        ser.write(f"{x},{y}\n".encode())
        ser.close()
    except Exception as e:
        print(f"[WARN] COM20 send error: {e}")

# ========== GUI 类 ==========
class HandGestureGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("手势识别 - PyQt5")
        self.setGeometry(100, 100, 900, 700)

        layout = QHBoxLayout()
        self.image_label = QLabel()
        self.image_label.setFixedSize(640, 480)
        layout.addWidget(self.image_label)

        right_panel = QVBoxLayout()

        self.gesture_label = QLabel("当前手势: 等待中")
        self.gesture_label.setFont(QFont("Arial", 16))
        right_panel.addWidget(self.gesture_label)

        self.fps_label = QLabel("FPS: 0")
        self.fps_label.setFont(QFont("Arial", 12))
        right_panel.addWidget(self.fps_label)

        self.status_label = QLabel("状态: 就绪")
        self.status_label.setFont(QFont("Arial", 12))
        right_panel.addWidget(self.status_label)

        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        right_panel.addWidget(log_group)

        right_panel.addStretch()
        layout.addLayout(right_panel)
        self.setLayout(layout)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.pTime = 0
        self.last_gesture = " "
        self.gesture_cooldown = 0
        self.face_tracking = False
        self.face_coords = []

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def log(self, message):
        self.log_text.append(message)

    def update_frame(self):
        success, img = self.cap.read()
        if not success:
            return

        img = cv2.flip(img, 1)
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(imgRGB)

        current_gesture = " "
        if results.multi_hand_landmarks:
            for hand in results.multi_hand_landmarks:
                mpDraw.draw_landmarks(img, hand, mpHands.HAND_CONNECTIONS)
                list_lms = []
                for i in range(21):
                    pos_x = hand.landmark[i].x * img.shape[1]
                    pos_y = hand.landmark[i].y * img.shape[0]
                    list_lms.append([int(pos_x), int(pos_y)])
                list_lms = np.array(list_lms)

                hull_index = [0, 1, 2, 3, 6, 10, 14, 19, 18, 17, 10]
                hull = cv2.convexHull(list_lms[hull_index, :])

                up_fingers = []
                finger_tips = [4, 8, 12, 16, 20]
                for i in finger_tips:
                    pt = (int(list_lms[i][0]), int(list_lms[i][1]))
                    if cv2.pointPolygonTest(hull, pt, True) < 0:
                        up_fingers.append(i)

                current_gesture = get_str_guester(up_fingers, list_lms)

        if self.gesture_cooldown > 0:
            self.gesture_cooldown -= 1
        elif current_gesture != self.last_gesture and current_gesture != " ":
            self.last_gesture = current_gesture
            self.gesture_cooldown = 10
            self.gesture_label.setText(f"当前手势: {current_gesture}")
            self.log(f"[手势] 识别到: {current_gesture}")
            send_gesture_to_mcu(current_gesture)

        cTime = time.time()
        fps = 1 / (cTime - self.pTime) if self.pTime > 0 else 0
        self.pTime = cTime
        self.fps_label.setText(f"FPS: {int(fps)}")

        img_display = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = img_display.shape
        bytes_per_line = ch * w
        qt_img = QImage(img_display.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_img))

    def closeEvent(self, event):
        self.cap.release()
        event.accept()

# ========== 主函数 ==========
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HandGestureGUI()
    window.show()
    sys.exit(app.exec_())
