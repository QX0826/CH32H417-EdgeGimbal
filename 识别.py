import mediapipe as mp
import cv2
import numpy as np
import serial
import time
import threading
import os

# ========== InsightFace 人脸检测 ==========
from insightface.app import FaceAnalysis

# 初始化 InsightFace
print("[INIT] Loading InsightFace model...", flush=True)
face_app = FaceAnalysis(name='buffalo_l', allowed_modules=['detection'], providers=['CPUExecutionProvider'])
face_app.prepare(ctx_id=0, det_size=(320, 320))  # 缩小检测尺寸，速度提升4倍
print("[INIT] InsightFace model loaded successfully!", flush=True)

# ========== 目标追踪状态 ==========

class KalmanFilter1D:
    """一阶卡尔曼滤波器，用于坐标平滑"""
    def __init__(self, process_noise=1e-3, measurement_noise=1e-1):
        self.x = None  # 状态估计值
        self.P = 1.0   # 估计误差
        self.Q = process_noise      # 过程噪声（越小越平滑）
        self.R = measurement_noise  # 测量噪声（越大越平滑）

    def update(self, measurement):
        if self.x is None:
            self.x = measurement
            return self.x
        # 预测步
        self.P += self.Q
        # 更新步
        K = self.P / (self.P + self.R)  # 卡尔曼增益
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return self.x

    def predict(self):
        return self.x if self.x is not None else 0


class PID:
    """
    PID 控制器：根据误差大小动态调整舵机运动量
    - 大偏差时快速追赶，接近中心时自动减速，不过冲
    - 带积分限幅，防止积分饱和
    """
    def __init__(self, Kp, Ki, Kd, integral_limit=50.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    def update(self, error, dt=0.033):
        # 积分项（带限幅）
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        # 微分项
        derivative = (error - self.prev_error) / max(dt, 1e-6)
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        """重置积分和微分，切换追踪目标时调用"""
        self.integral = 0.0
        self.prev_error = 0.0


def iou(boxA, boxB):
    """计算两个框的IoU(交并比)，用于帧间目标匹配"""
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    inter = max(0, xB - xA) * max(0, yB - yA)
    areaA = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    areaB = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    union = areaA + areaB - inter
    return inter / union if union > 0 else 0

tracked_bbox = None  # 当前追踪目标 [x1,y1,x2,y2]
LOST_THRESHOLD = 30  # 丢失多少帧后重新选目标
lost_count = 0
detect_interval = 3  # 初始检测间隔
detect_counter = 0

# ========== 运动预测状态 ==========
prev_center_x = None  # 上一帧目标中心x
prev_center_y = None  # 上一帧目标中心y
vx = 0.0  # 目标x方向速度（像素/帧）
vy = 0.0  # 目标y方向速度（像素/帧）
SPEED_ALPHA = 0.6 # 速度平滑系数（0-1，越大越信任最新速度）

def get_angle(v1, v2):
    """
    计算两个向量之间的夹角（角度制）
    """
    dot_product = np.dot(v1, v2)
    norm_v1 = np.sqrt(np.sum(v1 * v1))
    norm_v2 = np.sqrt(np.sum(v2 * v2))
    cos_angle = dot_product / (norm_v1 * norm_v2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle) / 3.14 * 180
    return angle


def get_str_guester(up_fingers, list_lms):
    """
    基于手指关键点识别手势类型
    """
    if len(up_fingers) == 0:
        return "fist"

    # 计算手掌尺度（腕部到中指根部的距离）用于归一化，使得远近距离判断均准确
    hand_scale = np.linalg.norm(list_lms[0] - list_lms[9])
    if hand_scale < 1.0:
        hand_scale = 1.0

    if len(up_fingers) == 1 and up_fingers[0] == 8:
        v1 = list_lms[6] - list_lms[7]
        v2 = list_lms[8] - list_lms[7]
        angle = get_angle(v1, v2)
        if angle < 160:
            return " "
        else:
            return "1"

    elif len(up_fingers) == 2 and up_fingers[0] == 8 and up_fingers[1] == 12:
        return "2"

    elif len(up_fingers) == 3 and up_fingers[0] == 4 and up_fingers[1] == 8 and up_fingers[2] == 12:
        tips = [4, 8, 12]
        tip_distances = []
        for i in range(len(tips)):
            for j in range(i + 1, len(tips)):
                dist = np.linalg.norm(list_lms[tips[i]] - list_lms[tips[j]])
                tip_distances.append(dist)
        avg_tip_dist = np.mean(tip_distances) if tip_distances else 0
        # 归一化判定阈值，使用 0.75 * hand_scale 代替固定 60 像素
        if avg_tip_dist < 0.75 * hand_scale:
            return "7"
        else:
            return "3"

    elif len(up_fingers) == 4 and up_fingers[0] == 8 and up_fingers[1] == 12 and up_fingers[2] == 16 and up_fingers[3] == 20:
        return "4"

    elif len(up_fingers) == 5:
        return "5"

    elif len(up_fingers) == 2 and up_fingers[0] == 4 and up_fingers[1] == 20:
        return "6"

    else:
        return " "


def init_serial(port, baudrate, max_retries=3):
    """初始化串口连接，支持重试机制"""
    for i in range(max_retries):
        try:
            ser = serial.Serial(port, baudrate, timeout=0.5)
            if ser.isOpen():
                print(f"Successfully opened serial port {port}")
                return ser
        except Exception as e:
            print(f"Attempt {i + 1} failed: {e}")
            time.sleep(1)
    print(f"Failed to open serial port {port} after {max_retries} attempts")
    return None


def read_from_serial(ser, buffer):
    """串口数据读取线程函数"""
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    buffer.append(data)
        except Exception as e:
            print(f"Serial read error: {e}")
            time.sleep(0.1)


if __name__ == "__main__":
    """
    主程序：多模态边缘智能稳像云台系统控制核心
    - com9: 手势控制串口（9600bps）
    - com10: 人脸跟踪串口（115200bps）
    - com27:  MCU通信串口（115200bps）
    """

    # ========== 硬件初始化 ==========
    serial_hand = init_serial('com9', 9600)
    serial_face = init_serial('com10', 115200)
    serial_mcu = init_serial('com27', 115200)

    if not all([serial_hand, serial_face, serial_mcu]):
        print("[WARN] Some serial ports failed. Running in camera-only mode (no MCU control).")
        # 不退出，继续运行以测试人脸识别

    # ========== 多线程串口读取 ==========
    mcu_buffer = []
    if serial_mcu is not None:
        mcu_thread = threading.Thread(target=read_from_serial, args=(serial_mcu, mcu_buffer))
        mcu_thread.daemon = True
        mcu_thread.start()

    # ========== 视觉系统初始化 ==========
    # Windows 下使用 cv2.CAP_DSHOW 后台能极大提高摄像头稳定性和断线恢复速度
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("[ERROR] Camera failed to open!", flush=True)
    else:
        print("[INIT] Camera ready.", flush=True)

    # MediaPipe 手势识别
    mpHands = mp.solutions.hands
    hands = mpHands.Hands(
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    mpDraw = mp.solutions.drawing_utils

    # ========== 系统状态变量 ==========
    coord_transmission_mode = False
    hand_stable_count = 0
    STABLE_THRESHOLD = 5
    last_hand_flag = " "
    current_hand_flag = " "
    hand_detect_counter = 0
    hand_flag = " "

    # ========== 死区参数 ==========
    DEAD_ZONE_X = 30   # 水平方向：画面中心 ±30 像素内不响应（消除抖动）
    DEAD_ZONE_Y = 25   # 垂直方向：画面中心 ±25 像素内不响应

    # X轴实测：40~110，中心75；  Y轴实测：140~230，中心175
    TRACK_X_CENTER = 75.0;   TRACK_X_HALF = 35.0   # 上下各 35 度，增大X轴活动灵敏度与幅度
    TRACK_Y_CENTER = 175.0;  TRACK_Y_HALF = 45.0   # 左右各 45 度
    TRACK_X_MIN = 40.0;      TRACK_X_MAX = 110.0   # 相应调大X轴运动范围，使其变动范围更广
    TRACK_Y_MIN = 140.0;     TRACK_Y_MAX = 230.0

    # ===== 两级平滑参数 =====
    # 第一级：人脸检测坐标输入平滑（消除 InsightFace bbox 逐帧噪声）
    # 越小越平滑，越大越跟手。0.35 = 轻度滤波，保留快速移动响应
    INPUT_ALPHA  = 0.55
    # 第二级：输出角度平滑（丝滑过渡感，数值越大对应轴反应越快）
    SMOOTH_ALPHA_YAW   = 0.45  # Y轴（水平偏航 Yaw）的平滑系数
    SMOOTH_ALPHA_PITCH = 0.65  # X轴（垂直俯仰 Pitch）的平滑系数

    # 当前指令角度（初始 = 开机位置）
    cmd_yaw   = 175.0   # Y轴
    cmd_pitch =  75.0   # X轴

    # 输入平滑状态（初始 = 画面中心）
    face_cx_s = 320.0   # 平滑后的人脸中心 X（像素）
    face_cy_s = 240.0   # 平滑后的人脸中心 Y（像素）

    # 卡尔曼滤波器（对 PID 输出做最终平滑）
    kalman_x = KalmanFilter1D(process_noise=1e-3, measurement_noise=2.0)
    kalman_y = KalmanFilter1D(process_noise=1e-3, measurement_noise=2.0)

    # 数据发送控制
    last_sent_time = time.time()
    last_hand_sent = ""
    last_face_sent = None
    last_time = time.time()
    COORD_CHANGE_THRESHOLD = 0.1  # 支持 0.1° 级别的角度更新，以实现平滑追踪

    # 手势发送控制
    hand_send_count = 0
    MAX_HAND_SEND_COUNT = 5  # 增加手势发送重试次数，确保单片机收到模式转换命令
    current_sending_gesture = " "
    is_sending_gesture = False
    last_gesture_change_time = 0
    GESTURE_CHANGE_COOLDOWN = 0.5

    # MCU通信控制
    mcu_5_received = False
    mcu_5_send_count = 0
    MCU_5_SEND_MAX = 4

    # ========== 主循环 ==========
    print("[INFO] System started. Press 'q' to quit.", flush=True)

    fail_count = 0
    while True:
        try:
            success, img = cap.read()
            if not success:
                fail_count += 1
                if fail_count > 50:
                    print("[WARN] Camera read failed 50 times, reopening...")
                    cap.release()
                    time.sleep(1.5)  # 稍微延长等待，给硬件驱动释放时间
                    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    fail_count = 0
                time.sleep(0.02)
                continue
            fail_count = 0

            # ========== MCU数据处理 ==========
            if mcu_buffer:
                data = mcu_buffer.pop(0)
                if data == '5':
                    print("Received '5' from MCU - Voice activation detected")
                    mcu_5_received = True
                    mcu_5_send_count = 0
                    coord_transmission_mode = False
                elif data == '0':
                    print("Received '0' from MCU - Voice stop/reset detected")
                    mcu_5_received = False
                    coord_transmission_mode = False
                    # 复位：直接发一次初始角度给云台
                    cmd_yaw   = 175.0
                    cmd_pitch =  75.0
                    try:
                        if serial_face is not None:
                            serial_face.write(f"75.0,175.0\n".encode())
                        print("[VOICE STOP] 已发送复位指令: (75.0, 175.0)")
                    except Exception as e:
                        print(f"[VOICE STOP] 复位发送失败: {e}")

            # ========== 图像预处理 ==========
            img = cv2.flip(img, 1)
            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # ========== InsightFace 人脸检测 + 追踪 ==========
            current_face_coord = None

            # 仅在人脸跟踪模式激活（coord_transmission_mode 或 mcu_5_received 为 True）时才运行人脸检测与计算，以节省 CPU/GPU 资源
            if coord_transmission_mode or mcu_5_received:
                detect_counter += 1
                # 动态检测间隔：速度快时更频繁检测
                if detect_counter >= detect_interval or tracked_bbox is None:
                    detect_counter = 0
                    faces = face_app.get(img)

                    if len(faces) > 0:
                        face_bboxes = [f.bbox.astype(int) for f in faces]
                        face_areas = [(b[2]-b[0]) * (b[3]-b[1]) for b in face_bboxes]

                        best_match_idx = -1
                        best_iou = 0
                        if tracked_bbox is not None:
                            for i, bbox in enumerate(face_bboxes):
                                iou_val = iou(tracked_bbox, bbox)
                                if iou_val > best_iou:
                                    best_iou = iou_val
                                    best_match_idx = i

                        if best_match_idx >= 0 and best_iou > 0.3:
                            tracked_bbox = face_bboxes[best_match_idx]
                            lost_count = 0
                        else:
                            best_match_idx = max(range(len(face_areas)), key=lambda i: face_areas[i])
                            tracked_bbox = face_bboxes[best_match_idx]
                            lost_count = 0

                        # 更新运动速度
                        x1, y1, x2, y2 = tracked_bbox
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        if prev_center_x is not None:
                            new_vx = cx - prev_center_x
                            new_vy = cy - prev_center_y
                            vx = SPEED_ALPHA * new_vx + (1 - SPEED_ALPHA) * vx
                            vy = SPEED_ALPHA * new_vy + (1 - SPEED_ALPHA) * vy
                        prev_center_x = cx
                        prev_center_y = cy

                        # 动态调整检测间隔
                        speed = np.sqrt(vx**2 + vy**2)
                        if speed > 40:
                            detect_interval = 1  # 快速移动：每帧检测
                        elif speed > 15:
                            detect_interval = 2  # 中速：每2帧
                        else:
                            detect_interval = 3  # 慢速：每3帧

                    else:
                        lost_count += 1
                        if lost_count > LOST_THRESHOLD:
                            tracked_bbox = None
                            prev_center_x = None
                            prev_center_y = None
                            vx = 0.0
                            vy = 0.0
                            kalman_x = KalmanFilter1D(process_noise=1e-3, measurement_noise=2.0)
                            kalman_y = KalmanFilter1D(process_noise=1e-3, measurement_noise=2.0)

                # 帧间运动预测：用速度预测bbox位置（检测间隔帧和短暂丢失帧都走这里）
                elif tracked_bbox is not None and lost_count < LOST_THRESHOLD and (vx != 0 or vy != 0):
                    x1, y1, x2, y2 = tracked_bbox
                    tracked_bbox = [int(x1 + vx), int(y1 + vy), int(x2 + vx), int(y2 + vy)]

                # 用追踪目标更新坐标
                if tracked_bbox is not None:
                    x1, y1, x2, y2 = tracked_bbox
                    w, h = x2 - x1, y2 - y1
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"Tracking" if lost_count == 0 else f"Predicting"
                    cv2.putText(img, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    center_x = x1 + w // 2
                    center_y = y1 + h // 2

                    # === 第一级：输入平滑（消除检测噪声）===
                    # 即使人脸静止，bbox 每帧会抖动 ±2~3 像素；EMA 将其压制到 <0.5 像素
                    face_cx_s += INPUT_ALPHA * (center_x - face_cx_s)
                    face_cy_s += INPUT_ALPHA * (center_y - face_cy_s)

                    err_x = face_cx_s - 320.0   # 平滑后的水平误差
                    err_y = face_cy_s - 240.0   # 平滑后的垂直误差

                    in_dz_x = abs(err_x) < DEAD_ZONE_X
                    in_dz_y = abs(err_y) < DEAD_ZONE_Y

                    # 死区可视化（绿=静止，黄=追踪中）
                    dz_color = (0, 255, 0) if (in_dz_x and in_dz_y) else (0, 255, 255)
                    cv2.rectangle(img,
                                  (320 - DEAD_ZONE_X, 240 - DEAD_ZONE_Y),
                                  (320 + DEAD_ZONE_X, 240 + DEAD_ZONE_Y),
                                  dz_color, 1)

                    # === 第二级：输出平滑（丝滑过渡）===
                    # 死区外：将像素误差映射到目标角度，EMA 平滑趋近
                    # 死区内：冻结当前角度（不向中心漂移）
                    if not in_dz_x:
                        tgt_yaw = TRACK_Y_CENTER + (err_x / 320.0 * TRACK_Y_HALF)
                        cmd_yaw += SMOOTH_ALPHA_YAW * (tgt_yaw - cmd_yaw)
                        cmd_yaw  = max(TRACK_Y_MIN, min(TRACK_Y_MAX, cmd_yaw))

                    if not in_dz_y:
                        tgt_pitch = TRACK_X_CENTER - (err_y / 240.0 * TRACK_X_HALF)
                        cmd_pitch += SMOOTH_ALPHA_PITCH * (tgt_pitch - cmd_pitch)
                        cmd_pitch  = max(TRACK_X_MIN, min(TRACK_X_MAX, cmd_pitch))

                    current_face_coord = (round(cmd_pitch, 1), round(cmd_yaw, 1))
                else:
                    cv2.putText(img, "No face detected", (10, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                # 非人脸跟踪模式，清除追踪状态并提示
                tracked_bbox = None
                prev_center_x = None
                prev_center_y = None
                vx = 0.0
                vy = 0.0
                cv2.putText(img, "Face Tracking: OFF", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 如果是在人脸跟踪模式下，降低手势检测的频率（例如每 6 帧检测一次）以大幅提升跟踪时的流畅度
            run_hand_detection = True
            if coord_transmission_mode:
                hand_detect_counter += 1
                if hand_detect_counter % 6 != 0:
                    run_hand_detection = False
            else:
                hand_detect_counter = 0

            # ========== 手势识别 ==========
            # 仅在实际运行手势检测的帧重置手势，跳过检测的帧保留上一帧手势以维持发送状态
            if run_hand_detection:
                hand_flag = " "

            if run_hand_detection:
                results = hands.process(imgRGB)

                if results.multi_hand_landmarks:
                    detected_gestures = []
                    for hand_landmarks in results.multi_hand_landmarks:
                        list_lms = []
                        for i in range(21):
                            pos_x = hand_landmarks.landmark[i].x * img.shape[1]
                            pos_y = hand_landmarks.landmark[i].y * img.shape[0]
                            list_lms.append([int(pos_x), int(pos_y)])
                        list_lms = np.array(list_lms)

                        # 使用自适应凸包法判断手指是否伸直。凸包法对2和3等手势有极高的准确度。
                        # 通过引入 hand_scale 自适应容差，完美解决远距离抖动噪声问题。
                        hull_index = [0, 1, 2, 3, 6, 10, 14, 19, 18, 17, 10]
                        hull = cv2.convexHull(list_lms[hull_index, :])

                        # 计算手掌尺度（以腕部 0 到中指根部 9 的距离作为基准）
                        hand_scale = np.linalg.norm(list_lms[0] - list_lms[9])
                        # 自适应边缘容差，防止远距离图像噪声导致手指抖动误判
                        margin = max(1.5, hand_scale * 0.08)

                        up_fingers = []
                        finger_tips = [4, 8, 12, 16, 20]
                        for i in finger_tips:
                            pt = (int(list_lms[i][0]), int(list_lms[i][1]))
                            # 必须明显在凸包外部（超出容差距离）才判定为伸出，提高抗噪性能
                            if cv2.pointPolygonTest(hull, pt, True) < -margin:
                                up_fingers.append(i)

                        gesture = get_str_guester(up_fingers, list_lms)
                        detected_gestures.append((gesture, hand_landmarks))

                    # 判定逻辑：
                    # 1. 只有检测到两只手且两只手都为 "5" 时，才判定为手势 "5"（以启动跟踪）
                    # 2. 单只手显示 "5" 被过滤忽略，避免误触发。
                    # 3. 其他手势（如拳头 fist 或 manual control 数字）正常单手判定即可。
                    gestures = [item[0] for item in detected_gestures]
                    if len(gestures) == 2 and gestures[0] == "5" and gestures[1] == "5":
                        current_hand_flag = "5"
                    else:
                        # 过滤掉单手 "5" 以及空手势
                        valid_gestures = [g for g in gestures if g != "5" and g != " "]
                        if "fist" in valid_gestures:
                            current_hand_flag = "fist"
                        elif len(valid_gestures) > 0:
                            current_hand_flag = valid_gestures[0]
                        else:
                            current_hand_flag = " "

                    # 如果在人脸跟踪模式下，只允许识别为 "fist"（握拳），其他手势直接忽略（不画骨架点、不显示、不触发）
                    if coord_transmission_mode and current_hand_flag != "fist":
                        pass
                    else:
                        for _, hand_landmarks in detected_gestures:
                            mpDraw.draw_landmarks(img, hand_landmarks, mpHands.HAND_CONNECTIONS)

                        # 手势稳定性检测
                        if current_hand_flag == last_hand_flag:
                            hand_stable_count += 1
                        else:
                            hand_stable_count = 0
                            last_hand_flag = current_hand_flag

                        # 人脸跟踪时由于跳帧检测，要求连续检测到指定次数以防止误判，非跟踪状态要求连续确定5次
                        stable_threshold = 3 if coord_transmission_mode else STABLE_THRESHOLD
                        if hand_stable_count >= stable_threshold:
                            hand_flag = current_hand_flag
                            cv2.putText(img, hand_flag, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # ========== 多模态控制逻辑 ==========
            current_time = time.time()

            if hand_flag == "5":
                if not mcu_5_received:
                    coord_transmission_mode = True
                    if current_sending_gesture != "5":
                        hand_send_count = 0
                        current_sending_gesture = "5"
                        is_sending_gesture = True
                        last_gesture_change_time = current_time
                        # 开始追踪：重置指令角度到初始位置
                        cmd_yaw   = 175.0
                        cmd_pitch =  75.0

            elif hand_flag == "fist":
                coord_transmission_mode = False
                mcu_5_received = False
                if current_sending_gesture != "fist":
                    hand_send_count = 0
                    current_sending_gesture = "fist"
                    is_sending_gesture = True
                    last_gesture_change_time = current_time
                    # ---- 复位：直接发一次初始角度给云台 ----
                    cmd_yaw   = 175.0
                    cmd_pitch =  75.0
                    try:
                        if serial_face is not None:
                            serial_face.write(f"75.0,175.0\n".encode())
                        print("[FIST] 已发送复位指令: (75.0, 175.0)")
                    except Exception as e:
                        print(f"[FIST] 复位发送失败: {e}")


            elif hand_flag in ["1", "2", "3", "4", "6", "7"]:
                coord_transmission_mode = False
                mcu_5_received = False
                if current_sending_gesture != hand_flag and (
                        current_time - last_gesture_change_time) > GESTURE_CHANGE_COOLDOWN:
                    hand_send_count = 0
                    current_sending_gesture = hand_flag
                    is_sending_gesture = True
                    last_gesture_change_time = current_time

            else:
                current_sending_gesture = " "
                is_sending_gesture = False

            # ========== 串口数据发送 ==========
            # 30Hz 发送：每帧 PID 增量小，MCU 收到连续细腻的步进而非离散大跳变
            if current_time - last_sent_time > 0.033:

                if mcu_5_received and mcu_5_send_count < MCU_5_SEND_MAX:
                    try:
                        if serial_hand is not None:
                            serial_hand.write("5\n".encode())
                        mcu_5_send_count += 1
                        print(f"Sent 5 via com9: {mcu_5_send_count}/{MCU_5_SEND_MAX}")
                        if mcu_5_send_count >= MCU_5_SEND_MAX:
                            coord_transmission_mode = True
                    except serial.SerialException as e:
                        print(f"com9 write error: {e}")

                if is_sending_gesture and hand_send_count < MAX_HAND_SEND_COUNT and current_sending_gesture in ["1",
                                                                                                                "2",
                                                                                                                "3",
                                                                                                                "4",
                                                                                                                "5",
                                                                                                                "6",
                                                                                                                "7",
                                                                                                                "fist"]:
                    gesture_str = "0\n" if current_sending_gesture == "fist" else f"{current_sending_gesture}\n"
                    gesture_bytes = gesture_str.encode()
                    try:
                        if serial_hand is not None:
                            serial_hand.write(gesture_bytes)
                        if serial_face is not None:
                            serial_face.write(gesture_bytes)
                        if serial_mcu is not None:
                            serial_mcu.write(gesture_bytes)
                        hand_send_count += 1
                        print(f"Sent hand: {current_sending_gesture} ({hand_send_count}/{MAX_HAND_SEND_COUNT}) to all ports")
                        if hand_send_count >= MAX_HAND_SEND_COUNT:
                            is_sending_gesture = False
                    except serial.SerialException as e:
                        print(f"com9 write error: {e}")

                if coord_transmission_mode and current_face_coord:
                    if last_face_sent is None or \
                            abs(current_face_coord[0] - last_face_sent[0]) > COORD_CHANGE_THRESHOLD or \
                            abs(current_face_coord[1] - last_face_sent[1]) > COORD_CHANGE_THRESHOLD:
                        try:
                            if serial_face is not None:
                                serial_face.write(f"{current_face_coord[0]},{current_face_coord[1]}\n".encode())
                            print(f"Sent face via com10: {current_face_coord}")
                            last_face_sent = current_face_coord
                        except serial.SerialException as e:
                            print(f"com10 write error: {e}")

                last_sent_time = current_time

            # ========== 界面显示 ==========
            mode_text = f"com9(Hand): {current_sending_gesture} ({hand_send_count}/{MAX_HAND_SEND_COUNT}) | com10(Face): {'ON' if coord_transmission_mode else 'OFF'} | com27(MCU): Active"
            if mcu_5_received:
                mode_text += f" | MCU-5: {mcu_5_send_count}/{MCU_5_SEND_MAX}"
            cv2.putText(img, mode_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 显示FPS
            fps = 1 / (current_time - last_time) if (current_time - last_time) > 0 else 0
            cv2.putText(img, f"FPS: {fps:.1f}", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            last_time = current_time

            cv2.imshow("Gesture and Face Detection", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"Error in main loop: {e}")
            cap.release()
            time.sleep(1.5)
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)

            if serial_hand is not None and not serial_hand.isOpen():
                serial_hand = init_serial('com9', 9600)
            if serial_face is not None and not serial_face.isOpen():
                serial_face = init_serial('com10', 115200)
            if serial_mcu is not None and not serial_mcu.isOpen():
                serial_mcu = init_serial('com27', 115200)

    # ========== 资源清理 ==========
    cap.release()
    cv2.destroyAllWindows()
    for ser in [serial_hand, serial_face, serial_mcu]:
        if ser is not None and ser.isOpen():
            ser.close()
