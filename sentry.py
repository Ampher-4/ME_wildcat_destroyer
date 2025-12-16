# cat_sentry.py
# 单文件：YOLO 识别 + 舵机控制 + 爆闪 + 扫描逻辑
import sys
import os
import time
import cv2
from ultralytics.models.yolo import YOLO
import RPi.GPIO as GPIO

# =========================
# 舵机设置
# =========================
SERVO_PIN = 18       # PWM 引脚
LED_PIN = 23         # 爆闪 LED

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # SG90: 50Hz
servo.start(0)

# SG90 工具函数：角度→占空比
def set_servo_angle(angle):
    angle = max(0, min(180, angle))
    duty = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty)


# =========================
# 爆闪函数
# =========================
def flash_led(times=3, interval=0.1):
    for _ in range(times):
        GPIO.output(LED_PIN, True)
        time.sleep(interval)
        GPIO.output(LED_PIN, False)
        time.sleep(interval)


# =========================
# 扫描模式参数
# =========================
scan_angle = 0
scan_direction = 1  # 1:右走, -1:左走
SCAN_SPEED = 1       # 每帧移动多少°
SCAN_MIN = 20        # 最左扫描角度
SCAN_MAX = 160       # 最右扫描角度

def AI_tick():
    pass

# =========================
# 主检测与控制逻辑
# =========================
def run_sentry(camera_index=0, operatingmode=0):
    """
    params:
        camera_index: 摄像头索引
        operatingmode: 0 = auto, 1 = manual
    """
    global scan_angle, scan_direction

    # 加载 YOLO
    model = YOLO("yolov8n.pt")

    # 摄像头
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 10)

    window_name = "Cat Sentry View"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("哨戒猫炮塔启动！按 q 或 Ctrl+C 退出...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("读取相机失败")
                continue

            # YOLO 推理（只检测猫）
            results = model(frame, classes=[15], conf=0.5)
            result = results[0]

            detected = False

            if len(result.boxes) > 0:
                # 只取第一个猫（你也可以换成最大面积）
                box = result.boxes[0]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                # 画检测框
                cv2.rectangle(frame, (x1, y1), (x2, y2),
                              (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"cat {conf:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                # 计算猫中心
                cat_x = (x1 + x2) // 2
                frame_center = frame.shape[1] // 2

                # 画中心线（调试非常有用）
                cv2.line(frame,
                         (frame_center, 0),
                         (frame_center, frame.shape[0]),
                         (255, 0, 0), 1)

                # 偏差（像素差）
                error_x = cat_x - frame_center
                correction = error_x * 0.05  # 需要你现场调参

                # 更新云台角度
                scan_angle -= correction
                scan_angle = max(SCAN_MIN, min(SCAN_MAX, scan_angle))
                set_servo_angle(scan_angle)

                flash_led(times=1, interval=0.05)
                detected = True

                print(f"锁定猫 angle={scan_angle:.2f}° error={error_x}")

            if not detected:
                # 扫描模式
                scan_angle += SCAN_SPEED * scan_direction
                if scan_angle >= SCAN_MAX:
                    scan_direction = -1
                if scan_angle <= SCAN_MIN:
                    scan_direction = 1

                set_servo_angle(scan_angle)

            # 显示画面
            cv2.imshow(window_name, frame)

            # 按 q 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n手动中断")

    finally:
        servo.stop()
        GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()



if __name__ == "__main__":
    cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    #init the system
    cfgfile = 'sentry.cfg'
    operatingmode = 0
    if os.path.exists(cfgfile):
        with open(cfgfile, 'r') as f:
            operatingmode = int(f.readline().strip())

    run_sentry(camera_index=cam, operatingmode=operatingmode)
