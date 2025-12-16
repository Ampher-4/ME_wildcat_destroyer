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

    print("哨戒猫炮塔启动！Ctrl+C 退出...")

    try:
        while True:

            # AI tick
            ret, frame = cap.read()
            if not ret:
                print("读取相机失败")
                continue

            # YOLO 只检测猫
            result = model(frame, classes=[15], conf=0.5)[0]

            if len(result.boxes) > 0:
                # 取最大置信度猫目标（一般只有一只）
                box = result.boxes[0]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # 计算猫中心
                cat_x = (x1 + x2) // 2
                frame_center = frame.shape[1] // 2

                # 偏差（像素差）
                error_x = cat_x - frame_center

                # 转成旋转角度校正（比例可调整）
                correction = error_x * 0.05  # 每像素 0.05°（需自己调参）

                # 更新云台角度
                scan_angle -= correction
                scan_angle = max(SCAN_MIN, min(SCAN_MAX, scan_angle))
                set_servo_angle(scan_angle)

                print(f"锁定猫！angle={scan_angle:.2f}°")

                flash_led(times=1, interval=0.05)

            else:
                # 没有检测到猫 → 扫描模式
                scan_angle += SCAN_SPEED * scan_direction
                if scan_angle >= SCAN_MAX:
                    scan_direction = -1
                if scan_angle <= SCAN_MIN:
                    scan_direction = 1

                set_servo_angle(scan_angle)
                print(f"扫描中... angle={scan_angle}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

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
