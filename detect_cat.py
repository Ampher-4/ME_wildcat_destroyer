import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import pyaudio
import audioop

############################
# 配置区域
############################

VIDEO_DEVICE = "/dev/video0"

# HC-SR04 GPIO
TRIG_PIN = 23
ECHO_PIN = 24

# 28BYJ-48 GPIO（IN1~IN4）
STEPPER_PINS = [17, 18, 27, 22]

# 音频参数
AUDIO_DEVICE_INDEX = 1     # 用 arecord -l 查看
AUDIO_THRESHOLD = 500      # 声音能量阈值（可调）

DISTANCE_THRESHOLD_CM = 10
SLEEP_AFTER_FEED = 60

############################
# GPIO 初始化
############################

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

for pin in STEPPER_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

############################
# 步进电机参数
############################

HALF_STEP_SEQ = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1],
]

STEPS_PER_REV = 4096
STEPS_180 = STEPS_PER_REV // 2

############################
# 功能函数
############################

def detect_cat(frame):
    """
    Placeholder：这里替换成你自己的 YOLO 推理
    返回 True / False
    """
    # TODO: 接你的 YOLO 代码
    return True   # 暂时假设检测到猫


def get_distance_cm():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)

    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        start = time.time()

    end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        end = time.time()

    duration = end - start
    distance = duration * 34300 / 2
    return distance


def detect_sound():
    p = pyaudio.PyAudio()

    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=16000,
                    input=True,
                    input_device_index=AUDIO_DEVICE_INDEX,
                    frames_per_buffer=1024)

    data = stream.read(1024, exception_on_overflow=False)
    rms = audioop.rms(data, 2)

    stream.stop_stream()
    stream.close()
    p.terminate()

    print(f"[Audio] RMS = {rms}")
    return rms > AUDIO_THRESHOLD


def step_motor(steps, delay=0.001, reverse=False):
    seq = HALF_STEP_SEQ[::-1] if reverse else HALF_STEP_SEQ
    for _ in range(steps):
        for step in seq:
            for pin, val in zip(STEPPER_PINS, step):
                GPIO.output(pin, val)
            time.sleep(delay)


def feed():
    print("[Motor] Feeding...")
    step_motor(STEPS_180)
    time.sleep(1)
    step_motor(STEPS_180, reverse=True)
    print("[Motor] Done")

############################
# 主循环
############################

def main():
    cap = cv2.VideoCapture(VIDEO_DEVICE)

    if not cap.isOpened():
        print("Camera open failed")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            if detect_cat(frame):
                distance = get_distance_cm()
                print(f"[Distance] {distance:.2f} cm")

                if distance < DISTANCE_THRESHOLD_CM:
                    if detect_sound():
                        feed()
                        print("[System] Sleeping...")
                        time.sleep(SLEEP_AFTER_FEED)

            time.sleep(0.1)

    finally:
        cap.release()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
