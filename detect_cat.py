import time
import cv2
import numpy as np
from ultralytics.models.yolo import YOLO
import RPi.GPIO as GPIO
import pyaudio
import sounddevice as sd
import numpy as np

AUDIO_DEVICE_INDEX = 1    # DroidCam
SAMPLE_RATE = 16000
RECORD_SECONDS = 0.5
AUDIO_THRESHOLD = 0.02   # RMS 阈值（经验值）


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
CAT_CLASS_ID = 15
def detect_cat(frame, model):
    
    h, w, _ = frame.shape
    frame_area = h * w

    results = model(frame, verbose=False)

    for r in results:
        if r.boxes is None:
            continue

        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            if cls_id != CAT_CLASS_ID or conf < 0.6:
                continue

            x1, y1, x2, y2 = box.xyxy[0]
            bbox_area = (x2 - x1) * (y2 - y1)
            area_ratio = bbox_area / frame_area

            print(f"[YOLO] Cat conf={conf:.2f}, area={area_ratio:.2f}")

            return True

    return False



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
#    p = pyaudio.PyAudio()
#
#    stream = p.open(format=pyaudio.paInt16,
#                    channels=1,
#                    rate=16000,
#                    input=True,
#                    input_device_index=AUDIO_DEVICE_INDEX,
#                    frames_per_buffer=1024)
#
#    data = stream.read(1024, exception_on_overflow=False)
#    rms = audioop.rms(data, 2)

#    stream.stop_stream()
#    stream.close()
#    p.terminate()
#
#    print(f"[Audio] RMS = {rms}")
#    return rms > AUDIO_THRESHOLD
    audio = sd.rec(
        int(RECORD_SECONDS * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype='float32',
        device=AUDIO_DEVICE_INDEX
    )

    sd.wait()

    rms = np.sqrt(np.mean(audio**2))
    print(f"[Audio] RMS = {rms:.4f}")

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

    model = YOLO("yolov8n.pt")


    cap = cv2.VideoCapture(VIDEO_DEVICE)

    if not cap.isOpened():
        print("Camera open failed")
        return

    # 设置摄像头参数（降低分辨率提升嵌入式性能）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 10)  # 降低帧率减少资源占用

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame from camera, did you plug camera yet?")
                continue

            if detect_cat(frame, model):
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
