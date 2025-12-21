# cat_sentry.py
# 单文件：YOLO 识别 + 舵机控制 + 爆闪 + 扫描逻辑
import sys
import os
import time
import cv2
from ultralytics.models.yolo import YOLO
from multiprocessing import Process, Queue, Event
import queue
import pigpio
import threading


# =========================
# vifeo thread
# =========================

frame_queue = Queue(maxsize=1)
result_queue = Queue(maxsize=1)
stop_event = Event()
debug = False
if sys.argv[1] == 'debug':
    debug = True



def camera_thread(cap, frame_queue):
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.resize(frame, (320, 320))

        if frame_queue.full():
            try: 
                frame_queue.get_nowait()  # 丢弃旧帧
            except queue.Empty:
                pass

        frame_queue.put(frame)
        print('cam put frame')

def inference_process(frame_queue, result_queue):
    model = YOLO("yolov8n_ncnn_model")

    while not stop_event.is_set():
        print('infer loop!')
        try:
            frame = frame_queue.get(timeout = 0.2)
            print('get!')
        except :
            continue

        start = time.time()
        results = model(frame, classes=[15], conf=0.5)
        cost = (time.time() - start) * 1000

        detected = False
        error_x = 0

        r = results[0]
        if len(r.boxes) > 0:
            print('detected!')
            box = r.boxes[0]
            x1, _, x2, _ = map(int, box.xyxy[0])
            cat_x = (x1 + x2) // 2
            frame_center = frame.shape[1] // 2
            error_x = cat_x - frame_center
            detected = True

        if result_queue.full():
            result_queue.get_nowait()

        result_queue.put_nowait({
            "detected": detected,
            "error_x": error_x,
            "cost_ms": cost
        })


# =========================
# 舵机设置
# =========================
SERVO_PIN_horizon = 18       # PWM 引脚
SERVO_PIN_vertical = 24      # 备用垂直舵机引脚
LED_PIN = 23         # 爆闪 LED


# SG90 工具函数：角度→占空比
def set_servo_angle(angle, pin=SERVO_PIN_horizon):
    angle = max(0, min(180, angle))

    # SG90 常见参数（可微调）
    min_pulse = 500    # μs
    max_pulse = 2500   # μs

    pulse_width = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)

    pi.set_servo_pulsewidth(pin, pulse_width)
    


# =========================
# 爆闪函数
# =========================
def flash_led(times=3, interval=0.1):
    for _ in range(times):
        pi.write(LED_PIN, 1)
        time.sleep(interval)
        pi.write(LED_PIN, 0)
        time.sleep(interval)


# =========================
# 扫描模式参数
# =========================
scan_angle = 90
scan_direction = 1  # 1:右走, -1:左走
SCAN_SPEED = 1       # 每帧移动多少°
SCAN_MIN = 20        # 最左扫描角度
SCAN_MAX = 160       # 最右扫描角度

def AI_tick():
    pass


def control_loop(result_queue):
    scan_angle = 90
    scan_direction = 1

    while not stop_event.is_set():
        try: 
            if not result_queue.empty():
                r = result_queue.get()
                print(f"infer {r['cost_ms']:.1f} ms")

                # 有目标
                if r["detected"]:
                    correction = r["error_x"] * 0.1
                    scan_angle -= correction
                    scan_angle = max(SCAN_MIN, min(SCAN_MAX, scan_angle))
                    
                    if debug:
                        print(f'locked : {scan_angle}')
                    else:
                        set_servo_angle(scan_angle)
                        flash_led(interval=0.05)
                    continue

            # 扫描模式
            scan_angle += SCAN_SPEED * scan_direction
            if scan_angle >= SCAN_MAX:
                scan_direction = -1
            if scan_angle <= SCAN_MIN:
                scan_direction = 1

            if debug:
                print(f'scanning : {scan_angle}')
            else:
                set_servo_angle(scan_angle)
            time.sleep(0.05)
        except KeyboardInterrupt:
            stop_event.set()
            if debug:
                print('safe exit')
            else:
                pass
        finally:
            set_servo_angle(90) #reset
            set_servo_angle(90, pin=SERVO_PIN_vertical)
            pi.stop()




if __name__ == "__main__":
    print("gpio debug")
    if debug:
        pass
    else:
        pi = pigpio.pi()
        print(pi.connected)
        pi.set_mode(SERVO_PIN_horizon, pigpio.OUTPUT)
        pi.set_mode(SERVO_PIN_vertical, pigpio.OUTPUT)
        pi.set_mode(LED_PIN, pigpio.OUTPUT)
        pi.write(LED_PIN, 0)

    cam = 0

    #init the system
    cfgfile = 'sentry.cfg'
    operatingmode = 0
    if os.path.exists(cfgfile):
        with open(cfgfile, 'r') as f:
            operatingmode = int(f.readline().strip())

    cap = cv2.VideoCapture(cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
    cap.set(cv2.CAP_PROP_FPS, 10)

    # 启动采集线程
    p_cam = Process(target=camera_thread, args=(cap, frame_queue))
    p_inf = Process(target=inference_process, args=(frame_queue, result_queue))

    p_cam.start()
    p_inf.start()

    control_loop(result_queue)
    cap.release()
    print('safe exit')



###### Deprecated


# =========================
# 主检测与控制逻辑
# =========================
#def run_sentry(camera_index=0, operatingmode=0, cv2cap=None):
#    """
#    params:
#        camera_index: 摄像头索引
#        operatingmode: 0 = auto, 1 = manual
#    """
#    global scan_angle, scan_direction
#
#    # 加载 YOLO
#    model = YOLO("yolov8n_ncnn_model")
#
##    cv2.namedWindow("Sentry Cat Turret", cv2.WINDOW_NORMAL)
#
#
#    print("哨戒猫炮塔启动！按 q 或 Ctrl+C 退出...")
#
#    try:
#        while True:
#            with frame_lock:
#                if latest_frame is None:
#                    continue
#                frame = latest_frame.copy()
#
#
#            # YOLO 推理（只检测猫）
#            starttime = time.time()
#            results = model(frame, classes=[15], conf=0.5)
#            result = results[0]
#            starttime = time.time() - starttime
#            print(f"eval time: {starttime*1000:.1f} ms")
#
#            detected = False
#
#            if len(result.boxes) > 0:
#                # 只取第一个猫（你也可以换成最大面积）
#                box = result.boxes[0]
#                x1, y1, x2, y2 = map(int, box.xyxy[0])
#                conf = float(box.conf[0])
#
#                # cv2 debug
##                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
##                cv2.putText(
##                    frame,
##                    f"cat {conf:.2f}",
##                    (x1, y1 - 10),
##                    cv2.FONT_HERSHEY_SIMPLEX,
##                    0.6,
##                    (0, 255, 0),
##                    2
##                )
#
#
#                # 计算猫中心
#                cat_x = (x1 + x2) // 2
#                frame_center = frame.shape[1] // 2
#
##                #cv2 debug
##                cv2.line(frame,
##                         (frame_center, 0),
##                         (frame_center, frame.shape[0]),
##                         (255, 0, 0), 1)
#
#                # 偏差（像素差）
#                error_x = cat_x - frame_center
#                correction = error_x * 0.1  # adjust this !!!
#
#                # 更新云台角度
#                scan_angle -= correction
#                scan_angle = max(SCAN_MIN, min(SCAN_MAX, scan_angle))
#                set_servo_angle(scan_angle)
#
#                flash_led(interval=0.05)
#                detected = True
#
#                print(f"锁定猫 angle={scan_angle:.2f}° error={error_x}")
#
#            if not detected:
#                # 扫描模式
#                scan_angle += SCAN_SPEED * scan_direction
#                if scan_angle >= SCAN_MAX:
#                    scan_direction = -1
#                if scan_angle <= SCAN_MIN:
#                    scan_direction = 1
#
#                set_servo_angle(scan_angle)
#
#
#            if cv2.waitKey(1) & 0xFF == ord('q'):
#                break
#
#            time.sleep(0.05)
#
#    except KeyboardInterrupt:
#        print("\n手动中断")
#
#    finally:
#        cap.release()
#        set_servo_angle(90)