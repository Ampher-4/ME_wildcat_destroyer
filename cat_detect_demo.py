#linux only script
from ultralytics.models.yolo import YOLO
import cv2
import sys


def cat_detection_from_camera(camera_index=0, conf_threshold=0.5):
    """
    demo for cat detection from camera using YOLOv8 on embedded devices.
    read /dev/camera0 on default. 
    """
    # 加载YOLOv8预训练模型（自动下载轻量化模型，适配嵌入式）
    # 若嵌入式设备无网络，可先下载yolov8n.pt到本地后指定路径
    model = YOLO("yolov8n.pt")  # n版为最小模型，速度最快，适合嵌入式

    # 打开摄像头（适配嵌入式V4L2驱动）
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("fail to read camera", file=sys.stderr)
        sys.exit(1)

    # 设置摄像头参数（降低分辨率提升嵌入式性能）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 10)  # 降低帧率减少资源占用

    print("开始猫咪检测，按 'ctrl + c' 退出...")
    print("检测结果格式：[类别] 置信度 | 坐标(左上x,左上y,右下x,右下y)")

    try:
        while True:
            # 读取一帧图像
            ret, frame = cap.read()
            if not ret:
                print("警告：无法读取摄像头帧，重试...", file=sys.stderr)
                continue

            # 运行YOLO检测（仅检测猫类别，class=15对应COCO数据集的cat）
            results = model(frame, classes=[15], conf=conf_threshold)

            # 解析检测结果并输出坐标
            cat_detections = []
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # 获取坐标（xyxy格式）
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # 获取置信度
                    conf = float(box.conf[0])
                    # 获取类别名称
                    cls_name = model.names[int(box.cls[0])]
                    
                    # 保存检测结果
                    cat_detections.append({
                        "class": cls_name,
                        "confidence": conf,
                        "coordinates": (x1, y1, x2, y2)
                    })
                    
                    # 输出坐标（核心需求）
                    print(f"[{cls_name}] {conf:.2f} | 坐标：({x1}, {y1}, {x2}, {y2})")

            # 若无检测结果，输出提示
            if not cat_detections:
                print("未检测到猫咪")

            # 按q退出（嵌入式若无需GUI，可注释此段）
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n检测被手动终止")
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 支持命令行指定摄像头索引（如python3 cat_detection.py 1）
    camera_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cat_detection_from_camera(camera_index=camera_idx, conf_threshold=0.5)