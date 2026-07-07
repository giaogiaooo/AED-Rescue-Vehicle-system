#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# 尝试导入 RKNNLite，用于 RK3588 NPU 推理
try:
    from rknnlite.api import RKNNLite
    HAS_RKNN = True
except ImportError:
    HAS_RKNN = False

# ===================== 系统配置 =====================
RKNN_MODEL = "fall.rknn"
RTSP_URL = "rtsp://admin:@192.168.1.86"

INPUT_SIZE = 640
CONF_THRESH = 0.85
NMS_THRESH = 0.45

FALL_TRIGGER_FRAMES = 10  # 必须连续检测到 10 帧
# ====================================================

# ===================== 核心算法辅助 =====================
def letterbox(img, new_shape=(640, 640), color=(114, 114, 114)):
    """对图像进行YOLO标准前处理(缩放+黑边填充)"""
    h, w = img.shape[:2]
    scale = min(new_shape[0] / h, new_shape[1] / w)
    nw, nh = int(w * scale), int(h * scale)
    dw, dh = (new_shape[1] - nw) / 2, (new_shape[0] - nh) / 2
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, scale, dw, dh

def post_process(pred, scale, pad_w, pad_h):
    """NMS后处理提取目标框"""
    if pred.shape[0] < pred.shape[1]: pred = pred.T
    scores = np.max(pred[:, 4:], axis=1)
    cls_ids = np.argmax(pred[:, 4:], axis=1)
    mask = scores > CONF_THRESH
    valid_pred, valid_scores, valid_cls = pred[mask], scores[mask], cls_ids[mask]

    boxes = []
    for i in range(len(valid_pred)):
        cx, cy, bw, bh = valid_pred[i, :4]
        cx, cy = (cx - pad_w) / scale, (cy - pad_h) / scale
        bw, bh = bw / scale, bh / scale
        x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
        boxes.append([x1, y1, int(bw), int(bh)])

    results = []
    if len(boxes) > 0:
        indices = cv2.dnn.NMSBoxes(boxes, valid_scores.tolist(), CONF_THRESH, NMS_THRESH)
        if len(indices) > 0:
            for i in indices.flatten():
                results.append({"box": boxes[i], "score": valid_scores[i], "class_id": valid_cls[i]})
    return results

# ===================== 线程1: 流媒体极速读取 =====================
class CaptureThread(threading.Thread):
    def __init__(self, url):
        super().__init__(name="CaptureThread")
        self.url = url
        self.frame = None
        self.running = True
        self.is_connected = False
        self.lock = threading.Lock()
        self.daemon = True

    def run(self):
        # 强制底层TCP传输与零缓冲
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|timeout;5000000"
        while self.running:
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            if cap.isOpened():
                self.is_connected = True
                while self.running:
                    ret, frame = cap.read()
                    if not ret: break
                    with self.lock:
                        self.frame = frame
            
            self.is_connected = False
            cap.release()
            time.sleep(1)

    def get_latest(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.is_connected, self.frame.copy()

# ===================== ROS2 节点 =====================
class FallDetectionNode(Node):
    def __init__(self):
        super().__init__('aed_fall_detection_node')
        self.get_logger().info('Initializing Edge Fall Detection Node...')

        # 创建 ROS2 Publisher
        self.event_pub = self.create_publisher(Bool, '/fall_event', 10)
        self.info_pub = self.create_publisher(String, '/fall_info', 10)

        # 状态机防抖变量
        self.current_state_is_fall = False
        self.fall_counter = 0

        # 启动视频拉流线程
        self.capture_thread = CaptureThread(RTSP_URL)
        self.capture_thread.start()

        # 启动独立的 NPU 推理线程
        self.inference_thread = threading.Thread(target=self.inference_loop, name="InferenceThread")
        self.inference_thread.daemon = True
        self.inference_thread.start()

    def inference_loop(self):
        # 初始化 RKNN
        if HAS_RKNN and os.path.exists(RKNN_MODEL):
            rknn = RKNNLite()
            rknn.load_rknn(RKNN_MODEL)
            try:
                core_mask = RKNNLite.NPU_CORE_0_1_2
            except AttributeError:
                core_mask = 7
            rknn.init_runtime(core_mask=core_mask)
            self.get_logger().info("NPU Runtime Initialized (RK3588 Tri-Core).")
        else:
            self.get_logger().warn("RKNN Environment not found. Running in simulation mode.")
            rknn = None

        # 设置OpenCV窗口属性(允许自由缩放)
        cv2.namedWindow("RKNN Fall Detection Inference", cv2.WINDOW_NORMAL)

        while rclpy.ok():
            is_conn, frame = self.capture_thread.get_latest()
            if not is_conn or frame is None:
                time.sleep(0.05)
                continue
            
            is_fall_detected = False
            max_conf = 0.0
            dets = []

            # ================= 执行 NPU 推理 =================
            if rknn:
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_processed, scale, pad_w, pad_h = letterbox(img_rgb, (INPUT_SIZE, INPUT_SIZE))
                input_data = np.expand_dims(img_processed, axis=0).astype(np.uint8)

                outputs = rknn.inference(inputs=[input_data])
                pred = np.squeeze(outputs[0])
                dets = post_process(pred, scale, pad_w, pad_h)

                is_fall_detected = len(dets) > 0
                max_conf = max([d['score'] for d in dets]) if is_fall_detected else 0.0

            # ================= 状态机防抖逻辑 =================
            if is_fall_detected:
                if self.fall_counter < FALL_TRIGGER_FRAMES:
                    self.fall_counter += 1
                    self.get_logger().info(f"[FALL] Counter={self.fall_counter}")
            else:
                if self.fall_counter > 0:
                    self.fall_counter -= 1

            # 连续 10 帧检测确认跌倒 (False -> True)
            if self.fall_counter >= FALL_TRIGGER_FRAMES:
                if not self.current_state_is_fall:
                    self.current_state_is_fall = True
                    self.get_logger().info("[FALL] Confirmed Fall Event")
                    self.publish_string_info(True, max_conf)
            # 恢复正常 (True -> False)
            elif self.fall_counter == 0:
                if self.current_state_is_fall:
                    self.current_state_is_fall = False
                    self.get_logger().info("[FALL] Recovered / Normal")
                    self.publish_string_info(False, 0.0)

            # 持续发布当前事件状态
            msg_bool = Bool()
            msg_bool.data = self.current_state_is_fall
            self.event_pub.publish(msg_bool)

            # ================= 图像渲染与可视化 =================
            if rknn is None:
                cv2.putText(frame, "SIMULATION MODE - NO RKNN", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
            else:
                # 1. 绘制检测框 (Bounding Box)
                for det in dets:
                    x, y, w, h = det['box']
                    conf = det['score']
                    # 绘制红色方框表示跌倒目标
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    label = f"Fall: {conf:.2f}"
                    cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # 2. 绘制全局状态信息
                if self.current_state_is_fall:
                    cv2.putText(frame, "STATUS: FALL DETECTED!!!", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                else:
                    cv2.putText(frame, "STATUS: NORMAL", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                
                # 3. 绘制防抖计数器 (以便观察连续检测过程)
                if self.fall_counter > 0 and not self.current_state_is_fall:
                    cv2.putText(frame, f"Warning Counter: {self.fall_counter}/{FALL_TRIGGER_FRAMES}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

            # 4. 显示画面
            cv2.imshow("RKNN Fall Detection Inference", frame)
            
            # 按下 'q' 键可以安全退出程序
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Quit requested via Video Window.")
                # 通过抛出异常或调用shutdown关闭ROS2
                rclpy.shutdown()
                break

        if rknn:
            rknn.release()
        cv2.destroyAllWindows()

    def publish_string_info(self, state, conf):
        now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        if state:
            msg_str = String()
            msg_str.data = f"FALL DETECTED\nTIME={now_str}\nCONF={conf:.2f}"
            self.info_pub.publish(msg_str)

def main(args=None):
    rclpy.init(args=args)
    node = FallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fall detection node.')
    finally:
        node.capture_thread.running = False
        node.destroy_node()
        # 检查rclpy是否已经shutdown (防止在GUI按q退出后重复shutdown报错)
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()