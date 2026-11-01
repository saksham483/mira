#!/usr/bin/env python3
"""
ZED SDK Custom Object Tracking + YOLOv11 TensorRT (Generic Multi-Class).
Publishes:
  - /vision/detections_3d (vision_msgs/Detection3DArray) : 3D centroids, dimensions, tracking IDs
  - /vision/detections    (vision_msgs/Detection2DArray) : 2D bounding boxes for Behavior Tree
  - /vision/target_pose   (geometry_msgs/PoseStamped)    : Closest/highest-confidence 3D target
Streams:
  - GStreamer H.264 preview over TCP port 5000 for ffplay
"""

import argparse
import subprocess
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
import pyzed.sl as sl
from ultralytics import YOLO

from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import (
    Detection3DArray,
    Detection3D,
    BoundingBox3D,
    ObjectHypothesisWithPose,
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
)


class SafeTcpStreamer:
    """Streams 640x360 H.264 preview over TCP without EGL/X11 dependencies."""
    def __init__(self, port=5000, fps=30):
        self.port = port
        self.fps = fps
        self.proc = None

    def start(self, width, height):
        cmd = [
            "gst-launch-1.0", "-q",
            "fdsrc", "!",
            "rawvideoparse", f"width={width}", f"height={height}", "format=bgr", f"framerate={self.fps}/1", "!",
            "videoconvert", "!",
            "video/x-raw,format=I420", "!",
            "x264enc", "tune=zerolatency", "speed-preset=ultrafast", "key-int-max=30", "bitrate=1500", "!",
            "h264parse", "config-interval=-1", "!",
            "mpegtsmux", "!",
            "tcpserversink", "host=0.0.0.0", f"port={self.port}", "sync=false"
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    def write(self, frame):
        if self.proc is None:
            h, w = frame.shape[:2]
            self.start(w, h)
        try:
            self.proc.stdin.write(frame.tobytes())
        except Exception:
            pass

    def close(self):
        if self.proc:
            try:
                self.proc.stdin.close()
                self.proc.terminate()
            except Exception:
                pass


def get_color(class_id: int):
    """Generate a consistent distinct color for each class ID."""
    np.random.seed(class_id)
    color = tuple(map(int, np.random.randint(50, 255, size=3)))
    return color


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=str, default="/workspace/src/mira2_perception/models/weights/yolo11n.engine")
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--classes", type=str, default="", help="Comma-separated class names to filter (default: empty = all)")
    p.add_argument("--svo", type=str, default="")
    p.add_argument("--stream-port", type=int, default=5000)
    p.add_argument("--depth-mode", type=str, default="ULTRA", choices=["PERFORMANCE", "QUALITY", "ULTRA"])
    p.add_argument("--no-stream", action="store_true", help="Run pure headless without streaming")
    return p.parse_args()


class GenericZedTracker(Node):
    def __init__(self, args):
        super().__init__('generic_zed_tracker')
        self.args = args
        self.allowed_classes = [c.strip().lower() for c in args.classes.split(",") if c.strip()]
        
        # ROS 2 Publishers
        self.det_3d_pub = self.create_publisher(Detection3DArray, '/vision/detections_3d', 10)
        self.det_2d_pub = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.pose_pub   = self.create_publisher(PoseStamped, '/vision/target_pose', 10)
        
        self.streamer = None if args.no_stream else SafeTcpStreamer(port=args.stream_port)
        if self.streamer:
            self.get_logger().info(f"TCP Streamer active on port {args.stream_port}")
            
        # 1. Load TensorRT Engine
        self.get_logger().info(f"Loading YOLO engine: {args.model}")
        self.model = YOLO(args.model, task='detect')
        self.get_logger().info(f"Loaded model with classes: {list(self.model.names.values())}")
        
        # 2. Initialize ZED Camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        
        if args.depth_mode == "PERFORMANCE":
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        elif args.depth_mode == "QUALITY":
            init_params.depth_mode = sl.DEPTH_MODE.QUALITY
        else:
            init_params.depth_mode = sl.DEPTH_MODE.ULTRA
            
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        
        if args.svo:
            init_params.set_from_svo_file(args.svo)
            init_params.svo_real_time_mode = False
            
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().fatal(f"ZED Open Failed: {status}")
            exit(1)
            
        # 3. Enable Positional Tracking & Custom Object Detection
        tracking_params = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(tracking_params)
        
        obj_params = sl.ObjectDetectionParameters()
        obj_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_params.enable_tracking = True
        self.zed.enable_object_detection(obj_params)
        
        self.obj_runtime_params = sl.ObjectDetectionRuntimeParameters()
        self.runtime_params = sl.RuntimeParameters()
        
        self.image_left = sl.Mat()
        self.objects = sl.Objects()
        self.get_logger().info("Generic ZED Tracker Ready. Running capture loop...")

    def run_loop(self):
        last_log = time.time()
        fps_count = 0
        
        while rclpy.ok():
            # --- STAGE 1: ZED Grab ---
            t0 = time.perf_counter()
            if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                continue
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            frame_bgra = self.image_left.get_data()
            img_bgr = np.ascontiguousarray(frame_bgra[:, :, :3])
            img_h, img_w = img_bgr.shape[:2]
            t_grab = (time.perf_counter() - t0) * 1000
            
            # --- STAGE 2: YOLO Inference ---
            t0 = time.perf_counter()
            results = self.model(img_bgr, conf=self.args.conf, verbose=False)[0]
            t_yolo = (time.perf_counter() - t0) * 1000
            
            # --- STAGE 3: Custom Box Formatting ---
            custom_boxes = []
            det_2d_msg = Detection2DArray()
            stamp = self.get_clock().now().to_msg()
            det_2d_msg.header.stamp = stamp
            det_2d_msg.header.frame_id = 'zed_left_camera_frame'

            for box in results.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id].lower()
                
                # Filter classes if user passed --classes argument
                if self.allowed_classes and cls_name not in self.allowed_classes:
                    continue
                    
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                
                # Ingest Box into ZED
                box_2d = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.int32)
                custom_box = sl.CustomBoxObjectData()
                custom_box.bounding_box_2d = box_2d
                custom_box.label = cls_id
                custom_box.probability = conf
                custom_box.is_grounded = False
                custom_boxes.append(custom_box)
                
                # Populate 2D Detection (for Behavior Tree / mira2_actions)
                d2d = Detection2D()
                d2d.header = det_2d_msg.header
                d2d.id = cls_name  # BT matches target object by id
                d2d.bbox.center.position.x = float((x1 + x2) / (2.0 * img_w))
                d2d.bbox.center.position.y = float((y1 + y2) / (2.0 * img_h))
                d2d.bbox.size_x = float((x2 - x1) / img_w)
                d2d.bbox.size_y = float((y2 - y1) / img_h)
                det_2d_msg.detections.append(d2d)
                
            self.det_2d_pub.publish(det_2d_msg)
                
            # --- STAGE 4: ZED 3D Tracking ---
            t0 = time.perf_counter()
            self.zed.ingest_custom_box_objects(custom_boxes)
            self.zed.retrieve_objects(self.objects, self.obj_runtime_params)
            t_track = (time.perf_counter() - t0) * 1000
            
            # --- STAGE 5: ROS 2 3D Publishing ---
            det_3d_msg = Detection3DArray()
            det_3d_msg.header.stamp = stamp
            det_3d_msg.header.frame_id = 'zed_left_camera_frame'
            
            tracked_summary = []
            closest_dist = float('inf')
            primary_pose = None
            
            for obj in self.objects.object_list:
                if obj.tracking_state not in [sl.OBJECT_TRACKING_STATE.OK, sl.OBJECT_TRACKING_STATE.SEARCHING]:
                    continue
                cls_id = int(obj.raw_label)
                cls_name = self.model.names.get(cls_id, f"class_{cls_id}").lower()
                x, y, z = obj.position
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    continue
                    
                corners_3d = obj.bounding_box
                if len(corners_3d) < 8:
                    continue
                    
                width = np.linalg.norm(corners_3d[0] - corners_3d[1])
                height = np.linalg.norm(corners_3d[1] - corners_3d[2])
                depth = np.linalg.norm(corners_3d[0] - corners_3d[4])
                
                tracked_summary.append(f"{cls_name}#{obj.id}({z:.1f}m)")

                # Standard vision_msgs/Detection3D
                det3d = Detection3D()
                det3d.header = det_3d_msg.header
                det3d.id = str(obj.id)
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = float(obj.confidence / 100.0)
                hyp.pose.pose.position.x = float(x)
                hyp.pose.pose.position.y = float(y)
                hyp.pose.pose.position.z = float(z)
                hyp.pose.pose.orientation.w = 1.0
                det3d.results.append(hyp)
                
                det3d.bbox.center.position.x = float(x)
                det3d.bbox.center.position.y = float(y)
                det3d.bbox.center.position.z = float(z)
                det3d.bbox.center.orientation.w = 1.0
                det3d.bbox.size.x = float(width)
                det3d.bbox.size.y = float(height)
                det3d.bbox.size.z = float(depth)
                det_3d_msg.detections.append(det3d)
                
                # Check closest object for primary target pose
                dist = np.sqrt(x*x + y*y + z*z)
                if dist < closest_dist:
                    closest_dist = dist
                    primary_pose = PoseStamped()
                    primary_pose.header = det_3d_msg.header
                    primary_pose.pose.position.x = float(x)
                    primary_pose.pose.position.y = float(y)
                    primary_pose.pose.position.z = float(z)
                    primary_pose.pose.orientation.w = 1.0
                
                # Visual Overlays
                if self.streamer:
                    color = get_color(cls_id)
                    x1, y1 = obj.bounding_box_2d[0]
                    x2, y2 = obj.bounding_box_2d[2]
                    cv2.rectangle(img_bgr, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    label = f"{cls_name} #{obj.id} | X:{x:+.2f} Y:{y:+.2f} Z:{z:.2f}m"
                    cv2.putText(img_bgr, label, (int(x1), int(y1) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            self.det_3d_pub.publish(det_3d_msg)
            if primary_pose:
                self.pose_pub.publish(primary_pose)

            # --- STAGE 6: Stream Preview ---
            t0 = time.perf_counter()
            if self.streamer:
                preview = cv2.resize(img_bgr, (640, 360))
                self.streamer.write(preview)
            t_stream = (time.perf_counter() - t0) * 1000
            
            # Terminal Telemetry Dashboard
            fps_count += 1
            now = time.time()
            if now - last_log >= 1.0:
                fps = fps_count / (now - last_log)
                fps_count = 0
                last_log = now
                summary_str = ", ".join(tracked_summary) if tracked_summary else "none"
                print(
                    f"[FPS: {fps:4.1f}] | Grab: {t_grab:4.1f}ms | YOLO: {t_yolo:4.1f}ms | "
                    f"Track: {t_track:4.1f}ms | Tracked: {len(tracked_summary)} -> [{summary_str}]",
                    flush=True
                )
                
            rclpy.spin_once(self, timeout_sec=0.0001)

    def destroy_node(self):
        if self.streamer:
            self.streamer.close()
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    parsed_args = parse_args()
    node = GenericZedTracker(parsed_args)
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
