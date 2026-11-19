"""Publish Ultralytics detections through the upstream yolo_msgs interface."""
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_msgs.msg import Detection, DetectionArray, Point2D

from .sensor_qos import declare_sensor_qos


class Detector(Node):
    def __init__(self):
        super().__init__('course_detector')
        self.declare_parameter('model', 'yolov8n-seg.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/rgbd/image')
        self.declare_parameter('threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('image_size', 640)
        sensor_qos = declare_sensor_qos(self)
        from ultralytics import YOLO
        import torch
        self.model = YOLO(self.get_parameter('model').value)
        self.model.model.float()
        self.model.to(self.get_parameter('device').value)
        self.get_logger().info(f'视觉推理设备：{next(self.model.model.parameters()).device}；FP32；'
                               f'torch={torch.__version__}，CUDA={torch.version.cuda}')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(DetectionArray, '/yolo/detections', 10)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.detect,
            sensor_qos)

    def detect(self, image):
        result = self.model.predict(
            self.bridge.imgmsg_to_cv2(image, 'bgr8'),
            device=self.get_parameter('device').value,
            conf=self.get_parameter('threshold').value,
            imgsz=self.get_parameter('image_size').value,
            iou=self.get_parameter('iou_threshold').value, verbose=False)[0]
        msg = DetectionArray()
        msg.header = image.header
        if result.boxes is not None:
            for index, (box, label, score) in enumerate(zip(result.boxes.xywh.tolist(),
                                         result.boxes.cls.tolist(),
                                         result.boxes.conf.tolist())):
                det = Detection()
                det.class_id = int(label)
                det.class_name = result.names[int(label)]
                det.score = float(score)
                det.bbox.center.position.x, det.bbox.center.position.y = box[:2]
                det.bbox.size.x, det.bbox.size.y = box[2:]
                if result.masks is not None:
                    det.mask.height, det.mask.width = image.height, image.width
                    det.mask.data = [Point2D(x=float(x), y=float(y))
                                     for x, y in result.masks.xy[index]]
                msg.detections.append(det)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = Detector()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
