"""Publish Ultralytics detections through the upstream yolo_msgs interface."""
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_msgs.msg import Detection, DetectionArray, Point2D

from .sensor_qos import current_sensor_stamp, declare_sensor_qos
from .detection_fusion import auxiliary_candidate


class Detector(Node):
    def __init__(self):
        super().__init__('course_detector')
        self.declare_parameter('model', 'yolov8n-seg.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/rgbd/image')
        self.declare_parameter('threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('maximum_input_age',.4)
        self.declare_parameter('ball_model','')
        self.declare_parameter('auxiliary_model','')
        self.declare_parameter('auxiliary_categories',['sports ball'])
        self.declare_parameter('auxiliary_prompts','')
        sensor_qos = declare_sensor_qos(self,depth=1)
        from ultralytics import YOLO
        import torch
        self.model = YOLO(self.get_parameter('model').value)
        self.model.model.float()
        self.model.to(self.get_parameter('device').value)
        ball_model=self.get_parameter('auxiliary_model').value or self.get_parameter('ball_model').value
        self.auxiliary_categories=set(self.get_parameter('auxiliary_categories').value)
        self.ball_model=YOLO(ball_model) if ball_model else None
        self.last_ball_inference=-1.
        self.ball_labels_seen=set()
        if self.ball_model is not None:
            self.ball_model.model.float()
            self.ball_model.to(self.get_parameter('device').value)
            prompts=self.get_parameter('auxiliary_prompts').value
            if prompts:self.ball_model.load_prompt_embeddings(prompts)
            self.get_logger().info('视觉辅助候选：最多2Hz；补充类别='+','.join(sorted(self.auxiliary_categories))+
                                   '；同类重叠保留主检测，气球不合并。')
        self.get_logger().info(f'视觉推理设备：{next(self.model.model.parameters()).device}；FP32；'
                               f'torch={torch.__version__}，CUDA={torch.version.cuda}')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(DetectionArray, '/yolo/detections', 10)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.detect,
            sensor_qos)

    def detect(self, image):
        stamp = image.header.stamp.sec+image.header.stamp.nanosec/1e9
        if not current_sensor_stamp(stamp,self.get_clock().now().nanoseconds/1e9,
                                    self.get_parameter('maximum_input_age').value):
            return
        pixels=self.bridge.imgmsg_to_cv2(image, 'bgr8')
        result = self.model.predict(
            pixels,
            device=self.get_parameter('device').value,
            conf=self.get_parameter('threshold').value,
            imgsz=self.get_parameter('image_size').value,
            iou=self.get_parameter('iou_threshold').value, verbose=False)[0]
        results=[(result,False)]
        primary=[] if result.boxes is None else [(result.names[int(c)],box)
            for c,box in zip(result.boxes.cls.tolist(),result.boxes.xyxy.tolist())]
        now=self.get_clock().now().nanoseconds/1e9
        primary_ball=result.boxes is not None and any(
            result.names[int(label)]=='sports ball' for label in result.boxes.cls.tolist())
        if (self.ball_model is not None and not (primary_ball and self.auxiliary_categories=={'sports ball'}) and
                (now-self.last_ball_inference>=.5 or now<self.last_ball_inference)):
            self.last_ball_inference=now
            extra=self.ball_model.predict(pixels,device=self.get_parameter('device').value,
                conf=self.get_parameter('threshold').value,imgsz=self.get_parameter('image_size').value,
                iou=self.get_parameter('iou_threshold').value,agnostic_nms=True,verbose=False)[0]
            results.append((extra,True))
        if self.ball_model is not None and not current_sensor_stamp(
                stamp,self.get_clock().now().nanoseconds/1e9,self.get_parameter('maximum_input_age').value):
            return
        msg = DetectionArray()
        msg.header = image.header
        for result,ball_only in results:
            if result.boxes is None:
                continue
            for index, (box, label, score) in enumerate(zip(result.boxes.xywh.tolist(),
                                         result.boxes.cls.tolist(),
                                         result.boxes.conf.tolist())):
                name=result.names[int(label)]
                canonical=None
                if ball_only:
                    canonical=auxiliary_candidate(name,result.boxes.xyxy[index].tolist(),primary,self.auxiliary_categories)
                    if canonical is None:continue
                if ball_only and name not in self.ball_labels_seen:
                    self.ball_labels_seen.add(name)
                    self.get_logger().info(f'辅助检测标签 {name} → {canonical[0]}；分数与原图掩膜保留。')
                det = Detection()
                det.class_id = canonical[1] if ball_only else int(label)
                det.class_name = canonical[0] if ball_only else name
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
