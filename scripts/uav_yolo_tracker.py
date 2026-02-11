import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import BoundingBox2D, ObjectHypothesisWithPose



class UavYoloTracker(Node):
    def __init__(self):
        super().__init__("uav_yolo_tracker")
        self.bridge = CvBridge()

        self.model_path = '/root/dev_ws/src/my_package/config/best.pt'
        self.tracker = 'bytetrack.yaml'
        self.classes = [0, 1]

        self.model = YOLO(self.model_path)

        self.boxes_pub = self.create_publisher(Detection2DArray, "/uav/detection_boxes", 10)


        self.img_sub = self.create_subscription(Image,'/uav/camera/image_raw',self.on_image,10)

        self.get_logger().info('UAV YOLO tracker is running')


    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h,w = frame.shape[:2]
            result = self.model.track(
                source = frame,
                tracker = self.tracker,
                persist = True,
                classes = self.classes,
                stream = False,
                verbose = False)
            res = result[0]

            out = Detection2DArray()
            out.header = msg.header

            if res.boxes is not None and len(res.boxes) > 0:
                for box in res.boxes:
                    
                    cls = int(box.cls.item())
                    x,y,w,h = box.xywh[0].tolist()
                    score = float(box.conf.item()) if box.conf is not None else 0.0

                    det = Detection2D()
                    det.header = out.header

                    bbox = BoundingBox2D()
                    bbox.center.position.x = float(x)
                    bbox.center.position.y = float(y)
                    bbox.size_x = float(w)
                    bbox.size_y = float(h)
                    det.bbox = bbox

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(cls)
                    hyp.hypothesis.score = score
                    det.results.append(hyp)

                    out.detections.append(det)

                self.boxes_pub.publish(out)



            else:
                self.boxes_pub.publish(out)
                return
            
        except Exception as e:
            self.get_logger().error(f'Yolo tracker error : {e}')

def main():
    rclpy.init()
    node = UavYoloTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
    