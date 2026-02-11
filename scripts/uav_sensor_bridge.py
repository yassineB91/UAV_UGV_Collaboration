import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped



from sensor_msgs.msg import Image, Range, Imu
from nav_msgs.msg import Odometry


class UavSensorBridge(Node):
    def __init__(self):
        super().__init__('uav_sensor_bridge')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.img_pub = self.create_publisher(Image,'/uav/camera/image_raw',10)
        self.imu_pub = self.create_publisher(Imu,'/uav/imu',10)
        self.odom_pub = self.create_publisher(Odometry,'/uav/odom',10)
        self.alt_pub = self.create_publisher(Range,'/uav/altitude',10)

        self.img_sub = self.create_subscription(Image,'/iris_depth_camera/rgb_camera/image_raw',self.on_img, sensor_qos)
        self.imu_sub = self.create_subscription(Imu,'/mavros/imu/data',self.on_imu ,sensor_qos)
        self.odom_sub = self.create_subscription(Odometry,'/mavros/local_position/odom', self.on_odom, sensor_qos)
        self.pose_sub = self.create_subscription(PoseStamped,'/mavros/local_position/pose', self.on_pose ,sensor_qos)


        

        self.get_logger().info('UAV sensor bridge is running')

    def on_img(self, msg: Image):
        self.img_pub.publish(msg)

    def on_imu(self, msg: Imu):
        self.imu_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)

    def on_pose(self, msg: PoseStamped):
        out = Range()
        out.header = msg.header
        out.range = float(msg.pose.position.z)
        out.radiation_type = Range.INFRARED
        out.field_of_view = 0.5
        out.min_range = 0.0
        out.max_range = 10000.0
        self.alt_pub.publish(out)



def main():
    rclpy.init()
    node = UavSensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()