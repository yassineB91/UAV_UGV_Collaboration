import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Range, Imu, LaserScan
from nav_msgs.msg import Odometry


class ugvSensorBridge(Node):
    def __init__(self):
        super().__init__('ugv_sensor_bridge')


        self.img_pub = self.create_publisher(Image,'/ugv/camera/image_raw',10)
        self.imu_pub = self.create_publisher(Imu,'/ugv/imu',10)
        self.odom_pub = self.create_publisher(Odometry,'/ugv/odom',10)
        self.alt_pub = self.create_publisher(LaserScan,'/ugv/scan',10)

        self.img_sub = self.create_subscription(Image,'/camera/image_raw',self.on_img, 10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.on_imu ,10)
        self.odom_sub = self.create_subscription(Odometry,'/robot1/diff_cont/odom',self.on_odom,10)
        self.alt_sub = self.create_subscription(LaserScan,'/scan', self.on_scan ,10)

        

        self.get_logger().info('UGV sensor bridge is running')

    def on_img(self, msg: Image):
        self.img_pub.publish(msg)

    def on_imu(self, msg: Imu):
        self.imu_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)

    def on_scan(self, msg: LaserScan):
        self.alt_pub.publish(msg)


def main():
    rclpy.init()
    node = ugvSensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()