import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2,  PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudConverter(Node):
    def __init__(self):
        super().__init__("point_cloud_converter_node_py")
        self.get_logger().info("point_cloud_converter_node_py create successful!")
        self.sub = self.create_subscription(
            PointCloud2,
            "/points_raw",
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            "/points_raw_converted",
            10
        )

    def callback(self, msg):
        # 读取原始点云所有点
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))

        if len(points) == 0:
            return

        # 给每个点根据仰角计算 ring 编号
        new_points = []
        N_SCANS = 24 # 和 URDF 里垂直采样数一致

        for p in points:
            x, y, z, intensity = p
            # 计算仰角（弧度转角度）
            elev = np.arctan2(z, np.sqrt(x*x + y*y)) * 57.3
            # 将仰角映射到线号（0~90度映射到0~23）
            ring = int(elev / 90.0 * N_SCANS - 1)
            ring = max(0, min(ring, N_SCANS - 1))
            # 时间戳设为0
            new_points.append([x, y, z, intensity, ring, 0.0])

        # 3. 定义新的点云字段（加上 ring 和 time）
        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',      offset=16, datatype=PointField.UINT16,  count=1),
            PointField(name='time',      offset=18, datatype=PointField.FLOAT32, count=1),
        ]

        # 4. 发布新点云
        new_msg = pc2.create_cloud(msg.header, fields, new_points)
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = PointCloudConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
