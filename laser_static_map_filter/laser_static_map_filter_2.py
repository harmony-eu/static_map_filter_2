"""
Created on ... by Julian Keller

Ported to ROS2 Humble on 17.10.2022 by Giulio Schiavi
"""
import copy
import numpy as np

from scipy.spatial.transform import Rotation as R

import rclpy
import tf2_ros

from rclpy.node import Node
from sensor_msgs_py.point_cloud2 import read_points_numpy, create_cloud_xyz32, read_points

from sensor_msgs.msg import PointCloud2
from tf2_geometry_msgs import PointStamped




def do_transform_cloud(cloud_np, transform_msg):
    """
    cloud_np: np.ndarray: pointcloud of shape (N, 3)
    transform_msg: geometry_msgs/msg/Transform: transformation message
    """
    q = transform_msg.rotation
    r = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    v = transform_msg.translation
    cloud_np = np.transpose(cloud_np)[:3, :]
    cloud_tf = np.matmul(r, cloud_np) + np.array([[float(v.x)], [float(v.y)], [float(v.z)]]) 
    return np.transpose(cloud_tf)


class LaserStaticMapFilter(Node):

    def __init__(self):
        super().__init__('laser_static_map_filter')
        self._scan_sub = self.create_subscription(PointCloud2, "/pcl", self._scan_callback_2, 1)

    def _scan_callback_2(self, scan: PointCloud2):
        # fix corrupted pointcloud
        # w = copy.deepcopy(scan.fields[2])
        # w.name = "intensity"
        # w.offset = 12
        # scan.fields.append(w)

        # points_np = read_points_numpy(scan, skip_nans=True, reshape_organized_cloud=False)
        points = read_points(scan)
        for point in points: print(point)
        return
        points_np = np.array([[point[0], point[1], point[2]] for point in points])
        # guard against empty pointclouds
        if len(np.shape(points_np)) < 2: return
        # pointcloud.header.frame_id -> self._ogm.reference_frame
        points_np_tf = do_transform_cloud(copy.deepcopy(points_np), self._current_transform.transform)
        points_f  = list()

        for idx in range(max(np.shape(points_np))):
            if self._filter(points_np_tf[idx]):
                # this makes sure that points_f is in the same frame as the original pointcloud
                points_f.append([points_np[idx, 0], points_np[idx, 1], points_np[idx, 2]])
        points_np_f = np.array(points_f)
        scan_f: PointCloud2 = create_cloud_xyz32(header=scan.header, points=points_np_f)
        scan_f.header.frame_id = scan.header.frame_id
        self._filtered_pub.publish(scan_f)


def main(args=None):
    rclpy.init(args=args)

    map_filter = LaserStaticMapFilter()

    rclpy.spin(map_filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

