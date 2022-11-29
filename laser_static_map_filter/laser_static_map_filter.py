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
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.srv import GetMap
from sensor_msgs.msg import PointCloud2
from tf2_geometry_msgs import PointStamped

from .occupancy_grid import OccupancyGridManager


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
        self._ogm = OccupancyGridManager()
        # subscribe to map topic
        self._map_sub = self.create_subscription(OccupancyGrid, "/map", self._ogm._occ_grid_cb, 1)
        self._map_updates_sub = self.create_subscription(OccupancyGridUpdate, "/map_updates", self._ogm._occ_grid_update_cb, 1)

        # request map
        self.map_client = self.create_client(GetMap, '/map_server/map')
        self.map_request = GetMap.Request()
        while not self.map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(message="Waiting for map service", throttle_duration_sec=3.0)
        self.send_request()

        self._scan_sub = self.create_subscription(PointCloud2, "/scan", self._scan_callback_2, 1)
        self._filtered_pub = self.create_publisher(PointCloud2, "/scan_filtered", 1)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node=self)
        self._point_tmp = PointStamped()
        self._current_transform = None

    def send_request(self):
        """
        The ROS2 mapserver only publishes the map once (at startup)
        Requests the map from the map server via the get_map service
        """
        future = self.map_client.call_async(self.map_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Got map via service call')
        else:
            self.get_logger().info('Could not get map via service call')
        self._ogm._occ_grid_cb(future.result().map)

    def _filter(self, point_np):
        try:
            cost = self._ogm.get_cost_from_world_x_y(point_np[0], point_np[1])
        except(IndexError):
            self.get_logger().warning(message="Received points outside of gridmap.", throttle_duration_sec=3.0)
            return False
        return cost >= 0 and cost < 100

    def _check(self, pointcloud):
        if self._ogm is None or self._ogm._occ_grid_metadata is None and self._ogm._grid_data is None:
            self.get_logger().warning(message="Did not receive map yet, cannot process scan.", throttle_duration_sec=3.0)
            return False
        try:
            target_frame = self._ogm.reference_frame
            source_frame = pointcloud.header.frame_id
            if source_frame[0] == "/": source_frame = source_frame[1:]
            if target_frame[0] == "/": target_frame = target_frame[1:]
            t = self._tf_buffer.lookup_transform(target_frame=target_frame, 
                                                 source_frame=source_frame, 
                                                 time=rclpy.time.Time())
            self._current_transform = t
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warning(message="TF from " + str(self._ogm.reference_frame)\
                + " to " + (pointcloud.header.frame_id) + " is not ready.", throttle_duration_sec=3.0)
            return False
        return True

    def _scan_callback_2(self, scan: PointCloud2):
        if not self._check(scan): return
        # fix corrupted pointcloud
        # w = copy.deepcopy(scan.fields[2])
        # w.name = "intensity"
        # w.offset = 12
        # scan.fields.append(w)


        # points_np = read_points_numpy(scan, skip_nans=True, reshape_organized_cloud=False)
        points = read_points(scan, skip_nans=True, reshape_organized_cloud=False)
        points_np = np.array([[point[0], point[1], point[2]] for point in points])
        # guard against empty pointclouds
        if len(np.shape(points_np)) < 2: return
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

