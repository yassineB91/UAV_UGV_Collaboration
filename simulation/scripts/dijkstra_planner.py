#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from queue import PriorityQueue

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid, Path

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose


class GraphNode:
    def __init__(self, x: int, y: int, cost: int = 0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return isinstance(other, GraphNode) and self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        dx, dy = other
        return GraphNode(self.x + dx, self.y + dy)


class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_node")

        # Declare parameters BEFORE reading them
        ##self.declare_parameter("use_sim_time", False)
        self.declare_parameter("base_frame", "robot1/base_footprint")
        self.declare_parameter("allow_unknown", False)

        self.get_logger().info(f"use_sim_time = {self.get_parameter('use_sim_time').value}")

        # ✅ Match your /map publisher (VOLATILE + RELIABLE)
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)

        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 10)

        self.map_: OccupancyGrid | None = None
        self.visited_map_ = OccupancyGrid()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)

    def wait_for_map(self) -> OccupancyGrid | None:
        if self.map_ is not None:
            return self.map_

        self.get_logger().info("Waiting for /map ...")
        i = 0
        while rclpy.ok() and self.map_ is None:
            i += 1
            if i % 50 == 0:
                self.get_logger().warn("Still waiting for /map ...")
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.map_

    def wait_for_transform(self, target_frame: str, source_frame: str):
        self.get_logger().info(f"Waiting for TF: {target_frame} <- {source_frame}")
        i = 0
        while rclpy.ok():
            try:
                t = rclpy.time.Time(clock_type=self.get_clock().clock_type)  # latest
                if self.tf_buffer.can_transform(target_frame, source_frame, t, timeout=Duration(seconds=0.2)):
                    return self.tf_buffer.lookup_transform(
                        target_frame, source_frame, t, timeout=Duration(seconds=0.2)
                    )
            except TransformException:
                pass

            i += 1
            if i % 50 == 0:
                self.get_logger().warn(f"Still waiting for TF: {target_frame} <- {source_frame}")
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def goal_callback(self, goal_msg: PoseStamped):
        if self.wait_for_map() is None:
            return

        allow_unknown = bool(self.get_parameter("allow_unknown").value)
        map_frame = self.map_.header.frame_id
        base_frame = str(self.get_parameter("base_frame").value)

        self.visited_map_.data = [-1] * (self.map_.info.height * self.map_.info.width)

        map_to_base_tf = self.wait_for_transform(map_frame, base_frame)
        if map_to_base_tf is None:
            return

        start_pose = Pose()
        start_pose.position.x = map_to_base_tf.transform.translation.x
        start_pose.position.y = map_to_base_tf.transform.translation.y
        start_pose.orientation = map_to_base_tf.transform.rotation

        goal_in_map = goal_msg
        if goal_msg.header.frame_id and goal_msg.header.frame_id != map_frame:
            tf_map_goal = self.wait_for_transform(map_frame, goal_msg.header.frame_id)
            if tf_map_goal is None:
                return
            goal_in_map = PoseStamped()
            goal_in_map.header.frame_id = map_frame
            goal_in_map.pose = do_transform_pose(goal_msg.pose, tf_map_goal)

        path = self.plan(start_pose, goal_in_map.pose, allow_unknown=allow_unknown)
        if path.poses:
            self.get_logger().info("Shortest path found.")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose, allow_unknown: bool = False) -> Path:
        explore_directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        if not self.pose_on_map(start_node) or not self.pose_on_map(goal_node):
            return Path(header=self._path_header())

        if not self.is_free(start_node, allow_unknown) or not self.is_free(goal_node, allow_unknown):
            return Path(header=self._path_header())

        pending_nodes = PriorityQueue()
        visited_nodes = set()

        pending_nodes.put(start_node)
        visited_nodes.add(start_node)

        active_node = None

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()
            if active_node == goal_node:
                break

            for dx, dy in explore_directions:
                new_node = active_node + (dx, dy)
                if not self.pose_on_map(new_node):
                    continue
                if new_node in visited_nodes:
                    continue
                if not self.is_free(new_node, allow_unknown):
                    continue

                new_node.cost = active_node.cost + 1
                new_node.prev = active_node
                pending_nodes.put(new_node)
                visited_nodes.add(new_node)

            self.visited_map_.data[self.pose_to_cell(active_node)] = 10
            self.map_pub.publish(self.visited_map_)

        if active_node is None or active_node != goal_node:
            return Path(header=self._path_header())

        path = Path()
        path.header = self._path_header()

        while active_node and active_node.prev and rclpy.ok():
            ps = PoseStamped()
            ps.header.frame_id = self.map_.header.frame_id
            ps.pose = self.grid_to_world(active_node)
            path.poses.append(ps)
            active_node = active_node.prev

        path.poses.reverse()
        return path

    def _path_header(self):
        h = Path().header
        h.frame_id = self.map_.header.frame_id
        return h

    def is_free(self, node: GraphNode, allow_unknown: bool) -> bool:
        occ = self.map_.data[self.pose_to_cell(node)]
        if occ == 0:
            return True
        if occ == -1:
            return allow_unknown
        return False

    def world_to_grid(self, pose: Pose) -> GraphNode:
        origin = self.map_.info.origin.position
        res = self.map_.info.resolution
        return GraphNode(
            int((pose.position.x - origin.x) / res),
            int((pose.position.y - origin.y) / res),
        )

    def grid_to_world(self, node: GraphNode) -> Pose:
        origin = self.map_.info.origin.position
        res = self.map_.info.resolution
        pose = Pose()
        pose.position.x = node.x * res + origin.x + 0.5 * res
        pose.position.y = node.y * res + origin.y + 0.5 * res
        pose.orientation.w = 1.0
        return pose

    def pose_on_map(self, node: GraphNode) -> bool:
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def pose_to_cell(self, node: GraphNode) -> int:
        return node.y * self.map_.info.width + node.x


def main():
    rclpy.init()
    node = DijkstraPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()