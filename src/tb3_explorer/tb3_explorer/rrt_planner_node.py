#!/usr/bin/env python3
import math
import numpy as np
import random
from typing import List, Tuple, Dict, Optional
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

# Simple grid-based RRT with goal bias

class Node2D:
    def __init__(self, x: float, y: float, parent: Optional[int]):
        self.x = x
        self.y = y
        self.parent = parent


def world_to_grid(wx: float, wy: float, origin_x: float, origin_y: float, res: float) -> Tuple[int, int]:
    gx = int((wx - origin_x) / res)
    gy = int((wy - origin_y) / res)
    return gx, gy


def grid_to_world(gx: int, gy: int, origin_x: float, origin_y: float, res: float) -> Tuple[float, float]:
    wx = origin_x + (gx + 0.5) * res
    wy = origin_y + (gy + 0.5) * res
    return wx, wy



class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        # Planning parameters
        self.declare_parameter('step_size', 0.25)  # meters
        self.declare_parameter('max_iters', 4000)
        self.declare_parameter('goal_tolerance', 0.3)  # meters
        self.declare_parameter('goal_bias', 0.1)  # probability to sample goal
        self.declare_parameter('occ_threshold', 50)  # occupancy >= threshold is obstacle
        self.declare_parameter('inflate_radius', 0.15)  # meters, conservative safety
        self.declare_parameter('random_seed', 0)
        self.declare_parameter('single_shot', True)  # publish once per start/goal pair
        self.declare_parameter('planner_type', 'rrt_star')  # 'rrt' or 'rrt_star'
        self.declare_parameter('rrtstar_radius', 0.75)  # neighborhood radius (meters)

        self.step_size = float(self.get_parameter('step_size').value)
        self.max_iters = int(self.get_parameter('max_iters').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_bias = float(self.get_parameter('goal_bias').value)
        self.occ_threshold = int(self.get_parameter('occ_threshold').value)
        self.inflate_radius = float(self.get_parameter('inflate_radius').value)
        seed = int(self.get_parameter('random_seed').value)
        self.single_shot = bool(self.get_parameter('single_shot').value)
        self.planner_type = str(self.get_parameter('planner_type').value).lower()
        self.rrtstar_radius = float(self.get_parameter('rrtstar_radius').value)
        if seed:
            random.seed(seed)

        # State
        self.map_msg: Optional[OccupancyGrid] = None
        self.start_pose: Optional[PoseStamped] = None
        self.goal_pose: Optional[PoseStamped] = None
        self.plan_needed: bool = False
        self.last_start_xy: Optional[Tuple[float, float]] = None
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self.last_path_len: Optional[float] = None

        # I/O
        # Use Transient Local to receive latched map even if this node starts late
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'rrt_path', 10)
        self.tree_pub = self.create_publisher(MarkerArray, 'rrt_tree', 10)
        
        # Subscribe to RViz tools for setting start and goal
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self._on_goal, 10)
        self.create_subscription(PoseStamped, '/rrt_start', self._on_start, 10)
        self.create_subscription(PoseStamped, '/rrt_goal', self._on_goal, 10)
        self.get_logger().info('RRT planner ready - use RViz tools to set start/goal')

        # Timer to attempt planning whenever inputs available
        self.create_timer(0.5, self._tick)
        self.get_logger().info('RRT planner node ready. Use RViz: /initialpose + /goal_pose (or /move_base_simple/goal); or /rrt_start + /rrt_goal.')

    def _on_map(self, msg: OccupancyGrid):
        first_map = self.map_msg is None
        self.map_msg = msg
        if first_map:
            self.get_logger().info('Received map (first)')
            # Trigger a single plan if start/goal already present
            if self.start_pose is not None and self.goal_pose is not None:
                self.plan_needed = True

    def _on_start(self, msg: PoseStamped):
        self.start_pose = msg
        self.get_logger().info('Received start pose')
        self.plan_needed = True

    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        # Convert to PoseStamped for internal use
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.start_pose = ps
        self.get_logger().info('Received start pose from /initialpose')
        self.plan_needed = True


    def _on_goal(self, msg: PoseStamped):
        self.goal_pose = msg
        try:
            fx = float(msg.pose.position.x)
            fy = float(msg.pose.position.y)
            self.get_logger().info(f"Received goal pose on {msg._connection_header.get('topic', '/goal')} frame={msg.header.frame_id} (%.2f, %.2f)" % (fx, fy))
        except Exception:
            self.get_logger().info(f"Received goal pose on frame={msg.header.frame_id}")
        self.plan_needed = True

    def _tick(self):
        if self.map_msg is None:
            return
            
        # We need both start and goal poses to plan
        if self.start_pose is None or self.goal_pose is None:
            return

        # Only (re)plan when flagged
        if not self.plan_needed and self.single_shot:
            return

        start_xy = (float(self.start_pose.pose.position.x), float(self.start_pose.pose.position.y))
        goal_xy = (float(self.goal_pose.pose.position.x), float(self.goal_pose.pose.position.y))

        # If single_shot and start/goal unchanged, skip
        if self.single_shot and self.last_start_xy == start_xy and self.last_goal_xy == goal_xy and self.last_path_len is not None:
            self.plan_needed = False
            return

        if self.planner_type in ('rrt_star', 'rrt*'):
            path_pts = self.plan_rrt_star(self.map_msg, self.start_pose, self.goal_pose)
        else:
            path_pts = self.plan_rrt(self.map_msg, self.start_pose, self.goal_pose)
        if not path_pts:
            self.get_logger().warn('RRT failed to find a path.')
            self._publish_tree([])
            # Avoid spamming; wait for new goal/start/map
            self.plan_needed = False
            return

        # Compute path length in meters
        total_len = 0.0
        for i in range(1, len(path_pts)):
            x1, y1 = path_pts[i-1]
            x2, y2 = path_pts[i]
            total_len += math.hypot(x2 - x1, y2 - y1)

        # If not single_shot, publish only if improved by 5% over last length
        if not self.single_shot and self.last_path_len is not None:
            if total_len >= 0.95 * self.last_path_len:
                # Not an improvement; skip publishing to avoid RViz spam
                self.plan_needed = False
                return

        self._publish_path(path_pts)
        self.get_logger().info(f'Published path with {len(path_pts)} poses. Length: {total_len:.2f} m')

        # Save last plan footprint and reset flag
        self.last_start_xy = start_xy
        self.last_goal_xy = goal_xy
        self.last_path_len = total_len
        self.plan_needed = False

    # ------------------ RRT core ------------------
    def plan_rrt(self, grid: OccupancyGrid, start: PoseStamped, goal: PoseStamped) -> List[Tuple[float, float]]:
        self.get_logger().info('Planning with RRT')
        info = grid.info
        W, H, res = info.width, info.height, info.resolution
        origin_x, origin_y = info.origin.position.x, info.origin.position.y
        data = grid.data

        def inb(gx: int, gy: int) -> bool:
            return 0 <= gx < W and 0 <= gy < H

        inflate_cells = max(0, int(self.inflate_radius / res))

        def is_free_world(wx: float, wy: float) -> bool:
            gx, gy = world_to_grid(wx, wy, origin_x, origin_y, res)
            if not inb(gx, gy):
                return False
            return is_free_grid(gx, gy)

        def is_free_grid(gx: int, gy: int) -> bool:
            # obstacle inflation: check a square around the cell
            if not inb(gx, gy):
                return False
            for yy in range(gy - inflate_cells, gy + inflate_cells + 1):
                for xx in range(gx - inflate_cells, gx + inflate_cells + 1):
                    if not inb(xx, yy):
                        return False  # out of bounds is considered collision
                    occ = data[yy * W + xx]
                    if occ >= self.occ_threshold:
                        return False
            return True

        def collision_free(p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
            # sample along the segment at grid resolution/2
            wx1, wy1 = p1
            wx2, wy2 = p2
            dist = math.hypot(wx2 - wx1, wy2 - wy1)
            if dist < 1e-6:
                return True
            steps = max(2, int(dist / (res * 0.5)))
            for i in range(steps + 1):
                t = i / steps
                wx = wx1 + t * (wx2 - wx1)
                wy = wy1 + t * (wy2 - wy1)
                if not is_free_world(wx, wy):
                    return False
            return True

        # bounds
        minx, miny = origin_x, origin_y
        maxx, maxy = origin_x + W * res, origin_y + H * res

        start_xy = (float(start.pose.position.x), float(start.pose.position.y))
        goal_xy = (float(goal.pose.position.x), float(goal.pose.position.y))

        if not is_free_world(*start_xy):
            self.get_logger().warn('Start pose is in collision or out of bounds.')
            return []
        if not is_free_world(*goal_xy):
            self.get_logger().warn('Goal pose is in collision or out of bounds.')
            return []

        nodes: List[Node2D] = [Node2D(start_xy[0], start_xy[1], parent=None)]
        edges_for_viz: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []

        def nearest(wx: float, wy: float) -> int:
            best_i = 0
            best_d = float('inf')
            for i, n in enumerate(nodes):
                d = (n.x - wx) ** 2 + (n.y - wy) ** 2
                if d < best_d:
                    best_d = d
                    best_i = i
            return best_i

        def sample() -> Tuple[float, float]:
            if random.random() < self.goal_bias:
                return goal_xy
            return (
                random.uniform(minx, maxx),
                random.uniform(miny, maxy),
            )

        for it in range(self.max_iters):
            sx, sy = sample()
            ni = nearest(sx, sy)
            nx, ny = nodes[ni].x, nodes[ni].y

            # steer
            dx, dy = sx - nx, sy - ny
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            ux, uy = dx / d, dy / d
            newx = nx + ux * self.step_size
            newy = ny + uy * self.step_size

            # bounds check
            if newx < minx or newx > maxx or newy < miny or newy > maxy:
                continue
            if not collision_free((nx, ny), (newx, newy)):
                continue

            nodes.append(Node2D(newx, newy, parent=ni))
            edges_for_viz.append(((nx, ny), (newx, newy)))

            # goal check
            if math.hypot(newx - goal_xy[0], newy - goal_xy[1]) <= self.goal_tolerance:
                # connect to goal if collision-free
                if collision_free((newx, newy), goal_xy):
                    nodes.append(Node2D(goal_xy[0], goal_xy[1], parent=len(nodes) - 1))
                    path = self._reconstruct(nodes, len(nodes) - 1)
                    self._publish_tree(edges_for_viz)
                    return path

        # failed
        self._publish_tree(edges_for_viz)
        return []

    def plan_rrt_star(self, grid: OccupancyGrid, start: PoseStamped, goal: PoseStamped) -> List[Tuple[float, float]]:
        self.get_logger().info('Planning with RRT*')
        info = grid.info
        W, H, res = info.width, info.height, info.resolution
        origin_x, origin_y = info.origin.position.x, info.origin.position.y
        data = grid.data

        def inb(gx: int, gy: int) -> bool:
            return 0 <= gx < W and 0 <= gy < H

        inflate_cells = max(0, int(self.inflate_radius / res))

        def is_free_world(wx: float, wy: float) -> bool:
            gx, gy = world_to_grid(wx, wy, origin_x, origin_y, res)
            if not inb(gx, gy):
                return False
            return is_free_grid(gx, gy)

        def is_free_grid(gx: int, gy: int) -> bool:
            if not inb(gx, gy):
                return False
            for yy in range(gy - inflate_cells, gy + inflate_cells + 1):
                for xx in range(gx - inflate_cells, gx + inflate_cells + 1):
                    if not inb(xx, yy):
                        return False
                    occ = data[yy * W + xx]
                    if occ >= self.occ_threshold:
                        return False
            return True

        def collision_free(p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
            wx1, wy1 = p1
            wx2, wy2 = p2
            dist = math.hypot(wx2 - wx1, wy2 - wy1)
            if dist < 1e-6:
                return True
            steps = max(2, int(dist / (res * 0.5)))
            for i in range(steps + 1):
                t = i / steps
                wx = wx1 + t * (wx2 - wx1)
                wy = wy1 + t * (wy2 - wy1)
                if not is_free_world(wx, wy):
                    return False
            return True

        # bounds
        minx, miny = origin_x, origin_y
        maxx, maxy = origin_x + W * res, origin_y + H * res

        start_xy = (float(start.pose.position.x), float(start.pose.position.y))
        goal_xy = (float(goal.pose.position.x), float(goal.pose.position.y))

        if not is_free_world(*start_xy):
            self.get_logger().warn('Start pose is in collision or out of bounds.')
            return []
        if not is_free_world(*goal_xy):
            self.get_logger().warn('Goal pose is in collision or out of bounds.')
            return []

        nodes: List[Node2D] = [Node2D(start_xy[0], start_xy[1], parent=None)]
        cost: List[float] = [0.0]
        edges_for_viz: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []

        def nearest(wx: float, wy: float) -> int:
            best_i = 0
            best_d = float('inf')
            for i, n in enumerate(nodes):
                d = (n.x - wx) ** 2 + (n.y - wy) ** 2
                if d < best_d:
                    best_d = d
                    best_i = i
            return best_i

        def near_indices(wx: float, wy: float, radius: float) -> List[int]:
            res_list: List[int] = []
            r2 = radius * radius
            for i, n in enumerate(nodes):
                if (n.x - wx) ** 2 + (n.y - wy) ** 2 <= r2:
                    res_list.append(i)
            return res_list

        best_goal_idx: Optional[int] = None
        best_goal_cost = float('inf')

        for it in range(self.max_iters):
            # Sample with goal bias
            if random.random() < self.goal_bias:
                sx, sy = goal_xy
            else:
                sx = random.uniform(minx, maxx)
                sy = random.uniform(miny, maxy)

            ni = nearest(sx, sy)
            nx, ny = nodes[ni].x, nodes[ni].y

            # Steer toward sample
            dx, dy = sx - nx, sy - ny
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            ux, uy = dx / d, dy / d
            newx = nx + ux * self.step_size
            newy = ny + uy * self.step_size

            if newx < minx or newx > maxx or newy < miny or newy > maxy:
                continue
            if not collision_free((nx, ny), (newx, newy)):
                continue

            # Choose best parent among neighbors
            neigh = near_indices(newx, newy, self.rrtstar_radius)
            best_parent = ni
            best_new_cost = cost[ni] + math.hypot(newx - nx, newy - ny)
            for j in neigh:
                px, py = nodes[j].x, nodes[j].y
                if collision_free((px, py), (newx, newy)):
                    c = cost[j] + math.hypot(newx - px, newy - py)
                    if c < best_new_cost:
                        best_new_cost = c
                        best_parent = j

            nodes.append(Node2D(newx, newy, parent=best_parent))
            cost.append(best_new_cost)
            edges_for_viz.append(((nodes[best_parent].x, nodes[best_parent].y), (newx, newy)))
            new_idx = len(nodes) - 1

            # Rewire neighbors through new node if cheaper
            for j in neigh:
                if j == new_idx:
                    continue
                px, py = nodes[j].x, nodes[j].y
                new_cost_via_new = cost[new_idx] + math.hypot(px - newx, py - newy)
                if new_cost_via_new + 1e-6 < cost[j] and collision_free((newx, newy), (px, py)):
                    nodes[j].parent = new_idx
                    cost[j] = new_cost_via_new
                    edges_for_viz.append(((newx, newy), (px, py)))

            # Check goal connection and keep the best
            if math.hypot(newx - goal_xy[0], newy - goal_xy[1]) <= self.goal_tolerance:
                if collision_free((newx, newy), goal_xy):
                    goal_cost = cost[new_idx] + math.hypot(goal_xy[0] - newx, goal_xy[1] - newy)
                    if goal_cost < best_goal_cost:
                        # Add goal as a node with parent new_idx
                        nodes.append(Node2D(goal_xy[0], goal_xy[1], parent=new_idx))
                        cost.append(goal_cost)
                        best_goal_idx = len(nodes) - 1
                        best_goal_cost = goal_cost
                        edges_for_viz.append(((newx, newy), (goal_xy[0], goal_xy[1])))

        # Done: if a goal was found, return its path
        self._publish_tree(edges_for_viz)
        if best_goal_idx is not None:
            return self._reconstruct(nodes, best_goal_idx)
        return []

    def _reconstruct(self, nodes: List[Node2D], goal_idx: int) -> List[Tuple[float, float]]:
        path: List[Tuple[float, float]] = []
        i = goal_idx
        while i is not None:
            n = nodes[i]
            path.append((n.x, n.y))
            i = n.parent
        path.reverse()
        return path

    def _publish_path(self, path_pts):
        """Publish the planned path in the appropriate format for the current mode"""
        try:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            
            for i, (x, y) in enumerate(path_pts):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                
                # Calculate orientation to face the next point
                if i < len(path_pts) - 1:
                    next_x, next_y = path_pts[i + 1]
                    yaw = math.atan2(next_y - y, next_x - x)
                    q = quaternion_from_euler(0, 0, yaw)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                else:
                    # For the last point, keep the same orientation as previous
                    pose.pose.orientation.w = 1.0
                
                path_msg.poses.append(pose)
            
            # Always publish to the visualization topic
            self.path_pub.publish(path_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing path: {e}')

    def _publish_tree(self, edges: List[Tuple[Tuple[float, float], Tuple[float, float]]]):
        try:
            marker_array = MarkerArray()
            
            # Clear all previous markers
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'rrt_tree'
            marker.id = 0
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            self.tree_pub.publish(marker_array)
            
            # If no edges, just clear the markers
            if not edges:
                return
                
            # Create a new marker for the tree
            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'rrt_tree'
            marker.id = 1
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.01  # line width
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.8
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            
            # Add all edges to the marker
            for (x1, y1), (x2, y2) in edges:
                p1 = Point()
                p1.x = x1
                p1.y = y1
                p1.z = 0.0
                p2 = Point()
                p2.x = x2
                p2.y = y2
                p2.z = 0.0
                marker.points.append(p1)
                marker.points.append(p2)
            
            marker_array.markers.append(marker)
            self.tree_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().warn(f'Failed to publish RRT tree: {str(e)}')

def main():
    rclpy.init()
    rclpy.spin(RRTPlannerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
