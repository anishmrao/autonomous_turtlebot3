#!/usr/bin/env python3
# simplest frontier explorer (ROS 2 Humble)
# - subscribes /map (OccupancyGrid)
# - finds frontiers (free next to unknown)
# - clusters by 8-neigh BFS, picks a centroid
# - sends NavigateToPose in the 'map' frame

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

FREE, OCC, UNK = 0, 100, -1

class SimpleFrontierExplorer(Node):
    def __init__(self):
        super().__init__('simple_frontier_explorer')
        self.map_msg = None
        self.sent_goals = []       # recent (x, y) world coords
        self.current_goal = None

        # tiny set of knobs
        self.min_frontier_size = 20
        self.goal_sep_m = 0.8
        self.replan_period_s = 2.0
        # while navigating, allow periodic replanning to a better frontier
        self.replan_while_moving_period_s = 60.0  # seconds

        # goal selection mode: 'closest' or 'largest'
        self.declare_parameter('selection_mode', 'closest')

        # maximum size of a frontier cluster before we split it into subclusters
        # set via ROS parameter 'max_cluster_size'
        self.declare_parameter('max_cluster_size', 120)
        try:
            self.max_cluster_size = int(self.get_parameter('max_cluster_size').get_parameter_value().integer_value)
        except Exception:
            # fallback for older rclpy API access
            self.max_cluster_size = int(self.get_parameter('max_cluster_size').value)

        # scoring weights for goal selection
        # lower score is better: score = w_d * distance - w_s * size
        self.weight_distance = 1.0
        self.weight_size = 1.0

        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.create_timer(self.replan_period_s, self.tick)

        self.nav = BasicNavigator()  # assume Nav2 already active

        # track last time we (re)planned/sent a goal
        self.last_plan_time = self.get_clock().now()

        # one-time startup spin to improve initial perception
        self.startup_spin_started = False
        self.startup_spin_done = False
        self.startup_spin_angle = 2.0 * math.pi  # radians

        # RViz visualization publishers
        self.frontier_marker_pub = self.create_publisher(Marker, '/explorer/frontiers', 10)
        self.centroid_marker_pub = self.create_publisher(Marker, '/explorer/centroid', 10)

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def tick(self):
        if self.map_msg is None:
            return

        # perform a one-time slow spin on startup before frontier logic
        if not self.startup_spin_done:
            if not self.startup_spin_started:
                try:
                    self.get_logger().info('Starting initial spin...')
                    self.nav.spin(self.startup_spin_angle)
                    self.startup_spin_started = True
                    return
                except Exception as e:
                    self.get_logger().warn(f'Initial spin failed to start: {e}. Proceeding without spin.')
                    self.startup_spin_done = True
            else:
                # waiting for spin to complete
                if not self.nav.isTaskComplete():
                    return
                result = self.nav.getResult()
                self.get_logger().info(f'Initial spin result: {result}')
                self.startup_spin_done = True
                # fall through to frontier logic after spin completes

        # if navigating, allow periodic replanning instead of waiting to finish
        if self.current_goal is not None:
            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                self.get_logger().info(f'Goal result: {result}')
                self.current_goal = None
            else:
                now = self.get_clock().now()
                dt = (now - self.last_plan_time).nanoseconds * 1e-9
                if dt < self.replan_while_moving_period_s:
                    return
                # else fall-through to recompute a potentially better goal

        goal = self.pick_frontier_goal(self.map_msg)
        if goal is None:
            self.get_logger().info('No frontiers found.')
            return

        wx, wy = goal
        # if we already have an active goal, only send a new one if it meaningfully differs
        if self.current_goal is not None:
            cgx, cgy = self.current_goal
            if math.hypot(wx - cgx, wy - cgy) < 0.5:  # 0.5 m hysteresis
                # refresh last_plan_time to delay immediate replans
                self.last_plan_time = self.get_clock().now()
                return
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wx)
        ps.pose.position.y = float(wy)
        ps.pose.orientation.w = 1.0  # let Nav2 handle approach yaw

        self.get_logger().info(f'Navigating to frontier: ({wx:.2f}, {wy:.2f})')
        self.nav.goToPose(ps)
        self.current_goal = (wx, wy)
        self.sent_goals.append((wx, wy))
        if len(self.sent_goals) > 10:
            self.sent_goals.pop(0)
        self.last_plan_time = self.get_clock().now()

    # ---- frontier detection + simple selection ----
    def pick_frontier_goal(self, grid: OccupancyGrid):
        info = grid.info
        W, H, res = info.width, info.height, info.resolution
        data = grid.data
        if W == 0 or H == 0:
            return None

        def idx(x, y): return y * W + x
        def inb(x, y): return 0 <= x < W and 0 <= y < H
        def is_free(x, y): return data[idx(x, y)] == FREE
        def is_unk(x, y):  return data[idx(x, y)] == UNK
        def g2w(gx, gy):
            return (info.origin.position.x + (gx + 0.5) * res,
                    info.origin.position.y + (gy + 0.5) * res)

        # mark frontier cells: free with at least one 8-neigh unknown
        frontier = [[False]*W for _ in range(H)]
        for y in range(H):
            base = y * W
            for x in range(W):
                if data[base + x] != FREE:
                    continue
                # check 8 neighbors (including diagonals)
                neighs = [
                    (x-1, y-1), (x, y-1), (x+1, y-1),
                    (x-1, y),             (x+1, y),
                    (x-1, y+1), (x, y+1), (x+1, y+1),
                ]
                if any(inb(nx, ny) and is_unk(nx, ny) for nx, ny in neighs):
                    frontier[y][x] = True
        # Visualize frontier points
        self._publish_frontier_points(frontier, info, res)

        # cluster by BFS (8-neigh) and compute integer centroid and size
        visited = [[False]*W for _ in range(H)]
        clusters = []  # (cx, cy, size)
        for y in range(H):
            for x in range(W):
                if not frontier[y][x] or visited[y][x]:
                    continue
                q = [(x, y)]
                xs, ys = [], []
                # cap the cluster size directly during BFS; mark visited only when popping
                while q and len(xs) < self.max_cluster_size:
                    cx, cy = q.pop()
                    if visited[cy][cx] or not frontier[cy][cx]:
                        continue
                    visited[cy][cx] = True
                    xs.append(cx); ys.append(cy)
                    for nx, ny in (
                        (cx-1, cy-1), (cx, cy-1), (cx+1, cy-1),
                        (cx-1, cy),               (cx+1, cy),
                        (cx-1, cy+1), (cx, cy+1), (cx+1, cy+1),
                    ):
                        if inb(nx, ny) and frontier[ny][nx] and not visited[ny][nx]:
                            q.append((nx, ny))
                if len(xs) >= self.min_frontier_size:
                    # pick integer centroid of this capped cluster
                    cx_i = sum(xs)//len(xs)
                    cy_i = sum(ys)//len(ys)
                    clusters.append((cx_i, cy_i, len(xs)))

        if not clusters:
            return None

        # choose goal by mode: 'closest' (distance) or 'largest' (cluster size)
        mode_param = self.get_parameter('selection_mode')
        try:
            selection_mode = mode_param.get_parameter_value().string_value
        except Exception:
            selection_mode = 'closest'

        # reference point for distance computations (robot pose or map origin)
        ref_pose = None
        try:
            ref_pose = self.nav.getCurrentPose()
        except Exception:
            ref_pose = None
        if ref_pose and ref_pose.header.frame_id:
            refx = float(ref_pose.pose.position.x)
            refy = float(ref_pose.pose.position.y)
        else:
            self.get_logger().warn('No robot pose, using map origin as reference point')
            refx, refy = info.origin.position.x, info.origin.position.y

        # Visualize ALL cluster centroids (in world coords) and then choose a goal
        all_centroids_world = []
        for cx, cy, _sz in clusters:
            wx_all, wy_all = g2w(cx, cy)
            all_centroids_world.append((wx_all, wy_all))
        # Clear and publish all centroids every tick with distances
        self._publish_centroids(all_centroids_world, refx=refx, refy=refy)

        if selection_mode == 'largest':
            # pick the cluster with the largest size, with separation filter; tie-breaker: nearest
            candidates = []  # (-size, distance, wx, wy)
            for cx, cy, sz in clusters:
                wx, wy = g2w(cx, cy)
                if any(math.hypot(wx-gx, wy-gy) < self.goal_sep_m for gx, gy in self.sent_goals[-5:]):
                    continue
                d = math.hypot(wx - refx, wy - refy)
                candidates.append((-float(sz), d, wx, wy))
            if not candidates:
                return None
            candidates.sort(key=lambda t: (t[0], t[1]))
            wx, wy = candidates[0][2], candidates[0][3]
        else:
            # default: closest
            candidates = []  # (distance, wx, wy)
            for cx, cy, sz in clusters:
                wx, wy = g2w(cx, cy)
                if any(math.hypot(wx-gx, wy-gy) < self.goal_sep_m for gx, gy in self.sent_goals[-5:]):
                    continue
                d = math.hypot(wx - refx, wy - refy)
                candidates.append((d, wx, wy))
            if not candidates:
                return None
            candidates.sort(key=lambda t: t[0])
            wx, wy = candidates[0][1], candidates[0][2]

        return (wx, wy)

    # ---- visualization helpers ----
    def _publish_frontier_points(self, frontier, info, res):
        try:
            W, H = info.width, info.height
            def g2w(gx, gy):
                return (info.origin.position.x + (gx + 0.5) * res,
                        info.origin.position.y + (gy + 0.5) * res)

            # Clear previous frontier markers
            dm = Marker()
            dm.action = Marker.DELETEALL
            self.frontier_marker_pub.publish(dm)

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = 0
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0

            pts = []
            # budget = 10000
            # stride = max(1, int((W * H) / max(1, budget)))
            stride = 1
            count = 0
            for yy in range(0, H, 1):
                for xx in range(0, W, stride):
                    if frontier[yy][xx]:
                        wx, wy = g2w(xx, yy)
                        pts.append(Point(x=float(wx), y=float(wy), z=0.0))
                #         count += 1
                #         if count >= budget:
                #             break
                # if count >= budget:
                #     break
            m.points = pts
            self.frontier_marker_pub.publish(m)
        except Exception as e:
            self.get_logger().warn(f'Frontier marker publish failed: {e}')

    def _publish_centroids(self, centroids_world, refx=None, refy=None):
        """Publish all centroids as a SPHERE_LIST and distances as TEXT markers.
        Clears previous markers each tick.
        centroids_world: list of (wx, wy)
        refx, refy: reference position to compute distances (optional)
        """
        try:
            # Clear previous centroid markers
            dm = Marker()
            dm.action = Marker.DELETEALL
            self.centroid_marker_pub.publish(dm)

            # If no centroids, nothing else to publish
            if not centroids_world:
                return

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'centroids'
            m.id = 0
            m.type = Marker.SPHERE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.9
            m.points = [Point(x=float(wx), y=float(wy), z=0.0) for (wx, wy) in centroids_world]
            self.centroid_marker_pub.publish(m)

            # Optionally overlay distance labels for each centroid
            if refx is not None and refy is not None:
                for i, (wx, wy) in enumerate(centroids_world):
                    t = Marker()
                    t.header.frame_id = 'map'
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.ns = 'centroid_dist'
                    t.id = i
                    t.type = Marker.TEXT_VIEW_FACING
                    t.action = Marker.ADD
                    t.pose.position.x = float(wx)
                    t.pose.position.y = float(wy)
                    t.pose.position.z = 0.2
                    t.scale.z = 0.2  # text height
                    t.color.r = 1.0
                    t.color.g = 0.4
                    t.color.b = 0.0
                    t.color.a = 1.0
                    dist = math.hypot(float(wx) - refx, float(wy) - refy)
                    t.text = f"{dist:.2f} m"
                    self.centroid_marker_pub.publish(t)
        except Exception as e:
            self.get_logger().warn(f'Centroids marker publish failed: {e}')

def main():
    rclpy.init()
    rclpy.spin(SimpleFrontierExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
