import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient  
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path  
from nav2_msgs.action import FollowPath      
from sensor_msgs.msg import LaserScan
from math import sqrt, cos, sin, atan2, pi
import heapq
import numpy as np

class AStarMPPIPlanner(Node):
    def __init__(self):
        super().__init__('a_star_mppi_planner')  

        self.collision_distance = 0.3    
        self.scan_angle_width = 30      
        self.obstacle_inflation = 2        

        self.current_x = 0.0
        self.current_y = 0.0
        self.is_localized = False
       
        self.original_map_data = None    
        self.current_map_data = None    
       
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        self.scan_msg = None            
       
        self.current_path_grid = []
        self.current_goal_grid = None

        # --- Action Client ---
        self._action_client = ActionClient(self, FollowPath, 'follow_path')

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
       
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            ))

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.create_timer(1.0, self.publish_initial_pose)
        self.initial_pose_published = False

        self.create_timer(0.1, self.check_and_replan)
       
        self.get_logger().info("A* + MPPI Planner (Dynamic Re-planning) Started!")

    def check_and_replan(self):
        if not self.is_localized or self.current_map_data is None or \
           not self.current_path_grid or self.current_goal_grid is None:
            return

        is_blocked = False
        for gx, gy in self.current_path_grid:
            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                if self.current_map_data[gy][gx] > 90:
                    is_blocked = True
                    break
       
        if is_blocked:
            self.get_logger().warn("Obstacle on path! Replanning...")
            start_grid = self.world_to_grid(self.current_x, self.current_y)
            self.plan_and_send_path(start_grid, self.current_goal_grid)

    def publish_initial_pose(self):
        if self.initial_pose_published: return
        if self.original_map_data is not None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = 0.0
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance = [0.1] * 36
            self.initial_pose_pub.publish(msg)
            self.get_logger().info("Sent /initialpose (0,0)")
            self.initial_pose_published = True

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.update_obstacle_map()

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.is_localized = True

    def map_callback(self, msg):
        self.get_logger().info(f"Map Received! Size: {msg.info.width}x{msg.info.height}")
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        raw_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.original_map_data = raw_data
        self.current_map_data = raw_data.copy()

    def update_obstacle_map(self):
        if self.scan_msg is None or self.current_map_data is None: return
       
        self.current_map_data = self.original_map_data.copy()

        ranges = np.array(self.scan_msg.ranges)
        angle_min = self.scan_msg.angle_min
        angle_increment = self.scan_msg.angle_increment
       
        for i, dist in enumerate(ranges):
            if dist < 0.1 or dist > 5.0: continue

            scan_angle = angle_min + (i * angle_increment)
           
            if -1.0 < scan_angle < 1.0:
                pass
               
        pass 

    def goal_callback(self, msg):
        self.get_logger().info("Goal Received!")
        if self.original_map_data is None:
            self.get_logger().warn("Map not ready yet!")
            return
        if not self.is_localized:
            self.get_logger().warn("Robot not localized!")
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.get_logger().info(f"New Goal: ({goal_x:.2f}, {goal_y:.2f})")

        start_grid = self.world_to_grid(self.current_x, self.current_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
       
        self.current_goal_grid = goal_grid
       
        self.plan_and_send_path(start_grid, goal_grid)

    def plan_and_send_path(self, start, goal):
        path_grid = self.a_star(start, goal)
       
        if path_grid:
            self.get_logger().info(f"Path Planned! Length: {len(path_grid)}")
            self.current_path_grid = path_grid

            nav_path = Path()
            nav_path.header.frame_id = 'map'
            nav_path.header.stamp = self.get_clock().now().to_msg()
           
            for gx, gy in path_grid:
                wx, wy = self.grid_to_world(gx, gy)
                pose = PoseStamped()
                pose.header = nav_path.header
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                nav_path.poses.append(pose)

            self.send_path_to_controller(nav_path)
        else:
            self.get_logger().error("Path Not Found! (Blocked)")
            self.current_path_grid = []

    def send_path_to_controller(self, path_msg):
       
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        pass 

    def feedback_callback(self, feedback_msg):
        pass 

    def a_star(self, start, goal):
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        close_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (f_score[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(start)
                return data[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = (current[0] + i, current[1] + j)
                if 0 <= neighbor[0] < self.map_width and 0 <= neighbor[1] < self.map_height:
                    if self.current_map_data[neighbor[1]][neighbor[0]] > 90 or self.current_map_data[neighbor[1]][neighbor[0]] == -1:
                        continue
                else:
                    continue
                if neighbor in close_set: continue
                move_cost = 1.414 if i != 0 and j != 0 else 1.0
                tentative_g_score = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (f_score[neighbor], neighbor))
        return None

    def heuristic(self, a, b):
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) * 1.1

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.map_origin_x) / self.map_resolution)
        gy = int((wy - self.map_origin_y) / self.map_resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        wx = gx * self.map_resolution + self.map_origin_x
        wy = gy * self.map_resolution + self.map_origin_y
        return (wx, wy)

def main(args=None):
    rclpy.init(args=args)
    node = AStarMPPIPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
