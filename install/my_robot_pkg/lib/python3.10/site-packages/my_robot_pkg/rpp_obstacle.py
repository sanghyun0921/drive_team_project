#rpp_obstacle.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import FollowPath
from sensor_msgs.msg import LaserScan
from math import sqrt, cos, sin
import heapq
import numpy as np

class AStarRPPPlanner(Node):
    def __init__(self):
        super().__init__('a_star_rpp_planner')

        # --- 설정 변수 ---
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # --- 상태 변수 ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.is_localized = False
        
        self.original_map_data = None    
        self.current_map_data = None    
        
        # 현재 경로 및 목표 (재탐색용)
        self.current_path_grid = []
        self.current_goal_grid = None
        
        # 현재 실행 중인 Action Goal Handle (취소용)
        self._goal_handle = None

        # --- Action Client (RPP Controller와 통신) ---
        # 'controller_server'가 제공하는 'follow_path' 액션 서버에 연결
        self._action_client = ActionClient(self, FollowPath, 'follow_path')

        # --- 퍼블리셔 & 서브스크라이버 ---
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # 초기 위치 전송 타이머
        self.create_timer(1.0, self.publish_initial_pose)
        self.initial_pose_published = False

        # [경로 재탐색 타이머] 0.2초마다 경로상 장애물 검사
        self.create_timer(0.2, self.check_and_replan)
        
        self.get_logger().info("A* + RPP Controller Node Started!")

    # ==========================================================
    # 1. 초기화 및 데이터 수신부
    # ==========================================================
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

    def scan_callback(self, msg):
        """
        라이다 데이터를 받아 내부 맵(current_map_data)에 장애물을 업데이트합니다.
        이 데이터는 A* 재탐색 시 사용됩니다.
        """
        if self.original_map_data is None: return

        # 매번 원본 맵에서 시작 (이전 프레임의 동적 장애물 잔상 제거)
        self.current_map_data = self.original_map_data.copy()

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # 로봇의 현재 Yaw 값을 모르면 정확한 매핑이 어렵지만, 
        # 간단한 회피를 위해 로봇 주변에 장애물이 있다고 가정하거나
        # TF를 사용해야 합니다. 여기서는 '장애물이 있다'는 사실 자체를 중요시하여
        # A*가 막힌 경로를 피하게 유도합니다. (간이 구현: 라이다 포인트 매핑)
        
        # *참고*: 실제로는 로봇의 Orientation(Yaw)을 반영해야 정확합니다.
        # 여기서는 간단히 로봇의 (x,y) 기준으로 scan 데이터를 뿌립니다.
        
        for i, dist in enumerate(msg.ranges):
            if dist < 0.1 or dist > 3.0: continue # 유효 거리

            # 로봇 좌표계 -> 맵 좌표계 (Yaw 고려 X, 실제 사용 시 TF 필요)
            # Yaw 없이 사용하면 로봇이 회전할 때 맵 상의 장애물이 엉뚱한 곳에 찍힐 수 있습니다.
            # 하지만 RPP가 지역 회피를 담당하므로, 여기서는 '경로 막힘 감지'용으로 씁니다.
            theta = angle_min + i * angle_increment
            # 로봇의 현재 위치 기준
            global_angle = self.current_yaw + theta
            ox = self.current_x + dist * cos(theta) # 실제로는 로봇의 Yaw를 더해야 함
            oy = self.current_y + dist * sin(theta)

            gx, gy = self.world_to_grid(ox, oy)
            
            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                # 맵에 장애물(100) 표시
                self.current_map_data[gy][gx] = 100 

    # ==========================================================
    # 2. 경로 계획 및 Action 전송부
    # ==========================================================
    def goal_callback(self, msg):
        self.get_logger().info(">>> Goal Received!")
        if self.original_map_data is None or not self.is_localized:
            self.get_logger().warn("Not ready (Map or Pose missing)")
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        start_grid = self.world_to_grid(self.current_x, self.current_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        self.current_goal_grid = goal_grid
        
        # 최초 경로 생성 및 전송
        self.plan_and_send_path(start_grid, goal_grid)

    def plan_and_send_path(self, start, goal):
        path_grid = self.a_star(start, goal)
        
        if path_grid:
            self.get_logger().info(f"Path Planned! Length: {len(path_grid)}")
            self.current_path_grid = path_grid

            # 1. nav_msgs/Path 메시지 생성
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

            # 2. Action Server(RPP)로 경로 전송
            self.send_path_action(nav_path)
        else:
            self.get_logger().error("Path Not Found! (Blocked by Map or Obstacle)")
            self.current_path_grid = []

    def send_path_action(self, path_msg):
        # 만약 이미 실행 중인 목표가 있다면 취소 요청 (새 경로로 덮어쓰기 위해)
        # (Nav2 Action Server는 보통 새 목표가 오면 이전 목표를 선점(Preempt)합니다.)
        
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        
        # [중요] YAML 파일에 정의된 Controller ID와 일치해야 함
        goal_msg.controller_id = 'FollowPath' 
        goal_msg.goal_checker_id = 'general_goal_checker'

        self.get_logger().info("Sending Path to RPP Controller...")
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by Controller Server :(')
            return
        
        # self.get_logger().info('Goal accepted by RPP Controller!')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # 결과 처리 (성공, 실패 등)
        pass

    def feedback_callback(self, feedback_msg):
        # RPP 컨트롤러로부터 주행 상태(남은 거리, 속도 등)를 받음
        # 필요 시 여기서 로깅하거나 특정 로직 수행 가능
        pass

    # ==========================================================
    # 3. 경로 재탐색 로직 (Dynamic Replanning)
    # ==========================================================
    def check_and_replan(self):
        """
        주기적으로 현재 경로 상에 새로운 장애물이 생겼는지 확인합니다.
        """
        if not self.current_path_grid or self.current_goal_grid is None:
            return

        is_blocked = False
        
        # 현재 경로의 포인트들을 검사
        # (너무 가까운 포인트는 로봇 자체일 수 있으므로 인덱스 5부터 검사 등 튜닝 가능)
        for i, (gx, gy) in enumerate(self.current_path_grid):
            # 로봇과 너무 가까운 점은 제외 (현재 위치가 장애물로 인식될 오류 방지)
            if i < 3: continue 

            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                # scan_callback에서 업데이트된 current_map_data 확인
                # 90 이상이면 장애물로 간주
                if self.current_map_data[gy][gx] > 90:
                    is_blocked = True
                    break
        
        if is_blocked:
            self.get_logger().warn("⚠️ Path Blocked by Dynamic Obstacle! Replanning...")
            
            # 현재 실행 중인 RPP 작업을 취소 (옵션: 새 목표 보내면 자동 선점되므로 생략 가능하나 명시적으로 처리)
            # 여기서는 바로 새 경로를 계산해서 덮어씌우는 방식을 사용
            
            start_grid = self.world_to_grid(self.current_x, self.current_y)
            self.plan_and_send_path(start_grid, self.current_goal_grid)


    def a_star(self, start, goal):
        # 8방향 이동 정의
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        close_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        oheap = []
        heapq.heappush(oheap, (f_score[start], start))

        # [기능 3] 무한 루프 방지용 반복 횟수 제한
        iterations = 0
        max_iterations = 10000 

        while oheap:
            iterations += 1
            if iterations > max_iterations:
                self.get_logger().warn("A* Max iterations reached!")
                return None

            current = heapq.heappop(oheap)[1]

            # 1. 목표 도착 확인
            if current == goal:
                return self.reconstruct_path(came_from, current, start)

            # [핵심 기능 1] Analytic Expansion (직선 주행 최적화)
            # 현재 노드에서 목표까지 장애물이 없다면 탐색을 즉시 종료하고 연결
            # (매번 하면 느리므로 5번에 1번 정도 시도하거나, 거리가 가까울 때 시도)
            dist_to_goal = self.heuristic(current, goal)
            if dist_to_goal < 50.0:  # 목표가 가까울 때만 시도 (튜닝 가능)
                if self.is_line_of_sight_clear(current, goal):
                    self.get_logger().info("Analytic Expansion Triggered!")
                    came_from[goal] = current
                    return self.reconstruct_path(came_from, goal, start)

            close_set.add(current)

            for i, j in neighbors:
                neighbor = (current[0] + i, current[1] + j)

                # 맵 범위 체크
                if not (0 <= neighbor[0] < self.map_width and 0 <= neighbor[1] < self.map_height):
                    continue

                # 장애물 체크 (벽이면 통과 불가)
                if self.current_map_data[neighbor[1]][neighbor[0]] > 90:
                    continue

                if neighbor in close_set: continue

                # [핵심 기능 2] Cost-Aware Traversal (위험 비용 추가)
                # 기본 이동 비용 + 맵의 비용(위험도)을 반영
                base_cost = 1.414 if i != 0 and j != 0 else 1.0
                map_cost = self.current_map_data[neighbor[1]][neighbor[0]]
                
                # 비용 공식: 기본이동 + (맵비용 * 가중치)
                # 맵 비용이 높을수록(장애물 근처) g_score가 확 커져서 그 길을 피하게 됨
                traversal_cost = base_cost + (map_cost * 0.05) 

                tentative_g_score = g_score[current] + traversal_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (f_score[neighbor], neighbor))

        return None

    # 경로 복원 함수 (코드를 깔끔하게 분리)
    def reconstruct_path(self, came_from, current, start):
        data = []
        while current in came_from:
            data.append(current)
            current = came_from[current]
        data.append(start)
        return data[::-1]

    # [보조 함수] Bresenham 알고리즘을 이용한 시야(Line of Sight) 체크
    def is_line_of_sight_clear(self, start, end):
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if self.current_map_data[y][x] > 90: return False # 장애물 충돌
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if self.current_map_data[y][x] > 90: return False # 장애물 충돌
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        # 마지막 지점 확인
        if self.current_map_data[y][x] > 90: return False
        return True

    # 8방향 그리드에서는 유클리드보다 Octile 거리가 더 정확함 (선택 사항)
    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        # Octile Distance: 대각선으로 최대한 가고 나머지는 직선으로
        # F = (sqrt(2) - 1) * min(dx, dy) + max(dx, dy)
        return (0.414 * min(dx, dy) + max(dx, dy)) * 2.0 # 가중치 2.0 유지

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
    node = AStarRPPPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()