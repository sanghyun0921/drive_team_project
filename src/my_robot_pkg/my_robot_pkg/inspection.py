#inspection.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import google.generativeai as genai
import json
import time
import math
import os
from PIL import Image as PILImage
from rclpy.executors import MultiThreadedExecutor

GEMINI_API_KEY = "본인의 API 키"

# [중요] 실제 맵 좌표 (CustomDriver가 갈 수 있는 곳)
LOCATION_DB = {
    "Snack_Section": {"x": 1.5, "y": 0.5, "yaw": 0.0, "item": "Box"},
    "Beverage_Section": {"x": -1.0, "y": 2.0, "yaw": 1.57, "item": "Cola Can"},
}

class InspectionBot(Node):
    def __init__(self):
        super().__init__('mission_control_bot')
        
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-3-flash-preview')
        
        self.bridge = CvBridge()
        self.latest_cv_image = None
        
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile_sensor_data)
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        self.current_x = None
        self.current_y = None

        print("--- AI Inspector Ready ---")
        print("--- 별도의 터미널에서 a_star_rpp_planner2가 실행 중이어야 합니다 ---")

    def image_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Image fail: {e}")

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def capture_image(self):
        if self.latest_cv_image is not None:
            rgb_image = cv2.cvtColor(self.latest_cv_image, cv2.COLOR_BGR2RGB)
            return PILImage.fromarray(rgb_image)
        return None

    def send_goal_and_wait(self, coords):
        if self.current_x is None:
            print("로봇 위치(AMCL)를 아직 모릅니다. 잠시 대기...")
            time.sleep(2)
            if self.current_x is None:
                print("로봇 위치 수신 실패. Localization 확인 필요.")
                return False

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = coords['x']
        goal.pose.position.y = coords['y']
        goal.pose.orientation.z = math.sin(coords['yaw'] / 2.0)
        goal.pose.orientation.w = math.cos(coords['yaw'] / 2.0)

        print(f" CustomDriver에게 이동 지시 -> ({coords['x']}, {coords['y']})")
        self.goal_pub.publish(goal)

        print("이동 중... (도착 감시 중)")
        start_time = time.time()
        
        while True:
            if self.current_x is not None:
                dist = math.hypot(goal.pose.position.x - self.current_x, 
                                  goal.pose.position.y - self.current_y)
                
                if dist < 0.15:
                    print(f"도착 확인! (오차: {dist:.2f}m)")
                    return True
            
            time.sleep(0.5)

    def run_inspection_mission(self, location_key):
        target_info = LOCATION_DB.get(location_key)
        if not target_info:
            print("존재하지 않는 위치입니다.")
            return
        
        arrived = self.send_goal_and_wait(target_info)

        if arrived:
            print("카메라 안정화 대기 (1초)...")
            time.sleep(1.0)
            
            print("이미지 촬영 및 분석 중...")
            pil_img = self.capture_image()
         
            result = self.analyze_image(pil_img, target_info['item'])
            print(f"\n📋 [Gemini 분석 결과]")
            print(f" - 대상: {target_info['item']}")
            print(f" - 판정: {'🟢 정상' if result['is_correct'] else '🔴 불량'}")
            print(f" - 이유: {result['reasoning']}\n")

            else:
                print("카메라 오류: 이미지를 가져올 수 없습니다.")
        else:
            print("미션 실패: 로봇이 도착하지 못했습니다.")

    def analyze_image(self, image, item_name):
        prompt = f"""
        You are a warehouse inspection robot.
        Look at this image. I am checking if the '{item_name}' is placed correctly.
        
        Criteria for 'Correctly Placed':
        1. The item must be visible.
        2. The item must be standing upright (not fallen over).
        3. The item must be neatly aligned.
        
        Output format (JSON only):
        {{
            "is_correct": boolean,
            "reasoning": "Explain why it is correct or incorrect in 1 sentence."
        }}
        """
        try:
            response = self.model.generate_content([prompt, image])
            text = response.text.strip().replace('```json', '').replace('```', '')
            return json.loads(text)
        except:
            return {"is_correct": False, "reasoning": "AI Error"}

def main():
    rclpy.init()
    bot = MissionControlBot()

    executor = MultiThreadedExecutor()
    executor.add_node(bot)
    
    import threading
    spinner_thread = threading.Thread(target=executor.spin, daemon=True)
    spinner_thread.start()

    try:
        time.sleep(2) 
        
        while True:
            print("\n--- 검사 위치 선택 ---")
            for k in LOCATION_DB.keys(): print(f"- {k}")
            loc = input("입력 (q=종료): ")
            
            if loc == 'q': break
            
            bot.run_inspection_mission(loc)
            
    finally:
        bot.destroy_node()
        rclpy.shutdown()
        spinner_thread.join()

if __name__ == '__main__':
    main()