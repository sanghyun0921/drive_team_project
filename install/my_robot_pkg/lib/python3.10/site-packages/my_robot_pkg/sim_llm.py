import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from cv_bridge import CvBridge
import cv2
import google.generativeai as genai
import json
import time
from PIL import Image as PILImage
from rclpy.executors import MultiThreadedExecutor

# [중요] 여기에 본인의 API 키를 입력하세요
GEMINI_API_KEY = "AIzaSyA8clYsUoxDV2c0_TZ83g3dj6rWyCUSyjc"

# Gazebo 맵(turtlebot3_world) 기준 테스트 좌표
LOCATION_DB = {
    "Snack_Section": {"x": 1.5, "y": 0.5, "yaw": 0.0, "item": "Box"},
    "Beverage_Section": {"x": -1.0, "y": 2.0, "yaw": 1.57, "item": "Cola Can"},
}

class InspectionBot(Node):
    def __init__(self):
        super().__init__('inspection_bot')
        
        # [수정 1] 시뮬레이션 시간 사용 강제 설정 (Gazebo와 시간 동기화 필수)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.navigator = BasicNavigator()
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel('gemini-3-flash-preview')
        
        self.bridge = CvBridge()
        self.latest_cv_image = None
        
        # [수정 2] 시뮬레이션 카메라 토픽으로 변경 (/image_raw -> /camera/image_raw)
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            qos_profile_sensor_data
        )
        
        print("--- [Simulation] AI Inspection Robot Ready ---")

    def image_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    def capture_image(self):
        if self.latest_cv_image is not None:
            rgb_image = cv2.cvtColor(self.latest_cv_image, cv2.COLOR_BGR2RGB)
            return PILImage.fromarray(rgb_image)
        return None

    def run_inspection_mission(self, location_key):
        target_info = LOCATION_DB.get(location_key)
        if not target_info:
            print("존재하지 않는 위치입니다.")
            return

        print(f"🚀 {location_key} (으)로 이동하여 [{target_info['item']}] 검사 시작...")
        self.go_to_pose(target_info)

        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            print("✅ 현장 도착. 카메라 안정화 대기 중...")
            time.sleep(1.0)
            
            print("📸 이미지 캡처 및 AI 분석 요청...")
            pil_img = self.capture_image()
            
            if pil_img:
                if self.latest_cv_image is not None:
                    # 
                    cv2.imshow(f"Inspection View: {location_key}", self.latest_cv_image)
                    print("👀 팝업창으로 사진을 확인하세요. (아무 키나 누르면 닫힘)")
                    cv2.waitKey(0) 
                    cv2.destroyAllWindows()
                      
                result = self.analyze_image(pil_img, target_info['item'])
                print(f"\n📋 [검사 결과 보고서]")
                print(f" - 대상: {target_info['item']}")
                print(f" - 상태: {'🟢 정상' if result['is_correct'] else '🔴 불량'}")
                print(f" - AI 소견: {result['reasoning']}\n")
            else:
                print("❌ 카메라 이미지를 받아올 수 없습니다. (Gazebo 카메라 확인 필요)")
        else:
            print("⚠️ 이동 실패.")

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
            cleaned_text = response.text.strip().replace('```json', '').replace('```', '')
            return json.loads(cleaned_text)
        except Exception as e:
            print(f"AI Analysis Error: {e}")
            return {"is_correct": False, "reasoning": "Error in AI processing"}

    def go_to_pose(self, coords):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        # [수정 3] 시뮬레이션 시간(Sim Time)으로 타임스탬프 찍기
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = coords['x']
        goal.pose.position.y = coords['y']
        
        import math
        goal.pose.orientation.z = math.sin(coords['yaw'] / 2.0)
        goal.pose.orientation.w = math.cos(coords['yaw'] / 2.0)
        
        self.navigator.goToPose(goal)
        while not self.navigator.isTaskComplete():
            pass

def main():
    rclpy.init()
    bot = InspectionBot()

    executor = MultiThreadedExecutor()
    executor.add_node(bot)
    
    import threading
    spinner_thread = threading.Thread(target=executor.spin, daemon=True)
    spinner_thread.start()

    try:
        while True:
            loc = input("검사할 위치를 입력하세요 (Snack_Section, Beverage_Section, q=종료): ")
            if loc == 'q':
                break
            bot.run_inspection_mission(loc)
    finally:
        bot.destroy_node()
        rclpy.shutdown()
        spinner_thread.join()

if __name__ == '__main__':
    main()