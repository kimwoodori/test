import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from pymycobot.mycobot import MyCobot
import time

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.subscription = self.create_subscription(
            Int32, 'robot_command', self.command_callback, 10)
        self.suction_publisher = self.create_publisher(String, 'suction_command', 10)
        
        # 로봇 초기화
        self.mycobot = MyCobot('/dev/ttyACM2', 115200)  # 실제 포트를 환경에 맞게 수정
        self.get_logger().info("Robot Control Node Initialized")
        self.initialize_robot()

    def initialize_robot(self):
        """로봇 초기화"""
        self.mycobot.set_gripper_mode(0)
        self.mycobot.init_eletric_gripper()
        self.mycobot.send_angles([0, 0, 0, 0, 0, 0], 50)
        time.sleep(4)
        self.mycobot.set_gripper_state(0, 50)
        time.sleep(3)
        self.get_logger().info("로봇 초기화 완료")

    def command_callback(self, msg):
        """명령에 따른 작업 수행"""
        command = msg.data
        if command == 1:  # 작업 시작 명령
            self.execute_task()
        elif command == 0:  # 초기화 명령
            self.reset_robot()
        else:
            self.get_logger().warn(f"알 수 없는 명령: {command}")

    def execute_task(self):
        """작업 수행"""
        self.get_logger().info("작업 시작: 재료 이동 및 조리")
        self.material_handling()
        self.prepare_cooking()
        self.cooking_task()
        self.plate_handling('pick')  # 흡착 명령을 사용해 접시 잡기
        self.get_logger().info("작업 완료")

    def reset_robot(self):
        """초기 위치로 복귀"""
        self.get_logger().info("로봇 초기화 진행 중...")
        self.mycobot.send_angles([0, 0, 0, 0, 0, 0], 100)
        time.sleep(2)
        self.mycobot.set_gripper_state(0, 50)
        time.sleep(1)
        self.get_logger().info("로봇 초기화 완료")

    def plate_handling(self, action):
        """접시 흡착 및 해제"""
        if action == 'pick':  # 흡착 (접시 잡기)
            self.get_logger().info("흡착 명령 전달: Suction ON")
            # suction_node에 흡착 명령 전달
            msg = String()
            msg.data = 'Suction ON'  # 흡착 명령
            self.suction_publisher.publish(msg)  # suction_node에 명령 발행
            time.sleep(5)  # 흡착 완료까지 잠시 대기
            self.get_logger().info("접시 잡기 완료")
        elif action == 'place':  # 해제 (접시 놓기)
            self.get_logger().info("해제 명령 전달: Suction OFF")
            # suction_node에 해제 명령 전달
            msg = String()
            msg.data = 'Suction OFF'  # 해제 명령
            self.suction_publisher.publish(msg)  # suction_node에 명령 발행
            time.sleep(3)  # 해제 완료까지 잠시 대기
            self.get_logger().info("접시 놓기 완료")
        else:
            self.get_logger().warn(f"알 수 없는 액션: {action}")

    def material_handling(self):
        """재료 이동 및 준비"""
        self.mycobot.send_angles([89.64, 0.7, 4.3, -5.0, -91.14, -4.57], 60)
        time.sleep(3)
        self.get_logger().info("컨베이어 대기 지점 도착")
        self.mycobot.send_angles([88.85, 28.65, 54.49, -60.99, -89.91, -0.61], 20)
        time.sleep(4)
        self.get_logger().info("재료 그릇 잡기 위치 도착")
        self.mycobot.set_gripper_state(1, 50)
        time.sleep(3)
        self.get_logger().info("재료 그릇 잡기 완료")
        self.mycobot.send_angles([88.15, -31.46, 98.87, -47.98, -86.74, -0.79], 60)
        time.sleep(3)
        self.get_logger().info("재료를 팬 위로 이동")
        self.mycobot.send_angles([154.95, -15.38, 103.27, -50.88, -79.45, 150], 60)
        time.sleep(4)
        self.get_logger().info("재료 팬 위에 위치 완료")

    def prepare_cooking(self):
        """조리 준비"""
        self.mycobot.send_angles([154.86, -51.06, 96.59, -23.9, -80.77, 0], 50)
        time.sleep(3)
        self.get_logger().info("컵 복귀 준비")
        self.mycobot.send_angles([-90, 0, 0, 0, 90, 0], 100)
        time.sleep(3)  # 제자리 준비
        self.mycobot.send_angles([-80.85, -79.1, 63.89, 23.9, 39.55, -6.94], 20)
        time.sleep(6)
        self.get_logger().info("도착")
        self.mycobot.set_gripper_state(0, 50)
        time.sleep(3)
        self.get_logger().info("그리퍼 열기 완료")

    def cooking_task(self):
        """조리 작업"""
        self.mycobot.send_angles([0, 0, 0, 0, 107, 0], 100)
        time.sleep(3)
        self.get_logger().info("조리도구 준비 완료")
        self.mycobot.send_angles([-10.1, -57.65, -36.05, 102.26, 107.31, 1.4], 20)
        time.sleep(6)
        self.mycobot.set_gripper_state(1, 50)
        time.sleep(3)
        self.get_logger().info("조리도구 잡기 완료")
        self.mycobot.send_angles([-13.97, -14.06, -99.75, 108.1, 107.92, 0.7], 20)
        time.sleep(5)
        self.mycobot.send_angles([-4.21, -39.46, -89.91, 126.56, 106.69, 1.49], 10)
        time.sleep(2)
        self.get_logger().info("조리 완료")
        self.mycobot.set_gripper_state(0, 50)
        time.sleep(3)  # 빼야할 부분 << 이건 그냥 의미 없음

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Robot Control Node 종료...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
