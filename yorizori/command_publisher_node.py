import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading

class CommandPublisherNode(Node):
    def __init__(self):
        super().__init__('command_publisher_node')
        self.publisher = self.create_publisher(Int32, 'robot_command', 10)
        self.get_logger().info('Command Publisher Node Started!')
        self.thread = threading.Thread(target=self.publish_command)
        self.thread.daemon = True
        self.thread.start()

    def publish_command(self):
        while rclpy.ok():  # 노드가 실행 중일 때 반복
            try:
                user_input = int(input("명령 입력 (1: 작업 시작, 0: 초기화): "))
                if user_input not in [0, 1]:
                    self.get_logger().warn("0 또는 1만 입력 가능합니다.")
                    continue

                msg = Int32()
                msg.data = user_input
                self.publisher.publish(msg)
                self.get_logger().info(f"명령 발행: {msg.data}")
            except ValueError:
                self.get_logger().warn("유효한 숫자를 입력하세요.")
            except KeyboardInterrupt:
                self.get_logger().info("종료 신호 수신. 노드를 종료합니다.")
                break

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisherNode()
    try:
        rclpy.spin(node)  # 노드가 실행 상태를 유지하도록 함
    except KeyboardInterrupt:
        node.get_logger().info("Command Publisher Node 종료...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
