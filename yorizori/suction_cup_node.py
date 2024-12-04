import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class SuctionControlNode(Node):
    def __init__(self):
        super().__init__('suction_control_node')
        
        # ROS2 publisher 설정
        self.publisher = self.create_publisher(String, 'suction_response', 10)
        
        # 소켓 서버 설정
        self.server_ip = '0.0.0.0'
        self.server_port = 9000
        self.get_logger().info("Starting server...")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.server_ip, self.server_port))
        self.server.listen(1)
        self.get_logger().info(f"Server listening on {self.server_ip}:{self.server_port}")

        self.client = None
        self.addr = None

    def start_server(self):
        try:
            while True:
                self.get_logger().info("Waiting for connection...")
                self.client, self.addr = self.server.accept()
                self.get_logger().info(f"Connection from {self.addr}")

                while True:
                    command = input("Enter command (Suction ON, Suction OFF, q: Quit): ")
                    if command == 'q':
                        self.get_logger().info("Shutting down server...")
                        return

                    if command in ['Suction ON', 'Suction OFF']:
                        self.client.send(command.encode())  # ESP32로 명령 전송
                        response = self.client.recv(1024).decode()  # ESP32 응답 수신
                        self.get_logger().info(f"ESP32 response: {response}")

                        # ROS2 토픽으로 응답 발행
                        msg = String()
                        msg.data = response
                        self.publisher.publish(msg)
                    else:
                        self.get_logger().warn("유효한 명령을 입력하세요: 'Suction ON', 'Suction OFF', 'q'.")
        except Exception as e:
            self.get_logger().error(f"Error occurred: {e}")
        finally:
            if self.client:
                self.client.close()
            self.server.close()
            self.get_logger().info("Server closed.")

def main(args=None):
    rclpy.init(args=args)
    node = SuctionControlNode()
    try:
        node.start_server()
    except KeyboardInterrupt:
        node.get_logger().info("Suction Control Node 종료...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
