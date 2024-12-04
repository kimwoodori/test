import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
class ESP32MotorControlNode(Node):
    def __init__(self):
        super().__init__('esp32_motor_control_node')

        # ROS2 Publisher 설정
        self.publisher = self.create_publisher(String, 'esp32_response', 10)

        # 소켓 서버 설정
        self.server_ip = '0.0.0.0'
        self.server_port = 10000

        self.get_logger().info("Starting server...")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 포트 재사용 설정
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
                    command = input("Enter command (1: Motor1 CW, 2: Motor1 CCW, 3: Motor2 CW, 4: Motor2 CCW, q: Quit): ")
                    if command == 'q':
                        self.get_logger().info("Shutting down server...")
                        return

                    if command in ['1', '2', '3', '4']:
                        self.client.send(command.encode())  # 명령 전송
                        response = self.client.recv(1024).decode()  # ESP32 응답 수신
                        self.get_logger().info(f"ESP32 response: {response}")

                        # ROS2 Topic으로 응답 Publish
                        msg = String()
                        msg.data = response
                        self.publisher.publish(msg)
                    else:
                        self.get_logger().warn("Invalid command! Please enter 1, 2, 3, 4, or q.")
        except Exception as e:
            self.get_logger().error(f"Error occurred: {e}")
        finally:
            if self.client:
                self.client.close()
            self.server.close()
            self.get_logger().info("Server closed.")

    def cleanup(self):
        if self.server:
            self.server.close()
        if self.client:
            self.client.close()
        self.get_logger().info("Cleaned up sockets.")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32MotorControlNode()

    try:
        node.start_server()
    except KeyboardInterrupt:
        node.get_logger().info("ESP32 Motor Control Node shutting down...")
    finally:
        node.cleanup()
        rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
