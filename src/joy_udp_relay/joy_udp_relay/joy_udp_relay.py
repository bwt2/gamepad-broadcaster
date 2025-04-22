import rclpy
import socket
import threading

from rclpy.node import Node
from sensor_msgs.msg import Joy

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

class JoyUDPRelay(Node):

    def __init__(self):
        super().__init__('joy_udp_relay')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        threading.Thread(target=self.receive_loop, daemon=True).start()

    def receive_loop(self):
        axes = [0.0]*6
        buttons = [0]*12
        while True:
            data, _ = self.sock.recvfrom(1024)
            msg = data.decode()
            print(msg)
            code, state = msg.split(":")
            state = float(state)
            # Map event.code to index manually:
            if code == "ABS_X": axes[0] = state / 32767.0
            elif code == "ABS_Y": axes[1] = state / 32767.0
            elif code == "BTN_SOUTH": buttons[0] = int(state)
            # Add more mappings as needed

            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.axes = axes
            joy_msg.buttons = buttons
            self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)

    relay_node = JoyUDPRelay()
    rclpy.spin(relay_node)
    
    relay_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()