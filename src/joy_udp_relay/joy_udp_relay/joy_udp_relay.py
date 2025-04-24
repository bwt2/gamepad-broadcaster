import rclpy
import socket
from rclpy.node import Node
from sensor_msgs.msg import Joy
from .joy_indices import IndexButton, IndexAxis

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
AUTOREPEAT_RATE_HZ = 20

def hz_to_ns(hz: float) -> float:
    return 1/(hz*10e9)

class JoyUDPRelay(Node):
    def __init__(self):
        super().__init__('joy_udp_relay')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.autorepeat_interval_ns = hz_to_ns(AUTOREPEAT_RATE_HZ)
        self.last_pub_ = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(self.autorepeat_interval_ns * 10e9) # want 20hz
        self.get_logger().info(f"Joy UDP Relay binding to UDP {UDP_IP}:{UDP_PORT}")

        self.receive_loop()
        

    def receive_loop(self) -> None:
        self.get_logger().info("Joy UDP Relay successfully initialized")
        self.last_pub_ = self.get_clock().now()
        axes = [0.0]*6
        buttons = [0]*21

        while True:
            should_publish = False

            try:
                data, _ = self.sock.recvfrom(1024)
                should_publish = True
                axes = [0.0]*6
                buttons = [0]*21
                msg = data.decode()
                events = msg.split(" ")
                for event in events:
                    code, state = event.split(":")
                    match code:
                        case "BTN_EAST":
                            buttons[IndexButton.B.value] = int(state)
                        case "BTN_SOUTH":
                            buttons[IndexButton.A.value] = int(state)
                        case "BTN_WEST":
                            buttons[IndexButton.X.value] = int(state)
                        case "BTN_NORTH":
                            buttons[IndexButton.Y.value] = int(state)       
                        case "BTN_TR":
                            buttons[IndexButton.RIGHTSHOULDER.value] = int(state)                 
                        case "BTN_TL":
                            buttons[IndexButton.LEFTSHOULDER.value] = int(state)  
                        case "BTN_SELECT":
                            buttons[IndexButton.SELECT.value] = int(state)
                        case "BTN_START":
                            buttons[IndexButton.START.value] = int(state)                            
                        case "ABS_HAT0X":
                            state = int(state)
                            if state == -1:
                                buttons[IndexButton.DPAD_LEFT.value] = -state 
                            elif state == 1:
                                buttons[IndexButton.DPAD_RIGHT.value] = state                    
                        case "ABS_HAT0Y":
                            state = int(state)
                            if state == -1:
                                buttons[IndexButton.DPAD_UP.value] = -state        
                            elif state == 1:
                                buttons[IndexButton.DPAD_DOWN.value] = state         
                        case "ABS_Z":
                            axes[IndexAxis.TRIGGERLEFT.value] = float(state)/255.0
                        case "ABS_RZ":
                            axes[IndexAxis.TRIGGERRIGHT.value] = float(state)/255.0
                        case "ABS_X":
                            axes[IndexAxis.LEFTX.value] = float(state)/32767.0
                        case "ABS_Y":
                            axes[IndexAxis.LEFTY.value] = float(state)/32767.0
                        case "ABS_RX":
                            axes[IndexAxis.RIGHTX.value] = float(state)/32767.0
                        case "ABS_RY":
                            axes[IndexAxis.RIGHTY.value] = float(state)/32767.0
                        case "SYN_REPORT":
                            break
                        case _:
                            self.get_logger().warn(f"Unknown button press {code}:{state} ignored")
                
            except TimeoutError:
                should_publish = False # be explicit

            if not should_publish:
                now = self.get_clock().now()
                diff = now - self.last_pub_
                if diff.nanoseconds > self.autorepeat_interval_ns:
                    self.last_pub_ = now
                    should_publish = True

            if should_publish:
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