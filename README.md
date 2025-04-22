# Gamepad ROS2 Broadcaster
## About
Tool to broadcast joystick inputs on Windows to [ROS joy](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy) node via UDP `127.0.0.1:5005`.

```mermaid
flowchart LR
    id["Gamepad Broadcaster"] -- "UDP" --> id["Joy UDP Relay"] -- "Joy messages" --> id["ROS2 Joy Node"]
```

## Gamepad Broadcaster
Broadcast gamepad inputs from Windows via socket UDP. 

```powershell
cd .\src\gamepad_broadcaster\
python gp_broadcaster.py
```

You can optionally specify the UDP port using `--p` (default port `5005`). 

```powershell
python gp_broadcaster.py --p 5555
```

## Joy UDP Relay
This node is the interface transforming joystick states from UDP `127.0.0.1:5005` to joy messages for the ROS2 joy node.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run joy_udp_relay relay
```