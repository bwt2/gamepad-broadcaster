# Gamepad ROS2 Broadcaster
## About
Tool to broadcast joystick inputs on Windows as [joy messahes](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy) to the `/joy` topic via UDP `127.0.0.1:5005`.
This was tested on an XBOX 360 controller.

```mermaid
flowchart LR
    gb["Gamepad Broadcaster"] -- "UDP" --> jur["Joy UDP Relay"] 
    jur -- "Joy messages" --> id["/joy topic"]
```

## Gamepad Broadcaster
Broadcast gamepad inputs from Windows via socket UDP. First, `cd .\src\gamepad_broadcaster\` and run

```powershell
# OPTIONAL: setup .venv
python gp_broadcaster.py
```

See the extra arguments can be used to specify the port and debugging level of the broadcaster using the `-h` flag.

```powershell
python gp_broadcaster.py -h
```
```powershell
usage: gp_broadcaster.py [-h] [-p PORT] [-d]

Gamepad broadcaster - sends gamepad inputs over UDP to be received by ROS2 nodes with socket listeners

options:
  -h, --help       show this help message and exit
  -p, --port PORT  UDP port to send data to (default: 5005)
  -d, --debug      turn on debug logs (default: off)

Source: https://github.com/bwt2/gamepad-ros2-broadcaster
```

You can test if broadcaster works using

```powershell
python .\test\gp_listener.py
```

## Joy UDP Relay
This node is the interface transforming joystick states from UDP `127.0.0.1:5005` to the `/joy` topic.


```bash
# source your ros2 installation here
colcon build --symlink-install
source install/setup.bash
ros2 run joy_udp_relay start
```