# Gamepad Broadcaster
Broadcast gamepad inputs via socket UDP. Tool to broadcast joystick inputs to ROS nodes listening to UDP `127.0.0.1:5005` on Windows.

```bash
python gp_broadcaster.py
```

You can optionally specify the UDP port using `--p` (default port `5005`). 

```bash
python gp_broadcaster.py --p 5555
```