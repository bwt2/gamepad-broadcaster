import socket
import sys
from inputs import devices, GamePad
from colorama import init as init_colorama
from colorama import Fore
import argparse

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
DEBUG = False

def select_gamepad() -> GamePad:
    gamepads: list[GamePad] = [dev for dev in devices.gamepads]
    if len(gamepads) == 0:
        print(Fore.RED + "No connected gamepads found")
        exit()

    gamepad_choice: GamePad = gamepads[0]

    if len(gamepads) != 1:
        print("Multiple gamepads detected. " + Fore.YELLOW +  f"Please enter a gamepad number (1-{len(gamepads)}):")
        for gamepad_id, gamepad in enumerate(gamepads):
            print(f"({(gamepad_id+1):02d}) {gamepad}")
        
        choice: int = None
        while True:
            print("Select gamepad: ", end="")
            try:
                choice = int(input())
                if choice <= 0 or choice > len(gamepads):
                    print(Fore.RED + f"Enter a gamepad number between 1-{len(gamepads)}")
                    continue
                else:
                    gamepad_choice = gamepads[choice-1]
                    break
            except ValueError:
                print(Fore.RED + f"Enter a gamepad number between 1-{len(gamepads)}")
                continue

    print("Connected to gamepad: " + Fore.GREEN +  str(gamepad_choice))
    return gamepad_choice

def main():
    gamepad: GamePad = select_gamepad()

    print(Fore.YELLOW + "CTRL+C to shut down program")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    msg = ""
    while True:
        gamepad_events = gamepad.read()
        for event in gamepad_events:
            msg += f"{event.code}:{event.state} "
            
        if event.code == "SYN_REPORT":
            if DEBUG: print(f"SENDING | {msg} ")
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
            msg = ""

if __name__ == "__main__":
    init_colorama(autoreset=True)

    parser = argparse.ArgumentParser(
        description="Gamepad broadcaster - sends gamepad inputs over UDP to be received by ROS2 nodes with socket listeners",
        epilog="Source: https://github.com/bwt2/gamepad-ros2-broadcaster"
    )
    
    parser.add_argument(
        "-p", "--port",
        type=int,
        default=5005,
        help="UDP port to send data to (default: 5005)"
    )

    parser.add_argument(
        "-d", "--debug",
        action="store_true",
        help="turn on debug logs (default: off)"
    )
     
    args = parser.parse_args()

    DEBUG = args.debug
    UDP_PORT = args.port

    try:
        main()
    except KeyboardInterrupt:
        print(Fore.GREEN + "\nShutting down gracefully...")
        exit(130)