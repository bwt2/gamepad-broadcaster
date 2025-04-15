import socket
import sys
from inputs import devices, GamePad
from colorama import init as init_colorama
from colorama import Fore

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

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
    print(Fore.YELLOW + "CTRL+C to shut down program")

    gamepad: GamePad = select_gamepad()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        gamepad_events = gamepad.read()
        for event in gamepad_events:
            msg = f"{event.code}:{event.state}"
            print(f"{event.code}:{event.state} ", end="")
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
            

if __name__ == "__main__":
    init_colorama(autoreset=True)

    if len(sys.argv) >= 4:
        print(Fore.RED + f"Too many arguments, python gp_broadcaster --p <port>")
        exit() 
        
    if len(sys.argv) == 3:
        if (sys.argv[1] == "--p"):
            try:
                UDP_PORT = int(sys.argv[2])
                print("Using UDP port: " + Fore.GREEN + str(UDP_PORT))
            except ValueError:
                print(Fore.RED + f"Invalid UDP port \"{sys.argv[2]}\" specified")
                exit()
    
    try:
        main()
    except KeyboardInterrupt:
        print(Fore.GREEN + "\nShutting down gracefully...")
        exit(130)