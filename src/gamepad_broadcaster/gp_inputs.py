from inputs import devices, GamePad
from colorama import Fore
import socket

def select_gamepad() -> GamePad:
    gamepads: list[GamePad] = [dev for dev in devices.gamepads]
    if len(gamepads) == 0:
        print(Fore.RED + "No connected gamepads found" + Fore.RESET)
        exit()

    gamepad_choice: GamePad = gamepads[0]

    if len(gamepads) != 1:
        print("Multiple gamepads detected. " + Fore.YELLOW +  f"Please enter a gamepad number (1-{len(gamepads)}):" + Fore.RESET)
        for gamepad_id, gamepad in enumerate(gamepads):
            print(f"({(gamepad_id+1):02d}) {gamepad}")
        
        choice: int = None
        while True:
            print("Select gamepad: ", end="")
            try:
                choice = int(input())
                if choice <= 0 or choice > len(gamepads):
                    print(Fore.RED + f"Enter a gamepad number between 1-{len(gamepads)}" + Fore.RESET)
                    continue
                else:
                    gamepad_choice = gamepads[choice-1]
                    break
            except ValueError:
                print(Fore.RED + f"Enter a gamepad number between 1-{len(gamepads)}" + Fore.RESET)
                continue

    print("Connected to gamepad: " + Fore.GREEN +  str(gamepad_choice) + Fore.RESET)
    return gamepad_choice


def inputs_main(UDP_IP: str, UDP_PORT: str, DEBUG: bool):
    gamepad: GamePad = select_gamepad()

    print(Fore.YELLOW + "CTRL+C to shut down program" + Fore.RESET)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        gamepad_events = gamepad.read()
        for event in gamepad_events:
            msg = f"{event.code}:{event.state}"
            if DEBUG:
                if event.code == "SYN_REPORT": 
                    print(f"{event.code}:{event.state} ", end="\n")
                else:
                    print(f"{event.code}:{event.state} ", end="")
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))