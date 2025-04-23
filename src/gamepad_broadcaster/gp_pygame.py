import pygame
from colorama import Fore
import socket

def init_gamepad() -> pygame.joystick.JoystickType:
    pygame.joystick.init()

    # init gamepad to allows pygame to receive events from it
    joysticks: list[pygame.joystick.JoystickType] = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

    if len(joysticks) == 0:
        print(Fore.RED + "No connected gamepads found" + Fore.RESET)
        exit()

    joystick_choice = joysticks[0]

    if len(joysticks) != 1:
        print("Multiple gamepads detected. " + Fore.YELLOW +  f"Please enter a gamepad number (1-{len(joysticks)}):" + Fore.RESET)
        for joystick_id, joystick in enumerate(joysticks):
            print(f"({(joystick_id+1):02d}) {joystick.get_name()}")
        
        choice: int = None
        while True:
            print("Select gamepad: ", end="")
            try:
                choice = int(input())
                if choice <= 0 or choice > len(joysticks):
                    print(Fore.RED + f"Enter a gamepad number between 1-{len(joysticks)}" + Fore.RESET)
                    continue
                else:
                    joystick_choice = joysticks[choice-1]
                    break
            except ValueError:
                print(Fore.RED + f"Enter a gamepad number between 1-{len(joysticks)}" + Fore.RESET)
                continue

    print("Connected to gamepad: " + Fore.GREEN +  joystick_choice.get_name() + Fore.RESET)
    return joystick_choice


def pygame_main(UDP_IP: str, UDP_PORT: str, DEBUG: bool):
    pygame.init()
    gamepad: pygame.joystick.JoystickType = init_gamepad()
    print(Fore.YELLOW + "CTRL+C to shut down program" + Fore.RESET)
    sock: socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:

        for event in pygame.event.get():
            match event.type:
                case pygame.JOYAXISMOTION:
                    msg = f"AXIS:{event.axis}:{event.value:.4f}"
                case pygame.JOYBUTTONDOWN:
                    msg = f"BUTTON_DOWN:{event.button}:1"
                case pygame.JOYBUTTONUP:
                    msg = f"BUTTON_UP:{event.button}:0"
                case pygame.JOYHATMOTION:
                    msg = f"HAT:{event.hat}:{event.value}"
                case _:
                    continue

            if DEBUG:
                print(msg)
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))