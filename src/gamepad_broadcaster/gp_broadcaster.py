import argparse
import pygame
from inputs import UnpluggedError
from colorama import just_fix_windows_console, Fore

from gp_inputs import inputs_main
from gp_pygame import pygame_main

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

if __name__ == "__main__":
    just_fix_windows_console()

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

    parser.add_argument(
        "-i", "--inputs",
        action="store_true",
        help="use `inputs` instead of `pygame` to process gamepad inputs (default: off)"
    )
     
    args = parser.parse_args()
    DEBUG = args.debug
    UDP_PORT = args.port
    INPUTS = args.inputs

    if INPUTS:
        try:
            inputs_main(UDP_IP, UDP_PORT, DEBUG)
        except KeyboardInterrupt:
            print(Fore.GREEN + "\nShutting down gracefully..." + Fore.RESET)
            exit(130)
        except UnpluggedError as e:
            print(e)
            exit(1)
    else:
        try:
            pygame_main(UDP_IP, UDP_PORT, DEBUG)
        except KeyboardInterrupt:
            pygame.joystick.quit()
            print(Fore.GREEN + "\nShutting down gracefully..." + Fore.RESET)
            exit(130)
        finally:
            pygame.quit()