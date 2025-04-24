from enum import Enum

class IndexButton(Enum):
    A, CROSS = 0, 0
    B, CIRCLE = 1, 1
    X, SQUARE = 2, 2
    Y, TRIANGLE = 3, 3
    BACK, SELECT = 4, 4
    GUIDE, MIDDLE = 5, 5
    START = 6
    LEFTSTICK = 7
    RIGHTSTICK = 8
    LEFTSHOULDER = 9
    RIGHTSHOULDER = 10
    DPAD_UP = 11
    DPAD_DOWN = 12
    DPAD_LEFT = 13
    DPAD_RIGHT = 14
    MISC1 = 15
    PADDLE1 = 16
    PADDLE2 = 17
    PADDLE3 = 18
    PADDLE4 = 19
    TOUCHPAD = 20

class IndexAxis(Enum):
    LEFTX = 0
    LEFTY = 1
    RIGHTX = 2
    RIGHTY = 3
    TRIGGERLEFT = 4
    TRIGGERRIGHT = 5