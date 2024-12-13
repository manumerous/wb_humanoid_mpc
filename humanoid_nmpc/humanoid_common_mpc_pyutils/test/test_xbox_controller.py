"""****************************************************************************
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************"""

import pygame
import sys
import time


def init():

    pygame.init()
    joystick_count = 0
    connection_counter = 0
    joystick = None
    while joystick_count <= 0:
        pygame.joystick.quit()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count <= 0:
            connection_counter += 1
            if connection_counter > 10:
                print("No joystick detected.")
                connection_counter = 0
            time.sleep(0.5)
        else:
            # Initialize the first joystick
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Initialized joystick: {joystick.get_name()}")
            joystick = joystick
            print("Joystick connected.")

    return joystick


def get_joystick_inputs(joystick):
    pygame.event.pump()
    # Axes for the left and right sticks (Left stick: 0 left-right, 1 up-down. Right stick: 3 left-right, 4 up-down)
    x_left = -joystick.get_axis(1)
    y_left = -joystick.get_axis(0)
    x_right = -joystick.get_axis(3)
    y_right = -joystick.get_axis(2)
    # Triggers (LT and RT are often on axis 2, but this can vary)
    # Normalize LT to 0 (not pressed) to 1 (fully pressed)
    lt = (joystick.get_axis(6) + 1) / 2
    rt = (joystick.get_axis(5) + 1) / 2

    return x_left, y_left, x_right, y_right, lt, rt


def main():
    joystick = init()
    try:
        while True:
            x_left, y_left, x_right, y_right, lt, rt = get_joystick_inputs(joystick)
            print(f"Left Stick X: {x_left:.2f}, Y: {y_left:.2f}")
            print(f"Right Stick X: {x_right:.2f}, Y: {y_right:.2f}")
            print(f"LT: {lt:.2f}, RT: {rt:.2f}")
            pygame.time.wait(100)  # update every 100 milliseconds

    except KeyboardInterrupt:
        print("Exiting program.")
        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    main()
