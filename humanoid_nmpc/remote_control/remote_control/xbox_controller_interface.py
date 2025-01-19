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
import time
import subprocess
from dataclasses import dataclass
from humanoid_mpc_msgs.msg import WalkingVelocityCommand


@dataclass
class ControllerInput:
    x_left: float = 0
    y_left: float = 0
    x_right: float = 0
    y_right: float = 0
    lt: int = 0
    rt: int = 0


def get_usb_devices():
    result = subprocess.run(["lsusb"], stdout=subprocess.PIPE)
    devices = result.stdout.decode("utf-8").split("\n")
    return devices


def get_bluetooth_devices():
    result = subprocess.run(["hcitool", "con"], stdout=subprocess.PIPE)
    devices = result.stdout.decode("utf-8").split("\n")
    return devices


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class XBoxControllerInterface:
    def __init__(self, publisher_rate):
        pygame.init()
        self.joystick_connected = False
        self.get_joystick_connection()

        self.current_pelvis_height_target = 0.8
        self.min_pelvis_height = 0.2
        self.max_pelvis_height = 1.0
        self.exp = 1.5

        self.publisher_rate = publisher_rate

        self.bluetooth_connection = False

    def get_joystick_connection(self):
        pygame.joystick.quit()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()

        if joystick_count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            joystick_name = joystick.get_name()

            # Xbox controllers over Bluetooth typically have specific names
            print("joystick_name", joystick_name)
            self.bluetooth_connection = "Wireless Controller" in joystick_name
            self.joystick = joystick
            self.joystick_connected = True
            print(
                f"Connected to {joystick_name} via {'Bluetooth' if self.bluetooth_connection else 'USB'}"
            )
        else:
            self.joystick_connected = False

    def get_joystick_inputs(self):
        joystick_count = pygame.joystick.get_count()
        if joystick_count < 1:
            self.get_joystick_connection()
        pygame.event.pump()

        input = ControllerInput()

        if self.bluetooth_connection:
            raw_x_left = -self.joystick.get_axis(1)
            raw_y_left = -self.joystick.get_axis(0)
            raw_x_right = -self.joystick.get_axis(3)
            raw_y_right = -self.joystick.get_axis(2)
            # Triggers (LT and RT are often on axis 2, but this can vary)
            # Normalize LT to 0 (not pressed) to 1 (fully pressed)
            input.lt = (self.joystick.get_axis(6) + 1) / 2
            input.rt = (self.joystick.get_axis(5) + 1) / 2

        else:
            # Settings for wired controller
            # Read joystick axes and invert the values as necessary
            raw_x_left = -self.joystick.get_axis(1)
            raw_y_left = -self.joystick.get_axis(0)
            raw_x_right = -self.joystick.get_axis(4)
            raw_y_right = -self.joystick.get_axis(3)

            # Normalize LT to 0 (not pressed) to 1 (fully pressed)
            input.lt = (self.joystick.get_axis(2) + 1) / 2
            input.rt = (self.joystick.get_axis(5) + 1) / 2

        # exponential scaling
        raw_x_left = 0.2 * raw_x_left + 0.8 * raw_x_left**3
        raw_y_left = 0.2 * raw_y_left + 0.8 * raw_y_left**3
        raw_x_right = 0.2 * raw_x_right + 0.8 * raw_x_right**3
        raw_y_right = 0.2 * raw_y_right + 0.8 * raw_y_right**3

        # Clip the values if they are below 0.1 in absolute value
        input.x_left = raw_x_left if abs(raw_x_left) >= 0.02 else 0.0
        input.y_left = raw_y_left if abs(raw_y_left) >= 0.02 else 0.0
        input.x_right = raw_x_right if abs(raw_x_right) >= 0.02 else 0.0
        input.y_right = raw_y_right if abs(raw_y_right) >= 0.02 else 0.0

        return input

    def get_walking_command_msg(self):
        if self.joystick_connected:
            try:
                input = self.get_joystick_inputs()
                msg = WalkingVelocityCommand()

                # Setting normalized inputs
                msg.linear_velocity_x = input.x_left
                msg.linear_velocity_y = input.y_left
                msg.angular_velocity_z = input.y_right

                # adapt pelvis height py maximum 4 cm per call, equals 1m per second with timer callback of 25Hz
                pelvis_height_vel = input.rt - input.lt
                self.current_pelvis_height_target += (
                    pelvis_height_vel / self.publisher_rate
                )
                self.current_pelvis_height_target = clamp(
                    self.current_pelvis_height_target,
                    self.min_pelvis_height,
                    self.max_pelvis_height,
                )
                msg.desired_pelvis_height = self.current_pelvis_height_target
                return True, msg
            except:

                print(
                    "Lost Joystick Connection. Start to scan for connection in the background."
                )
                self.joystick_connected = False
                return False, None
        else:
            print("Could not read inputs since no Joystick is connected!")
            return False, None
