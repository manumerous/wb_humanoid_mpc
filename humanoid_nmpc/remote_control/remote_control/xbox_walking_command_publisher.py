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
import rclpy
import math
import subprocess
from rclpy.node import Node
from dataclasses import dataclass
from humanoid_mpc_msgs.msg import WalkingVelocityCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy


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


class XBoxWalkingCommandPublisher(Node):
    def __init__(self):
        super().__init__("xbox_walking_command_publisher")
        pygame.init()
        self.get_joystick_connection()

        self.max_vel_x = 1.0
        self.max_vel_y = 1.0
        self.max_vel_yaw = 1.0  # rad/s
        self.current_pelvis_height_target = 0.8
        self.min_pelvis_height = 0.2
        self.max_pelvis_height = 1.0

        self.bluetooth_connection = False

        self.publisher_rate = 25  # Hz

        # Create a QoS profile with Best Effort reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=25,  # Set the depth, which is the size of the message queue
        )

        self.publisher_ = self.create_publisher(
            WalkingVelocityCommand,
            "/humanoid/walking_velocity_command",
            qos_profile,
        )
        self.timer = self.create_timer(1 / self.publisher_rate, self.timer_callback)

    def get_joystick_connection(self):
        joystick_count = 0
        connection_counter = 0
        self.joystick = None
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
                self.joystick = joystick
                print("Joystick connected.")

                joystick_name = joystick.get_name()
                usb_devices = get_usb_devices()
                bluetooth_devices = get_bluetooth_devices()
                # Check if the joystick is in USB or Bluetooth device lists

                if any(joystick_name in device for device in bluetooth_devices):
                    print(f"Joystick is connected via Bluetooth.")
                    self.bluetooth_connection = True
                elif any(joystick_name in device for device in usb_devices):
                    print(f"Joystick is connected via USB.")
                    self.bluetooth_connection = False
                else:
                    print(f"Connection type of Joystick is unknown.")

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

        # Clip the values if they are below 0.1 in absolute value
        input.x_left = raw_x_left if abs(raw_x_left) >= 0.1 else 0
        input.y_left = raw_y_left if abs(raw_y_left) >= 0.1 else 0
        input.x_right = raw_x_right if abs(raw_x_right) >= 0.1 else 0
        input.y_right = raw_y_right if abs(raw_y_right) >= 0.1 else 0

        return input

    def get_walking_command_msg(self, input: ControllerInput):
        msg = WalkingVelocityCommand()

        msg.linear_velocity_x = input.x_left * self.max_vel_x
        msg.linear_velocity_y = input.y_left * self.max_vel_y
        msg.angular_velocity_z = input.y_right * self.max_vel_yaw

        # adapt pelvis height py maximum 4 cm per call, equals 1m per second with timer callback of 25Hz
        pelvis_height_vel = input.rt - input.lt
        self.current_pelvis_height_target += pelvis_height_vel / self.publisher_rate
        self.current_pelvis_height_target = clamp(
            self.current_pelvis_height_target,
            self.min_pelvis_height,
            self.max_pelvis_height,
        )
        msg.desired_pelvis_height = self.current_pelvis_height_target
        return msg

    def timer_callback(self):
        msg = self.get_walking_command_msg(self.get_joystick_inputs())
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = XBoxWalkingCommandPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
