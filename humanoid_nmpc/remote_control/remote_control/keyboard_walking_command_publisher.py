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

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from humanoid_mpc_msgs.msg import WalkingVelocityCommand
import sys
import termios
import tty
import fcntl
import os


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class KeyboardWalkingCommandPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_walking_command_publisher")

        self.max_vel_x = 1.0
        self.max_vel_y = 1.0
        self.max_vel_yaw = 1.0  # rad/s

        self.x_vel = 0
        self.y_vel = 0
        self.yaw_vel = 0

        self.current_pelvis_height_target = 0.8
        self.min_pelvis_height = 0.2
        self.max_pelvis_height = 1.0
        self.delta_pelvis_height = 0.0

        # Create a QoS profile with Best Effort reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=25,  # Set the depth, which is the size of the message queue
        )

        self.publisher_ = self.create_publisher(
            WalkingVelocityCommand, "/humanoid/walking_velocity_command", qos_profile
        )
        self.publisher_rate = 25  # Hz
        self.timer = self.create_timer(1 / self.publisher_rate, self.timer_callback)

        # Set up the terminal for non-blocking input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Set non-blocking mode
        fcntl.fcntl(sys.stdin, fcntl.F_SETFL, os.O_NONBLOCK)

    def __del__(self):
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        try:
            return sys.stdin.read(1)
        except IOError:
            return None

    def flush_input(self):
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)

    def process_key(self, key):
        if key == "\x1b":
            next1, next2 = self.get_key(), self.get_key()
            if next1 == "[":
                if next2 == "A":  # Up arrow
                    self.x_vel = self.max_vel_x
                elif next2 == "B":  # Down arrow
                    self.x_vel = -self.max_vel_x
                if next2 == "C":  # Right arrow
                    self.y_vel = -self.max_vel_y
                elif next2 == "D":  # Left arrow
                    self.y_vel = self.max_vel_y
        if key == "a":
            self.yaw_vel = self.max_vel_yaw
        elif key == "d":
            self.yaw_vel = -self.max_vel_yaw
        if key == "w":
            self.delta_pelvis_height = 0.01
        elif key == "s":
            self.delta_pelvis_height = -0.01
        else:
            # Reset velocities when no recognized key is pressed
            self.x_vel = 0
            self.y_vel = 0
            self.yaw_vel = 0
            self.delta_pelvis_height = 0.0

    def get_walking_command_msg(self):
        msg = WalkingVelocityCommand()
        msg.linear_velocity_x = float(self.x_vel)
        msg.linear_velocity_y = float(self.y_vel)
        msg.angular_velocity_z = float(self.yaw_vel)

        self.current_pelvis_height_target += self.delta_pelvis_height
        self.current_pelvis_height_target = clamp(
            self.current_pelvis_height_target,
            self.min_pelvis_height,
            self.max_pelvis_height,
        )
        msg.desired_pelvis_height = self.current_pelvis_height_target
        return msg

    def timer_callback(self):
        key = self.get_key()
        if key:
            self.process_key(key)
            self.flush_input()  # Clear any buffered input
        else:
            # Reset velocities when no key is pressed
            self.x_vel = 0
            self.y_vel = 0
            self.yaw_vel = 0

        msg = self.get_walking_command_msg()
        self.publisher_.publish(msg)
        print(
            f"Command: x={self.x_vel:.2f}, y={self.y_vel:.2f}, yaw={self.yaw_vel:.2f}, stepping={'allowed' if self.allow_stepping else 'disallowed'}",
            end="\r",
        )


def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardWalkingCommandPublisher()

    try:
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
