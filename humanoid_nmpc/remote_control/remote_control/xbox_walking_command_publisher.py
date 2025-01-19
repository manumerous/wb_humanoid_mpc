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
from rclpy.node import Node
from dataclasses import dataclass
from humanoid_mpc_msgs.msg import WalkingVelocityCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy
from remote_control import XBoxControllerInterface


class XBoxWalkingCommandPublisher(Node):
    def __init__(self):
        super().__init__("xbox_walking_command_publisher")

        self.publisher_rate = 25  # Hz

        self.xbox_controller_interface = XBoxControllerInterface(self.publisher_rate)

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

        self.counter = 0

    def timer_callback(self):
        if self.xbox_controller_interface.joystick_connected:
            success, msg = self.xbox_controller_interface.get_walking_command_msg()
            if success: self.publisher_.publish(msg)
        else:
            if self.counter >= (2 * self.publisher_rate):
                self.xbox_controller_interface.get_joystick_connection()
                self.counter = 0
            self.counter = self.counter + 1


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
