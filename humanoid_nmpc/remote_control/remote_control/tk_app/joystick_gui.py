"""****************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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

import tkinter as tk
from math import sqrt


class JoystickGui(tk.Frame):
    def __init__(self, master, auto_center_var=False, fix_y_axis=False):
        super().__init__(master)
        self.configure(bg="#2c2c2c")  # Dark background

        # Create canvas with modern dark theme
        self.canvas = tk.Canvas(
            self,
            width=250,
            height=250,
            bg="#2c2c2c",
            highlightthickness=0,  # Remove border
        )
        self.canvas.pack(padx=10, pady=10)

        # Create the base circle
        self.base_x = 125
        self.base_y = 125
        self.base_radius = 100
        self.handle_radius = 10

        self.auto_center_var = auto_center_var
        self.fix_y_axis = fix_y_axis

        # Create the base (outer circle)
        self.base = self.canvas.create_oval(
            self.base_x - self.base_radius,
            self.base_y - self.base_radius,
            self.base_x + self.base_radius,
            self.base_y + self.base_radius,
            outline="#4a90e2",
            width=2,
            fill="#363636",  # Slightly lighter than background
        )

        if self.fix_y_axis:
            # Create the handle (inner circle)
            self.horizontal_indicator = self.canvas.create_rectangle(
                self.base_x - self.base_radius,
                self.base_y - 10,
                self.base_x + self.base_radius,
                self.base_y + 10,
                fill="#2c2c2c",  # Modern blue
                outline="#4a90e2",  # Lighter blue for depth
            )

        # Create the handle (inner circle)
        self.handle = self.canvas.create_oval(
            self.base_x - self.handle_radius,
            self.base_y - self.handle_radius,
            self.base_x + self.handle_radius,
            self.base_y + self.handle_radius,
            fill="#4a90e2",  # Modern blue
            outline="#5ca0f2",  # Lighter blue for depth
        )

        # Bind mouse events to canvas
        self.canvas.bind("<Button-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.drag)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drag)

        # Current position of handle
        self.current_x = self.base_x
        self.current_y = self.base_y

        # Initialize dragging state
        self.dragging = False

        self.x_norm = 0.0
        self.y_norm = 0.0

    def start_drag(self, event):
        dx = event.x - self.current_x
        dy = event.y - self.current_y
        if sqrt(dx * dx + dy * dy) <= self.handle_radius:
            self.dragging = True

    def drag(self, event):
        if not self.dragging:
            return

        dx = event.x - self.base_x
        dy = event.y - self.base_y
        distance = sqrt(dx * dx + dy * dy)

        if distance > self.base_radius:
            ratio = self.base_radius / distance
            dx *= ratio
            dy *= ratio

        new_x = self.base_x + dx
        if self.fix_y_axis:
            new_y = self.base_y
            dy = 0
        else:
            new_y = self.base_y + dy

        self.canvas.coords(
            self.handle,
            new_x - self.handle_radius,
            new_y - self.handle_radius,
            new_x + self.handle_radius,
            new_y + self.handle_radius,
        )

        self.current_x = new_x
        self.current_y = new_y

        self.x_norm = -dy / self.base_radius
        self.y_norm = -dx / self.base_radius

    def stop_drag(self, event):
        if self.auto_center_var.get():
            self.set_position()

    def set_position(self, x_norm=0.0, y_norm=0.0):
        self.dragging = False
        self.current_x = self.base_x - y_norm * self.base_radius
        self.current_y = self.base_y - x_norm * self.base_radius
        self.canvas.coords(
            self.handle,
            self.current_x - self.handle_radius,
            self.current_y - self.handle_radius,
            self.current_x + self.handle_radius,
            self.current_y + self.handle_radius,
        )
        self.x_norm = x_norm
        self.y_norm = y_norm
