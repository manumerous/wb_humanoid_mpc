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


class LEDIndicatorGui(tk.Frame):
    def __init__(
        self, parent, label_text="LED", size=20, color_on="#4a90e2", color_off="#363636"
    ):
        super().__init__(parent)
        self.configure(bg="#2c2c2c")  # Match dark background
        self.color_on = color_on
        self.color_off = color_off

        # Create Label with matching style
        self.label = tk.Label(
            self,
            text=label_text,
            bg="#2c2c2c",
            fg="#ffffff",
            font=("Helvetica", 10),  # Match other controls' font
        )
        self.label.pack(side=tk.LEFT, padx=5)

        # Create Canvas for LED
        self.canvas = tk.Canvas(
            self, width=size, height=size, bg="#2c2c2c", highlightthickness=0
        )
        self.canvas.pack(side=tk.LEFT)

        # Create the LED circle with modern styling
        padding = size * 0.1
        self.led = self.canvas.create_oval(
            padding,
            padding,
            size - padding,
            size - padding,
            fill=self.color_off,
            outline="#4a90e2",  # Match button border color
            width=1,
        )

    def set_state(self, state):
        """Set LED state: True for on, False for off"""
        if state:
            # When on, use the modern blue color with a light border
            self.canvas.itemconfig(
                self.led, fill=self.color_on, outline="#5ca0f2"
            )  # Lighter blue for depth
        else:
            # When off, use the darker gray with standard border
            self.canvas.itemconfig(self.led, fill=self.color_off, outline="#4a90e2")
