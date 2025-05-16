#!/bin/bash

# Set a fallback font config if fbterm can't pick up the system one
export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

# Launch fbterm with a nice monospace font
# You can use 'NotoMono', 'DejaVu Sans Mono', etc. (run `fc-list :mono` to check installed monospace fonts)

FONT="Noto Mono"

fbterm -s 24 -- bash -c "python3 /home/pi/Terminal-Boids/main.py"
