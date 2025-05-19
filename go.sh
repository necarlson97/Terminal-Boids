#!/bin/bash

# Wait until the framebuffer and tty1 exist
while [ ! -e /dev/fb0 ] || [ ! -e /dev/tty1 ]; do
  echo "Waiting for fb0 and tty1..."
  sleep 1
done

# Switch to tty1 (only works from root context or with systemd)
chvt 1

# Run the boids script in fbterm
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

exec fbterm -s 24 -- bash -c "cd /home/pi/Terminal-Boids && python3 main.py"
