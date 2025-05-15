import curses
import time
import logging
from boids import BoidSimulation
from braille_display import BrailleDisplay

# Can't use stdout print debugging with curses,
# so we use logs
logging.basicConfig(
    filename="debug.log",
    filemode="w",
    level=logging.DEBUG,
    format="%(asctime)s - %(levelname)s - %(message)s"
)


FPS = 30
NUM_BOIDS = 200
DEBUG = False

def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)

    # Color pair constants
    COLOR_GOOD = 1
    COLOR_BAD = 2
    curses.start_color()
    curses.init_pair(COLOR_GOOD, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(COLOR_BAD, curses.COLOR_RED, curses.COLOR_BLACK)
    HAWK_COLOR = 3  # any available number
    curses.init_pair(HAWK_COLOR, curses.COLOR_MAGENTA, curses.COLOR_BLACK)

    max_y, max_x = stdscr.getmaxyx()
    max_x -= 1
    sim_width = max_x * 2       # Each Braille char = 2 horizontal pixels
    sim_height = max_y * 4      # Each Braille char = 4 vertical pixels

    sim = BoidSimulation(
        width=sim_width, height=sim_height, num_boids=NUM_BOIDS)
    display = BrailleDisplay(width=sim_width, height=sim_height)

    frame_duration = 1 / FPS
    while True:
        frame_start = time.perf_counter()

        sim.update()
        display.clear()
        display.plot_boids(sim.positions)
        display.render(stdscr, sim.hawks)

        # Calculate raw frame processing time (before sleep)
        process_time = time.perf_counter() - frame_start
        if DEBUG:
            # Draw potential-FPS counter
            potential_fps = 1.0 / process_time if process_time > 0 else 999.0

            # Draw FPS overlay
            fps_str = f"{potential_fps:5.0f} FPS"
            color = COLOR_GOOD if potential_fps >= FPS else COLOR_BAD
            stdscr.addstr(0, 0, fps_str, curses.color_pair(color))

        stdscr.refresh()

        # Sleep to maintain target frame rate
        remaining = frame_duration - process_time
        if remaining > 0:
            time.sleep(remaining)


if __name__ == "__main__":
    curses.wrapper(main)
