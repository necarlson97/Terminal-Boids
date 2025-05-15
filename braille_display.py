import curses
import logging

BRAILLE_OFFSET = 0x2800

# Map (x,y) pixel coords to Braille cell coords and bitmask
DOT_MAP = {
    (0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (0, 3): 0x40,
    (1, 0): 0x08, (1, 1): 0x10, (1, 2): 0x20, (1, 3): 0x80,
}

class BrailleDisplay:

    def __init__(self, width, height):
        self.width_chars = width // 2
        self.height_chars = height // 4
        self.grid = [
            [0 for _ in range(self.width_chars)]
            for _ in range(self.height_chars)
        ]

    def clear(self):
        for y in range(self.height_chars):
            for x in range(self.width_chars):
                self.grid[y][x] = 0

    def plot_boids(self, positions):
        for x, y in positions:
            cx, px = divmod(int(x), 2)
            cy, py = divmod(int(y), 4)
            if 0 <= cx < self.width_chars and 0 <= cy < self.height_chars:
                self.grid[cy][cx] |= DOT_MAP.get((px, py), 0)

    def render(self, stdscr, hawks=[]):
        for y in range(self.height_chars):
            for x in range(self.width_chars):
                code = BRAILLE_OFFSET + self.grid[y][x]
                char = chr(code)
                stdscr.addch(y, x, char)

        for hawk in hawks:
            x, y = int(hawk.pos[0] / 2), int(hawk.pos[1] / 4)
            if 0 <= y < self.height_chars and 0 <= x < self.width_chars:
                stdscr.addstr(y, x,
                    hawk.direction_char(), curses.color_pair(3))

        stdscr.refresh()
