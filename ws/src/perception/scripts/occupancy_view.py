"""Live 8x8 grid view of /chess/occupancy. Run alongside the sim launch."""
"""This is just a fun way of looking at the occupancy data, and is not used by any other code"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


GREEN = "\033[92m"
DIM = "\033[90m"
BOLD = "\033[1m"
RESET = "\033[0m"


class Viewer(Node):
    def __init__(self):
        super().__init__("occupancy_view")
        self.create_subscription(UInt8MultiArray, "/chess/occupancy", self._cb, 10)
        self._last = None
        self._count = 0
        self._t0 = time.monotonic()
        self.create_timer(0.1, self._render)

    def _cb(self, msg):
        self._last = list(msg.data)
        self._count += 1

    def _render(self):
        os.system("clear")
        elapsed = time.monotonic() - self._t0
        rate = self._count / elapsed if elapsed > 0 else 0.0
        print(f"{BOLD}/chess/occupancy{RESET}    "
              f"frames: {self._count}    rate: {rate:.1f} Hz    "
              f"topic: /chess/occupancy")
        print()
        if self._last is None:
            print(f"{DIM}waiting for first message...{RESET}")
            return
        d = self._last
        occ = sum(d)
        print(f"  occupied squares: {BOLD}{occ}{RESET} / 64")
        print()
        print("       a   b   c   d   e   f   g   h")
        print("     +---+---+---+---+---+---+---+---+")
        for r in range(7, -1, -1):
            row = []
            for f in range(8):
                if d[r * 8 + f]:
                    row.append(f"{GREEN}#{RESET}")
                else:
                    row.append(f"{DIM}.{RESET}")
            print(f"  {r+1}  | " + " | ".join(row) + " |")
            print("     +---+---+---+---+---+---+---+---+")


def main():
    rclpy.init()
    n = Viewer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
