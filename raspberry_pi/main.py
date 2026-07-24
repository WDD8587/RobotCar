#!/usr/bin/env python3
"""
RobotCar — Main application on Raspberry Pi 4B

Three-thread architecture:
  Thread 1 (100 Hz): SPI bridge — motor control + telemetry
  Thread 2 (15 Hz):  Vision — OpenCV tennis/lane detection
  Thread 3:          Keyboard input

Usage: python3 main.py [--vision]
"""

import sys
import time
import threading
import signal
from spi_bridge import SpiBridge

# Vision module is optional (requires OpenCV)
try:
    from vision import VisionTracker
    HAS_VISION = True
except ImportError:
    HAS_VISION = False


class RobotCar:
    """Main robot controller."""

    WHEEL_BASE_MM = 150.0  # distance between wheels

    def __init__(self, enable_vision=True):
        self.bridge = SpiBridge()
        self.vision = None
        self.running = True
        self.vl = 0.0  # left wheel speed mm/s
        self.vr = 0.0  # right wheel speed mm/s

        if enable_vision and HAS_VISION:
            self.vision = VisionTracker()
            print("[Vision] Tennis/lane tracking enabled")
        elif enable_vision:
            print("[Vision] OpenCV not available, vision disabled")

    def set_velocity_arc(self, linear_mm_s: float, angular_deg_s: float):
        """Convert linear + angular velocity to differential wheel speeds."""
        # Angular velocity in rad/s
        w = angular_deg_s * 3.14159 / 180.0
        self.vl = linear_mm_s - w * self.WHEEL_BASE_MM / 2.0
        self.vr = linear_mm_s + w * self.WHEEL_BASE_MM / 2.0

    def spi_loop(self):
        """Thread 1: SPI bridge at 100 Hz."""
        print("[SPI] Bridge thread started (100 Hz)")
        while self.running:
            self.bridge.exchange(self.vl, self.vr)
            time.sleep(0.01)

    def vision_loop(self):
        """Thread 2: Vision at 15 Hz."""
        if not self.vision:
            return
        print("[Vision] Tracking thread started (15 Hz)")
        while self.running:
            result = self.vision.process_frame()
            if result:
                x, y, radius = result
                # Tennis at (x, y) with radius r
                # Simple proportional control: center tennis at x=320
                error_x = x - 320
                self.set_velocity_arc(
                    linear_mm_s=100.0 if radius < 30 else -50.0 if radius > 40 else 0,
                    angular_deg_s=-error_x * 0.5  # P controller for turning
                )
            time.sleep(0.066)  # 15 Hz

    def keyboard_loop(self):
        """Thread 3: Keyboard control."""
        print("[KB] WASD to move, Q to quit, SPACE to stop")
        import tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)

        try:
            while self.running:
                c = sys.stdin.read(1)
                if c == 'w':    self.set_velocity_arc(200, 0)
                elif c == 's':  self.set_velocity_arc(-200, 0)
                elif c == 'a':  self.set_velocity_arc(0, -90)
                elif c == 'd':  self.set_velocity_arc(0, 90)
                elif c == ' ':  self.set_velocity_arc(0, 0)
                elif c == 'q':  self.running = False
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, 'running', False))

        t_spi = threading.Thread(target=self.spi_loop, daemon=True)
        t_spi.start()

        if self.vision:
            t_vis = threading.Thread(target=self.vision_loop, daemon=True)
            t_vis.start()

        print("\nRobotCar ready. WASD to drive, Q to quit.\n")
        self.keyboard_loop()

        self.bridge.stop()
        self.bridge.close()
        print("RobotCar stopped.")


if __name__ == '__main__':
    rc = RobotCar(enable_vision='--vision' in sys.argv)
    rc.run()
