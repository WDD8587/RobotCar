#!/usr/bin/env python3
"""
Vision module — tennis tracking + lane detection (from original car project)

Tennis: HoughCircles + HSV color filter → (x, y, radius)
Lane:   R-channel binary threshold + edge detection → lane center offset

Reuses the core algorithms from the original RaspberryCar project,
but wrapped as a class for the upgraded architecture.
"""

import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera


class VisionTracker:
    """Tennis/lane tracker using OpenCV on Raspberry Pi."""

    def __init__(self, resolution=(640, 480), framerate=30):
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.raw_capture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(
            self.raw_capture, format="bgr", use_video_port=True)
        self.mode = 'tennis'  # 'tennis' or 'lane'

        # Tennis tracking state
        self.x_mov_ave = resolution[0] / 2
        self.r_mov_ave = 35

    def detect_tennis(self, frame):
        """Detect tennis ball using HoughCircles + HSV color filter."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # HoughCircles
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=100, param2=30, minRadius=10, maxRadius=100
        )

        if circles is None:
            return (0, 0, 0)

        circles = np.round(circles[0, :]).astype("int")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        best_ratio = 0
        best_circle = None

        for (x, y, r) in circles:
            # Extract ROI around the circle
            x1 = max(0, x - r)
            y1 = max(0, y - r)
            x2 = min(frame.shape[1], x + r)
            y2 = min(frame.shape[0], y + r)
            roi = hsv[y1:y2, x1:x2]

            # Tennis ball color: H ~25-50, S > 50, V > 50
            mask = cv2.inRange(roi, (25, 50, 50), (50, 255, 255))
            ratio = np.sum(mask > 0) / (roi.shape[0] * roi.shape[1])

            if ratio > best_ratio and ratio > 0.15:
                best_ratio = ratio
                best_circle = (x, y, r)

        if best_circle and best_ratio > 0.15:
            x, y, r = best_circle
            # Moving average
            self.x_mov_ave = 0.4 * x + 0.6 * self.x_mov_ave
            self.r_mov_ave = 0.4 * r + 0.6 * self.r_mov_ave
            return (int(self.x_mov_ave), y, int(self.r_mov_ave))

        return (0, 0, 0)

    def process_frame(self):
        """Capture one frame and run detection."""
        try:
            raw = next(self.stream)
            frame = raw.array
            self.raw_capture.truncate(0)

            if self.mode == 'tennis':
                return self.detect_tennis(frame)
        except StopIteration:
            pass
        return None

    def close(self):
        self.camera.close()
