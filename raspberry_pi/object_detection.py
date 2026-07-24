#!/usr/bin/env python3
"""
Object Detection module — TensorFlow Lite + MobileNet SSD / EfficientDet

Detects 90 COCO classes on Raspberry Pi 4B at 1-3 fps.

Two modes:
  1. COCO pre-trained: detects 90 classes out of the box, no training needed
  2. Custom trained:   transfer learning on your own dataset (e.g., "dock", "stain")

Setup (one-time):
  pip install tflite-runtime  # NOT tensorflow! tflite-runtime is 1/10 the size
  wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/
       coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
  unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
  → produces detect.tflite + labelmap.txt

Usage:
  from object_detection import ObjectDetector
  det = ObjectDetector('detect.tflite', 'labelmap.txt')
  results = det.detect(frame)  # returns list of (label, confidence, x, y, w, h)

Architecture note (interview):
  This is the same model class used in Ecovacs X2 / Dreame X50 for:
  - Pet waste detection (fine-tuned on custom dataset)
  - Cable/obstacle recognition
  - Furniture/room type classification
"""

import numpy as np
import time
from typing import List, Tuple, Optional

try:
    from tflite_runtime.interpreter import Interpreter
    HAS_TFLITE = True
except ImportError:
    try:
        from tensorflow.lite.python.interpreter import Interpreter
        HAS_TFLITE = True
    except ImportError:
        HAS_TFLITE = False
        print("[WARN] tflite-runtime not installed. Object detection disabled.")
        print("  Install: pip install tflite-runtime")

# COCO 90-class labels (MobileNet SSD was trained on this)
COCO_LABELS = [
    '???', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', '???',
    'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
    'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', '???',
    'backpack', 'umbrella', '???', '???', 'handbag', 'tie', 'suitcase',
    'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
    'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', '???', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', '???', 'dining table', '???', '???', 'toilet',
    '???', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', '???',
    'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
    'toothbrush'
]


class ObjectDetector:
    """TFLite object detector — runs on Raspberry Pi CPU."""

    def __init__(self, model_path: str, label_path: Optional[str] = None):
        if not HAS_TFLITE:
            raise RuntimeError("tflite_runtime not installed")

        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details  = self.interpreter.get_input_details()[0]
        self.output_details = self.interpreter.get_output_details()

        # Determine input size from model
        _, self.input_h, self.input_w, _ = self.input_details['shape']

        # Load labels
        self.labels = []
        if label_path:
            with open(label_path) as f:
                self.labels = [line.strip() for line in f.readlines()]
        else:
            self.labels = COCO_LABELS

        self.frame_count = 0
        self.total_time = 0

    def detect(self, frame: np.ndarray, threshold: float = 0.5) -> List[Tuple[str, float, int, int, int, int]]:
        """
        Run object detection on a BGR frame.

        Args:
            frame:      BGR image (480x640 typical from PiCamera)
            threshold:  minimum confidence to report (0.0-1.0)

        Returns:
            List of (label, confidence, x, y, w, h) for each detected object
            (x, y) is top-left corner, (w, h) is bounding box size
        """
        t_start = time.time()

        # Preprocess: resize to model input, normalize to [0,1]
        img = cv2.resize(frame, (self.input_w, self.input_h))
        img = np.expand_dims(img, axis=0).astype(np.float32) / 255.0

        # Run inference
        self.interpreter.set_tensor(self.input_details['index'], img)
        self.interpreter.invoke()

        # Parse output boxes
        # SSD MobileNet outputs: [batch, num_detections, 7]
        #   [image_id, label, score, xmin, ymin, xmax, ymax] × N
        boxes   = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        scores  = self.interpreter.get_tensor(self.output_details[2]['index'])[0]

        fh, fw, _ = frame.shape
        results = []

        for i in range(len(scores)):
            if scores[i] < threshold:
                continue
            class_id = int(classes[i])
            label = self.labels[class_id] if class_id < len(self.labels) else f'class_{class_id}'

            # SSD coordinates are normalized [0,1]
            ymin, xmin, ymax, xmax = boxes[i]
            x = int(xmin * fw)
            y = int(ymin * fh)
            w = int((xmax - xmin) * fw)
            h = int((ymax - ymin) * fh)

            results.append((label, float(scores[i]), x, y, w, h))

        # Track FPS
        self.frame_count += 1
        self.total_time += (time.time() - t_start)

        return results

    def draw_detections(self, frame: np.ndarray, results: List, color=(0, 255, 0)):
        """Draw bounding boxes and labels on frame."""
        for label, conf, x, y, w, h in results:
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = f"{label} {conf:.2f}"
            cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)
        return frame

    @property
    def fps(self):
        if self.total_time == 0:
            return 0
        return self.frame_count / self.total_time


# ============================================================================
# Training guide for custom objects (interview talking points)
# ============================================================================

"""
How to train for custom objects (e.g., "dock", "pet_waste", "cable"):

1. Collect ~200 images per class, annotate with LabelImg (VOC format)
2. Use TensorFlow Model Maker (zero-code transfer learning):

   from tflite_model_maker import object_detector
   data = object_detector.DataLoader.from_pascal_voc(
       'images/', 'annotations/', ['dock', 'cable', 'obstacle'])
   model = object_detector.create(data, model_spec='efficientdet_lite0')
   model.export(export_dir='./trained_model/')

3. Deploy: copy model.tflite to RPi, load with ObjectDetector('model.tflite')

4. On-device training NOT recommended for RPi — too slow. Train on PC, deploy.

Production note (interview):
  Dreame X50 series fine-tunes a MobileNet SSD on in-house datasets for:
  - Pet waste (reported by users as #1 request)
  - Socks / small objects (to avoid sucking them up)
  - Room type classification (living room vs kitchen → different cleaning strategies)
  Training happens on GPU servers. The TFLite model (~4MB) is deployed via OTA.
"""


if __name__ == '__main__':
    import cv2
    import sys

    if len(sys.argv) < 2:
        print("Usage: python3 object_detection.py detect.tflite [labelmap.txt]")
        print("Download model: wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/")
        print("                coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip")
        sys.exit(1)

    det = ObjectDetector(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = det.detect(frame, threshold=0.5)
        frame = det.draw_detections(frame, results)

        cv2.putText(frame, f"FPS: {det.fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
