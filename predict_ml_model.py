"""
predict_ml_model.py
-------------

Description:
    This is the inference module for the YOLOv8 fine tuned model

Author:
    Umair Cheema <cheemzgpt@gmail.com>

Version:
    1.0.0

License:
    Apache License 2.0 (https://www.apache.org/licenses/LICENSE-2.0)

Date Created:
    2024-12-15

Last Modified:
    2024-12-15

Python Version:
    3.8+

Usage:
    Can be run from the command line /Terminal /Shell
    Example:
        python predict_ml_model.py

Dependencies:

"""

import cv2
from ultralytics import YOLO

# Load the finetuned YOLOv8 model
model = YOLO("runs/detect/train7/weights/best.pt")

vc = cv2.VideoCapture(0)

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False

while vc.isOpened():
   
    success, frame = vc.read()
    
    if success:
        results = model(frame)
        annotated_result = results[0].plot()  
        cv2.imshow("Toy detection", annotated_result)
        class_labels = results[0].names  # Class labels (indices of the detected classes)
        bounding_boxes = results[0].boxes  # Bounding boxes (x_center, y_center, width, height)

        # Print the results
        print("Class Labels:", class_labels[0])
        print("Bounding Boxes:", bounding_boxes)
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

vc.release()
cv2.destroyAllWindows()
