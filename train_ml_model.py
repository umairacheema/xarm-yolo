"""
train_ml_model.py
-------------

Description:
    This module is used to fine tune YOLOv8 model using toys dataset 

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
        python train_ml_model.py

Dependencies:
    roboticstoolbox

"""
from ultralytics import YOLO

# Load YOLO v8 model
model = YOLO("yolov8n.pt")

#Show model information
model.info()

# Fine Tune the model on toys dataset
train_results = model.train(
    data="data.yaml", 
    epochs=100,
    imgsz=640,
    device="cpu"
)
#Validate
model.val()

#Save the model
model.save('yolov8_finetuned.pt')
