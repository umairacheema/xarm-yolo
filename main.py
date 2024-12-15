"""
main.py
-------------

Description:
    This is the main module for the Forward Kinematics, Inverse Kinematics and 
    Object detection demonstration using xArm 1S robotic arm

Author:
    Umair Cheema <cheemzgpt@gmail.com>

Version:
    1.0.0

License:
    Apache License 2.0 (https://www.apache.org/licenses/LICENSE-2.0)

Date Created:
    2024-12-01

Last Modified:
    2024-12-15

Python Version:
    3.8+

Usage:
    Can be run from the command line /Terminal /Shell
    Example:
        python main.py

Dependencies:

"""


import cv2
import threading
import numpy as np
import roboticstoolbox as rtb
from ultralytics import YOLO
from xarm_model import XARM
from xarm_controller import XARMController
from roboticstoolbox.backends.PyPlot import PyPlot


#Global variable to check robot activity
is_robot_active = False

def main():
    #initialize connection with physical robot
    print(f'\nConnecting to the xArm over USB ...')
    xarm = XARMController()
    
    print(f'Moving physical robot to initial position ...')
    xarm.reset()
    pick_and_place_toy_kinematics(xarm)
    start_toy_detection_and_removal(xarm)


def start_toy_detection_and_removal(xarm):
    global is_robot_active
    # Load the finetuned YOLOv8 model
    model = YOLO("runs/detect/train7/weights/best.pt")
    # Open camera
    vc = cv2.VideoCapture(0)
    if vc.isOpened():
        success, frame = vc.read()
    else:
        success = False
    
    while vc.isOpened():
        success, frame = vc.read()
        if success:
            results = model(frame)
            annotated_result = results[0].plot()  
            cv2.imshow("Toy detection", annotated_result)
            class_labels = results[0].names  # Class labels (indices of the detected classes)
            bounding_boxes = results[0].boxes  # Bounding boxes (x_center, y_center, width, height)
            probabilities = results[0].probs
            # Print the results
            print("Class Labels:", class_labels[0])
            print("Bounding Boxes:", bounding_boxes)
            print("Probability:", probabilities)
            
            #If toy is detected pick and place in the box
            if class_labels[0] == 'toys':
                if is_robot_active == False:
                    #pick_and_place_toy(xarm)
                    prob = bounding_boxes.conf.detach().cpu().numpy()
                    if len(prob)>0:
                        prob = prob[0]
                        if prob >= 0.80:
                            thread = threading.Thread(target=pick_and_place_toy, args=(xarm,))
                            thread.start()
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Break the loop if stream ends
            break

    vc.release()
    cv2.destroyAllWindows()



def pick_and_place_toy(xarm):
    global is_robot_active 
    is_robot_active = True
    xarm.move_end_effector(-90)
    xarm.set_joint_state([0.25,-37.5,80.25,-5,-50])
    xarm.move_end_effector(110)
    xarm.set_joint_state([0.25,-7.5,10,-5,-50])
    xarm.set_joint_state([99.5,-43.25,5.5,-50,-49.75])
    xarm.move_end_effector(-90)
    xarm.reset()
    is_robot_active = False

def pick_and_place_toy_kinematics(xarm):
    global is_robot_active 
    is_robot_active = True
    #initialize simulation
    print(f'Launching robotics toolbox based environment ...')
    xarm_sim = XARM()
    env = PyPlot()
    env.launch()
    env.add(xarm_sim)
    print(f'Moving simulated robot to initial position ...')
    xarm_sim.q = physical_robot_to_model(xarm_sim.qz)
    env.step()
    #Create a list for forward kinematics SE3 objects
    Tfk = []
    print(f"\n\n Starting Forward Kinematics Demonstration")
    xarm_sim.q = physical_robot_to_model([0.25,-37.5,80.25,-5,-50])
    Tfk.append(xarm_sim.fkine(xarm_sim.q))
    env.step()
    xarm.move_end_effector(-90)
    xarm.set_joint_state([0.25,-37.5,80.25,-5,-50])
    xarm.move_end_effector(90)
    xarm_sim.q = physical_robot_to_model([0.25,-7.5,10,-5,-50])
    Tfk.append(xarm_sim.fkine(xarm_sim.q))
    env.step()
    xarm.set_joint_state([0.25,-7.5,10,-5,-50])
    xarm_sim.q = physical_robot_to_model([99.5,-43.25,5.5,-50,-49.75])
    Tfk.append(xarm_sim.fkine(xarm_sim.q))
    env.step()
    xarm.set_joint_state([99.5,-43.25,5.5,-50,-49.75])
    xarm.move_end_effector(-90)
    xarm_sim.q = physical_robot_to_model(xarm_sim.qz)
    Tfk.append(xarm_sim.fkine(xarm_sim.q))
    env.step()
    xarm.reset()
    print(f"\n\n Starting Inverse Kinematics Demonstration")
    solutions = []
    q0 = [0.,0.,0.,0.,0.]
    for Tep in Tfk:
        sol = xarm_sim.ikine_LM(Tep, ilimit=1000, slimit=1000, q0=q0)
        success = sol.success
        print(success)
        solutions.append(sol.q)
        q0 = sol.q
    env = PyPlot()
    env.launch()
    env.add(xarm_sim)
    for sol in solutions:
        xarm_sim.q = sol
        q = model_to_physical_robot(sol)
        env.step()
        xarm.set_joint_state(list(q))
        print(f"Inverse Kinematic Solution:")
        print(q)
    xarm.reset()
    is_robot_active = False


def sanitize_Teps(Tfk):
    sanitized = []
    for Tep in Tfk:
        print(type(Tep))
        empty = np.eye(4,dtype=float)
        empty[0,3] = Tep.t[0]
        empty[1,3] = Tep.t[1]
        empty[2,3] = Tep.t[2]
        sanitized.append(empty)
    return sanitized



def physical_robot_to_model(q):
    q = np.asarray(q).astype(float)
    q[0] += 90.0
    q[1] *= -1.0
    q = degree_to_radians(q)
    return q

def model_to_physical_robot(q):
    q = np.asarray(q).astype(float)
    q = radians_to_degrees(q)
    q[0] -= 90.0
    q[1] *= -1.0
    #Restrict the movement to joint limits
    for i,val in enumerate(q):
        if val > 90:
            q[i] = 90
        elif val <= -90:
            q[i] = -90
    return q


def degree_to_radians(angles):
    _angles = np.asarray(angles)
    _angles = np.deg2rad(_angles)
    return _angles

def radians_to_degrees(angles):
    _angles = np.asarray(angles)
    _angles = np.rad2deg(_angles)
    return _angles


if __name__ == "__main__":
    main()
