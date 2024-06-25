import rospy
import numpy as np
import baxter #here we are importing the baxter.py interface. (cause it's in this same folder, but in your project please clone the repo as submodule and import the interface as described in the readme)
import sys

"""
Usage: Execute the following command in the terminal:
python DataCrawler.py

Input: 
move direction distance
rotate direction angle
reverse
exit


direction: up, down, left, right, forward, backward (for move) and wrist_cw, wrist_ccw, flex_arm, extend_arm (for rotate)
angle: angle in degrees
distance: distance in cm
"""

rospy.init_node("testing")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm="left")
rospy.sleep(2.0)

robot.set_robot_state(True)

reverse_operation = None


def rotation(direction, angle):
    global reverse_operation
    angles = robot.joint_angle()
    print(angles)
    angle = np.deg2rad(angle)
    if direction == "wrist_cw":
        angles["left_w2"] += angle
        reverse_operation = ("rotate","wrist_ccw", np.rad2deg(angle))
    elif direction == "wrist_ccw":
        angles["left_w2"] -= angle
        reverse_operation = ("rotate","wrist_cw", np.rad2deg(angle))
    elif direction == "flex_arm":
        angles["left_w1"] -= angle
        reverse_operation = ("rotate","extend_arm", np.rad2deg(angle))
    elif direction == "extend_arm":
        angles["left_w1"] += angle
        reverse_operation = ("rotate","flex_arm", np.rad2deg(angle))
    else:
        print("Invalid direction")
        return
    for i in range(30):
        robot.set_joint_position(angles)
        rospy.sleep(0.2)
    

def movement(direction, distance):
    global reverse_operation
    p = robot._endpoint_state.pose.position
    q = robot._endpoint_state.pose.orientation
    distance = distance / 100
    if direction == "up":
        p.z += distance
        reverse_operation = ("move","down", distance * 100)
    elif direction == "down":
        p.z -= distance
        reverse_operation = ("move","up", distance * 100)
    elif direction == "left":
        p.y += distance
        reverse_operation = ("move","right", distance * 100)
    elif direction == "right":
        p.y -= distance
        reverse_operation = ("move","left", distance * 100)
    elif direction == "forward":
        p.x += distance
        reverse_operation = ("move","backward", distance * 100)
    elif direction == "backward":
        p.x -= distance
        reverse_operation = ("move","forward", distance * 100)
    else:
        print("Invalid direction")
        return
    robot.set_cartesian_position([p.x, p.y, p.z], [ q.x, q.y, q.z, q.w])


while True:
    command = input("Enter command: ")
    if command == "exit":
        break
    command = command.split()
    move = command[0]
    if move == "rotate":
        direction = command[1]
        angle = float(command[2])
        rotation(direction, angle)
    elif move == "move":
        direction = command[1]
        distance = float(command[2])
        movement(direction, distance)
    elif move == "reverse":
        if reverse_operation[0] == "rotate":
            print(reverse_operation)
            rotation(reverse_operation[1], reverse_operation[2])
        elif reverse_operation[0] == "move":
            movement(reverse_operation[1], reverse_operation[2])
    else:
        print("Invalid command")
        continue



