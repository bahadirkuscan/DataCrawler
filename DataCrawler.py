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
robot_left = baxter.BaxterRobot(rate=100, arm="left")
rospy.sleep(2.0)
robot_right = baxter.BaxterRobot(rate=100, arm="right")

robot_left.set_robot_state(True)
robot_right.set_robot_state(True)

print(robot_left._endpoint_state.pose.position)
robot_right.gripper_calibrate()
robot_right.gripper_prepare_to_grip()
robot_right.move_to_neutral()
robot_left.move_to_neutral()

reverse_operation = None

object_positions = [[0.7829306320888794, 0.2939874955968246, 0.1827199408116185], [0.6839650164218424, 0.08158967511950699, 0.2476260224906196], [0.6858101478459308, -0.0700958564708805, 0.26235887693132587], [0.6695379643138924, -0.19581662422156422, 0.26987945530784874]]


def rotation(direction, angle):
    global reverse_operation
    angles = robot_left.joint_angle()
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
        robot_left.set_joint_position(angles)
        rospy.sleep(0.2)
    

def movement(direction, distance):
    global reverse_operation
    p = robot_left._endpoint_state.pose.position
    q = robot_left._endpoint_state.pose.orientation
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
    robot_left.set_cartesian_position([p.x, p.y, p.z], [ q.x, q.y, q.z, q.w])
    

def grip(object_position):
    global robot_right, robot_left
    #robot_left.move_to_neutral()
    go_to_object(object_position, "right")
    """
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2] - 0.4], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.gripper_grip()
    rospy.sleep(2.0)
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2]], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2] - 0.4], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.gripper_release()
    rospy.sleep(2.0)
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2]], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.move_to_neutral()
    """
    
def go_to_object(object_position, arm):
    global robot_right, robot_left
    if arm == "left":
        robot = robot_left
    else:
        robot = robot_right
    robot.set_cartesian_position(object_position, [0,1,0,0])
    rospy.sleep(2.0)
    
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
            rotation(reverse_operation[1], reverse_operation[2])
        elif reverse_operation[0] == "move":
            movement(reverse_operation[1], reverse_operation[2])
    elif move == "grip":
        grip(object_positions[int(command[1])])
    elif move == "move_sensor":
        go_to_object(object_positions[int(command[1])], "left")
    else:
        print("Invalid command")
        continue


