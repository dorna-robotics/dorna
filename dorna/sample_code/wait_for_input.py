"""
In this code we wait for the input1 to turn 1 and then do our job.
"""
from dorna import Dorna
import json
import time

# creat Dorna object and connect to the robot
robot = Dorna()
robot.connect()

while True:
	result = robot.io() # read the io
	result = json.loads(result) # translate the json file into a python dictionary
	if result["in1"] == 1:

		break
	time.sleep(0.001) # run this every 1ms until you get in1 equal to 1

# Now do the remaining job	
robot.play({"command": "move", "prm":{"path": "joint", "movement":1, "j0": 10}})


