from dorna import Dorna
import json
import time

# creat Dorna object and connect to the robot
robot = Dorna()
robot.connect()

# home all the joints
robot.home(["j0", "j1", "j2", "j3"])

"""
move to j0 = 0, ..., j4 = 0
wait for the motion to be done, timeout = 1000 seconds
"""
result = robot.play({"command": "move", "prm":{"path": "joint", "movement":0, "joint":[0, 0, 0, 0, 0]}})
result = json.loads(result)
wait = robot._wait_for_command(result, time.time()+1000)


# move in cartesian space, -10 inches toward X direction
robot.play({"command": "move", "prm":{"path": "line", "movement":1, "x":-10}})


