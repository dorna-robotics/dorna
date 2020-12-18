# Dorna2 
This is a Python API for [Dorna 2][dorna] robotic arm.

## Installation
Notice that the program has been tested only on Python 3.7+.

### Download
First, use `git clone` to download the repository:  
```bash
git clone https://github.com/dorna-robotics/dorna2-python.git
```
Or simply download the [zip file](https://github.com/dorna-robotics/dorna/archive/master.zip), and uncompress the file.  

### Install
Next, go to the downloaded directory, where the `setup.py` file is located, and run:
```bash
python setup.py install
```

## Getting started
Import `dorna2` module.
``` python
from dorna2 import dorna

robot = dorna()
ip = "127.0.0.1"
host = 443
robot.connect(ip, host)

# your code

robot.close() # always close the socket when you are done
```  

## Connection
The robot WebSocket server runs on `ws://robot_ip_address:443`. Where `robot_ip_address` is the IP address of the robot, and `443` is the port number.   
```python
# example: if ip = dorna
ws_url = "ws://dorna:443"

# example: if ip = 192.168.1.2
ws_url = "ws://192.168.1.2:443"
```

`.connect(host, port, wait=1, init=True)`
Connect to the robot WebSocket server at `ws://host:port`. This method is very similar to the Python `socket.connect((host, port))` method. 
`.sock` is the core Python `socket` object which is responsible for sending and receiving messages.  


`.close()`
Use this method to close the WS connection. It is a good practice to close an opened socket connection when your task is over and the connection is no longer required.
Notice that `.close()` instantly closes the socket and terminates the communication loop.
``` python
from dorna2 import dorna

robot = dorna()
robot.connect("192.168.1.10", 443) # connect to ws://192.168.1.10:443

# your code

robot.close() # always close the socket when you are done
```  

## Send message
`.play(message=None, **arg)`
Send a valid message to the robot. There are multiple ways to send your message. Here we show how to send one simple alarm command in three different ways:
1. JSON string format: `play('{"cmd": "alarm", "id": 100}')`
2. Python dictionary format: `play({'cmd': 'alarm', 'id': 100})` 
3. Key and value format: `play(cmd='alarm', id=100})`  

There are other helper functions to send a message to the robot:
- `.jmove(**arg)`: A helper function to send a `jmove` command. `jmove(j0=0, id=100)` is equivalent to sending `'{"cmd": "jmove", "j0": 0, "id": 100}'`. Basically the `"cmd"` key is set to `"jmove"`.  
- `.lmove(**arg)`: Similar to `.jmove()` but the command key is equal to `"lmove"`.
- `.cmove(**arg)`: Similar to `.cmove()` but the command key is equal to `"cmove"`.

## Receive message
`sys` is a dictionary that holds the messages received by the API. Notice that, `sys` initialized with an empty dictionary. Every time a new JSON received by the API, `sys` updates itself according to the received data.
``` python
print(robot.sys) # {}

# command id = 100
robot.play(cmd="jmove", rel=0, id=100, j1=90, j2=-90)
robot.wait(id=100, stat=2)

print(robot.sys) # {"id": 100, "stat": 2, ...}
``` 
The last 100 messages received by the API are stored in `msg` queue (`queue.Queue(100)`).
``` python
# print received messages 
while True:
	if not robot.msg.empty()
		print(robot.msg.get())
``` 
`.wait(time_out=0, **arg)`
Wait for a pattern of key and values to appear in the `.sys` dictionary. Use `wait` method to wait for the completion of a command with an `id`. 
``` python
# command id = 100
robot.play(cmd = "jmove", rel = 0, id = 100, j1 = 90, j2 = -90)
robot.wait(100)

# {"id": 100, "stat": 2} has been received
print("command with id = 100 has been completed")
``` 

[dorna]: https://dorna.ai
