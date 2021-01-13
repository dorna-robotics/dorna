# Dorna
[Dorna][dorna] is a 5-axis robotic arm, with industrial grade quality, offered at an affordable price for use in industrial or research applications. Dorna has maximum payload of about **1.1KG (2.5lbs)** and reach of about **500mm (20in)**. Dorna uses [g2core firmware][g2core] for its motor control and motion planning. On this Github repo you will find the API of Dorna that can be used to control the robot in Python.

## Wiki
For the full documentation visit the [dorna Wiki page][wiki].

## Installation

**GitHub**  
To install the Python package from GitHub, you need to clone the repository first.
```bash
git clone https://github.com/dorna-robotics/dorna.git
```
After download, go to the directory, and run:
```bash
python setup.py install
```
## ROS
For implementing Dorna with ROS, please visit the [rakutentech/dorna_arm_ros][ros] repository.

## API initialization

Import the module and create a `Dorna` object for interacting with the API:

``` python
from dorna import Dorna
robot = Dorna()
```
[dorna]:https://dorna.ai/
[wiki]:https://github.com/dorna-robotics/dorna/wiki
[g2core]: https://github.com/synthetos/g2/wiki
[latest]: https://github.com/dorna-robotics/dorna/releases/latest
[ros]:https://github.com/rakutentech/dorna_arm_ros
[ros2]:https://github.com/rakutentech/dorna_arm_ros