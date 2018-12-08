# Dorna
Dorna is a 5-axis robotic arm ([www.dorna.ai][dorna]), with industrial grade quality, offered at an affordable price for use in industrial or research applications. Dorna has maximum payload of about **2.5lbs (1.1KG)**, accuracy of **0.001in (0.02mm)** and reach of about **20in (500mm)**. Dorna uses [g2core firmware][g2core] for its motor control and motion planning. On this Github repo you will find the API of Dorna that can be used to control the robot in Python and is used to send commands to its firmware.

# Wiki
For the full documentation visit the [dorna Wiki page][wiki].

# Quick start

## Installation
The latest version of the API is available on [GitHub](https://github.com/dorna-robotics/dorna).  
<!--
**PyPI**  
To install the package from the PyPI server, simply use the `pip` command in command line:
```bash
pip install dorna
```
-->
**GitHub**  
Another option is to first clone the repository from GitHub directly:
```bash
git clone https://github.com/dorna-robotics/dorna.git
```
Then run the `setup.py` file from that directory,
```bash
python setup.py install
```
## API initialization

Import the module and create a `Dorna` object for interacting with the API:

``` python
from dorna import Dorna
robot = Dorna()
```
[dorna]:https://www.dorna.ai/
[wiki]:https://github.com/dorna-robotics/dorna/wiki
[g2core]: https://github.com/synthetos/g2/wiki
