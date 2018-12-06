# Dorna
The official Python library for the Dorna API V1.3

# Wiki
For the full documentation visit the [dorna Wiki page][wiki].

# Quick start

## Installation
The latest version of the API is available on [PyPI](https://pypi.org/project/dorna/) and [GitHub](https://github.com/dorna-robotics/dorna).  

**PyPI**  
To install the package from the PyPI server, simply use the `pip` command in command line:
```bash
pip install dorna
```
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
[wiki]:https://github.com/dorna-robotics/dorna/wiki
