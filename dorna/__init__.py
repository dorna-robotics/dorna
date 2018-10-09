import sys

__name__ = "dorna"
__version__ = "1.2"	

version = 10*sys.version_info[0] + sys.version_info[1]
plt = sys.platform

if plt == "win32": # win32
	if version >= 37:
		from .api_37_win import Dorna
	elif 36<= version < 37:
		from .api_36_win import Dorna
	else:
		from .api_35_win import Dorna

elif plt == "darwin": # mac
    if version >= 37:
        from .api_37_darwin import Dorna
    elif 36<= version < 37:
        from .api_36_darwin import Dorna
    else:
        from .api_35_darwin import Dorna

else: # linux-gnu
	if version >= 37:
		from .api_37_linux import Dorna
	elif 36<= version < 37:
		from .api_36_linux import Dorna
	else:
		from .api_35_linux import Dorna
