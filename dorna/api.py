# =================================================================
# imports
# =================================================================
import threading
import time
import json
import yaml
import numpy as np
import serial
import math
import sys
from serial.tools import list_ports
from subprocess import Popen, PIPE
from pkg_resources import resource_filename
import os
import copy
import re

# =================================================================
# print
# =================================================================
def _printx(enable = True, *arg):
	if enable:
		print(arg)

class _port_usb(object):
	def __init__(self):
		self._port = None

	def port_list(self):
		#result = json.dumps([str(p[0]) for p in list_ports.comports()])
		result = [str(p[0]) for p in list_ports.comports()]
		self._log_add(result, "port_list")
		return json.dumps(result)

	def _port_open(self, port):
		if self._port: # port is already open
			return True

		prt = serial.Serial()
		prt.port = port
		prt.baudrate = 115200
		prt.bytesize = serial.EIGHTBITS  # number of bits per bytes
		prt.parity = serial.PARITY_NONE  # set parity check: no parity
		prt.stopbits = serial.STOPBITS_ONE  # number of stop bits
		prt.timeout = .001  # non-block read
		prt.writeTimeout = None  # timeout for write
		try:
			prt.open()
			self._port = prt
			return True
		except Exception as ex:
			return False


	def _port_read(self):
		response_line = self._port.readline()
		if response_line:
			response = str(response_line,'utf-8')
			try:
				_result = json.loads(response)
				return _result
			except Exception as ex:
				pass
		return False



# =================================================================
# handy methods
# =================================================================
class easy_method(object):
	def __init__(self):
		super(easy_method, self).__init__()

	"""
	prm:
		dict
		json-dict
	append
		True
		False
	sync
		True
		False	
	"""
	def set_io(self, prm, fulfill = True, append = True, sync = True):
		try:
			prm = json.loads(prm)
		except:
			pass

		if sync:
			command = [{"command": "set_io", "prm": prm, "fulfill": fulfill}]
		else:
			command = [{"command": "set_io_async", "prm": prm, "fulfill": fulfill}]
		return self.play(command, append)

	def set_io_backup(self, prm, fulfill = True, append = True):
		try:
			prm = json.loads(prm)
		except:
			pass

		command = [{"command": "set_io", "prm": prm, "fulfill": fulfill}]
		return self.play(command, append)


	"""
	prm:
		dict
		json-dict
	append
		True
		False
	"""
	def move(self, prm, fulfill = True, append = True):
		try:
			prm = json.loads(prm)
		except:
			pass

		command = [{"command": "move", "prm": prm, "fulfill": fulfill}]
		return self.play(command, append)


	"""
	prm:
		dict
		json-dict
	"""
	def set_toolhead(self, prm):
		try:
			prm = json.loads(prm)
		except:
			pass

		# mm to inch
		if self._config["unit"]["length"] == "mm":
			for k in prm:
				prm[k] = self._mm_to_inch(prm[k])

		command = [{"command": "set_toolhead", "prm": prm}]
		result = self.play(command, False)
		result = json.loads(result)
		if len(result) == 0:
			return self.toolhead()

		wait = self._wait_for_command(result, time.time()+5)

		#wait = self._wait_for_job(result, time.time(), 5)
		if not wait:
			return self.toolhead()

		# update config
		self._config["toolhead"]["x"] = prm["x"]


		# update xyz
		self._xyz = self._travel_to_xyz(np.copy(self._travel))
		self._log_add(json.loads(self.position("xyz")), "xyz")

		return self.toolhead()

	"""
	prm:
		dict
		json-dict
	"""
	def set_motion(self, prm):
		try:
			prm = json.loads(prm)
		except:
			pass

		command = [{"command": "set_motion", "prm": prm}]
		result = self.play(command, False)
		result = json.loads(result)
		if len(result) == 0:
			return self.motion()

		wait = self._wait_for_command(result, time.time()+5)
		# update config in return

		return self.motion()

	"""
	prm:
		float
		str-flaot
	"""
	def servo(self, prm, append = True):
		try:
			prm = json.loads(prm)
		except:
			pass

		#command = [{"command": "servo", "prm": prm}]
		command = [{"command": "set_io", "prm": {"servo": prm}}]
		return self.play(command, append)

	"""
	prm:
		binary
		str-binary
	"""
	def laser(self, prm,  append = True):
		try:
			prm = json.loads(prm)
		except:
			pass
		#command = {"command": "laser", "prm":prm}
		command = {"command": "set_io", "prm":{"laser": prm}}
		return self.play(command, append)

	"""
	prm:
		dict
		json-dict
	"""
	def output(self, prm,  append = True):
		try:
			prm = json.loads(prm)
		except:
			pass
		#command = {"command": "output", "prm":prm}
		command = {"command": "set_io", "prm":prm}
		return self.play(command,append)

	def halt(self):
		self._flush_commands(True)


	"""
	prm:
		dict
		json-dict
	"""
	def gcode(self, prm, fulfill = True, append = True):
		try:
			prm = json.loads(prm)
		except:
			pass

		command = [{"command": "gcode", "prm": prm, "fulfill": fulfill}]
		return self.play(command, append)



	"""
	prm:
		gcode = None, list, json list 
		gcode_path = None, string, json, list
	"""
	def play_gcode(self, gcode_path = None, gcode = None, **kwargs):
		
		data = False
		# open gcode_path
		if gcode_path:
			try:
				with open(gcode_path, 'r') as f:
					data = f.read().splitlines()							
			except:
				data = False

		# gcode: list, str, JSON, 
		if gcode:
			# str to data (dict or list)
			if type(gcode) == str:
				try:
					data = json.loads(gcode)
				except:
					data = False

			if type(data) == dict:
				data = [data]
			elif type(data) == list:
				if any([type(j) != dict or type(j) != str for j in data]):
					data = False
			else:
				data = False

		try:
			commands = [{"command": "g2core", "prm": d} for d in data]
		except:
			_rtn = {"error": 1 , "message": "not a valid input format"}
			self._log_add(_rtn, "play_gcode")
			return json.dumps(_rtn)

		# xyz space
		self.play({"command": "move", "prm": {"path": "line", "movement": 1, "x": 0}}, append = False)
		
		return self.play(commands)



"""
limit rules:
always validate limit
if the device passes the limits only joint works
"""

class Dorna(_port_usb, easy_method):

	def __init__(self, config_path = None):
		
		super(Dorna, self).__init__()
		
		# =================================================================
		# print
		# =================================================================
		self._prnt = False

		# =================================================================
		# module name: "api", "dorna"
		# =================================================================
		self._mn = "dorna"

		# =================================================================
		# log
		# =================================================================
		self._log = None

		# =================================================================
		# max_read_error
		# =================================================================
		self._read_error = 10

		# =================================================================
		# number of axis
		# =================================================================
		self._axis = 5

		# =================================================================
		# decimal
		# =================================================================
		self._delta_e = 0.001
		self._display_precision = 4
		self._compute_precision = 16

		# =================================================================
		# utility
		# =================================================================
		# segment size
		self._segment_size = 0.1
		# dimension
		self._bx = 3.759
		self._bz = 8.111
		self._l1 = 8.
		self._l2 = 6.

		# =================================================================
		# variable
		# =================================================================
		# init variable
		self._init_variable()


		# =================================================================
		# config
		# =================================================================

		# init config
		if not config_path:
				config_path = resource_filename(self._mn, 'config.yaml')
				_printx(self._prnt,"config_path: ", config_path)
		self._device["config"] = config_path
		self._init_config()

		# =================================================================
		# thread
		# =================================================================

		# thread
		self._stop = False

		command_thread = threading.Thread(target = self._command_thread)
		command_thread.start()

		send_thread = threading.Thread(target = self._send)
		send_thread.start()

		receive_thread = threading.Thread(target = self._receive)
		receive_thread.start()


	# =================================================================
	# update arduino
	# =================================================================
	def _baud(self, port_name):
		platform = sys.platform
		baud = []
		if platform == "win32": # windows
			baud.append("mode "+port_name+" BAUD=1200")
			#bossac = "resources/windows/bossac --port="+port_name+" -U true -e -w -v -b "+bin_path+ " -R"
		elif platform == "darwin": # mac
			baud.append("stty -f "+port_name+" 1200")
		else: # linux
			baud.append("sudo stty -F "+port_name+" 1200 hup")
			baud.append("sudo stty -F "+port_name+" 9600 hup")
		# baud
		try:
			for cmd in baud:
				time.sleep(0.5)
				sp = Popen(cmd, shell=True, stdout=PIPE,stderr=PIPE, bufsize=1, universal_newlines=True)
				sp.communicate()

			return True
		except Exception as ex:
			_printx(self._prnt, ex)
			return False

	def _bossac(self, port_name, bin_path):
		platform = sys.platform
		if platform == "win32": # windows
			bossac_path = resource_filename(self._mn, 'resources/windows/bossac')
			bossac = [bossac_path, "-p", port_name, "-U", "true", "-e", "-w", "-v", "-b", bin_path ,"-R"]

		elif platform == "darwin": # mac
			bossac_path = resource_filename(self._mn, 'resources/mac/bossac')
			bossac = ["sudo", bossac_path, "-U", "true", "-e", "-w", "-v", "-b", bin_path, "-R"]
			self._bossac_exe(bossac_path)

		else: # linux
			bossac_path = "bossac"
			if "/dev/" in port_name:
				port_name = port_name[5:]
			# shell is True: bossac = "sudo "+ bossac_path + " --port="+port_name+"  -U true -e -w -v -i -b -R " + bin_path
			bossac = ["sudo", bossac_path, "-p", port_name,"-U", "true", "-e", "-w", "-v", "-i", "-b", bin_path, "-R"]
		# installing bossac
		line_last = ""

		self._log_add({"status": 1, "message": "Updating..."}, "update_firmware")
		time.sleep(1)
		try:

			with Popen(bossac, shell=False, stdout=PIPE, bufsize=1, universal_newlines=True) as p:

				for line in p.stdout:

					line_last = line
					_printx(self._prnt,line)

					# check fro (fff/fff)
					if all([x in line for x in ["(","/"," pages)", "%"]]):
						try:
							# find the %
							_index = line.find("%")
							tmp_log = line[_index+1:]

							# find (
							_index = tmp_log.find("(")
							tmp_log = tmp_log[_index+1:]

							# find  pages)
							_index = tmp_log.find(" pages)")
							tmp_log = tmp_log[0:_index]

							# nom and denom
							nom, denom = tmp_log.split("/")
							nom = int(nom)
							denom = int(denom)

							_max_square = 32
							_number_sqr = math.floor(_max_square*nom/denom)
							_percentage = math.floor(100*nom/denom)
							# percentage 8 characters: "    100%"
							percentage_text = (8 - len(str(_percentage) + "%"))* " " +str(_percentage) + "%"
							# square 35 characters: " |███████████████████████████     |"
							square_text = " |" + _number_sqr*"\u2588" + (_max_square -_number_sqr )*" "+ "|"
							# command 17 characters: " 108/111 commands"
							_command_text = str(nom)+ "/"+str(denom)+ " pages"
							_command_text = (17-len(_command_text))*" "+ _command_text
							if nom != denom:
								print(percentage_text,square_text,_command_text,  end="\r")
							else:
								print(percentage_text,square_text,_command_text)

							# log_add
							#self._log_add({"status": 2, "nom": nom, "denom": denom}, "update_firmware")
						except Exception as ex:
							pass

		except Exception as ex:
			return False
		if line_last.strip() in ["CPU reset.", "Set boot flash true"]:
			return True
		return False

	def _bossac_reset(self, port_name):
		platform = sys.platform
		if platform == "win32": # windows
			bossac_path = resource_filename(self._mn, 'resources/windows/bossac')
			bossac = [bossac_path, "-p", port_name, "-b","-R"]

		elif platform == "darwin": # mac
			bossac_path = resource_filename(self._mn, 'resources/mac/bossac')
			bossac = ["sudo", bossac_path,"-b", "-R"]
		else: # linux
			bossac_path = "bossac"
			if "/dev/" in port_name:
				port_name = port_name[5:]
			bossac = ["sudo", bossac_path, "-p", port_name, "-b", "-R"]
		# installing bossac
		line_last = ""

		self._log_add({"status": 1, "message": "Reseting..."}, "update_firmware")
		time.sleep(1)
		try:

			with Popen(bossac, shell=False, stdout=PIPE, bufsize=1, universal_newlines=True) as p:

				for line in p.stdout:

					line_last = line
					_printx(self._prnt,line)

					# check fro (fff/fff)
					if all([x in line for x in ["(","/"," pages)", "%"]]):
						try:
							# find the %
							_index = line.find("%")
							tmp_log = line[_index+1:]

							# find (
							_index = tmp_log.find("(")
							tmp_log = tmp_log[_index+1:]

							# find  pages)
							_index = tmp_log.find(" pages)")
							tmp_log = tmp_log[0:_index]

							# nom and denom
							nom, denom = tmp_log.split("/")
							nom = int(nom)
							denom = int(denom)

							_max_square = 32
							_number_sqr = math.floor(_max_square*nom/denom)
							_percentage = math.floor(100*nom/denom)
							# percentage 8 characters: "    100%"
							percentage_text = (8 - len(str(_percentage) + "%"))* " " +str(_percentage) + "%"
							# square 35 characters: " |███████████████████████████     |"
							square_text = " |" + _number_sqr*"\u2588" + (_max_square -_number_sqr )*" "+ "|"
							# command 17 characters: " 108/111 commands"
							_command_text = str(nom)+ "/"+str(denom)+ " pages"
							_command_text = (17-len(_command_text))*" "+ _command_text
							if nom != denom:
								print(percentage_text,square_text,_command_text,  end="\r")
							else:
								print(percentage_text,square_text,_command_text)

							# log_add
							#self._log_add({"status": 2, "nom": nom, "denom": denom}, "update_firmware")
						except Exception as ex:
							pass

		except Exception as ex:
			return False
		if line_last.strip() in ["CPU reset.", "Set boot flash true"]:
			return True
		return False


	def _bossac_exe(self, bossac_path):
		# form the command
		bossac = ["sudo", "chmod", "+x", bossac_path]

		# run the command
		sp = Popen(bossac, shell=False, stdout=PIPE,stderr=PIPE, bufsize=1, universal_newlines=True)
		out, err = sp.communicate()
		out = out.splitlines()
		err = err.splitlines()
		for line in out + err:
			_printx(self._prnt, line)

	def reset_board(self, port_name = None):
		# bin_path = "./resources/firmware/firmware.bin"
		"""
		baud
		mac: stty -f port_name 1200
		windows: mode port_name BAUD=1200
		linux:
			stty -F port_name 1200 hup
			stty -F port_name 9600
		"""
		self._log_add({"status": 1, "message": "Progressing..."}, "update_firmware")
		print("Progressing...")
		

		num_try = 8

		### disconnect first ###
		self.disconnect()

		time.sleep(1)

		# check bossac exists
		if sys.platform not in ["win32", "darwin"]: # linux
			#sudo apt-get install bossa-cli
			sp = Popen(["sudo", "bossac"], shell=False, stdout=PIPE,stderr=PIPE, bufsize=1, universal_newlines=True)
			out, err = sp.communicate()
			out = out.splitlines()
			err = err.splitlines()
			for line in out + err:

				self._log_add({"status": 1, "message": line}, "update_firmware")
				if "not found" in line:
					# log
					_rtn = {"status" : 100, "message": "You need to install BOSSA flash programming utility. Run: sudo apt-get install bossa-cli"}
					self._log_add(_rtn, "update_firmware")
					return json.dumps(_rtn)

		# port name is given
		if port_name:
			result = json.loads(self._reset_board(port_name))
			if not result["status"]:
				# log
				return result
		else:
			port_list = json.loads(self.port_list())
			for port_name in port_list:
				result = json.loads(self._reset_board(port_name))
				if not result["status"]:
					# log
					return result

		# log
		_rtn = {"status" : 100, "message": "Reset failed"}
		self._log_add(_rtn, "update_firmware")
		return json.dumps(_rtn)

	def _reset_board(self, port_name):
		# port list
		port_list_before = json.loads(self.port_list())
		# port not found
		if port_name not in port_list_before:
			_rtn = {"status" : 100, "message": "USB port not found"}
			self._log_add(_rtn, "update_firmware")
			return json.dumps(_rtn)

		# iteration
		baud = False
		num_try = 8
		i = 0
		while i < num_try and not baud:
			i += 1
			# time sleep
			time.sleep(1)

			# set port_baud
			port_baud = port_name
			port_list_after = json.loads(self.port_list())
			if port_baud not in port_list_after:
				difference_list = [x for x in port_list_after if x not in port_list_before]
				if difference_list:
					port_baud =  difference_list[0]
				else:
					_rtn = {"status" : 100, "message": "Update failed"}
					return json.dumps(_rtn)

			# baud
			baud = self._baud(port_baud)

			if baud:
				time.sleep(1)

				# set port bossac
				port_bossac = port_baud
				port_list_bossac = json.loads(self.port_list())
				if port_bossac not in port_list_bossac:
					difference_list = [x for x in port_list_bossac if x not in port_list_after]
					if difference_list:
						port_bossac =  difference_list[0]
					else:
						return json.dumps({"status" : 100, "message": "Reset failed"})
				#bossac
				#if self._bossac(port_bossac, firmware_path):
				if self._bossac_reset(port_bossac):
					# log
					_rtn = {"status" : 0, "message": "Completed: board reseted on port "+ port_name + "."}
					self._log_add(_rtn, "update_firmware")
					return json.dumps(_rtn)

		return json.dumps({"status" : 100, "message": "Reset failed"})



	def update_firmware(self, port_name = None, firmware_path =  None):
		# bin_path = "./resources/firmware/firmware.bin"
		"""
		baud
		mac: stty -f port_name 1200
		windows: mode port_name BAUD=1200
		linux:
			stty -F port_name 1200 hup
			stty -F port_name 9600
		"""
		self._log_add({"status": 1, "message": "Progressing..."}, "update_firmware")
		print("Progressing...")
		

		num_try = 8

		### disconnect first ###
		self.disconnect()

		time.sleep(1)
		### firmware_path
		if not firmware_path:
			firmware_path = resource_filename(self._mn, 'resources/firmware/firmware.bin')

		# check bossac exists
		if sys.platform not in ["win32", "darwin"]: # linux
			#sudo apt-get install bossa-cli
			sp = Popen(["sudo", "bossac"], shell=False, stdout=PIPE,stderr=PIPE, bufsize=1, universal_newlines=True)
			out, err = sp.communicate()
			out = out.splitlines()
			err = err.splitlines()
			for line in out + err:

				self._log_add({"status": 1, "message": line}, "update_firmware")
				if "not found" in line:
					# log
					_rtn = {"status" : 100, "message": "You need to install BOSSA flash programming utility. Run: sudo apt-get install bossa-cli"}
					self._log_add(_rtn, "update_firmware")
					return json.dumps(_rtn)

		# port name is given
		if port_name:
			result = json.loads(self._update_firmware(port_name, firmware_path))
			if not result["status"]:
				# log
				return result
		else:
			port_list = json.loads(self.port_list())
			for port_name in port_list:
				result = json.loads(self._update_firmware(port_name, firmware_path))
				if not result["status"]:
					# log
					return result

		# log
		_rtn = {"status" : 100, "message": "Update failed"}
		self._log_add(_rtn, "update_firmware")
		return json.dumps(_rtn)


	def _update_firmware(self, port_name, firmware_path):
		# port list
		port_list_before = json.loads(self.port_list())
		# port not found
		if port_name not in port_list_before:
			_rtn = {"status" : 100, "message": "USB port not found"}
			self._log_add(_rtn, "update_firmware")
			return json.dumps(_rtn)

		# iteration
		baud = False
		num_try = 8
		i = 0
		while i < num_try and not baud:
			i += 1
			# time sleep
			time.sleep(1)

			# set port_baud
			port_baud = port_name
			port_list_after = json.loads(self.port_list())
			if port_baud not in port_list_after:
				difference_list = [x for x in port_list_after if x not in port_list_before]
				if difference_list:
					port_baud =  difference_list[0]
				else:
					_rtn = {"status" : 100, "message": "Update failed"}
					return json.dumps(_rtn)

			# baud
			baud = self._baud(port_baud)

			if baud:
				time.sleep(1)

				# set port bossac
				port_bossac = port_baud
				port_list_bossac = json.loads(self.port_list())
				if port_bossac not in port_list_bossac:
					difference_list = [x for x in port_list_bossac if x not in port_list_after]
					if difference_list:
						port_bossac =  difference_list[0]
					else:
						return json.dumps({"status" : 100, "message": "Update failed"})
				#bossac
				#if self._bossac(port_bossac, firmware_path):
				if self._bossac(port_bossac, firmware_path):
					# log
					_rtn = {"status" : 0, "message": "Completed: firmware updated on port "+ port_name+"."}
					self._log_add(_rtn, "update_firmware")
					return json.dumps(_rtn)

		return json.dumps({"status" : 100, "message": "Update failed"})


	# =================================================================
	# all the public attr
	# =================================================================

	#??? add log or not
	def device(self):
		#tmp = dict(self._device)
		tmp = copy.deepcopy(self._device)
		if tmp["state"] != None:
			tmp["state"] = math.ceil(tmp["state"])

		# config

		try:
			tmp["config"] = os.path.abspath(tmp["config"])
		except:
			pass

		return json.dumps(tmp)

	def _print_percentage(self, nom, denom, new_line = False):
		_max_square = 32
		_number_sqr = math.floor(_max_square*nom/denom)
		_percentage = math.floor(100*nom/denom)
		# percentage 8 characters: "    100%"
		percentage_text = (8 - len(str(_percentage) + "%"))* " " +str(_percentage) + "%"
		# square 35 characters: " |███████████████████████████     |"
		square_text = " |" + _number_sqr*"\u2588" + (_max_square -_number_sqr )*" "+ "|"
		# command 17 characters: " 108/111 commands"
		_command_text = str(nom)+ "/"+str(denom)+ " commands"
		_command_text = (17-len(_command_text))*" "+ _command_text
		if new_line:
			print(percentage_text,square_text,_command_text)
		else:
			print(percentage_text,square_text,_command_text,  end="\r")

	def _connect_percentage(self, _init_nom, denom, command_list, max_time):
		while len(command_list) and time.time() < max_time and self._device["connection"]:
			try:
				if command_list[0]["id"] <= self._system["command"][5][-1]["id"]:
					_init_nom += 1
					if _init_nom == denom:
						self._print_percentage(_init_nom, denom, True)
					else:
						self._print_percentage( _init_nom, denom)
					command_list.pop(0)
					# add to the log
					self._log_add({"nom":_init_nom, "denom": denom}, "connect_percentage")
				else:
					time.sleep(0.02)				
			except Exception as e:
				time.sleep(0.02)

	def _connect_percentage_backup(self, _init_nom, denom, command_list, max_time):
		while len(command_list) and time.time() < max_time and self._device["connection"]:
			if command_list[0]["id"] <= self._system["command"][5][-1]["id"]:
				_init_nom += 1
				if _init_nom == denom:
					self._print_percentage(_init_nom, denom, True)
				else:
					self._print_percentage( _init_nom, denom)
				command_list.pop(0)
				# add to the log
				self._log_add({"nom":_init_nom, "denom": denom}, "connect_percentage")
			else:
				time.sleep(0.02)				

	def _command_mask(self, command):
		#_allowed_keys = ["id", "state", "error", "message", "command","prm","fulfill", "key"]
		_state = [0, 0, 1,1,1,2]
		_remove_key = ["travel_final", "gc", "display"]
		_command = []
		for c in command:
			#x = dict(c)
			x = copy.deepcopy(c)
			if x["display"]:
				for r in _remove_key:
					x.pop(r, None)

				# change state
				x["state"] = _state[x["state"]]

				_command.append(x)
		return _command


	def _command_by_id(self, id_list):
		# sanitate only int or list of int
		if type(id_list) is not list:
			id_list = [id_list]
		# sort
		id_list.sort()


		# search
		_command = []
		command_index = 5

		# every id is an integer
		if any([type(x) != int for x in id_list]):
			return _command


		#_id = id_list.pop(0)
		while id_list and command_index >= 0:
			result = next((item for item in self._system["command"][command_index] if item["id"] == id_list[0]), None)
			if result != None:
				_command.append(result)
				id_list.pop(0)
			else:
				command_index += -1
		return _command

	def _command_by_state(self, state_list):
		# sanitate
		if type(state_list) is not list:
			state_list = [state_list]
		# sort
		state_list.sort(reverse=True)

		_command = []

		for state in state_list:
			_command += self._system["command"][state]

		return _command


	def command(self, prm):
		# json
		if type(prm) == str:
			try:
				prm = json.loads(prm)
			except:
				prm = False
		_result = []
		if type(prm) == dict and "id" in prm:
			_result = self._command_by_id(prm["id"])
		elif type(prm) == dict and "state" in prm:
			_state = [[0,1], [2,3,4], [5]]
			state = []
			if type(prm["state"]) != list:
				prm["state"] = [prm["state"]]

			for s in prm["state"]:
				try:
					state += _state[s]
				except:
					pass
			_result = self._command_by_state(state)

		_result = self._command_mask(_result)
		return json.dumps(_result)


	def xyz_to_joint(self, xyz):
		tmp_xyz =np.array(xyz[0:self._config["axis"]["number"]])
		# unit
		if self._config["unit"]["length"] == "mm":
			for i in range(0,3):
				tmp_xyz[i] = self._mm_to_inch(tmp_xyz[i])

		# xyz to joint
		return json.dumps(self._xyz_to_joint(tmp_xyz)["joint"].tolist())


	def joint_to_xyz(self, joint):
		tmp_joint = np.array(joint[0:self._config["axis"]["number"]])
		tmp_xyz = self._joint_to_xyz(tmp_joint)
		# unit
		if self._config["unit"]["length"] == "mm":
			for i in range(0,3):
				tmp_xyz[i] = self._inch_to_mm(tmp_xyz[i])

		return json.dumps(tmp_xyz.tolist())

	def position(self, space = "joint"):
		if space[0] == "j":
			return json.dumps(self._joint[0:self._config["axis"]["number"]].tolist())
		elif space[0] == "x":
			# unit
			if self._config["unit"]["length"] == "mm":
				tmp_xyz = self._xyz[0:self._config["axis"]["number"]].tolist()

				for i in range(0,3):
					tmp_xyz[i] = self._inch_to_mm(tmp_xyz[i])

				return json.dumps(tmp_xyz)
			return json.dumps(self._xyz[0:self._config["axis"]["number"]].tolist())

		rtn = np.array([None for _ in range(self._config["axis"]["number"])])
		return json.dumps(rtn.tolist())

	def homed(self):
		rtn = {}
		rtn["j0"] = int(self._home_robot["x"] == 1)
		rtn["j1"] = int(self._home_robot["y"] == 1)
		rtn["j2"] = int(self._home_robot["z"] == 1)
		try:
			if self._home_robot["a"]+self._home_robot["b"] == 2:
				rtn["j3"] = 1
			else:
				rtn["j3"] = 0
		except:
			rtn["j3"] = None
		rtn["j4"] = rtn["j3"]
		return json.dumps(rtn)

	def io(self):
		#tmp = dict(self._io)
		tmp = copy.deepcopy(self._io)
		tmp["laser"] = tmp["out5"]
		tmp.pop("out5", None)
		return json.dumps(tmp)

	def log_start(self, l):
		self._log = l
		self._log_id = 0

	def log(self):
		return json.dumps(self._log)

	def _log_add(self,msg,  _type = "info"):
		# make sure it is list
		if type(self._log) != list:
			return None
		self._log.append(json.dumps({"id": self._log_id, "time": time.time(), "type": _type,  "message": msg}))
		self._log_id += 1

	def _init_variable(self):
		# ???
		self.limit_base = [-175,175]

		# device_id
		"""
		connection => 0: disconnected, 1: connecting, 2: connected
		state => 0: stopped, 1 : running, 0.5 stopping
		"""
		self._device = {"id": None, "connection": 0,  "port": None, "fv": None, "config": None, "state": None, "version": "1.4.2"}

		# travel
		self._travel = np.array([None,None,None,None,None,None]) # 6 axis
		self._joint = self._travel_to_joint(np.copy(self._travel))
		self._xyz = self._travel_to_xyz(np.copy(self._travel))


		# home j0 ,j1, j2, (j3,j4)
		self._home_system = {"home": None, "homx": None, "homy": None, "homz": None, "homa": None, "homb": None}
		self._home_robot = {"x": None, "y": None, "z": None, "a": None, "b": None, "c": None}

		self._scale = {"speed": 0.5, "jerk": 0.5}

		# io
		self._io = {"out1": None, "out2": None , "out3": None, "out4": None,
					"out5": None,
					"in1": None, "in2":None, "in3": None, "in4":None,
					"do1mo": None , "do2mo": None, "do3mo": None,"do4mo": None,
					"di1mo": None, "di2mo": None,"di3mo": None, "di4mo": None,
					"servo": None}

		# servo
		#self._srv = None

		# system

		self._system = {"lines_to_send": 4,
						"qr": 48,
						"travel_final" : None,
						"command": [[], [], [], [] ,[], []],
						"command_id": -1,
						"valid_command_id": 0,
						"gc": None,
						"state": None,
						"probe": {}
						}

	# sanitate data and make sure it has a right format
	def _sanitate_command(self, data):
		# str to data (dict or list)
		if type(data) == str:
			try:
				data = json.loads(data)
			except:
				data = False


		if type(data) == dict:
			data = [data]
		elif type(data) == list:
			if any([type(j) != dict for j in data]):
				data = False
		else:
			data = False

		return data

	# format the commands
	def _format_command(self, commands):
		commands_tmp = []
		for command in commands:
			self._system["command_id"] += 1
			command_tmp = {"id": self._system["command_id"],
							"state": 0,
							"error": None,
							"message": None,
							"travel_final": None,
							"gc": [],
							"command": command["command"],
							"fulfill": True,
							"key": None,
							"display": True
							}

			# prm
			if "prm" in command:
				command_tmp["prm"] = command["prm"]

			# fulfill
			if "fulfill" in command:
				command_tmp["fulfill"] = command["fulfill"]

			# display
			if "display" in command:
				command_tmp["display"] = command["display"]

			# key
			if "key" in command:
				command_tmp["key"] = command["key"]

			commands_tmp.append(command_tmp)

		return commands_tmp

	def _port_close(self):
		try:
			self._port.close()
		except:
			pass

		self._port = None
		self._device["connection"] = 0
		self._device["state"] = None


		result = self.device()

		# add log
		self._log_add(json.loads(result), "device")
		# return
		return result

	def terminate(self):
		# make sure everything is finished
		while self._device["state"]:
			time.sleep(0.01)

		self._port_close()
		self._stop = True

	def disconnect(self):
		return self._port_close()

	def connect(self,port_name = None, file_init = None): # port: open, send: startup commands
		# open port
		# send Id
		# wait for it for 1 sec
		# send the rest
		# wait for end
		# change flag to ready
		#self._stop = False
		### search for all ports ###
		print("Progressing...")
		if port_name:
			return self._connect(port_name, file_init)
		else:
			port_all = json.loads(self.port_list())
			for port in port_all:
				result = self._connect(port, file_init)
				result = json.loads(result)
				if result["connection"] == 2:
					#self.set_joint([0,0,0,0,0])
					return self.device()

		return self._port_close()

	def _connect(self,port_name,file_init):
		try:
			"""
			# linux sudo permission
			if sys.platform not in ["win32" , "darwin"]:
				check_output("sudo chmod 666 " + port_name, shell=True).decode()
			"""
			# linux sudo permission
			if sys.platform != "win32":
				#Popen(bossac, shell=True, stdout=PIPE, bufsize=1, universal_newlines=True)
				#check_output("sudo chmod 777 " + port_name, shell=False).decode()
				with Popen(["sudo", "chmod", "777", port_name], shell=False, stdout=PIPE,stderr=PIPE, bufsize=1, universal_newlines=True) as p:
					pass

			# initial port open failed
			if not self._port_open(port_name):
				return self._port_close()


			# change to connecting status
			self._device["connection"] = 1

			job = [
				[{"command": "g2core", "prm": "{line:n}"}, {"command": "g2core", "prm": "{id: n, fv:n}"}],
				[{"command": "g2core", "prm": "{sr: n}"}],
				[{"command": "set_toolhead", "prm": {"x": self._config["toolhead"]["x"]}}, {"command": "move", "prm":{"path": "joint", "movement": 1, "speed": self._config["default_speed"]["joint"], "j0": 0, "jerk": list(self._config["default_jerk"]["joint"])}},{"command": "g2core", "prm": "{tt32:n}"}, {"command": "set_motion", "prm": self._config["motion"]}]
			]

			# file init
			if file_init:
				try:
					with open(file_init) as f:
					    content = f.readlines()
					# you may also want to remove whitespace characters like `\n` at the end of each line
					content = [x.strip() for x in content]
					# form g2core
					job.append([{"command": "g2core", "prm": "c"} for c in content])
				except:
					pass
			
			# number of jobs
			_init_num = [0]
			for j in job:
				_init_num.append(_init_num[-1] +len(j))

			for i, j in enumerate(job):
				result = self.play(j, True)
				result = json.loads(result)
				if type(result)!= list or len(result) == 0:
					return self._port_close()

				self._connect_percentage(_init_num[i], _init_num[-1], result, 5 + time.time() + 2*len(result))

				# wait a little bit
				_time = time.time()
				while self._device["state"]!= 0 and time.time() < _time + 1:
					time.sleep(0.005)

				# make sure the state is 0
				if self._device["state"] != 0:
					return self._port_close()

			self._device["connection"] =2
			self._device["port"] = port_name

			result = self.device()
			self._log_add(json.loads(result), "device")
			return result
		except Exception as x:
			_printx(self._prnt,"error: ",x)
			return self._port_close()



	def connect_backup(self,port_name = None): # port: open, send: startup commands
		# open port
		# send Id
		# wait for it for 1 sec
		# send the rest
		# wait for end
		# change flag to ready
		#self._stop = False
		### search for all ports ###
		print("Progressing...")
		if port_name:
			return self._connect(port_name)
		else:
			port_all = json.loads(self.port_list())
			for port in port_all:
				result = self._connect(port)
				result = json.loads(result)
				if result["connection"] == 2:
					return self.device()

		return self._port_close()

	def _connect_backup(self,port_name):
		try:
			"""
			# linux sudo permission
			if sys.platform not in ["win32" , "darwin"]:
				check_output("sudo chmod 666 " + port_name, shell=True).decode()
			"""
			# linux sudo permission
			if sys.platform != "win32":
				#Popen(bossac, shell=True, stdout=PIPE, bufsize=1, universal_newlines=True)
				#check_output("sudo chmod 777 " + port_name, shell=False).decode()
				with Popen(["sudo", "chmod", "777", port_name], shell=False, stdout=PIPE,stderr=PIPE, bufsize=1, universal_newlines=True) as p:
					pass

			# initial port open failed
			if not self._port_open(port_name):
				return self._port_close()

			# change to connecting status
			self._device["connection"] = 1

			job = [
				[{"command": "g2core", "prm": "{line:n}"}, {"command": "g2core", "prm": "{id: n, fv:n}"}],
				[{"command": "g2core", "prm": "{di1fn:4}"},{"command": "g2core", "prm": "{ej: 1}"},{"command": "g2core", "prm": "{jv:4}"},{"command": "g2core", "prm": "{sv:1}"},{"command": "g2core", "prm": "{si:200}"},{"command": "g2core", "prm": "{qv:2}"},{"command": "g2core", "prm": "{xam:1}"},{"command": "g2core", "prm": "{yam:1}"},{"command": "g2core", "prm": "{zam:1}"},{"command": "g2core", "prm": "{aam:1}"},{"command": "g2core", "prm": "{bam:1}"},{"command": "g2core", "prm": "{cam:1}"},{"command": "g2core", "prm": "{1:{sa:1.8}}"},{"command": "g2core", "prm": "{2:{sa:1.8}}"},{"command": "g2core", "prm": "{3:{sa:1.8}}"},{"command": "g2core", "prm": "{4:{sa:1.8}}"},{"command": "g2core", "prm": "{5:{sa:1.8}}"},{"command": "g2core", "prm": "{6:{sa:1.8}}"},{"command": "g2core", "prm": "{1:{tr:45}}"},{"command": "g2core", "prm": "{2:{tr:18}}"},{"command": "g2core", "prm": "{3:{tr:18}}"},{"command": "g2core", "prm": "{4:{tr:90}}"},{"command": "g2core", "prm": "{5:{tr:90}}"},{"command": "g2core", "prm": "{6:{tr:1.3535433}}"},{"command": "g2core", "prm": "{1:{mi:32}}"},{"command": "g2core", "prm": "{2:{mi:32}}"},{"command": "g2core", "prm": "{3:{mi:32}}"},{"command": "g2core", "prm": "{4:{mi:32}}"},{"command": "g2core", "prm": "{5:{mi:32}}"},{"command": "g2core", "prm": "{6:{mi:32}}"},{"command": "g2core", "prm": "{1:{ma:0}}"},{"command": "g2core", "prm": "{2:{ma:1}}"},{"command": "g2core", "prm": "{3:{ma:2}}"},{"command": "g2core", "prm": "{4:{ma:3}}"},{"command": "g2core", "prm": "{5:{ma:4}}"},{"command": "g2core", "prm": "{6:{ma:5}}"},{"command": "g2core", "prm": "{1:{po:0}}"},{"command": "g2core", "prm": "{2:{po:1}}"},{"command": "g2core", "prm": "{3:{po:0}}"},{"command": "g2core", "prm": "{4:{po:1}}"},{"command": "g2core", "prm": "{5:{po:1}}"},{"command": "g2core", "prm": "{6:{po:0}}"},{"command": "g2core", "prm": "{1:{pm:1}}"},{"command": "g2core", "prm": "{2:{pm:1}}"},{"command": "g2core", "prm": "{3:{pm:1}}"},{"command": "g2core", "prm": "{4:{pm:1}}"},{"command": "g2core", "prm": "{5:{pm:1}}"},{"command": "g2core", "prm": "{6:{pm:1}}"},{"command": "g2core", "prm": "{1:{pl:1.0}}"},{"command": "g2core", "prm": "{2:{pl:1.0}}"},{"command": "g2core", "prm": "{3:{pl:1.0}}"},{"command": "g2core", "prm": "{4:{pl:1.0}}"},{"command": "g2core", "prm": "{5:{pl:1.0}}"},{"command": "g2core", "prm": "{6:{pl:1.0}}"},{"command": "g2core", "prm": "{xtn:1}"},{"command": "g2core", "prm": "{xtm:1000}"},{"command": "g2core", "prm": "{xhi:6}"},{"command": "g2core", "prm": "{xhd:0}"},{"command": "g2core", "prm": "{xsv:4000}"},{"command": "g2core", "prm": "{xlv:500}"},{"command": "g2core", "prm": "{xlb:50}"},{"command": "g2core", "prm": "{xzb:69}"},{"command": "g2core", "prm": "{ytn:0}"},{"command": "g2core", "prm": "{ytm:420}"},{"command": "g2core", "prm": "{yhi:2}"},{"command": "g2core", "prm": "{yhd:0}"},{"command": "g2core", "prm": "{ysv:4000}"},{"command": "g2core", "prm": "{ylv:500}"},{"command": "g2core", "prm": "{ylb:50}"},{"command": "g2core", "prm": "{yzb:90}"},{"command": "g2core", "prm": "{ztn:100}"},{"command": "g2core", "prm": "{ztm:1500}"},{"command": "g2core", "prm": "{zhi:3}"},{"command": "g2core", "prm": "{zhd:1}"},{"command": "g2core", "prm": "{zsv:4000}"},{"command": "g2core", "prm": "{zlv:500}"},{"command": "g2core", "prm": "{zlb:50}"},{"command": "g2core", "prm": "{zzb:82}"},{"command": "g2core", "prm": "{atn:-1000}"},{"command": "g2core", "prm": "{btn:-1000}"},{"command": "g2core", "prm": "{atm:0}"},{"command": "g2core", "prm": "{btm:0}"},{"command": "g2core", "prm": "{ahi:4}"},{"command": "g2core", "prm": "{bhi:5}"},{"command": "g2core", "prm": "{ahd:1}"},{"command": "g2core", "prm": "{bhd:1}"},{"command": "g2core", "prm": "{asv:10000}"},{"command": "g2core", "prm": "{bsv:10000}"},{"command": "g2core", "prm": "{alv:5000}"},{"command": "g2core", "prm": "{blv:5000}"},{"command": "g2core", "prm": "{alb:30}"},{"command": "g2core", "prm": "{blb:30}"},{"command": "g2core", "prm": "{azb:60}"},{"command": "g2core", "prm": "{bzb:175}"},{"command": "g2core", "prm": "{xvm:30000}"},{"command": "g2core", "prm": "{yvm:30000}"},{"command": "g2core", "prm": "{zvm:30000}"},{"command": "g2core", "prm": "{avm:30000}"},{"command": "g2core", "prm": "{bvm:30000}"},{"command": "g2core", "prm": "{cvm:30000}"},{"command": "g2core", "prm": "{xfr:30000}"},{"command": "g2core", "prm": "{yfr:30000}"},{"command": "g2core", "prm": "{zfr:30000}"},{"command": "g2core", "prm": "{afr:30000}"},{"command": "g2core", "prm": "{bfr:30000}"},{"command": "g2core", "prm": "{cfr:30000}"}],
				[{"command": "g2core", "prm": "{sr: n}"}],
				[{"command": "set_toolhead", "prm": {"x": self._config["toolhead"]["x"]}}, {"command": "move", "prm":{"path": "joint", "movement": 1, "speed": self._config["default_speed"]["joint"], "j0": 0, "jerk": list(self._config["default_jerk"]["joint"])}},{"command": "g2core", "prm": "{tt32:n}"}, {"command": "set_motion", "prm": self._config["motion"]}]
			]

			job.pop(1)

			# number of jobs
			_init_num = [0]
			for j in job:
				_init_num.append(_init_num[-1] +len(j))

			for i, j in enumerate(job):
				result = self.play(j, False)
				result = json.loads(result)
				if type(result)!= list or len(result) == 0:
					return self._port_close()

				self._connect_percentage(_init_num[i], _init_num[-1], result, 5 + time.time() + 2*len(result))

				# wait a little bit
				_time = time.time()
				while self._device["state"]!= 0 and time.time() < _time + 1:
					time.sleep(0.005)

				# make sure the state is 0
				if self._device["state"] != 0:
					return self._port_close()

			self._device["connection"] =2
			self._device["port"] = port_name

			result = self.device()
			self._log_add(json.loads(result), "device")
			return result
		except Exception as x:
			_printx(self._prnt,"error: ",x)
			return self._port_close()

	def set_joint(self, prm, append = False):
		# joints are valid
		if any(self._joint == None):
			return None

		# json or not
		try:
			prm = json.loads(prm)
		except:
			pass

		# dictionary or list
		if type(prm) == list: # list
			if len(prm) > 6:
				return None
			prm_tmp = {"j"+str(i): prm[i] for i in range(len(prm))}
			prm = prm_tmp

		if type(prm) == dict:
			if not all([x in ["j0", "j1", "j2", "j3", "j4", "j5"] for x in prm.keys()]):
				return None
		# not empty
		if not prm:
			return None

		# set joint
		command = [{"command": "set_joint", "prm": prm}]
		result = self.play(command, append)
		result = json.loads(result)
		"""
		if result["id"] == None:
			return None
		wait = self._wait_for_job(result, time.time(), 1)
		if wait == "timeout":
			return None
		"""
		if len(result) == 0:
			return None

		wait = self._wait_for_command(result, time.time()+1)
		if not wait:
			return None

		# home robot
		home_robot = []
		if "j0" in prm:
			home_robot.append("x")
		if "j1" in prm:
			home_robot.append("y")
		if "j2" in prm:
			home_robot.append("z")
		if "j3" in prm or "j4" in prm:
			home_robot.append("a")
			home_robot.append("b")
		if "j5" in prm:
			home_robot.append("c")

		# tt32
		prm = {"tt32": {x: 1 for x in home_robot}}
		prm = json.dumps(prm)
		prm = prm.replace('"', "")
		command = {"command": "g2core", "prm": prm}
		result = self.play(command, False)
		result = json.loads(result)
		"""
		if result["id"] == None:
			return None
		wait = self._wait_for_job(result, time.time(), 1)
		if wait == "timeout":
			return None
		"""
		if len(result) == 0:
			return None

		wait = self._wait_for_command(result, time.time()+1)
		if not wait:
			return None

		# get the last tt32
		command = {"command": "g2core", "prm": "{tt32:n}"}
		result = self.play(command, False)
		result = json.loads(result)
		"""
		if result["id"] == None:
			return None
		wait = self._wait_for_job(result, time.time(), 1)
		if wait == "timeout":
			return None
		"""
		if len(result) == 0:
			return None

		wait = self._wait_for_command(result, time.time()+1)
		if not wait:
			return None

		return self.homed()


	# args: j0, j1, j2, j3 or j4
	# 		list or JSON list
	def home(self, prm):
		try:
			prm = json.loads(prm)
		except:
			pass

		# string or list
		if type(prm) != list:
			prm = [prm]

		result = None
		T = False
		for joint in prm:
			if all([T == False, joint in ["j3", "j4"]]):
				result = self._home_joint_3_4()
				T = True
			else:	
				result = self._home_joint(joint)
		return result

	# args: j0, j1, j2, j3 or j4
	# 		list or JSON list
	def home_backup(self, prm):
		try:
			prm = json.loads(prm)
		except:
			pass

		# string or list
		if type(prm) != list:
			prm = [prm]

		result = None
		for joint in prm:
			result = self._home_joint(joint)
		return result

	"""
	# remove all the probes
	{dixfn: 0}
	add the probe
	{di4fn: 4}

	G38.3
	"""
	def _home_joint_3_4(self):
		_input = [3,4]
		# set_joint
		self.set_joint({"j3": 0, "j4": 0}, True)

		# remove all the probe inputs
		for i in range(1,10):
			command = "{di"+str(i)+"fn: 0}"
			self.play({"command": "g2core", "prm":command}, append = True)
		time.sleep(0.5)
		# add di4fn: 4
		self.play({"command": "g2core", "prm":"{di4fn: 4}"}, append = True)
		time.sleep(0.5)
		
		# probe toward j4 -> 360
		_result = self.probe({"j"+str(_input[-1]): 360, "speed": 5000}, append = True)		
		if _result == None:
			return None
		_result = json.loads(_result)
		t3 = - _result[4]

		# back to where it started
		command = {"command": "move", "prm": {"path": "joint", "movement": 0, "j4": 0, "speed": 5000}}
		result = self.play(command, False)
		result = json.loads(result)
		if len(result) == 0:
			return None
		wait = self._wait_for_command(result, time.time()+1000)
		if not wait:
			return None


		# remove all the probe inputs
		for i in range(1,10):
			command = "{di"+str(i)+"fn: 0}"
			self.play({"command": "g2core", "prm":command}, append = True)
		time.sleep(0.5)
		# add di5fn: 4
		self.play({"command": "g2core", "prm":"{di5fn: 4}"}, append = True)
		time.sleep(0.5)

		# probe toward j4 -> -360
		_result = self.probe({"j"+str(_input[-1]): -360, "speed": 5000}, append = True)		
		if _result == None:
			return None
		_result = json.loads(_result)
		t4 = - _result[4]

		# back to where it started
		command = {"command": "move", "prm": {"path": "joint", "movement": 0, "j4": 0, "speed": 5000}}
		result = self.play(command, False)
		result = json.loads(result)
		if len(result) == 0:
			return None
		wait = self._wait_for_command(result, time.time()+1000)
		if not wait:
			return None

		travel = [-t3, -t4]
		#joint = [0.5*(travel[1]-travel[0]), -0.5*(travel[1]+travel[0])]
		joint = [0.5*(travel[1]-travel[0]), 0.5*(travel[1]+travel[0])]
		# set_joint
		#return self.set_joint({"j3": self._config["calibrate"]["j3"] + -(m4-m3)/2, "j4": self._config["calibrate"]["j4"] + (m4+m3)/2}, True)
		return self.set_joint({"j3": self._config["calibrate"]["j3"] + joint[0], "j4": self._config["calibrate"]["j4"] + joint[1]}, True)



	def _home_joint(self, joint):
		if joint not in ["j0","j1", "j2", "j3", "j4"]:
			return None

		# homing
		# job_1: home
		command = {"command": "home", "prm": [joint]}
		result = self.play(command, False)
		result = json.loads(result)

		if len(result) == 0:
			return None

		wait = self._wait_for_command(result, time.time()+120)
		if not wait:
			return None

		# calibration
		if joint in ["j3", "j4"]:
			clb = {"j3": self._config["calibrate"]["j3"], "j4": self._config["calibrate"]["j4"]}
		else:
			clb = {joint: self._config["calibrate"][joint]}

		# set_joint
		return self.set_joint(clb)

	def add_180(self):
		if any(self._joint == None):
			return None

		joint = np.copy(self._joint)
		if not self.set_joint({"j3": joint[3]+ 180, "j4": joint[4]+ 180}):
			return None

		return self.position("joint")

	# prm = {"j0":1, "j2":}
	def probe(self, prm, append = False):
		# read json
		try:
			prm = json.loads(prm)
		except:
			pass

		# probing
		# job_1: probe
		command = {"command": "probe", "prm": prm}
		result = self.play(command, append)
		result = json.loads(result)

		if len(result) == 0:
			return None

		wait = self._wait_for_command(result, time.time()+360)
		if not wait:
			return None

		# read probe
		# job2
		self._system["probe"]["e"] = 0
		command = {"command": "g2core", "prm": "{prb:n}"}
		result = self.play(command, False)
		result = json.loads(result)

		if len(result) == 0:
			return None
		wait = self._wait_for_command(result, time.time()+360)
		if not wait:
			return None

		if self._system["probe"]["e"]:
			try:
				_probe_travel = [self._system["probe"][k] for k in ["x", "y", "z", "a", "b", "c"]]
				_probe_joint = self._travel_to_joint(_probe_travel)
				return json.dumps(_probe_joint[0:self._config["axis"]["number"]].tolist())
			except Exception as ex:
				pass

		return None

	# prm = {"j0":1, "j2": ,... }
	def calibrate(self, prm):

		# robot is homed
		home = list(json.loads(self.homed()).values())
		if not all(home[0:5]):
			return None

		# current joint
		joint = np.copy(self._joint)

		# read json
		try:
			prm = json.loads(prm)
		except:
			pass

		# dictionary or list
		if type(prm) == list: # list
			if len(prm) > 6:
				return None
			prm_tmp = {"j"+str(i): prm[i] for i in range(len(prm))}
			prm = prm_tmp

		if type(prm) == dict:
			if not all([x in ["j0", "j1", "j2", "j3", "j4", "j5"] for x in prm.keys()]):
				return None
		# not empty
		if not prm:
			return None
		

		# set_joint
		if not self.set_joint(prm):
			return None

		"""
		# load config file
		with open(self._device["config"], 'r') as stream:
			_config_tmp = yaml.load(stream)
		"""
		# update config
		if "j0" in prm:
			self._config["calibrate"]["j0"] = self._config["calibrate"]["j0"] + prm["j0"]- joint[0]
			#_config_tmp["calibrate"]["j0"] = self._config["calibrate"]["j0"]
		if "j1" in prm:
			self._config["calibrate"]["j1"] = self._config["calibrate"]["j1"] + prm["j1"]- joint[1]
			#_config_tmp["calibrate"]["j1"] = self._config["calibrate"]["j1"]
		if "j2" in prm:
			self._config["calibrate"]["j2"] = self._config["calibrate"]["j2"] + prm["j2"]- joint[2]
			#_config_tmp["calibrate"]["j2"] = self._config["calibrate"]["j2"]
		if "j3" in prm:
			self._config["calibrate"]["j3"] = (self._config["calibrate"]["j3"] + prm["j3"]- joint[3])%360
			#_config_tmp["calibrate"]["j3"] = self._config["calibrate"]["j3"]
		if "j4" in prm:
			self._config["calibrate"]["j4"] = (self._config["calibrate"]["j4"] + prm["j4"]- joint[4])%360
			#_config_tmp["calibrate"]["j4"] = self._config["calibrate"]["j4"]

		"""
		# save calibrate
		with open(self._device["config"], 'w') as yaml_file:
			yaml.dump(_config_tmp, yaml_file, default_flow_style=False)
		"""
		
		# add to log
		self.config()

		return self.position("joint")
	
	def calibrate_backup(self, prm):

		# robot is homed
		home = list(json.loads(self.homed()).values())
		if not all(home[0:5]):
			return None

		# current joint
		joint = np.copy(self._joint)

		# read json
		try:
			prm = json.loads(prm)
		except:
			pass

		# dictionary or list
		if type(prm) == list: # list
			if len(prm) > 6:
				return None
			prm_tmp = {"j"+str(i): prm[i] for i in range(len(prm))}
			prm = prm_tmp

		if type(prm) == dict:
			if not all([x in ["j0", "j1", "j2", "j3", "j4", "j5"] for x in prm.keys()]):
				return None
		# not empty
		if not prm:
			return None

		# check prm and validate
		if "j0" in prm and any([prm["j0"] > self.limit_base[1], prm["j0"] < self.limit_base[0]]):
			return None
		if "j1" in prm and any([prm["j1"] > self.limit_base[1], prm["j1"] < self.limit_base[0]]):
			return None
		if "j2" in prm and any([prm["j2"] > self.limit_base[1], prm["j2"] < self.limit_base[0]]):
			return None


		# set_joint
		if not self.set_joint(prm):
			return None

		# update config
		if "j0" in prm:
			self._config["calibrate"]["j0"] = self._config["calibrate"]["j0"] + prm["j0"]- joint[0]
		if "j1" in prm:
			self._config["calibrate"]["j1"] = self._config["calibrate"]["j1"] + prm["j1"]- joint[1]
		if "j2" in prm:
			self._config["calibrate"]["j2"] = self._config["calibrate"]["j2"] + prm["j2"]- joint[2]
		if "j3" in prm:
			self._config["calibrate"]["j3"] = (self._config["calibrate"]["j3"] + prm["j3"]- joint[3])%360
		if "j4" in prm:
			self._config["calibrate"]["j4"] = (self._config["calibrate"]["j4"] + prm["j4"]- joint[4])%360

		# add to log
		self.config()

		return self.position("joint")

	# return: True False
	def _wait_for_command(self, command_list, max_time):
		while len(command_list) and time.time() < max_time and self._device["connection"]:
			if command_list[0]["id"] <= self._system["command"][5][-1]["id"]:
				command_list.pop(0)
			else:
				time.sleep(0.02)

		if len(command_list):
			return False
		return True

	def _append_commands(self,commands):
		command_id_list = list(range(self._system["command_id"]+1, self._system["command_id"]+len(commands)+1))
		# append M2
		commands +=  [{"command": "g2core", "prm": "M2", "display": False}]

		f_commands = self._format_command(commands)
		# add commands
		self._system["command"][0] += f_commands
		# add to log
		#_cmd_log = [{"id": cmd["id"],"key": cmd["key"], "state": cmd["state"]} for cmd in f_commands]
		self._log_add( [{"id": cmd["id"],"key": cmd["key"], "state": cmd["state"]} for cmd in f_commands], "line_update")

		# change device state to running
		self._device["state"] = 1
		# add to log
		#result = self.device()
		self._log_add(json.loads(self.device()), "device")

		return command_id_list

	def _flush_commands(self, halt = True):
		if halt:
			# valid commands id
			self._system["valid_command_id"] = self._system["command_id"] + 1
			# copy ???
			#_command_list = list(self._system["command"][0:3])
			_command_list = list(self._system["command"][0:4])
			# clean ???
			#self._system["command"][0:3] = [[], [], []]
			self._system["command"][0:4] = [[], [], [], []]
			_result = self.play({"command": "halt", "display": False})
			_time = time.time()
			while self._device["state"] != 0 and time.time() < _time + 1:
				time.sleep(0.001)

			# send jobs to log
			self._log_add([{"id": cmd["id"], "key":cmd["key"], "state":-1} for X in _command_list for cmd in  X], "line_update" )
			"""
			???
			for _command in _command_list:
				_command = self._command_mask(_command)
				for _cmd in _command:
					self._log_add(json.dumps(_cmd), "halt_command")
			"""
			# update travel final
			if self._device["state"] == 0:
				self._system["travel_final"] = np.copy(self._travel)
				return True
			self._log_add(True, "halt")
		else:
			# copy
			_command_list = list(self._system["command"][0:2])
			# clean
			self._system["command"][0:2] = [[], []]

			# update travel final
			if self._system["command"][2]:
				self._system["travel_final"] = np.copy(self._system["command"][2][0]["travel_final"])
			elif self._system["command"][3]:
				self._system["travel_final"] = np.copy(self._system["command"][3][0]["travel_final"])
			elif self._system["command"][4]:
				self._system["travel_final"] = np.copy(self._system["command"][4][0]["travel_final"])
			elif self._system["command"][5]:
				self._system["travel_final"] = np.copy(self._system["command"][5][0]["travel_final"])
			else:
				self._system["travel_final"] = np.copy(self._travel)

			# add new job
			self.play({"command": "g2core", "prm": "{id:n}", "display": False})

			# add it to log
			self._log_add([{"id": cmd["id"], "key":cmd["key"], "state":-1} for X in _command_list for cmd in  X], "line_update" )
			"""
			???
			# send jobs to log
			for _command in _command_list:
				_command = self._command_mask(_command)
				for _cmd in _command:
					self._log_add(json.dumps(_cmd), "pause_command")
			self._log_add(True, "pause")
			"""
			return _command_list

		return False

	"""
	append: append the commands to the last job
		if the job is clsoe to complete then it will wait for it and submit it as a new job
	fulfill
		[{"command_id": len(commands), "job_id":self._job["id"], "command": "g2core", "prm": "M2" + " n "+ str(2* self._job["id"] + 1)}]
	"""
	def play(self, commands, append = True):
		# connection exists
		if self._device["connection"] == 0:
			_rtn = {"error": 1 , "message": "the device is not connected"}
			self._log_add(_rtn, "play")
			return json.dumps(_rtn)

		# sanitate
		commands = self._sanitate_command(commands)

		# key
		try:
			key = commands[0]["key"]
		except:
			key = None

		if not commands:
			_rtn = {"error": 2 , "message": "not a valid format", "key": key}
			self._log_add(_rtn, "play")
			return json.dumps(_rtn)

		# system is stopped == 0
		# system is running == 1
		"""
		Find the last M2 and remove it
		"""
		# update travel
		if self._device["state"] == 0:
			self._system["travel_final"] = np.copy(self._travel)
		if append or self._device["state"] == 0:
			id_list = self._append_commands(commands)
			#return self.command(id = id_list)
			data =  self.command({"id": id_list})
			return data
		else:
			_flush_result = self._flush_commands()
			if _flush_result:
				return self.play(commands)
		_rtn = {"error": 3 , "message": "timeout, try again", "key": key}
		return json.dumps(_rtn)


	"""
	empty commands 0, 1, 2
	"""
	def pause(self):
		return json.dumps(self._flush_commands(False))

	"""
	if error and not fulfill then do not run that command
	# command[0] process and moves to command[1]
	"""
	def _command_thread(self):
		while not self._stop:
			self._command_compile()
			time.sleep(0.001)

	"""
	Every single command is a dict
	If there is any error, we send it all to the canceled commands
	self._system["command"][6]
	"""
	def _command_compile(self):
		# not disconnected
		if self._device["connection"] == 0:
			return False

		# get the last command[0]
		try:
			#_command = dict(self._system["command"][0][0])
			_command = copy.deepcopy(self._system["command"][0][0])
			if "prm" in _command:
				_prm = copy.deepcopy(_command["prm"])
			else:
				_prm = False
		except:
			return False

		"""
		# make sure it is synced
		# if not synced then this command is already in the canceled
		if _command and _command["id"] < self._system["valid_command_id"]:
			return False
		"""

		# json command to gc_list
		gcs = self._json_to_method(_command) #{'gc_list', 'travel_final', "status", "message"}
		# system is synced
		# if not synced then this command is already in the canceled
		if _command["id"] == self._system["command"][0][0]["id"]:
			self._system["command"][0].pop(0)
		else:
			# add to log ???
			self._log_add([{"id": _command["id"], "key": _command["key"], "state": -1}], "line_update")
			return False

		# wrong format: just ignore the command and add it to the canceled
		if type(gcs) is not dict:
			# modify command
			#_command["state"] = 6
			_command["error"] = 1
			_command["message"] = "wrong format"
			# add it to cancled
			#self._system["command"][6].append(_command)
			#self._log_add(_command, "ignore") ???
			# add to log ???
			self._log_add([{"id": _command["id"], "key": _command["key"], "state": -1}], "line_update")

			return False

		# state modification
		_command["state"] = 1
		# message
		if "message" in gcs:
			_command["message"] = gcs["message"]

		"""
		error
		if there is an error and fulfill is false
		then send evrything in command 0 to the cancled
		"""
		if "status" in gcs:
			_command["error"] = gcs["status"]
			if not _command["fulfill"] and _command["error"]:
				# add other commands except M2
				commands = [_command] + list(self._system["command"][0][0:-1])
				self._system["command"][0]= self._system["command"][0][-1]
				for command in  commands:
					command["message"] = "canceled because of an error in the command with the id : " + str(_command["id"])
				#_cmd_log = [{"id": cmd["id"], "key": cmd["key"], "state": -1} for cmd in commands]
				self._log_add( [{"id": cmd["id"], "key": cmd["key"], "state": -1} for cmd in commands], "line_update")
				return False
				"""
				???
				# add _command
				#_command["state"] = 6
				#self._system["command"][6].append(_command)
				self._log_add(_command, "error")
				# add other commands except M2
				commands = list(self._system["command"][0][0:-1])
				self._system["command"][0]= self._system["command"][0][0:-1]
				for command in  commands:
					#command["state"] = 6
					command["message"] = "canceled because of an error in the command with the id : " + str(_command["id"])
					#self._system["command"][6].append(command)
					self._log_add(command, "error")
				"""
		# no error
		if "gc_list" in gcs:
			# not empty
			_command["gc"] = self._method_to_gc(gcs["gc_list"], _command["id"])

		if "travel_final" in gcs:
			_command["travel_final"] = gcs["travel_final"]
			self._system["travel_final"] = gcs["travel_final"]


		# append to list 1
		if "prm" in _command:
			_command["prm"] = _prm
		self._system["command"][1].append(_command)
		# add to log ???
		self._log_add([{"id": _command["id"], "key": _command["key"], "state": _command["state"]}], "line_update")

		return True

	def _send(self):
		_gc = True
		while not self._stop: # main loop
			if self._device["connection"] > 0:
				# still gcodes in the list
				try:
					if type(_gc) != dict:
						# get gcode from the list
						_gc = {"id": self._system["command"][2][0]["id"],
								"gc": self._system["command"][2][0]["gc"].pop(0)
								}
					_gc = self._process_gc(_gc)
				except:
					# move the command from 2 to 3
					try:
						_command = self._system["command"][2].pop(0)
						_command["state"] = 3
						self._system["command"][3] += [_command]
						self._log_add([{"id": _command["id"], "key": _command["key"], "state": _command["state"]}], "line_update")
					except:
						pass

					# move the command from 1 to 2
					try:
						_command = self._system["command"][1].pop(0)
						_command["state"] = 2
						self._system["command"][2] += [_command]
						self._log_add([{"id": _command["id"], "key": _command["key"], "state": _command["state"]}], "line_update")

					except:
						pass

			time.sleep(0.001)

	def _receive(self):
		_error = 0
		while not self._stop: # main loop
			if self._device["connection"] > 0:
				try:
					rsp = self._port_read()
					if rsp:
						self._process_response(rsp)
						_error = 0

				except Exception as ex:
					_printx(self._prnt,"test_3", ex)
					_error += 1
					if _error > self._read_error:
						self._port_close()
						_error = 0

			time.sleep(.005)

	def _method_to_gc(self, gc_list, command_id = None):
		# send_list
		_send_list = []
		for gc in gc_list:
			if type(gc) is str:
				_send_list.append(gc)
			elif type(gc) is dict:
				_send_list.append(json.dumps(gc))


		# add line
		if _send_list and command_id is not None:
			if "{" in _send_list[-1]: # dictionary format
				_send_list.append(" n "+ str(2* command_id + 2))
			elif all([x not in _send_list[-1] for x in [" n ", "!", "%", "~"]]):
				_send_list[-1] += " n "+ str(2* command_id + 2)

		return _send_list

	def _json_to_method(self, command):
		try:
			if "prm" in command:
				return getattr(self, "_"+command['command'])(command['prm'])
			else:
				return getattr(self, "_"+command['command'])()
		except Exception as x:
			_printx(self._prnt,x)
			return False

	"""
	with every command send change the device state to 1
	"""
	def _process_gc(self, gc):
		# check format
		if type(gc) != dict or "gc" not in gc or "id" not in gc:
			return False

		# system is synced
		if gc["id"] < self._system["valid_command_id"]:
			return False

		# ignore m2
		if "M2" in gc["gc"] and len(self._system["command"][0]) + len(self._system["command"][1]):
			return False

		"""
		# system is bussy
		if not all([self._system["lines_to_send"] > 0, self._system["qr"] > 40]):
			return gc
		"""
		# system is bussy
		if gc["gc"] in ["!" , "%"]:
			pass
		elif not all([self._system["lines_to_send"] > 0, self._system["qr"] > 40]):
			return gc



		# send command
		_printx(self._prnt,"send: ", gc["gc"])

		# add log
		self._log_add(gc["gc"], "send")

		send_command = gc["gc"] + '\n'
		self._port.write(send_command.encode())
		# update line and line to send
		line = [x for x in gc["gc"].split('\n') if x]
		self._system["lines_to_send"] -= len([x for x in line if x not in ['%', '~', '!']])
		if gc["gc"] == "%":
			self._system["lines_to_send"] = 4
			_printx(self._prnt,"lines to send: ", self._system["lines_to_send"])

		# sleep
		sleep = 0.35 - 0.00625*self._system["qr"]
		_printx(self._prnt,"sleep: :", sleep)
		time.sleep(sleep)
		return True

	def _process_response(self, response):
		# response
		if 'r' in response:
			self._system["lines_to_send"]  = min(4, self._system["lines_to_send"] + 1)
			#_r = dict(response["r"])
			_r = copy.deepcopy(response["r"])
			if "f" in response:
				_r["f"] = response["f"]
			self._process_response(_r)
			return True

		# status report
		if 'sr' in response:
			# we should find the position updates if there is any
			#_r = dict(response["sr"])
			_r = copy.deepcopy(response["sr"])
			self._process_response(_r)
			return True

		_printx(self._prnt,"receive: ", response)

		# add to log
		self._log_add(response, "receive")

		# qr update
		self._qr_update(response)
		# travel update
		self._travel_update(response)
		# id
		self._id_update(response)
		# probe
		self._probe_update(response)
		# gpa, jt, ct
		self._motion_update(response)
		# homing
		self._home_update(response)
		# firmware version
		self._fv_update(response)
		# io update
		self._io_update(response)
		# id
		self._line_update(response)
		# end of script
		self._end_of_script(response)


	def _qr_update(self, response):
		if 'qr' in response and type(response["qr"]) == int:
			self._system["qr"] = response["qr"]

			# update line to send
			if "qi" in response and "qo" in response:
				if response["qr"] == 48 and response["qi"] == 0 and response["qo"] == 0:
					self._system["lines_to_send"] = 4


	def _id_update(self, response):
		if "id" in response:
			self._device["id"] = response["id"]


	def _probe_update(self, response):
		if "prb" in response:
			for k in response["prb"]:
				self._system["probe"][k] = response["prb"][k]


	def _motion_update(self, response):
		if "gpa" in response and type(response["gpa"]) != str:
			self._config["motion"]["gpa"] = response["gpa"]
		if "jt" in response and type(response["jt"]) != str:
			self._config["motion"]["jt"] = response["jt"]
		if "ct" in response and type(response["ct"]) != str:
			self._config["motion"]["ct"] = response["ct"]

	def _fv_update(self, response):
		if "fv" in response:
			self._device["fv"] = response["fv"]


	def _io_update(self, response):
		change = False
		# this is when outputs are not in response
		if "do6mo" in response:
			change = True
			self._io["out1"] = response["do6mo"]
		if "do7mo" in response:
			change = True
			self._io["out2"] = response["do7mo"]
		if "do8mo" in response:
			change = True
			self._io["out3"] = response["do8mo"]
		if "do9mo" in response:
			change = True
			self._io["out4"] = response["do9mo"]
		if "do10mo" in response:
			change = True
			self._io["out5"] = response["do10mo"]
		###

		if "out1" in response:
			change = True
			self._io["out1"] = response["out1"]
		if "out2" in response:
			change = True
			self._io["out2"] = response["out2"]
		if "out3" in response:
			change = True
			self._io["out3"] = response["out3"]
		if "out4" in response:
			change = True
			self._io["out4"] = response["out4"]
		if "out5" in response:
			change = True
			self._io["out5"] = response["out5"]

		if "do1mo" in response:
			change = True
			self._io["do1mo"] = response["do1mo"]
		if "do2mo" in response:
			change = True
			self._io["do2mo"] = response["do2mo"]
		if "do3mo" in response:
			change = True
			self._io["do3mo"] = response["do3mo"]
		if "do4mo" in response:
			change = True
			self._io["do4mo"] = response["do4mo"]

		if "in1" in response:
			change = True
			self._io["in1"] = response["in1"]
		if "in7" in response:
			change = True
			self._io["in2"] = response["in7"]
		if "in8" in response:
			change = True
			self._io["in3"] = response["in8"]
		if "in9" in response:
			change = True
			self._io["in4"] = response["in9"]

		if "di1mo" in response:
			change = True
			self._io["di1mo"] = response["di1mo"]
		if "di7mo" in response:
			change = True
			self._io["di2mo"] = response["di7mo"]
		if "di8mo" in response:
			change = True
			self._io["di3mo"] = response["di8mo"]
		if "di9mo" in response:
			change = True
			self._io["di4mo"] = response["di9mo"]

		if change:
			self._log_add(json.loads(self.io()), "io")

	def _travel_update(self, response):
		mapping = {"posx": 0, "posy": 1, "posz": 2, "posa": 3, "posb": 4, "posc": 5}
		joint = list(set(mapping.keys()) & set(response.keys()))

		# update travel
		for j in joint:
			self._travel[mapping[j]] = response[j]

		# update joint xyz
		if joint:
			try:
				self._joint = self._travel_to_joint(np.copy(self._travel))
				self._xyz = self._travel_to_xyz(np.copy(self._travel))

				# add to log
				self._log_add(json.loads(self.position()), "joint")
				self._log_add(json.loads(self.position("xyz")), "xyz")
			except:
				pass

	def _end_of_script(self, response):
		if "stat" in response:
			# update stat
			self._system["state"] = response["stat"]
			if response["stat"] == 4 and self._device["state"] == 0.5:
				self._device["state"] = 0
				self._log_add(json.loads(self.device()), "device")

	def _line_update(self, response):
		_command_update = []
		if "line" in response:
			if response["line"]%2 == 0:
				command_id = (response["line"]-2)/2

				# check for completing
				if command_id == self._system["command_id"]:
					self._device["state"] = 0.5
					self._log_add(json.loads(self.device()), "device")

				# 4 to 5
				try:
					while self._system["command"][4][0]["id"] < command_id:
						_command = self._system["command"][4].pop(0)
						_command["state"] = 5
						self._system["command"][5].append(_command)
						# ??? limit length
						self._system["command"][5] = self._system["command"][5][-100:]
						# add to log
						_command_update.append({"id": _command["id"], "state": 5,"key": _command["key"]})
				except:
					pass

				# 3 to 5
				try:
					while self._system["command"][3][0]["id"] < command_id:
						_command = self._system["command"][3].pop(0)
						_command["state"] = 5
						self._system["command"][5].append(_command)
						# ??? limit length
						self._system["command"][5] = self._system["command"][5][-100:]
						# add to log
						_command_update.append({"id": _command["id"], "state": 5, "key": _command["key"]})
				except:
					pass

				# 3 to 4
				try:
					while self._system["command"][3][0]["id"] == command_id:
						_command = self._system["command"][3].pop(0)
						_command["state"] = 4
						self._system["command"][4].append(_command)
						# add to log
						_command_update.append({"id": _command["id"], "state": 4, "key": _command["key"]})
				except:
					pass

				if _command_update:
					self._log_add(_command_update, "line_update")

	def _home_update(self, response):
		# update home_robot
		update = False
		if "tt32" in response:
			home_key = list(set(response["tt32"].keys()) & set(self._home_robot.keys()))
			_printx(self._prnt,"tt32: ", home_key)
			for k in home_key:
				update = True
				self._home_robot[k] = response["tt32"][k]

		# add to log
		if update:
			self._log_add(json.loads(self.homed()), "homed")

		# update home_system
		home_key = list(set(response.keys()) & set(self._home_system))
		for k in home_key:
			self._home_system[k] = response[k]


	# =================================================================
	# configuration
	# =================================================================
	"""
	if ["unit"]["length"] == "mm"
	bring everything to inch
	"""
	def _init_config(self):
		# Read YAML file
		with open(self._device["config"], 'r') as stream:
			self._config = yaml.load(stream, Loader=yaml.SafeLoader)

		if self._config["unit"]["length"] == "mm":
			# speed_xyz
			self._config["default_speed"]["xyz"] = self._mm_to_inch(self._config["default_speed"]["xyz"])
			# jerk_xyz
			self._config["default_jerk"]["xyz"] = self._jerk_mm_to_inch(self._config["default_jerk"]["xyz"])
			# toolhead
			for k in self._config["toolhead"]:
				self._config["toolhead"][k] = self._mm_to_inch(self._config["toolhead"][k])



	def save_config(self,save_path = None):
		if save_path:
			#self._config_path = save_path
			self._device["config"] = save_path

		with open(self._device["config"], 'w') as yaml_file:
			_config = self.config()
			_config = json.loads(_config)
			yaml.dump(_config, yaml_file, default_flow_style=False)

		# add log
		result = self.device()
		self._log_add(json.loads(result), "device")
		return result

	def scale(self):
		self._log_add(self._scale, "scale")
		return json.dumps(self._scale)

	def set_scale(self , prm):
		try:
			prm = json.loads(prm)
		except:
			pass
		if type(prm) is dict and "speed" in prm and 0 < prm["speed"] <= 1:
			self._scale["speed"] = prm["speed"]

		if type(prm) is dict and "jerk" in prm and 0 < prm["jerk"] <= 1:
			self._scale["jerk"] = prm["jerk"]

		# modify
		return self.scale()

	def config(self, prm = None):
		try:
			prm = json.loads(prm)
		except:
			pass

		# copy
		tmp_config = copy.deepcopy(self._config)


		# length
		tmp_config["default_jerk"]["joint"] = tmp_config["default_jerk"]["joint"][0:tmp_config["axis"]["number"]]
		tmp_config["default_jerk"]["xyz"] = tmp_config["default_jerk"]["xyz"][0:tmp_config["axis"]["number"]]

		# mm
		if tmp_config["unit"]["length"] == "mm":
			# speed_xyz
			tmp_config["default_speed"]["xyz"] = self._inch_to_mm(tmp_config["default_speed"]["xyz"])

			# jerk_xyz
			tmp_config["default_jerk"]["xyz"] = self._jerk_inch_to_mm(tmp_config["default_jerk"]["xyz"])
			# toolhead
			for k in tmp_config["toolhead"]:
				tmp_config["toolhead"][k] = self._inch_to_mm(tmp_config["toolhead"][k])

		# display keys
		if prm:
			_display_key = [k for k in tmp_config if k in prm]
			_rtn = {k: tmp_config[k] for k in _display_key}
			self._log_add(_rtn, "config")
			return json.dumps(_rtn)

		self._log_add(tmp_config, "config")
		return json.dumps(tmp_config)

	def axis(self):
		return self.config(["axis"])

	def set_axis(self , prm):
		try:
			prm = json.loads(prm)
		except:
			pass
		if type(prm) is dict and "number" in prm and prm["number"] in [5, 6]:
			self._config["axis"]["number"] = prm["number"]

		# modify
		return self.axis()


	def unit(self):
		#_rtn = self.config(["unit"])
		#self._log_add(json.loads(_rtn), "unit")
		return self.config(["unit"])

	def set_unit(self , prm):
		try:
			prm = json.loads(prm)
		except:
			pass
		if type(prm) is dict and "length" in prm and prm["length"] in ["inch", "mm"]:
			self._config["unit"]["length"] = prm["length"]

		# modify
		return self.config()

	def motion(self):
		return self.config(["motion"])
		#self._log_add(result, "motion")
		#return result

	def default_speed(self):
		return self.config(["default_speed"])

	def set_default_speed(self , prm):
		try:
			prm = json.loads(prm)
		except:
			pass

		# update config
		if "joint" in prm:
			self._config["default_speed"]["joint"] = prm["joint"]


		if "xyz" in prm:
			if self._config["unit"]["length"] == "mm":
				self._config["default_speed"]["xyz"] = self._mm_to_inch(prm["xyz"])
			else:
				self._config["default_speed"]["xyz"] = prm["xyz"]

		return self.default_speed()


	def default_jerk(self):
		return self.config(["default_jerk"])

	def set_default_jerk(self , prm):
		try:
			prm = json.loads(prm)
		except:
			pass

		# update config
		if "joint" in prm:
			if type(prm["joint"]) == list and len(prm["joint"]) == self._config["axis"]["number"]:
				self._config["default_jerk"]["joint"][0:self._config["axis"]["number"]] = prm["joint"]

		if "xyz" in prm:
			if type(prm["xyz"]) == list and len(prm["xyz"]) == self._config["axis"]["number"]:
				if self._config["unit"]["length"] == "mm":
					self._config["default_jerk"]["xyz"][0:self._config["axis"]["number"]] = self._jerk_mm_to_inch(prm["xyz"])
				else:
					self._config["default_jerk"]["xyz"][0:self._config["axis"]["number"]] = prm["xyz"]

		return self.default_jerk()


	def _inch_to_mm(self, x):
		if x == None:
			return None
		return x * 25.4

	def _mm_to_inch(self, x):
		if x == None:
			return None
		return x/25.4

	def _jerk_mm_to_inch(self, jerk):
		# xyz
		_jerk_tmp = [self._mm_to_inch(x) for x in jerk[0:3]]
		# abc
		_jerk_tmp += jerk[3:]
		return _jerk_tmp

	def _jerk_inch_to_mm(self, jerk):
		# xyz
		#_jerk_tmp = [self._mm_to_inch(x) for x in jerk[0:3]]
		_jerk_tmp = [self._inch_to_mm(x) for x in jerk[0:3]]
		# abc
		_jerk_tmp += jerk[3:]
		return _jerk_tmp
	"""
	limit {"j0": [min, max], "j1": [min, max],...}
	if soemthins is None then we ignore it
	"""
	def set_limit(self, limit):
		# json
		try:
			limit = json.loads(limit)
		except:
			pass

		
		if "j0" in limit and len(limit["j0"]) == 2 and limit["j0"][0]<= limit["j0"][1] :
			self._config["limit"]["j0"] = limit["j0"]

		if "j1" in limit and len(limit["j1"]) == 2 and limit["j1"][0]<= limit["j1"][1] :
			self._config["limit"]["j1"] = limit["j1"]

		if "j2" in limit and len(limit["j2"]) == 2 and limit["j2"][0]<= limit["j2"][1] :
			self._config["limit"]["j2"] = limit["j2"]

		# save limit
		#self.save_config()

		return self.limit()
	
	def set_limit_backup(self, limit):
		# json
		try:
			limit = json.loads(limit)
		except:
			pass


		if "j0" in limit and len(limit["j0"]) == 2 and self.limit_base[0]<=limit["j0"][0]<= limit["j0"][1] <= self.limit_base[1]:
			self._config["limit"]["j0"] = limit["j0"]

		if "j1" in limit and len(limit["j1"]) == 2 and self.limit_base[0]<=limit["j1"][0]<= limit["j1"][1] <= self.limit_base[1]:
			self._config["limit"]["j1"] = limit["j1"]

		if "j2" in limit and len(limit["j2"]) == 2 and self.limit_base[0]<=limit["j2"][0]<= limit["j2"][1] <= self.limit_base[1]:
			self._config["limit"]["j2"] = limit["j2"]

		# save limit
		#self.save_config()

		return self.limit()

	def limit(self):
		#result =  self.config(["limit"])
		# log toolhead
		#self._log_add({"limit": json.loads(result)}, "config")
		return self.config(["limit"])

	"""
	status:
		0: limit not passed
		100: limit passed
	joint: indices of joints that passed the limit
	"""
	def _limit_check(self, joint):
		limit_passed = [i for i in range(len(self._config["limit"])) if not(self._config["limit"]["j"+ str(i)][0] <= joint[i] <= self._config["limit"]["j"+ str(i)][1])]
		status = 0
		if limit_passed:
			status = 100
		return {"status": status, "joint": limit_passed}

	# =================================================================
	# utility
	# =================================================================

	"""
	forward kinematics: joint to xyz
	"""
	def f_k(self, joint):
		try:

			# joint to radian
			teta = [math.radians(j) for j in joint]


			# first we find x, y, z assuming base rotation is zero (teta_0 = 0). Then we rotate everything
			# then we rotate the robot around z axis for teta_0
			tmp = self._bx + self._l1 * math.cos(teta[1]) + self._l2 * math.cos(teta[1] + teta[2]) + self._config["toolhead"]["x"] * math.cos(teta[1] + teta[2] + teta[3])
			x = tmp * math.cos(teta[0])
			y = tmp * math.sin(teta[0])
			z = self._bz + self._l1 * math.sin(teta[1]) + self._l2 * math.sin(teta[1] + teta[2]) + self._config["toolhead"]["x"] * math.sin(teta[1] + teta[2] + teta[3])
			alpha = teta[1] + teta[2] + teta[3]
			beta = teta[4]

			alpha = math.degrees(alpha)
			beta = math.degrees(beta)

			_rtn = [x, y, z]
			if self._config["unit"]["length"] == "mm":
				_rtn = [self._inch_to_mm(c) for c in _rtn]

			return _rtn +[alpha, beta] + joint[5:]

		except:
			return None


	"""
	inverse kinematics: xyz to joint
	"""
	def i_k(self, xyz):
		try:
			x = xyz[0]
			y = xyz[1]
			z = xyz[2]
			
			if self._config["unit"]["length"] == "mm":
				x = self._mm_to_inch(x)
				y = self._mm_to_inch(y)
				z = self._mm_to_inch(z)			
			

			alpha = xyz[3]
			beta = xyz[4]

			alpha = math.radians(alpha)
			beta = math.radians(beta)

			# first we find the base rotation
			teta_0 = math.atan2(y, x)

			# next we assume base is not rotated and everything lives in x-z plane
			x = math.sqrt(x ** 2 + y ** 2)

			# next we update x and z based on base dimensions and hand orientation
			x -= (self._bx + self._config["toolhead"]["x"] * math.cos(alpha))
			z -= (self._bz + self._config["toolhead"]["x"] * math.sin(alpha))

			# at this point x and z are the summation of two vectors one from lower arm and one from upper arm of lengths l1 and l2
			# let L be the length of the overall vector
			# we can calculate the angle between l1 , l2 and L
			L = math.sqrt(x ** 2 + z ** 2)
			L = np.round(L,13) # ???
			# not valid
			if L > (self._l1 + self._l2) or self._l1 > (self._l2 + L) or self._l2 > (self._l1 + L):  # in this case there is no solution
				return None

			# init status
			status = 0
			if L > (self._l1 + self._l2) - self._delta_e or self._l1 > (self._l2 + L) - self._delta_e: # in this case there is no solution
				status = 1

			teta_l1_L = math.acos((self._l1 ** 2 + L ** 2 - self._l2 ** 2) / (2 * self._l1 * L))  # l1 angle to L
			teta_L_x = math.atan2(z, x)  # L angle to x axis
			teta_1 = teta_l1_L + teta_L_x
			# note that the other solution would be to set teta_1 = teta_L_x - teta_l1_L. But for the dynamics of the robot the first solution works better.
			teta_l1_l2 = math.acos((self._l1 ** 2 + self._l2 ** 2 - L ** 2) / (2 * self._l1 * self._l2))  # l1 angle to l2
			teta_2 = teta_l1_l2 - math.pi
			teta_3 = alpha - teta_1 - teta_2
			teta_4 = beta
			teta_0 = math.degrees(teta_0)
			teta_1 = math.degrees(teta_1)
			teta_2 = math.degrees(teta_2)
			teta_3 = math.degrees(teta_3)
			teta_4 = math.degrees(teta_4)

			return [teta_0, teta_1, teta_2, teta_3, teta_4] + xyz[5:]

		except:
			return None



	"""
	input: joint
	output: xyz
	"""
	def _joint_to_xyz(self, joint):
		if any(joint == None):
			return np.array([None for i in range(len(joint))])

		# joint to radian
		teta_0 = math.radians(joint[0])
		teta_1 = math.radians(joint[1])
		teta_2 = math.radians(joint[2])
		teta_3 = math.radians(joint[3])
		teta_4 = math.radians(joint[4])

		# first we find x, y, z assuming base rotation is zero (teta_0 = 0). Then we rotate everything
		# then we rotate the robot around z axis for teta_0
		tmp = self._bx + self._l1 * math.cos(teta_1) + self._l2 * math.cos(teta_1 + teta_2) + self._config["toolhead"]["x"] * math.cos(teta_1 + teta_2 + teta_3)
		x = tmp * math.cos(teta_0)
		y = tmp * math.sin(teta_0)
		z = self._bz + self._l1 * math.sin(teta_1) + self._l2 * math.sin(teta_1 + teta_2) + self._config["toolhead"]["x"] * math.sin(teta_1 + teta_2 + teta_3)
		alpha = teta_1 + teta_2 + teta_3
		beta = teta_4

		alpha = math.degrees(alpha)
		beta = math.degrees(beta)

		if len(joint) == 6:
			return np.array([x, y, z, alpha, beta, joint[5]]) # [x, y, z, alpha, beta, joints[5]]
		else:
			return np.array([x, y, z, alpha, beta]) # [x, y, z, alpha, beta]


	"""
	status: 0: valid and safe xyz
	status: 1: valid but not safe xyz
	status: 2: not a valid xyz
	"""
	def _xyz_to_joint(self,xyz):
		if any(xyz == None): # xyz contains None coordinate
			return {"joint": np.array([None for i in range(len(xyz))]), "status": 2}

		x = xyz[0]
		y = xyz[1]
		z = xyz[2]
		alpha = xyz[3]
		beta = xyz[4]

		alpha = math.radians(alpha)
		beta = math.radians(beta)

		# first we find the base rotation
		teta_0 = math.atan2(y, x)

		# next we assume base is not rotated and everything lives in x-z plane
		x = math.sqrt(x ** 2 + y ** 2)

		# next we update x and z based on base dimensions and hand orientation
		x -= (self._bx + self._config["toolhead"]["x"] * math.cos(alpha))
		z -= (self._bz + self._config["toolhead"]["x"] * math.sin(alpha))

		# at this point x and z are the summation of two vectors one from lower arm and one from upper arm of lengths l1 and l2
		# let L be the length of the overall vector
		# we can calculate the angle between l1 , l2 and L
		L = math.sqrt(x ** 2 + z ** 2)
		L = np.round(L,13) # ???
		# not valid
		if L > (self._l1 + self._l2) or self._l1 > (self._l2 + L) or self._l2 > (self._l1 + L):  # in this case there is no solution
			return {"joint": np.array([None for i in range(len(xyz))]), "status": 2}

		# init status
		status = 0
		if L > (self._l1 + self._l2) - self._delta_e or self._l1 > (self._l2 + L) - self._delta_e: # in this case there is no solution
			status = 1

		teta_l1_L = math.acos((self._l1 ** 2 + L ** 2 - self._l2 ** 2) / (2 * self._l1 * L))  # l1 angle to L
		teta_L_x = math.atan2(z, x)  # L angle to x axis
		teta_1 = teta_l1_L + teta_L_x
		# note that the other solution would be to set teta_1 = teta_L_x - teta_l1_L. But for the dynamics of the robot the first solution works better.
		teta_l1_l2 = math.acos((self._l1 ** 2 + self._l2 ** 2 - L ** 2) / (2 * self._l1 * self._l2))  # l1 angle to l2
		teta_2 = teta_l1_l2 - math.pi
		teta_3 = alpha - teta_1 - teta_2
		teta_4 = beta
		teta_0 = math.degrees(teta_0)
		teta_1 = math.degrees(teta_1)
		teta_2 = math.degrees(teta_2)
		teta_3 = math.degrees(teta_3)
		teta_4 = math.degrees(teta_4)


		if len(xyz) == 6:
			joint = np.array([teta_0, teta_1, teta_2, teta_3, teta_4, xyz[5]])
		else:
			joint = np.array([teta_0, teta_1, teta_2, teta_3, teta_4])

		return {"joint": joint, "status": status}

	# return: np.array
	def _travel_to_joint(self, travel):
		try:
			if travel[2] >= 800:  # travel is joint
				travel[2] -= 1000
				joint = np.copy(travel)
				joint[3] = 0.5 * (travel[4] - travel[3] )
				joint[4] = 0.5 * (-travel[4] - travel[3] )
				return joint
			else: # travel is xyz
				return self._xyz_to_joint(travel)["joint"]
		except:
			return np.array([None for i in range(len(travel))])

	def _travel_to_xyz(self, travel):
		try:
			if travel[2] >= 800:  # travel is joint
				joint = self._travel_to_joint(travel)
				return self._joint_to_xyz(joint)
			return travel
		except:
			return np.array([None for i in range(len(travel))])

	def _joint_to_travel(self, joint):
		joint[2] += 1000
		travel = np.copy(joint)
		travel[3] = -joint[3]-joint[4]
		travel[4] = joint[3]-joint[4]
		#return joint
		return travel

	def _xyz_to_travel(self, xyz):
		return xyz

	def _joint_validate(self, joint_init, joint_final):
		joint_achieve = np.copy(joint_final)

		status = 0
		message = ""
		for i in range(len(self._config["limit"])):

			if joint_init[i] > self._config["limit"]["j"+ str(i)][1]:
				# above
				joint_achieve[i] = max(joint_final[i], self._config["limit"]["j"+ str(i)][0])
				message = "initial position (joint: "+str(joint_init)+") is out of limit"
				status = 100
						
			elif self._config["limit"]["j"+ str(i)][0] <= joint_init[i] <= self._config["limit"]["j"+ str(i)][1]:
				# between
				joint_achieve[i] = min(max(joint_final[i], self._config["limit"]["j"+ str(i)][0]),self._config["limit"]["j"+ str(i)][1] )
			else:
				# under
				joint_achieve[i] = min(joint_final[i],self._config["limit"]["j"+ str(i)][1])
				message = "initial position (joint: "+str(joint_init)+") is out of limit"
				status = 0


		if not np.array_equal(joint_achieve, joint_final):
			message = "final position (joint: "+str(joint_final)+") is out of limit"
			status = 100
		return {"status": status, "joint_achieve": joint_achieve, "joint_final": joint_final, "message": message}


	def _joint_validate_backup(self, joint_init, joint_final):
		message = ""
		joint_achieve = np.copy(joint_final)
		# check for init
		check_init = self._limit_check(joint_init)
		if check_init["status"] != 0: # no need to check final
			message = "initial position (joint: "+str(joint_init)+") is out of limit"
			return {"status": check_init["status"], "joint_achieve": joint_achieve, "joint_final": joint_final, "message": message}

		# find the achieve
		for i in range(len(self._config["limit"])):
			joint_achieve[i] = min(max(self._config["limit"]["j"+ str(i)][0], joint_achieve[i]),self._config["limit"]["j"+ str(i)][1] )

		status = 0
		if not np.array_equal(joint_achieve, joint_final):
			status = 100
			message = "final position (joint: "+str(joint_final)+") is out of limit"
		return {"status": status, "joint_achieve": joint_achieve, "joint_final": joint_final, "message": message}

	"""
	give two xyz points, find the midle and see if the middle is achievable
	"""
	def _xyz_achieve(self, xyz_init, xyz_final):
		xyz_middle = (xyz_init + xyz_final)/2
		result = self._xyz_to_joint(xyz_middle)
		joint = result["joint"]
		status = result["status"]
		if status or self._limit_check(joint)["status"]:
			return [np.copy(xyz_init), np.copy(xyz_middle)]
		return [np.copy(xyz_middle), np.copy(xyz_final)]

	def _line_validate(self, xyz_init, xyz_final):
		message = ""
		# check xyz_init
		joint = self._xyz_to_joint(xyz_init)["joint"]
		if any(joint == None):
			message = "initial position is not valid"
			#return {"status": 100, "xyz_achieve": None, "xyz_final": xyz_final, "message": message}
			return {"status": 100, "xyz_achieve": np.array([None for _ in range(len(xyz_init))]), "xyz_final": xyz_final, "message": message}
		if self._limit_check(joint)["status"]:
			message = 'initial position is out of limit, in this case use the "move" command and set the "path":"joint" to get out of the limit zone'
			return {"status": 100, "xyz_achieve": np.array([None for _ in range(len(xyz_init))]), "xyz_final": xyz_final, "message": message}

		# same point
		if np.array_equal(xyz_final, xyz_init):
			return {"status": 0, "xyz_achieve": xyz_final, "xyz_final": xyz_final}

		# rotation around B (j4)
		if np.array_equal(xyz_final[0:4], xyz_init[0:4]):
			return {"status": 0, "xyz_achieve": xyz_final, "xyz_final": xyz_final}

		#direction
		direction = xyz_final - xyz_init
		L = math.floor(np.linalg.norm(direction)/self._segment_size)
		direction = self._segment_size*direction/np.linalg.norm(direction)
		xyz_achieve = np.copy(xyz_init)

		for i in range(1,L+1):
			xyz = xyz_init + i * direction
			# check xyz
			result = self._xyz_to_joint(xyz)
			joint = result["joint"]
			status = result["status"]
			if status or self._limit_check(joint)["status"]:
				xyz_r = xyz
				for i in range(10):
					[xyz_achieve, xyz_r] = self._xyz_achieve(xyz_achieve, xyz_r)

				message = "achievable position is (xyz: " + str(xyz_achieve) +")"
				return {"status": 100, "xyz_achieve": xyz_achieve, "xyz_final": xyz_final, "message": message}
			else:
				xyz_achieve = np.copy(xyz)

		# xyz_final validate
		xyz = np.copy(xyz_final)
		result = self._xyz_to_joint(xyz)
		joint = result["joint"]
		status = result["status"]
		if status or self._limit_check(joint)["status"]:
			xyz_r = xyz
			for i in range(10):
				[xyz_achieve, xyz_r] = self._xyz_achieve(xyz_achieve, xyz_r)

			message = "achievable position is (xyz: " + str(xyz_achieve) +")"
			return {"status": 100, "xyz_achieve": xyz_achieve, "xyz_final": xyz_final, "message": message}

		return {"status": 0, "xyz_achieve": xyz_final, "xyz_final": xyz_final}


	def _move_to_gc(self,travel_final, prm):
		gc = 'X{:07.4f} Y{:07.4f} Z{:07.4f} A{:07.4f} B{:07.4f} C{:07.4f}'.format(travel_final[0], travel_final[1], travel_final[2], travel_final[3], travel_final[4], travel_final[5])

		if "gc" in prm:
			gc = prm["gc"] + gc
		else:
			gc = "G1 " + gc

		try:
			gc = gc + 'F{:07.4f}'.format(prm["speed"]*self._scale["speed"])
		except:
			pass

		gc = 'G90 ' + gc
		return gc

	def _move_to_gc_backup(self,travel_final, prm):
		gc = 'X{:07.4f} Y{:07.4f} Z{:07.4f} A{:07.4f} B{:07.4f} C{:07.4f}'.format(travel_final[0], travel_final[1], travel_final[2], travel_final[3], travel_final[4], travel_final[5])

		if "gc" in prm:
			gc = prm["gc"] + gc
		else:
			gc = "G1 " + gc

		try:
			gc = gc + 'F{:07.4f}'.format(prm["speed"])
		except:
			pass

		gc = 'G90 ' + gc
		return gc


	def _move_to_gc_backup_(self,travel_final, prm):
		gc = 'X{:07.4f} Y{:07.4f} Z{:07.4f} A{:07.4f} B{:07.4f} C{:07.4f}'.format(travel_final[0], travel_final[1], travel_final[2], travel_final[3], travel_final[4], travel_final[5])
		gc = 'G1 ' + gc
		try:
			gc = gc + 'F{:07.4f}'.format(prm["speed"])
		except:
			pass

		gc = 'G90 ' + gc
		return gc

	def _ref_change(self, travel):
		return 'G28.3X{:07.4f} Y{:07.4f} Z{:07.4f} A{:07.4f} B{:07.4f} C{:07.4f}'.format(travel[0], travel[1], travel[2], travel[3], travel[4], travel[5])

	def _joint_final(self, prm, joint_init):
		if 'j0' in prm:
			joint_init[0] = prm['j0'] + prm["movement"]*joint_init[0]
		if 'j1' in prm:
			joint_init[1] = prm['j1'] + prm["movement"]*joint_init[1]
		if 'j2' in prm:
			joint_init[2] = prm['j2'] + prm["movement"]*joint_init[2]
		if 'j3' in prm:
			joint_init[3] = prm['j3'] + prm["movement"]*joint_init[3]
		if 'j4' in prm:
			joint_init[4] = prm['j4'] + prm["movement"]*joint_init[4]
		if 'j5' in prm:
			joint_init[5] = prm['j5'] + prm["movement"]*joint_init[5]
		return joint_init

	def _xyz_final (self, prm, xyz_init):
		if 'x' in prm:
			xyz_init[0] = prm['x'] + prm["movement"]*xyz_init[0]
		if 'y' in prm:
			xyz_init[1] = prm['y'] + prm["movement"]*xyz_init[1]
		if 'z' in prm:
			xyz_init[2] = prm['z'] + prm["movement"]*xyz_init[2]
		if 'a' in prm:
			xyz_init[3] = prm['a'] + prm["movement"]*xyz_init[3]
		if 'b' in prm:
			xyz_init[4] = prm['b'] + prm["movement"]*xyz_init[4]
		if 'c' in prm:
			xyz_init[5] = prm['c'] + prm["movement"]*xyz_init[5]
		return xyz_init

	# {"command": "M100", "prm":{"out5":1, "out1":0}}
	def _form_io(self,prm):
		"""
		prm = json.dumps(prm)
		prm = prm.replace('"', "")
		return {'gc_list': ["M100("+prm+")"], 'status':0}
		"""
		# this is when outputs are not showing
		"""
		outputs with
		1->6
		2->7
		3->8
		4->9
		5->10

		inputs
		1->1
		2->7
		3->8
		4->9
		"""
		#prm_tmp = dict(prm)
		prm_tmp = copy.deepcopy(prm)
		for x in prm:
			if x in ["in2", "in3", "in4"]:
				# add new key
				prm_tmp["in"+str(int(x[-1]) + 5)] = prm_tmp[x]
				# remove x
				prm_tmp.pop(x)

			elif x in ["di2mo", "di3mo", "di4mo"]:
				# add new key
				prm_tmp["di"+str(int(x[2:-2]) + 5)+"mo"] = prm_tmp[x]
				# remove x
				prm_tmp.pop(x)

			elif x in ["out1", "out2", "out3", "out4", "out5"]:
				prm_tmp["do"+str(int(x[-1])+ 5)+"mo"] = prm[x]

			elif x in ["do6mo", "do7mo", "do8mo", "do9mo", "do10mo"]:
				prm_tmp["out"+str(int(x[2:-2])- 5)] = prm[x]

			elif x in ["do1mo", "do2mo", "do3mo", "do4mo", "do5mo"]:
				prm_tmp["do"+str(int(x[2:-2])+ 5)+"mo"] = 0
				prm_tmp["out"+x[2:-2]] = 0

		prm = prm_tmp

		prm = json.dumps(prm)
		return prm.replace('"', "")		
	
	def _M100 (self, prm):
		prm = self._form_io(prm)
		return {'gc_list': ["M100("+prm+")"], 'status':0}
	
	def _M100_backup (self, prm):
		"""
		prm = json.dumps(prm)
		prm = prm.replace('"', "")
		return {'gc_list': ["M100("+prm+")"], 'status':0}
		"""
		# this is when outputs are not showing
		"""
		outputs with
		1->6
		2->7
		3->8
		4->9
		5->10

		inputs
		1->1
		2->7
		3->8
		4->9
		"""
		#prm_tmp = dict(prm)
		prm_tmp = copy.deepcopy(prm)
		for x in prm:
			if x in ["in2", "in3", "in4"]:
				# add new key
				prm_tmp["in"+str(int(x[-1]) + 5)] = prm_tmp[x]
				# remove x
				prm_tmp.pop(x)

			elif x in ["di2mo", "di3mo", "di4mo"]:
				# add new key
				prm_tmp["di"+str(int(x[2:-2]) + 5)+"mo"] = prm_tmp[x]
				# remove x
				prm_tmp.pop(x)

			elif x in ["out1", "out2", "out3", "out4", "out5"]:
				prm_tmp["do"+str(int(x[-1])+ 5)+"mo"] = prm[x]

			elif x in ["do6mo", "do7mo", "do8mo", "do9mo", "do10mo"]:
				prm_tmp["out"+str(int(x[2:-2])- 5)] = prm[x]

			elif x in ["do1mo", "do2mo", "do3mo", "do4mo", "do5mo"]:
				prm_tmp["do"+str(int(x[2:-2])+ 5)+"mo"] = 0
				prm_tmp["out"+x[2:-2]] = 0

		prm = prm_tmp

		prm = json.dumps(prm)
		prm = prm.replace('"', "")

		return {'gc_list': ["M100("+prm+")"], 'status':0}



	# {"command": "M100", "prm":{"in7":1}}
	def _M101 (self, prm):
		prm = json.dumps(prm)
		prm = prm.replace('"', "")
		return {'gc_list':["M101("+prm+")"], 'status':0}


	# =================================================================
	# method
	# =================================================================

	#{"command": "move", "prm": {"movement": "relative", "speed": 1000.0, "path": "line", "segment_size": 0.01, "j0": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0}}
	"""
	command: move
	movement: absolute:0, relative:1
	speed
	path: line, circle, joint
	input:j or x
	"""

	def _move(self,prm):
		if "joint" in prm:
			for i in range(len(prm["joint"])):
				if prm["joint"][i] is not None:
					prm["j"+str(i)] = prm["joint"][i]
		prm.pop("joint", None)

		if "xyz" in prm:
			_map = ["x", "y", "z", "a", "b", "c"]
			for i in range(len(prm["xyz"])):
				if prm["xyz"][i] is not None:
					prm[_map[i]] = prm["xyz"][i]
		prm.pop("xyz", None)

		# speed
		if "speed" in prm and prm["speed"] <= 0:
			return {'gc_list':[], 'status':100 , "message": "not a valid format"}


		"""
		unit
			position (xyz)
			speed
			jerk (not joint)
		"""
		if self._config["unit"]["length"] == "mm":
			_key = [k for k in ["x", "y", "z"] if k in prm]
			for k in _key:
				prm[k] = self._mm_to_inch(prm[k])

			if "speed" in prm and prm["path"] != "joint":
				prm["speed"] = self._mm_to_inch(prm["speed"])

			if "jerk" in prm and prm["path"] != "joint":
				_jerk = [self._mm_to_inch(x) for x in prm["jerk"]]
				prm["jerk"] = self._jerk_mm_to_inch(prm["jerk"])



		if "path" in prm and prm["path"] == "joint": # move_joint
			return self._move_joint(prm)
		elif "path" in prm and prm["path"] == "line": # move_line
			return self._move_line(prm)
		return {'gc_list':[], 'status':100 , "message": "not a valid format"}



	def _move_joint(self, prm):
		travel_init = np.copy(self._system["travel_final"])
		joint_init = self._travel_to_joint(np.copy(self._system["travel_final"]))

		# joint_final
		if any(['j0' in prm, 'j1' in prm, 'j2' in prm, 'j3' in prm, 'j4' in prm, 'j5' in prm]):
			joint_final = self._joint_final(prm, np.copy(joint_init))

		elif any(['x' in prm, 'y' in prm, 'z' in prm, 'a' in prm, 'b' in prm, 'c' in prm]):
			xyz_init = self._joint_to_xyz(np.copy(joint_init))
			xyz_final = self._xyz_final (prm, xyz_init)
			result = self._xyz_to_joint(xyz_final)
			joint_final = result["joint"]
			if result["status"]:
				return {'gc_list': [], 'travel_final':np.copy(self._system["travel_final"]), 'status':result["status"], "message": "final position is not valid"}


		# validate joint_final
		result = self._joint_validate(joint_init, joint_final)
		joint_achieve = result["joint_achieve"]

		# travel final
		prm_switch = False
		travel_final = self._joint_to_travel(np.copy(joint_achieve))
		gc_list = []
		if travel_init[2] < 800:
			gc_list.append(self._ref_change(self._joint_to_travel(joint_init)))
			prm_switch = True

		# jerk
		if "jerk" in prm:
			jrk = self._jerk(prm["jerk"])
			gc_list += self._M100(jrk)["gc_list"]

		else:
			if prm_switch:
				#jrk = self._jerk(list(self._config["default_jerk"]["joint"].values()))
				jrk = self._jerk(list(self._config["default_jerk"]["joint"]))
				gc_list += self._M100(jrk)["gc_list"]

		# speed
		if not "speed" in prm and prm_switch:
			prm["speed"] = self._config["default_speed"]["joint"]

		gc_list.append(self._move_to_gc(travel_final, prm))

		return {'gc_list': gc_list, 'travel_final':travel_final, 'status':result["status"], "message": result["message"]}


	def _move_line(self,prm):
		travel_init = np.copy(self._system["travel_final"])
		xyz_init = self._travel_to_xyz(np.copy(self._system["travel_final"]))
		# xyz_final
		if any(['j0' in prm, 'j1' in prm, 'j2' in prm, 'j3' in prm, 'j4' in prm, 'j5' in prm]):
			joint_init = self._xyz_to_joint(np.copy(xyz_init))["joint"]
			joint_final = self._joint_final (prm, joint_init)
			xyz_final = self._joint_to_xyz(joint_final)
		elif any(['x' in prm, 'y' in prm, 'z' in prm, 'a' in prm, 'b' in prm, 'c' in prm]):
			xyz_final = self._xyz_final (prm, np.copy(xyz_init))
		# validate xyz_final
		# send error if result is not valid ???
		result = self._line_validate(xyz_init, xyz_final)
		xyz_achieve = result["xyz_achieve"]
		if any(xyz_achieve == None):
			return {'gc_list':[], 'status':result["status"], "message": result["message"]}


		# travel final
		prm_switch = False
		travel_final = self._xyz_to_travel(xyz_achieve)
		gc_list = []
		if travel_init[2] >= 800:
			prm_switch = True
			gc_list.append(self._ref_change(self._xyz_to_travel(xyz_init)))
			###
			#jrk = {k: v[1] for k,v in rbt["jerk"]["xyz"]["job"].items()}
			#gc_list += self.jerk(jrk)["gc_list"]

		# jerk
		if "jerk" in prm:
			jrk = self._jerk(prm["jerk"])
			gc_list += self._M100(jrk)["gc_list"]
		else:
			if prm_switch:
				#jrk = self._jerk(list(self._config["default_jerk"]["xyz"].values()))
				jrk = self._jerk(list(self._config["default_jerk"]["xyz"]))
				gc_list += self._M100(jrk)["gc_list"]


		# speed
		if not "speed" in prm and prm_switch:
			prm["speed"] = self._config["default_speed"]["xyz"]

		gc_list.append(self._move_to_gc(travel_final, prm))

		if "message" in result:
			return {'gc_list':gc_list, 'travel_final':travel_final, 'status':result["status"], "message": result["message"]}
		return {'gc_list':gc_list, 'travel_final':travel_final, 'status':result["status"]}

		"""
		Supported params:
			offsets (I,J,K): center offset from current point
			end point (X,Y,Z): End point (optional)
			P: number of turns should be integer
			circle axis (M): Should be one of 0 (Z)(default), 1(Y), 2(X)
			movement: 0 absolute, 1 relative (default)
			rotation_dirxn: 0 cw (default), 1 (ccw)
		"""
		def move_circle(self, prm, fulfill=True, append=True):
			try:
				prm = json.loads(prm)
			except:
				pass
			# Set into cartesian coordinates
			self.play({"command": "move", "prm": {"path": "line", "movement": 1, "x":0}}, True)
			gcodes = []
			circle_axis = 0
			if 'movement' in prm and prm['movement'] == 0:
				gcodes.append('G90')
			else:
				gcodes.append('G91')
			if 'M' in prm:
				gcodes.append('G'+str(17+prm['M']))
				circle_axis=prm['M']
			else:
				gcodes.append('G17')

			if (circle_axis == 0 and 'K' in prm or
				circle_axis == 1 and 'J' in prm or
				circle_axis == 2 and 'I' in prm):
				print("Cannot provide offset along circle axis")

			circle_command = ''

			if 'rotation_dirxn' in prm and prm['rotation_dirxn'] == 1:
				circle_command = circle_command + 'G3'
			else:
				circle_command = circle_command + 'G2'

			for key in ['I', 'J', 'K', 'X', 'Y', 'Z', 'P']:
				if key in prm:
					circle_command = circle_command + ' ' + key + str(prm[key])
			gcodes.append(circle_command)

			for gcode in gcodes:
				self.play({"command": "g2core", "prm": {'gc': gcode}}, append=True)



	# {"command": "halt", "prm":0}
	def _halt(self):
		gc_list = ["!", "%"]
		_printx(self._prnt,"halt: ")
		return {'gc_list':gc_list, 'status':0}


	# *{"command": "probe", "prm": {"j0": , "j1": , ..., "speed"}}
	def _probe(self, prm):
		tmp_prm = {"movement": 0,
					"speed": 100,
					"jerk": [300, 300, 300, 300, 300, 300],
					"gc": "G38.3 ",
					"path": "joint"}
		tmp_prm.update(prm)
		"""
		prm["movement"] = 0
		prm["speed"] = 100
		prm["jerk"] = [300, 300, 300, 300, 300, 300]
		prm["gc"] = "G38.3 "
		prm["path"] = "joint"
		"""
		return self._move(tmp_prm)

	# *{"command": "homing", "prm": ["j0", "j1", ...]}
	def _home(self, args):
		_printx(self._prnt,"args: ", args)
		gc_list = []
		if self._system["travel_final"][2] < 800:
			joint = self._travel_to_joint(np.copy(self._system["travel_final"]))
			gc_list.append(self._ref_change(self._joint_to_travel(joint)))

		#home = {}
		command = ""
		if "j0" in args:
			command += "x0"
		if "j1" in args:
			command += "y0"
		if "j2" in args:
			command += "z0"
		if "j3" in args or "j4" in args:
			command += "a0b0"

		if command:
			gc_list.append("G28.2" + command)
			return {'gc_list': gc_list, 'status':0}

		return False

	# {"command": "set_joint", "prm": {"j0": 1.1, ...}}
	def _set_joint(self, prm):
		joint_current = self._travel_to_joint(np.copy(self._system["travel_final"]))

		if 'j0' in prm:
			joint_current[0] = prm['j0']

		if 'j1' in prm:
			joint_current[1] = prm['j1']

		if 'j2' in prm:
			joint_current[2] = prm['j2']

		if 'j3' in prm:
			joint_current[3] = prm['j3']

		if 'j4' in prm:
			joint_current[4] = prm['j4']

		if 'j5' in prm:
			joint_current[5] = prm['j5']
		travel = self._joint_to_travel(joint_current)
		command_send = "G28.3 "
		if 'j0' in prm:
			command_send = command_send + "X{:07.4f}".format(travel[0])

		if 'j1' in prm:
			command_send = command_send + "Y{:07.4f}".format(travel[1])

		if 'j2' in prm:
			command_send = command_send + "Z{:07.4f}".format(travel[2])

		if 'j3' in prm or "j4" in prm:
			command_send = command_send + "A{:07.4f}".format(travel[3])
			command_send = command_send + "B{:07.4f}".format(travel[4])


		"""
		if 'j3' in prm:
			command_send = command_send + "A{:07.4f}".format(travel[3])

		if 'j4' in prm:
			command_send = command_send + "B{:07.4f}".format(travel[4])
		"""
		if 'j5' in prm:
			command_send = command_send + "C{:07.4f}".format(travel[5])

		return {'gc_list':[command_send], 'status':0, "travel_final": travel}


	# {"command": "gcode", "prm":{"gc": , ...}}
	def _gcode (self, prm):
		if type(prm) != dict or "gc" not in prm:
			return {'gc_list':[], 'status':100 , "message": "not a valid format"}

		tmp_prm = {"path": "line", "movement": 0}
		gc_prm = {}
		line = prm["gc"].strip()
		line = line.lower()
		keys = list(set(re.findall("[xyzabcuvgf]+", line)))
		for k in keys:
			regexp = k + r"\s*(-?\d+(?:\.\d+)?)"
			values = re.findall(regexp, line)
			tmp = []
			for v in values:
				try:
					tmp.append(float(v))
				except:
					pass
			gc_prm[k] = tmp

			if k == "g":
				for v in gc_prm[k]:
					if v < 1.1:
						tmp_prm["method"] = "move"
					elif v == 90:
						tmp_prm["movement"] = 0
					elif v == 91:
						tmp_prm["movement"] = 1

			elif k == "f" and gc_prm[k]:
				tmp_prm["speed"] = gc_prm[k][0]

			elif gc_prm[k]:
				tmp_prm[k] = gc_prm[k][0]

		# make sure method exists
		if "method" in tmp_prm:
			# remove method key
			method = tmp_prm["method"]
			tmp_prm.pop('method', None)
			# remove gc parameter
			prm.pop("gc", None)
			# merge two dictionaries
			tmp_prm.update(prm)

			return getattr(self, "_"+method)(tmp_prm)



	# jerk : [1200, 500, 100, ...] ???
	def _jerk(self, jrk):
		_map = ["xjm", "yjm", "zjm", "ajm", "bjm", "cjm"]
		send = {}
		for i in range(min(len(_map), len(jrk))):
			#send[_map[i]] = jrk[i]
			send[_map[i]] = jrk[i] * self._scale["jerk"]

		return send

	# {"command": "servo", "prm":500}
	def _servo(self, prm):
		self._io["servo"] = prm
		cmd = []
		cmd.append("M3S"+str(prm))
		return {'gc_list':cmd, 'status':0}


	# {"command": "laser", "prm":1}
	def _laser(self, prm):
		result =  self._M100({"out5": prm})
		#result["gc_list"] = result["gc_list"]
		return result

	# {"command": "g2core", "prm":}
	def _g2core (self, prm):
		return {'gc_list':[prm], 'status':0}


	# {"command": "sleep", "prm":500}
	def _sleep(self, prm):
		cmd = []
		cmd.append("G4P"+str(prm))
		return {'gc_list':cmd, 'status':0}

	# {"command": "output", "prm":{"out1": 0, "out2":1, "out3":1}}
	def _output(self, prm):
		#output_key = list(set(prm.keys()) & set(["out1", "out2","out3","out4"]))
		#output = {x: int(prm[x])%2 for x in output_key }
		result = self._M100(prm)
		#result["gc_list"] = result["gc_list"]
		return result

	# "laser", "servo", "di1mo", "out1", "do1mo"
	def _set_io(self, prm):
		valid_m100 = ["di1mo", "di2mo", "di3mo", "di4mo", "do1mo" ,"do2mo", "do3mo", "do4mo", "out1", "out2", "out3", "out4", "out5"]
		cmd = []

		# laser
		if "laser" in prm:
			prm["out5"] = prm["laser"]

		# M100
		# di or do (1,2,3,4)mo, out(1,2,3,4 ,5)
		tmp = {x: prm[x] for x in prm if x in valid_m100}
		result = self._M100(tmp)

		# servo
		if "servo" in prm:
			result["gc_list"] += self._servo(prm["servo"])["gc_list"]

		return result

	# "laser", "servo", "di1mo", "out1", "do1mo"
	def _set_io_async(self, prm):
		valid_m100 = ["di1mo", "di2mo", "di3mo", "di4mo", "do1mo" ,"do2mo", "do3mo", "do4mo", "out1", "out2", "out3", "out4", "out5"]
		cmd = []

		# laser
		if "laser" in prm:
			prm["out5"] = prm["laser"]

		# M100
		# di or do (1,2,3,4)mo, out(1,2,3,4 ,5)
		tmp = {x: prm[x] for x in prm if x in valid_m100}
		#result = self._M100(tmp)
		tmp = self._form_io(tmp)
		result = {'gc_list': [tmp], 'status':0}

		# servo
		if "servo" in prm:
			result["gc_list"] += self._servo(prm["servo"])["gc_list"]

		return result		

	# {"command": "input", "prm":{"in1": 0, "in2":1, "in3":1}}
	def _wait_for_input(self, prm):
		result = False
		if "in1" in prm:
			mode = [False, True][prm["in1"]]
			result = self._M101({"in1": mode})
		elif "in2" in prm:
			mode = [False, True][prm["in2"]]
			result = self._M101({"in7": mode})
		elif "in3" in prm:
			mode = [False, True][prm["in3"]]
			result = self._M101({"in8": mode})
		elif "in4" in prm:
			mode = [False, True][prm["in4"]]
			result = self._M101({"in9": mode})

		if result:
			result["gc_list"] = result["gc_list"]

		return result


	# {"command": "set_motion", "prm":{"jt": 1, "ct": 0.01, "gpa": 2}}
	def _set_motion(self, prm):
		gc_list = []
		if "jt" in prm:
			gc_list.append("{jt : "+str(prm["jt"]) +"}")
		if "ct" in prm:
			gc_list.append("{ct : "+str(prm["ct"]) +"}")
		if "pcm" in prm:
			gc_list.append("{gpa : "+str(prm["gpa"]) +"}")

		return {'gc_list':gc_list, 'status':0, "travel_final": np.copy(self._system["travel_final"])}

	def motion(self):
		return self.config(["motion"])

	# {"command": "set_toolhead", "prm":{"x": 1.74}}
	def _set_toolhead(self, prm):
		# sanitation
		# initialization
		travel = np.copy(self._system["travel_final"])
		joint = self._travel_to_joint(np.copy(self._system["travel_final"]))
		travel_final = self._joint_to_travel(np.copy(joint))
		gc_list = []
		status = 0

		# change ik to joint
		if travel[2] < 800:
			gc_list.append(self._ref_change(travel_final))

		# update tool length
		gc_list.append("G10L1P1X"+ str(prm["x"]))

		# update config
		#self._config["toolhead"]["x"] = prm["x"]

		return {'gc_list':gc_list, 'status':0, "travel_final": travel_final}

	def toolhead(self):
		#result =  self.config(["toolhead"])
		# log toolhead
		#self._log_add({"toolhead": json.loads(result)}, "config")
		return self.config(["toolhead"])

