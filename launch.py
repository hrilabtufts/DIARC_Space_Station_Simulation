#!/usr/bin/python

from argparse import ArgumentParser
from datetime import datetime
from threading import Thread
from pathlib import Path
import time
import os
import subprocess
import tempfile
import signal
import sys
import shutil
import re

has_pyxdf = False

try :
	import pyxdf
	has_pyxdf = True
except ImportError:
	print('WARNING: Module pyxdf is not installed, LSL data cannot be analyzed after experiment')

parser = ArgumentParser(prog='python launch.py',
                    description='Script for orchestrating the launch of DIARC, ROS and the Unity Space Station environment for experimentation')

subparsers = parser.add_subparsers(dest='command', help='Commands to run', required=True)

build = subparsers.add_parser('build', help='Build the DIARC Space Station docker image')
build.add_argument('--no-cache', dest='cache', default=True, action='store_false', help='Rebuild image without using cache')

dev = subparsers.add_parser('dev', help='Run the simulation servers in dev mode')
dev.add_argument('-g', '--gui', default=False, action='store_true', help='Enable GUI settings in DIARC and ROS')
dev.add_argument('-r', '--robots', type=int, default=None, help='Set the number of robots to spawn in the trial')
dev.add_argument('-s', '--smm', default=None, action='store_true', help='Set the number of robots to spawn in the trial')
dev.add_argument('-n', '--noServer', default=False, action='store_true', help='Run without launching Unity server so it can be launched elsewhere')
dev.add_argument('-o', '--output', default=None, type=str, help='Directory to save log files to. If it does not exist, it will be created.')
dev.add_argument('-d', '--dataPort', default=8888, type=int, help='Port to run the data collection server on. Default (8888)')
dev.add_argument('-c', '--clients', default=1, type=int, help='Number of clients expected to join. Default (1)')
dev.add_argument('diarc', type=str, nargs='+')

run = subparsers.add_parser('run', help='Run the simulation server(s) in production mode')
run.add_argument('-g', '--gui', default=False, action='store_true', help='Enable GUI settings in DIARC and ROS')
run.add_argument('-r', '--robots', type=int, default=None, help='Set the number of robots to spawn in the trial')
run.add_argument('-s', '--smm', default=None, action='store_true', help='Set the number of robots to spawn in the trial')
run.add_argument('-n', '--noServer', default=False, action='store_true', help='Run without launching Unity server so it can be launched elsewhere')
run.add_argument('-o', '--output', default=None, type=str, help='Directory to save log files to. If it does not exist, it will be created.')
run.add_argument('-d', '--dataPort', default=None, type=int, help='Port to run the data collection server on. Default (8888)')
run.add_argument('-c', '--clients', default=1, type=int, help='Number of clients expected to join. Default (1)')

stop = subparsers.add_parser('stop', help='Gracefully stop all docker containers')
kill = subparsers.add_parser('kill', help='Kill all docker containers')

args = parser.parse_args()

global_dict = {}

class DIARCSpaceStation:
	docker_compose = './docker-compose.yaml'

	additional_services_tmpl = './config/docker/docker-compose-additional-services.yaml.tmpl'
	network_tmpl = './config/docker/docker-compose-network.yaml.tmpl'
	
	trade_properties_default = 'diarc.tradePropertiesFile=src/main/resources/default/trade.properties.default'
	trade_properties_smm = 'diarc.tradePropertiesFile=/root/trade.properties'

	trade_properties_hub_tmpl = './config/diarc/trade.hub.properties'
	trade_properties_spoke_tmpl = './config/diarc/trade.spoke.properties'

	gradle_properties_tmpl = './config/gradle/gradle.properties'

	ros_tmpl = './config/ros/startup_pr2_docker.launch.tmpl'

	docker_compose_robot_dev_tmpl = './config/docker/docker-compose-robot-dev.yaml.tmpl'
	docker_compose_robot_dev_headless_tmpl = './config/docker/docker-compose-robot-dev-headless.yaml.tmpl'
	docker_compose_robot_tmpl = './config/docker/docker-compose-robot.yaml.tmpl'
	docker_compose_robot_headless_tmpl = './config/docker/docker-compose-robot-headless.yaml.tmpl'

	unity_server_bin = './Space_Station_SMM_Server.x86_64'
	unity_server_log = '.config/unity3d/HRI_Lab_Tufts_University/Space_Station_SMM_Server/Player.log'
	unity_server_output = 'Space_Station_SMM_Server_Data/StreamingAssets/Output'
	unity_server_config = 'Space_Station_SMM_Server_Data/StreamingAssets/server_settings.json'

	required_keys = ['BIN' , 'UNITY_IP', 'DIARC_CONFIG', 'LLM_URL', 'LLM_PORT', 'SMM', 'NETWORK_PREFIX']
	
	ros_port = 9090
	unity_port = 1755
	network_start = 4
	robots = 0
	clients = 0

	ros_started = []
	diarc_started = []
	diarc_connected = []
	client_connected = []
	lsl_stream_closed = []

	unity_started = False
	lab_recorder_started = False

	write_to_log = False
	log_file = None

	has_lab_recorder = False

	def __init__ (self, args) :
		self._args = args
		print('DIARC Space Station Simulation')
		self._line()
		self.ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
		if args.command == 'build' :
			self._build()
		elif args.command == 'dev' :
			self._run(True)
		elif args.command == 'run' :
			self._run(False)
		elif args.command == 'kill' :
			self._kill()
		elif args.command == 'stop' :
			self._stop()
		else:
			parser.print_help()

	def _readEnv(self) :
		'''Load all .env values to self._env, removes need for python-dotenv dependency'''
		self._env = {}
		envFile = '.env'
		if not os.path.isfile(envFile) :
			print('WARNING: Using default.env variables. Create a .env file with your own variables to customize.')
			envFile = 'default.env'
		with open(envFile, 'r') as file :
			for line in file :
				if line.startswith('#') or line.strip() == '':
					continue
				key, value = line.strip().split('=', 1)
				value = value.replace('"', '')
				if value.lower() == 'false' :
					value = False
				elif value.lower() == 'true' :
					value = True
				self._env[key] = value
				print(f'[ENV] {key} = {value}')
		keys = list(self._env.keys())
		failed = False
		for key in self.required_keys :
			if key not in keys :
				print(f'[ENV] Missing required key "{key}" in .env file')
				failed = True
		if failed :
			exit(1)
		self._line()

	def _overrides(self) :
		'''Override ENV values with commandline settings'''
		if 'robots' in self._args and self._args.robots is not None :
			self._log(f'Overriding ENV value ROBOTS ({self._env["ROBOTS"]}) with {self._args.robots}')
			self._env['ROBOTS'] = str(self._args.robots)

		if 'smm' in self._args and self._args.smm is not None :
			self._log(f'Overriding ENV value SMM ({self._env["SMM"]}) with {self._args.smm}')
			self._env['SMM'] = self._args.smm

		if 'noServer' in self._args and self._args.noServer :
			if 'SERVER' in self._env :
				self._log(f'Overriding ENV value SERVER with --noServer flag')
				del self._env['SERVER']

		if 'dataPort' in self._args and self._args.dataPort is not None:
			if 'DATA_PORT' in self._env :
				self._log(f'Overriding ENV value DATA_PORT ({self._env["DATA_PORT"]}) with {self._args.dataPort}')
			self._env['DATA_PORT'] = str(self._args.dataPort)
		if 'dataPort' in self._args and self._args.dataPort is not None and 'DATA_PORT' not in self._env :
			self._env['DATA_PORT'] = '8888'


	def _build(self) :
		'''Build the image. Required prior to run or dev commands.'''
		print('build()')
		
		cmd = ['docker', 'build']

		if not self._args.cache :
			cmd += ['--no-cache']

		cmd += ['--progress=plain', '-t', 'hrilabtufts/unity_space_station', '-f', 'Dockerfile', '.']
		
		print(f'[BUILD] {" ".join(cmd)}')
		
		proc = subprocess.run(cmd, stdout=subprocess.PIPE)

		for line in proc.stdout :
			print(f'[BUILD] {line}')

	def _kill(self) :
		'''Immediately kill the DIARC/ROS docker container with any and all sideeffects of doing that'''
		print('kill()')
		cmd = "docker kill $(docker container ls | grep unity_space_station | awk '{print $1}')"
		print(f'[KILL] {cmd}')
		proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
		for line in proc.stdout :
			print(f'[KILL] {line}')

	def _stop(self) :
		'''Gracefully stop the DIARC/ROS docker container'''
		print('stop()')
		cmd = ['bash', './scripts/stop.sh']
		print(f'[STOP] {" ".join(cmd)}')
		proc = subprocess.run(cmd, stdout=subprocess.PIPE)
		for line in proc.stdout :
			print(f'[STOP] {line}')

	def _run(self, dev) :
		'''Start all processes for the run or dev commands'''
		print(f'{self._args.command.lower()}()')

		timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')

		self._logs(timestamp)
		self._readEnv()
		self._overrides()
		self._checkLabRecorder()
		self.unity_server_log = os.path.join(Path.home(), self.unity_server_log)
		
		robot_tmpl = './config/docker/docker-compose-robot-headless.yaml.tmpl'
		if dev :
			robot_tmpl = './config/docker/docker-compose-robot-dev-headless.yaml.tmpl'

		robots = int(self._env['ROBOTS'])
		llm_port = int(self._env['LLM_PORT'])
		clients = int(self._args.clients)
		ros_port = self.ros_port
		unity_port = self.unity_port
		network = self.network_start
		gui = False

		if robots > 2 :
			self._log(f'ERROR: Cannot support more than 2 robots with this script')
			exit(1)

		if dev and len(self._args.diarc) != robots :
			self._log(f'ERROR: Robot count ({robots}) != number of DIARC directories in command ({len(self._args.diarc)})')
			exit(1)

		if self.write_to_log and 'LSL' in self._env :
			global_dict['lsl'] = self._env['LSL']

		for i in range(robots) :
			self.ros_started.append(False)
			self.diarc_started.append(False)
			self.diarc_connected.append(False)

		#server stream
		self.lsl_stream_closed.append(False)

		for i in range(clients) :
			self.client_connected.append(False)
			#client streams
			self.lsl_stream_closed.append(False)

		if self._args.gui :
			self._log('WARN: GUI features not available in this version of the script.')
			#self._setupX()
			#robot_tmpl = './config/docker/docker-compose-robot.yaml.tmpl'
			#if dev :
			#	robot_tmpl = './config/docker/docker-compose-robot-dev.yaml.tmpl'
			#gui = True

		self.robots = robots
		self.clients = clients
		
		self._mkdir('./logs')
		self._mkdir('./cache')

		docker_compose = ['services:']

		additional_services = self._additionalServices()
		docker_compose += additional_services

		self._log(f'Generating config for robot(s) {robots}')

		for i in range(robots) :
			robot_name = f'robot{i+1}'
			robot_ip = f'{self._env["NETWORK_PREFIX"]}.0.{network}'
			llm_url = f'{self._env["LLM_URL"]}:{llm_port}'
			diarc = os.path.realpath(self._args.diarc[i])

			trade_properties_tmp = self._mktemp()
			trade_properties = self._tradeProperties(i)
			self._writeLines(trade_properties_tmp, trade_properties)

			gradle_properties_tmp = self._mktemp()
			gradle_properties = self._gradleProperties()
			self._writeLines(gradle_properties_tmp, gradle_properties)

			ros_tmp = self._mktemp()
			ros = self._rosConfig(ros_port, gui)
			self._line()
			for line in ros :
				self._log(line)
			self._line()
			self._writeLines(ros_tmp, ros)

			robot = self._robot(dev, gui, robot_name, timestamp, ros_port, ros_tmp, diarc, robot_ip, trade_properties_tmp, unity_port, gradle_properties_tmp, llm_url)

			docker_compose += robot

			ros_port += 1
			unity_port += 1
			llm_port += 1
			network += 1

		network = self._network()
		docker_compose += network

		self._line()
		for line in docker_compose :
			self._log(line)
		self._line()

		self._writeLines(self.docker_compose, docker_compose)
		self._waitForYes()
		self._dataServer(timestamp)
		self._dockerCompose()

	def _dockerCompose (self) :
		'''Start the docker compose process and store the subprocess in the global dict'''
		self._log(f'Launching via {self._env["BIN"]}...')
		cmd = self._env['BIN'] + ' up --remove-orphans'
		global_dict['docker_proc'] = subprocess.Popen(cmd.split(' '), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		for line in global_dict['docker_proc'].stdout :
			self._parseDockerComposeLog(line)

	def _parseDockerComposeLog (self, line) :
		'''Parse all lines from docker compose process to determine when ROS is started to start the Unity server'''
		line = line.decode('ascii').strip()
		self._log(line, False, True)
		
		if not self.ros_started[0] and 'robot1-' in line and 'process[tilt_shadow_filter-' in line :
			self.ros_started[0] = True

		if self.robots == 2 and not self.ros_started[1] and 'robot2-' in line and 'process[tilt_shadow_filter-' in line :
			self.ros_started[1] = True

		if not self.unity_started and False not in self.ros_started :
			self._unityServer()

		if not self.diarc_started[0] and 'robot1-' in line and 'Task :config:launch' in line :
			self.diarc_started[0] = True

		if self.robots == 2 and not self.diarc_started[1] and 'robot2-' in line and 'Task :config:launch' in line :
			self.diarc_started[1] = True

		if not self.diarc_connected[0] and 'Agent robot1 received message action = connected' in line :
			self.diarc_connected[0] = True

		if self.robots == 2 and not self.diarc_connected[1] and 'Agent robot2 received message action = connected' in line :
			self.diarc_connected[1] = True

	def _unityServer (self) :
		'''Start the Unity server in a subprocess if in .env file'''
		self.unity_started = True
		if 'SERVER' in self._env :
			self._log('STARTING UNITY SERVER')
			cmd = [ self.unity_server_bin ]
			global_dict['unity_logs'] = os.path.join(self._env['SERVER'], self.unity_server_output)
			global_dict['unity_proc'] = subprocess.Popen(cmd, cwd=self._env['SERVER'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
			self._unityServerLog()
		else :
			self._log(" *********************************************")
			self._log(" *                                           *")
			self._log(" *                                           *")
			self._log(" *           START UNITY SERVER NOW          *")
			self._log(" *                                           *")
			self._log(" *                                           *")
			self._log(" *********************************************")

	def _unityServerLogThread (self) :
		if os.path.isfile(self.unity_server_log) :
			cmd = ['tail', '-F', self.unity_server_log]
			global_dict['unity_server_log'] = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
			for line in global_dict['unity_server_log'].stdout :
				self._parseUnityServerLog(line)
		else :
			self._log(f'Cannot find Unity server log: {self.unity_server_log}')

	def _unityServerLog(self) :
		'''Wrapper function for launching the Unity server log thread -- from the Unity process, not the data streams'''
		Thread(target=self._unityServerLogThread).start()

	def _parseUnityServerLog (self, line) :
		line = line.decode('ascii').strip()
		if line == '' :
			return
		if line.startswith('(Filename: ') :
			return
		if 'The referenced script on this Behaviour (Game Object' in line :
			return
		
		self._log(line, True, False)
		
		if not self.client_connected[0] and 'SMM Player 1 joined server' in line :
			self.client_connected[0] = True

		if self.clients == 2 and not self.client_connected[0] and 'SMM Player 2 joined server' in line :
			self.client_connected[1] = True

		if self.has_lab_recorder and not self.lab_recorder_started and False not in self.client_connected : 
			self._labRecorder()


	def _additionalServices (self) :
		'''Generate the docker compose stanza for Kaldi and OpenTTS'''
		additional_services = self._readLines(self.additional_services_tmpl)
		additional_services_map = {
			'NETWORK_PREFIX' : self._env['NETWORK_PREFIX']
		}
		return self._replaceLines(additional_services, additional_services_map)

	def _network (self) :
		'''Generate the docker compose stanza for the network definition'''
		network = self._readLines(self.network_tmpl)
		network_map = {
			'NETWORK_PREFIX' : self._env['NETWORK_PREFIX']
		}
		return self._replaceLines(network, network_map)

	def _tradeProperties (self, i) :
		'''Generate the TRADE properties for a hub or spoke from a template'''
		trade_properties_tmpl = self.trade_properties_hub_tmpl
		if i > 0 :
			trade_properties_tmpl = self.trade_properties_spoke_tmpl
		trade_properties = self._readLines(trade_properties_tmpl)
		trade_properties_map = {
			'NETWORK_PREFIX' : self._env['NETWORK_PREFIX']
		}
		return self._replaceLines(trade_properties, trade_properties_map)

	def _gradleProperties (self) :
		'''Generate the gradle properties file for a hub or spoke TRADE configuration'''
		gradle_properties = self._readLines(self.gradle_properties_tmpl)
		trade_properties_line = self.trade_properties_default
		if self._env['SMM'] :
			trade_properties_line = self.trade_properties_smm
		gradle_properties_map = {
			'TRADE_PROPERTIES' : trade_properties_line
		}
		return self._replaceLines(gradle_properties, gradle_properties_map)

	def _rosConfig (self, ros_port, gui) :
		'''Generate the ROS launch file for starting the PR2'''
		ros = self._readLines(self.ros_tmpl)
		ros_map = {
			'DISPLAY_RVIZ' : str(gui),
			'DISPLAY_GAZEBO' : str(gui)
		}
		return self._replaceLines(ros, ros_map)

	def _robot (self, dev, gui, robot_name, timestamp, ros_port, ros_tmp, diarc, robot_ip, trade_properties_tmp, unity_port, gradle_properties_tmp, llm_url) :
		'''Generate the docker compose robot stanza(s)'''
		tmpl = self.docker_compose_robot_tmpl
		if not dev and gui :
			tmpl = self.docker_compose_robot_headless_tmpl
		elif dev and gui :
			tmpl = self.docker_compose_robot_dev_tmpl
		elif dev and not gui :
			tmpl = self.docker_compose_robot_dev_headless_tmpl
		self._log(f'Using docker compose template {tmpl}')
		docker_compose_entry = self._readLines(tmpl)
		docker_compose_map = {
			'ROSPORT' : str(ros_port),
			'TIMESTAMP' : timestamp,
			'ROS_TMP' : ros_tmp,
			'ROBOTNAME' : robot_name,
			'DIARC_SRC' : diarc,
			'ROBOT_IP' : robot_ip,
			'TRADE_PROPERTIES' : trade_properties_tmp,
			'UNITYPORT' : str(unity_port),
			'GRADLE_PROPERTIES' : gradle_properties_tmp,
 			'LLMURL' : llm_url
		}

		return self._replaceLines(docker_compose_entry, docker_compose_map)

	def _setupX (self) :
		'''Used to set up X auth for the GUI features but deprecated for now'''
		xauth_tmp = '/tmp/.docker.xauth'

		xhost_cmd = ['xhost', '+local:docker']
		self._shellPrint(xhost_cmd)

		xauth_cmd = ['export', f'XAUTH={xauth_tmp}']
		self._shellPrint(xauth_cmd)

		xauth_remove_cmd = ['sudo', 'rm', '-f', xauth_tmp]
		subprocess.run(xauth_remove_cmd)

		xauth_list_cmd = "xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/'"
		xauth_list = subprocess.check_output(xauth_list_cmd)
		xauth_list = xauth_list.decode('ascii').strip()

		if not os.path.isfile(xauth_tmp) :
			if not xauth_list.strip() == '' :
				xauth_list_write_cmd = f'echo {xauth_list} | xauth -f {xauth_tmp} nmerge -'
				self._shellPrint(xauth_list_write_cmd)
			else :
				xauth_touch_command = ['touch', xauth_tmp]
				self._shellPrint(xauth_touch_cmd)

			xauth_permissions_cmd = ['chmod', '777', xauth_tmp]
			self._shellPrint(xauth_permissions_cmd)

		xauth_confirm_cmd = ['file', xauth_tmp]
		self._shellPrint(xauth_confirm_cmd)

		xauth_ls_cmd = [f'ls -FAlh {xauth_tmp}']
		self._shellPrint(xauth_ls_cmd)

	def _shellPrint(self, cmd) :
		'''Run a command in a subprocess shell and print output. Intended to be run syncronously'''
		self._log(f'CMD: {cmd}')
		proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		for line in proc.stdout :
			self._log(line)

	def _readLines (self, path) :
		'''Read all lines in a file and store as a list'''
		lines = []
		with open(path, 'r') as file :
			lines = [line.rstrip() for line in file]
		return lines

	def _replaceLines(self, lines, replace_map) :
		'''Replace all instances of a dict key/value pair in all lines of a list'''
		outLines = []
		for line in lines :
			for key in replace_map :
				line = line.replace(key, replace_map[key])
			outLines.append( line )
		return outLines

	def _writeLines (self, path, lines) :
		'''Write all lines from a list to a file with newline delimitation'''
		with open(path, 'w') as file:
			for line in lines :
				file.write(f'{line}\n')
		self._log(f'Wrote file {path}')

	def _mkdir (self, path) :
		'''Make a directory if it does not exist'''
		if not os.path.exists(path) :
			os.makedirs(path)
			self._log(f'Created directory {path}')
		else :
			self._log(f'Directory {path} exists')

	def _mktemp (self) :
		'''Create a temporary file and return the path to be written to by another process'''
		tmp_file, filename = tempfile.mkstemp()
		return filename

	def _logs (self, timestamp) :
		'''If output is specified, add all log paths and required variables to be copied to output by the exitGracefully function'''
		if self._args.output is not None :
			self.write_to_log = True
			if os.path.exists(self._args.output) and os.path.isdir(self._args.output) :
				self._log(f'Output directory {self._args.output} exists')
			elif os.path.exists(self._args.output) and not os.path.isdir(self._args.output) :
				self._log(f'ERROR: Output path {self._args.output} exists and is not a directory, exiting...')
				exit(2)
			elif not os.path.exists(self._args.output) :
				self._log(f'Creating output directory {self._args.output}')
				self._mkdir(self._args.output)
			launch_log = os.path.join(self._args.output, f'{timestamp}_launch.log')
			global_dict['log_file'] = open(launch_log, 'a')
			global_dict['diarc_ros_logs'] = timestamp
			global_dict['output'] = self._args.output

	def _log (self, line, tag = True, remove_color = False) :
		'''Write a log line with optional command name tagged in brackets. If output dir specified write log line to a file there'''
		if remove_color :
			line = self.ansi_escape.sub('', line)
		if tag :
			print(f'[{self._args.command.upper()}] {line}')
		else : 
			print(line)
		if self.write_to_log and 'log_file' in global_dict :
			ts = datetime.now().strftime("%Y %m %d %H:%M:%S.%f")
			global_dict['log_file'].write(f'[{ts}] {line}\n')

	def _line(self) :
		'''Draw a line in the terminal'''
		print('------------------------------')

	def _waitForYes (self) :
		'''Pause the process to wait for user confirmation'''
		val = input('Are you ready to continue? (yes/no) : ')
		if val.strip() == '' or 'y' in val.lower() :
			self._log('Continuing')
		else :
			self._log('Cancelled')
			exit()

	def _dataServerThread (self, timestamp) :
		'''Start the file upload data server for collecting data from Unity clients'''
		cmd = ['python', './server.py', '-p', self._env['DATA_PORT'], '-t', timestamp]
		if self._args.output is not None :
			cmd += ['-o', self._args.output]
		global_dict['data_proc'] = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		self._log(f'Started Unity client data collection server on port: {self._env["DATA_PORT"]}')
		for line in global_dict['data_proc'].stdout :
			line = line.decode('ascii').strip()
			self._log(line, True, True)

	def _dataServer (self, timestamp) : 
		'''Wrapper method for launching the data collection server thread'''
		Thread(target=self._dataServerThread, args=[timestamp]).start()

	def _checkLabRecorder (self) :
		'''Perform which check for installed LabRecorder binary'''
		cmd = ['which', 'LabRecorder']
		output = subprocess.check_output(cmd)
		output = output.decode('ascii').strip()
		if output != '' :
			self.has_lab_recorder = True
			self._log(f'LabRecorder installed: {output}')
		else :
			self._log(f'WARN: LabRecorder is not installed, will not launch automatically')

	def _labRecorderThread (self) :
		'''Start the LabRecorder application'''
		cmd = ['LabRecorder']
		global_dict['lab_recorder_closed'] = False
		global_dict['lab_recorder'] = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		self._log(f'Started LabRecorder')
		for line in global_dict['lab_recorder'].stdout :
			self._parseLabRecorderLog(line)

	def _labRecorder (self) :
		'''Wrapper method for launching the LabRecorder application'''
		self.lab_recorder_started = True
		Thread(target=self._labRecorderThread).start()

	def _parseLabRecorderLog (self, line) :
		line = line.decode('ascii').strip()
		self._log(line, True, False)

		if not self.lsl_stream_closed[0] and 'Wrote footer for stream ' in line  and 'Server_Timestamp' in line:
			self.lsl_stream_closed[0] = True

		if not self.lsl_stream_closed[1] and 'Wrote footer for stream ' in line  and 'Player_1_Client_Timestamp' in line:
			self.lsl_stream_closed[1] = True

		if self.clients == 2 and not self.lsl_stream_closed[2] and 'Wrote footer for stream ' in line  and 'Player_2_Client_Timestamp' in line:
			self.lsl_stream_closed[2] = True

		if not global_dict['lab_recorder_closed'] and False not in self.lsl_stream_closed and 'Closing the file' in line :
			global_dict['lab_recorder_closed'] = True
			self._log('LabRecorder streams all closed')

def waitForEnter () :
	'''Pause the process to wait for user confirmation'''
	val = input('Your LabRecorder session does not appear to be stopped. Hit enter after stopping: ')

def copyDir (src, dest) :
	'''Copy one directory to another via cp command'''
	cmd = ['cp', '-r', src, dest]
	proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
	for line in proc.stdout :
		print(line.decode('ascii').strip())
	print(f'Copied {src} => {dest}')

def copyLogs (timestamp, output) :
	'''Find directories in ./logs matching the timestamp used to create them and copy into output directory'''
	dirs = os.listdir('./logs')
	for dir in dirs :
		if timestamp in dir :
			src = os.path.join('./logs', dir)
			copyDir(src, output)

def unityLogs (src, output) :
	'''Copy the logs from the Unity server into the output directory if specified'''
	dirs = os.listdir(src)
	dirs.sort(reverse=True)
	copyDir(os.path.join(src, dirs[0]), output)

def lslFile (src, output, timestamp) :
	'''Copy the LSL file to the output directory'''
	dest = os.path.join(output, f'{timestamp}_lsl_data.xdf')
	if os.path.isfile(src) :
		shutil.copy2(src, dest)
		print(f'Copied LSL file {src} => {output}')
	else :
		print(f'ERROR: Could not find LSL file {src}')

def exitGracefully () :
	'''Catch the CTRL-c SIGINT and run all cleanup jobs to shut down subprocesses and copy log files into output directory'''
	print('Exiting...')

	if 'lab_recorder' in global_dict and not global_dict['lab_recorder_closed'] :
		waitForEnter()

	#print(global_dict)
	if 'unity_proc' in global_dict :
		global_dict['unity_proc'].kill()
		print('Killed Unity server')

	if 'unity_server_log' in global_dict :
		global_dict['unity_server_log'].kill()
		print('Killed Unity server log tail process')

	if 'docker_proc' in global_dict :
		global_dict['docker_proc'].kill()
		print('Killed docker compose process')

	if 'data_proc' in global_dict :
		global_dict['data_proc'].kill()
		print('Killed data server process')

	if 'lab_recorder' in global_dict :
		global_dict['lab_recorder'].kill()
		print('Killed LabRecorder process')

	if 'diarc_ros_logs' in global_dict and 'output' in global_dict :
		copyLogs(global_dict['diarc_ros_logs'], global_dict['output'])
		print('Copied DIARC and ROS logs')

	if 'unity_logs' in global_dict and 'output' in global_dict :
		unityLogs(global_dict['unity_logs'], global_dict['output'])
		print('Copied Unity logs')

	if 'lsl' in global_dict and 'diarc_ros_logs' in global_dict and 'output' in global_dict :
		lslFile(global_dict['lsl'], global_dict['output'], global_dict['diarc_ros_logs'])
		print('Copied LSL data')

	if 'log_file' in global_dict :
		global_dict['log_file'].close()
		print('Closed launch.py log file')

if __name__ == '__main__':
	try:
		DIARCSpaceStation(args)
	except KeyboardInterrupt:
		pass
	finally:
		exitGracefully()
