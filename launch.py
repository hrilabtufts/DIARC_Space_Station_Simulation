#!/usr/bin/python

from argparse import ArgumentParser
from datetime import datetime
import os
import subprocess
import tempfile


parser = ArgumentParser(prog='DIARC Space Station Simulation',
                    description='Script for orchestrating the launch of DIARC, ROS and the Unity Space Station environment for experimentation')

subparsers = parser.add_subparsers(dest='command', help='Commands to run', required=True)

build = subparsers.add_parser('build', add_help=False, help='Build the DIARC Space Station docker image')
build.add_argument('--no-cache', dest='cache', default=True, action='store_false', help='Rebuild image without using cache')

dev = subparsers.add_parser('dev', add_help=False, help='Run the simulation servers in dev mode')
dev.add_argument('-g', '--gui', default=False, action='store_true', help='Enable GUI settings in DIARC and ROS')
dev.add_argument('-r', '--robots', type=int, default=None, help='Set the number of robots to spawn in the trial')
dev.add_argument('-s', '--smm', default=None, action='store_true', help='Set the number of robots to spawn in the trial')
dev.add_argument('diarc', type=str, nargs='+')

run = subparsers.add_parser('run', add_help=False, help='Run the simulation server(s) in production mode')
run.add_argument('-g', '--gui', default=False, action='store_true', help='Enable GUI settings in DIARC and ROS')
run.add_argument('-r', '--robots', type=int, default=None, help='Set the number of robots to spawn in the trial')
run.add_argument('-s', '--smm', default=None, action='store_true', help='Set the number of robots to spawn in the trial')

stop = subparsers.add_parser('stop', add_help=False, help='Gracefully stop all docker containers')
kill = subparsers.add_parser('kill', add_help=False, help='Kill all docker containers')

args = parser.parse_args()


class DIARCSpaceStation:
	docker_compose = './docker-compose.yaml'

	additional_services_tmpl = './config/docker/docker-compose-additional-services.yaml.tmpl'
	
	trade_properties_default = 'diarc.tradePropertiesFile=src/main/resources/default/trade.properties.default'
	trade_properties_smm = 'diarc.tradePropertiesFile=/root/trade.properties'

	trade_properties_hub_tmpl = './config/diarc/trade.hub.properties'
	trade_properties_spoke_tmpl = './config/diarc/trade.spoke.properties'

	gradle_properties_tmpl = './config/gradle/gradle.properties'

	ros_tmpl = './config/ros/startup_pr2_docker.launch.tmpl'
	
	ros_port = 9090
	unity_port = 1755
	network_start = 4

	def __init__ (self, args) :
		self._args = args
		print('DIARC Space Station Simulation')
		self._line()

		if args.command in ['run', 'dev'] :
			self._readEnv()

		if args.command == 'build' :
			self._build()
		elif args.command == 'dev' :
			self._dev()
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
				if line.startswith('#') :
					continue
				key, value = line.strip().split('=', 1)
				value = value.replace('"', '')
				if value.lower() == 'false' :
					value = False
				elif value.lower() == 'true' :
					value = True
				self._env[key] = value
				print(f'[ENV] {key} = {value}')
		self._line()

	def _overrides(self) :
		if 'robots' in self._args and self._args.robots is not None :
			self._log(f'Overriding ENV value ROBOTS ({self._env["ROBOTS"]}) with {self._args.robots}')
			self._env['ROBOTS'] = str(self._args.robots)

		if 'smm' in self._args and self._args.smm is not None :
			self._log(f'Overriding ENV value SMM ({self._env["SMM"]}) with {self._args.smm}')
			self._env['SMM'] = self._args.smm 


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

	def _dev(self) :
		print('dev()')
		print(self._args)
		self._overrides()

		robot_tmpl = './config/docker/docker-compose-robot-dev-headless.yaml.tmpl'

		timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
		robots = int(self._env['ROBOTS'])
		llm_port = int(self._env['LLM_PORT'])
		ros_port = self.ros_port
		unity_port = self.unity_port
		network = self.network_start
		gui = False

		if len(self._args.diarc) != robots :
			self._log(f'ERROR: Robot count ({robots}) != number of DIARC directories in command ({len(self._args.diarc)})')
			exit(1)

		if self._args.gui :
			self._log('WARN: GUI features not available in this version of the script.')
			#self._setupX()
			#robot_tmpl = './config/docker/docker-compose-robot-dev.yaml.tmpl'
			#gui = True
		
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

			trade_properties_tmp = self._mktemp()
			trade_properties = self._tradeProperties(i)
			self._writeLines(trade_properties_tmp, trade_properties)

			gradle_properties_tmp = self._mktemp()
			gradle_properties = self._gradleProperties()
			self._writeLines(gradle_properties_tmp, gradle_properties)

			ros_tmp = self._mktemp()
			ros = self._rosConfig(ros_port, gui)
			self._writeLines(ros_tmp, ros)


			ros_port += 1
			unity_port += 1
			llm_port += 1
			network += 1

		for line in docker_compose :
			self._log(line)

	def _additionalServices (self) :
		additional_services = self._readLines(self.additional_services_tmpl)
		additional_services_map = {
			'NETWORK_PREFIX' : self._env['NETWORK_PREFIX']
		}
		return self._replaceLines(additional_services, additional_services_map)

	def _tradeProperties (self, i) :
		trade_properties_tmpl = self.trade_properties_hub_tmpl
		if i > 0 :
			trade_properties_tmpl = self.trade_properties_spoke_tmpl
		trade_properties = self._readLines(trade_properties_tmpl)
		trade_properties_map = {
			'NETWORK_PREFIX' : self._env['NETWORK_PREFIX']
		}
		return self._replaceLines(trade_properties, trade_properties_map)

	def _gradleProperties (self) :
		gradle_properties = self._readLines(self.gradle_properties_tmpl)
		trade_properties_line = self.trade_properties_default
		if self._env['SMM'] :
			trade_properties_line = self.trade_properties_smm
		gradle_properties_map = {
			'TRADE_PROPERTIES' : trade_properties_line
		}
		return self._replaceLines(gradle_properties, gradle_properties_map)

	def _rosConfig (self, ros_port, gui) :

		ros = self._readLines(self.ros_tmpl)
		ros_map = {
			'DISPLAY_RVIZ' : str(gui),
			'DISPLAY_GAZEBO' : str(gui)
		}
		return self._replaceLines(ros, ros_map)

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
		self._log(f'CMD: {cmd}')
		proc = subprocess.run(cmd, stdout=subprocess.PIPE)
		for line in proc.stdout :
			self._log(line)

	def _readLines (self, path) :
		lines = []
		with open(path, 'r') as file :
			lines = [line.rstrip() for line in file]
		return lines

	def _replaceLines(self, lines, replace_map) :
		outLines = []
		for line in lines :
			for key in replace_map :
				line = line.replace(key, replace_map[key])
			outLines.append( line )
		return outLines

	def _writeLines (self, path, lines) :
		with open(path, 'w') as file:
			for line in lines :
				file.write(f'{line}\n')
		self._log(f'Wrote file {path}')

	def _mkdir (self, path) :
		if not os.path.exists(path) :
			os.makedirs(path)
			self._log(f'Created directory {path}')
		else :
			self._log(f'Directory {path} exists')

	def _mktemp (self) :
		tmp_file, filename = tempfile.mkstemp()
		return filename

	def _log (self, line) :
		print(f'[{self._args.command.upper()}] {line}')

	def _line(self) :
		print('------------------------------')


if __name__ == '__main__':
    DIARCSpaceStation(args)