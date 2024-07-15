#!/usr/bin/python

from argparse import ArgumentParser
import os.path
import subprocess

parser = ArgumentParser(prog='DIARC Space Station Simulation',
                    description='Script for orchestrating the launch of DIARC, ROS and the Unity Space Station environment for experimentation')

subparsers = parser.add_subparsers(dest='command', help='Commands to run', required=True)

build = subparsers.add_parser('build', parents=[parser], add_help=False, help='Build the DIARC Space Station docker image')
build.add_argument('--no-cache', dest='cache', default=True, action='store_false', help='Rebuild image without using cache')

stop = subparsers.add_parser('stop', parents=[parser], add_help=False, help='Gracefully stop all docker containers')
kill = subparsers.add_parser('kill', parents=[parser], add_help=False, help='Kill all docker containers')

args = parser.parse_args()
print(args)
class DIARCSpaceStation:
	def __init__ (self, args) :
		self._args = args
		print('DIARC Space Station Simulation')
		print('------------------------------')
		if args.command != 'help' :
			self._readEnv()
		if args.command == 'build' :
			self._build()
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
				key,value = line.strip().split('=', 1)
				self._env[key] = value
				print(f'[ENV] {key} = {value}')


	def _build(self) :
		print('build()')
		cmd = ['docker', 'build']
		if not self._args.cache :
			cmd += ['--no-cache']
		cmd += ['--progress=plain', '-t', 'hrilabtufts/unity_space_station', '-f', 'Dockerfile', '.']
		print(cmd)
		#proc = subprocess.run(['bash', './scripts/build.sh'], stdout=subprocess.PIPE)
		#for line in proc.stdout :
		#	print(f'[BUILD] {line}')

	def _kill(self) :

		proc = subprocess.Popen("docker kill $(docker container ls | grep unity_space_station | awk '{print $1}')", stdout=subprocess.PIPE, shell=True)
		for line in proc.stdout :
			print(f'[KILL] {line}')

	def _stop(self) :
		proc = subprocess.run(['bash', './scripts/stop.sh'], stdout=subprocess.PIPE)
		for line in proc.stdout :
			print(f'[STOP] {line}')

if __name__ == '__main__':
    DIARCSpaceStation(args)