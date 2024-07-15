#!/usr/bin/python

from argparse import ArgumentParser
import os.path
import subprocess

commands = [ 'build', 'dev', 'run', 'stop', 'kill']


parser = ArgumentParser(prog='DIARC Space Station Simulation',
                    description='Script for orchestrating the launch of DIARC, ROS and the Unity Space Station environment for experimentation')

parser.add_argument('command',  type=str, help=f'Command to execute: options {", ".join(commands)}')


args = parser.parse_args()


class DIARCSpaceStation:
	def __init__ (self, args) :
		print('DIARC Space Station Simulation')
		print('------------------------------')
		self._readEnv()
		if args.command == 'build' :
			self._build()
		elif args.command == 'kill' :
			self._kill()
		elif args.command == 'stop' :
			self._stop()

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
		proc = subprocess.run(['bash', './scripts/build.sh'], stdout=subprocess.PIPE)
		for line in proc.stdout :
			print(f'[BUILD] {line}')

	def _kill(self) :
		proc = subprocess.run(['bash', './scripts/kill.sh'], stdout=subprocess.PIPE)
		for line in proc.stdout :
			print(f'[KILL] {line}')

	def _stop(self) :
		proc = subprocess.run(['bash', './scripts/stop.sh'], stdout=subprocess.PIPE)
		for line in proc.stdout :
			print(f'[STOP] {line}')

if __name__ == '__main__':
    DIARCSpaceStation(args)