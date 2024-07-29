from argparse import ArgumentParser
import os

has_flask = False

try :
	from flask import Flask, flash, request, redirect, url_for
	has_flask = True
except ImportError:
    print('WARNING: Module flask is not installed, data from Unity clients cannot be collected automatically')
    exit()

parser = ArgumentParser(prog='python server.py')
parser.add_argument('-p', '--port', type=int, default=8888, help='Port to run this server on')
parser.add_argument('-t', '--timestamp', default='', help='Timestamp to use for storing files')
parser.add_argument('-o', '--output', default=None, help='Output directory')
args = parser.parse_args()

def mkdir (path) :
	'''Make a directory if it does not exist'''
	if not os.path.exists(path) :
		os.makedirs(path)
		print(f'Created directory {path}')
	else :
		print(f'Directory {path} exists')

server = Flask('cdc_server')

@server.route('/')
def index():
	return 'OK'

@server.route('/<datadir>', methods=['POST'])
def upload_file(datadir):
	for key in request.files.keys() :
		uploaded_file = request.files[key]
		break
	if uploaded_file.filename != '' and args.output is not None :
		mkdir(os.path.join(args.output, datadir))
		uploaded_file.save(os.path.join(args.output, datadir, uploaded_file.filename))
		print(f'Uploaded file {datadir}/{uploaded_file.filename}')
	return 'OK'

if __name__ == '__main__' :


	if has_flask :
		server.run(host='0.0.0.0', port=args.port)