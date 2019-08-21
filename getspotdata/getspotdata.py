# getspotdata.py
#
# Receive data from the Spot Demo and store them to a file
#
# Copyright (c) 2019 INFICON Ltd.
#

import sys
import serial
from serial import SerialException
import argparse

parser = argparse.ArgumentParser(description='Get streaming data from the SPOT demonstrator and store them to a file.')
parser.add_argument('-p', '--port', nargs=1, default=['COM1'], help='COM port of the Demonstrator')
parser.add_argument('-f', '--file', nargs=1, default=['out.csv'], help='name of the output csv file')
args = parser.parse_args()

try:
	ser = serial.Serial(args.port[0], 2000000, timeout=0.5)
except SerialException:
	print('Could not open serial port ' + args.port[0] + "!")
	sys.exit(1)

try:
	f = open(args.file[0], "w")
except PermissionError:
	print('Could not create file ' + args.file[0] + '. No permission.')
	sys.exit(1)

foundPound = False
started = False
running = True

print("Waiting for data stream...")

# initialize empty buffer
buf = ''

try:
	while running:
		received = ser.read(10000)
		buf += received.decode('ascii')
		lines = buf.split('\r\n')
		
		# save last element for later (is either empty or the begin of a new line
		buf = lines.pop()
		
		for line in lines:
			if started:
				# record data until a line starts with #
				if line[:1] == '#':
					running = False
					break
				f.write(line + '\n')
			elif foundPound:
				# wait for first line with data
				if line[0].isdigit():
					started = True
					print("Streaming started.")
			elif line[:1] == '#':
				foundPound = True

	print("Streaming from Spot Demonstrator was stopped.")

except KeyboardInterrupt:
	print("quit.")
