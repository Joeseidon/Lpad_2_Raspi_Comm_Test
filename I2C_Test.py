#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  I2C_Test.py
#  
#  Copyright 2017  <pi@raspberrypi>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

#I2C Connection
#	Raspi		|		Target
# 	pin3				pinA4
#	pin5				pinA5
#	pin6				GND

import smbus
import struct
import sys

def testsmbus(write=False,read=False):
	DEVICE_BUS = 1
	DEVICE_ADDR = 0x08
	print("I2C test on BUS: ",str(DEVICE_BUS)," to device addr: ",str(DEVICE_ADDR))
	bus = smbus.SMBus(DEVICE_BUS)
	command = 0
	if write:
		data = [0,0,0,0,77,44,25,52,77,53,0,0,0,0,0]
		print("Write Test Data: ",data)
	
		bus.write_i2c_block_data(DEVICE_ADDR, command, data)
	
	if read:
		datalist = bus.read_i2c_block_data(DEVICE_ADDR, command)
		print(datalist)
		print("Length: ",len(datalist))
		target = datalist[:4] #first four bytes hold float=scale data
		print("Target Data: ",target)
		pack = struct.pack('4B',*target) #convert the bytes to a string package for unpack
		value = struct.unpack('f',pack)  #converts string package of bytes to float
		print("Value: ",value)
		

if __name__ == '__main__':
	testsmbus(write=True)
	sys.exit()
