#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  ThreePhase.py
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

import sys
import os
import subprocess
from PyQt4 import QtGui, QtCore, uic

#I2C Connection
#	Raspi		|		Target
# 	pin3				pinA4
#	pin5				pinA5
#	pin6				GND

import smbus
import struct


class MyWindow(QtGui.QMainWindow):
	def __init__(self):
		#Create Window
		super(MyWindow, self).__init__()
		uic.loadUi('ThreePhase.ui',self)
		self.show()
		qr = self.frameGeometry()
		cp = QtGui.QDesktopWidget().availableGeometry().center()
		qr.moveCenter(cp)
		self.move(qr.topLeft())
		
		#Create Vars to hold output values
		self.freq = 1000 #hz
		self.gain = 0.8003
		self.offset = 0
		self.op_code = 1
		
		#Connect GUI features to functionality
		
		
		#Create I2C comm
		self.DEVICE_BUS = 1
		self.DEVICE_ADDR = 0x08
		self.bus = smbus.SMBus(self.DEVICE_BUS)
		
	def writeSettingsToGenerator(self,debug=False):
		command = 0
		
		msg_data = createMsg(freq=self.freq, gain=self.gain, offset=self.offset, op_code=self.op_code)
		if debug:
			print("Write Test Data: ",msg_data)
	
		bus.write_i2c_block_data(DEVICE_ADDR, command, msg_data)
		
	def dataConversionForTransfer(self,value):
		r = "{0:#0{1}x}".format(value,6)	
		topBits = int(r[:-2], 16)
		lowerBits = int("0x"+r[4:], 16)
		return topBits, lowerBits
	
	def createMsg(freq=5000, gain=0.8003, offset=0, op_code=1,debug=False):
		#limited to 8bits transfered in each index(i.e: [0xFF,0x43 ....])
		'''MSG structure:
			data = [
			0-2		: 	buffer
			3-4		: 	FREQ: 4 is the top 8 bits and 5 is the lower 8 bits
			5		: 	buffer
			6-7		:	GAIN: 7 top 8 bits, 8 lower 8 bits
			8		:	buffer
			9-10	:	OFFSET: 10 top 8 bits, 11 lower 8 bits
			11		:	buffer
			12-13	:	op_code: indicates to the gen when to produce waveform
			14-15	:	buffer
			]
			'''
		
		msg = [0,0,0]
		buf = 0
		if debug:
			print(msg)
		MSB,LSB = self.dataConversionForTransfer(freq)
		msg.append(MSB)
		msg.append(LSB)	
		msg.append(buf)
		if debug:
			print(msg)
		MSB,LSB = self.dataConversionForTransfer(int(gain*10)) #multiplied by 10 to avoid decimals
		msg.append(MSB)
		msg.append(LSB)	
		msg.append(buf)
		if debug:
			print(msg)
		MSB,LSB = self.dataConversionForTransfer(int(offset*10)) #multiplied by 10 to avoid decimals
		msg.append(MSB)
		msg.append(LSB)	
		msg.append(buf)
		if debug:
			print(msg)
		MSB,LSB = self.dataConversionForTransfer(op_code)
		msg.append(MSB)
		msg.append(LSB)	
		msg.append(buf)
		msg.append(buf)
		if debug:
			print(msg)
			print(len(msg))
		
		return msg
		

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
