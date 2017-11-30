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
# 	pin3				pin10
#	pin5				pin9
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
		self.op_code = 1 #idle
		self.channel_1_shift_value = 0
		self.channel_2_shift_value = 0
		self.channel_3_shift_value = 0
		
		#Create necessary local vars
		self.update_freq = 1000 #time in msec for timer experation
		self.msg_data = []
		self.dataHasChanged = False
		
		#Connect GUI features to functionality
			#connect frequency input
		self.connect(self.FreqDial, QtCore.SIGNAL('valueChanged(int)'), self.updateFreq)
			#connect gain input
		self.OffsetInput.valueChanged.connect(self.updateOffset)
			#connect offset input
		self.GainInput.valueChanged.connect(self.updateGain)
			#connect start button
		self.Startbtn.clicked.connect(self.startBtnPress)
			#connect stop button
		self.Stopbtn.clicked.connect(self.stopBtnPress)
		self.Stopbtn.setEnabled(False)
			#connect channel shift 
		self.Channel1_scale.valueChanged.connect(self.channelShiftUpdate)
		self.Channel2_scale.valueChanged.connect(self.channelShiftUpdate)
		self.Channel3_scale.valueChanged.connect(self.channelShiftUpdate)
			#connect reset button
		self.ResetBtn.clicked.connect(self.resetGenData)
		
		#Create I2C comm
		self.DEVICE_BUS = 1
		self.DEVICE_ADDR = 0x08
		try:
			self.bus = smbus.SMBus(self.DEVICE_BUS)
			self.updateSysStatus("Connected")
		except:
			self.updateSysStatus("I2C Connection Error Please Check Connections and Restart")
			
		#Create update timer
		self.timer = QtCore.QTimer()
		self.timer.setInterval(self.update_freq)
		self.timer.timeout.connect(self.timerExperation)
		self.timer.start()
	
	def resetGenData(self):
		#reset 3 phase generator settings
		self.FreqDial.setValue(1000)
		self.FreqLCD.display(self.freq)
		self.OffsetInput.setValue(0.0)
		self.GainInput.setValue(0.8)
		self.Startbtn.setEnabled(True)
		self.Stopbtn.setEnabled(False)
		self.Channel1_scale.setValue(0)
		self.Channel2_scale.setValue(0)
		self.Channel2_scale.setValue(0)
		self.op_code = 3
		#set global update value
		self.dataHasChanged = True
		
	def channelShiftUpdate(self):
		#update all channel shift values
		self.channel_1_shift_value = self.Channel1_scale.value()
		self.channel_2_shift_value = self.Channel2_scale.value()
		self.channel_3_shift_value = self.Channel3_scale.value()
		#set global update value
		self.dataHasChanged = True
		
	def updateSysStatus(self, msg):
		self.ErrorLbl.setText("Status: "+msg)
		
	def startBtnPress(self):
		#udpate op_code 
		self.op_code = 2 #start
		#set global update value
		self.dataHasChanged = True
		#disable startBtn and enable stopBtn
		self.Stopbtn.setEnabled(True)
		self.Startbtn.setEnabled(False)
		
	def stopBtnPress(self):
		#update op_code
		self.op_code = 3 #stop, cleanup, and then idle
		#set global update value
		self.dataHasChanged = True
		#disable stopBtn and enable startBtn
		self.Startbtn.setEnabled(True)
		self.Stopbtn.setEnabled(False)
		
	def updateFreq(self):
		#grab current dial value 
		self.freq = self.FreqDial.sliderPosition()
		#update LCD display to this value
		self.FreqLCD.display(self.freq)
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def updateOffset(self):
		#grab current spin box value
		self.offset=self.OffsetInput.value()		
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def updateGain(self):
		#grab current spin box value
		self.gain=self.GainInput.value()		
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def timerExperation(self):
		#every experation of the timer (as defined by update_freq)
		#this function will run. If udpates have been made to any of the 
		#four local parameters (freq, gain, offset, op_code)
		#a new message will be sent to the launchpad via the I2C 
		#interface to make these updates.
		if(self.dataHasChanged):
			#reset flag
			self.dataHasChanged = False
			
			#send new settings to the launchpad
			self.writeSettingsToGenerator()
			
	def writeSettingsToGenerator(self,debug=False):
		self.command = 0
		
		self.msg_data = self.createMsg(freq=self.freq, gain=self.gain, offset=self.offset, op_code=self.op_code)
		if debug:
			print("Write Test Data: ",self.msg_data)
		try:
			self.bus.write_i2c_block_data(self.DEVICE_ADDR, self.command, self.msg_data)
		except:
			self.updateSysStatus("I2C Write Error. Please Check Connections")
			
	def dataConversionForTransfer(self,value):
		r = "{0:#0{1}x}".format(value,6)	
		topBits = int(r[:-2], 16)
		lowerBits = int("0x"+r[4:], 16)
		return topBits, lowerBits
		
	def channelShiftToCode(self, channel_val):
		'''Channel Shift Codes to Pass Negatives
			Shift Value 	|		Transmit Code
				-1						0
				 0						1
				 1						2
		'''
		if(channel_val==-1):
			return 0
		elif(channel_val==0):
			return 1
		elif(channel_val==1):
			return 2
	
	def createMsg(self,
					freq			= 1000, 
					gain			= 0.8003, 
					offset			= 0, 
					op_code			= 1,
					channel1_shift 	= 0,
					channel2_shift 	= 0,
					channel3_shift 	= 0,
					debug			= False):
		#limited to 8bits transfered in each index(i.e: [0xFF,0x43 ....])			
		'''MSG structure:
			data = [
			0		: 	Command
			1		:	Buffer
			2		:	Freq MSB
			3		:	Freq LSB
			4		:	Buffer
			5		:	Gain
			6		:	Buffer
			7		:	Offset Sign
			8		:	Offset
			9		:	Buffer
			10		:	Op_Code
			11		:	Buffer
			12		:	Channel 1 Shift
			13		:	Channel 2 Shift
			14		:	Channel 3 Shift
			15		: 	Buffer	
			]
			'''
		bufferVal = 0
		
		#Add Buffer after Command
		msg = [bufferVal]
		if debug:
			print(msg)
			
		#Add Freq and Buffer
		MSB,LSB = self.dataConversionForTransfer(freq)
		msg.append(MSB)
		msg.append(LSB)	
		msg.append(bufferVal)
		if debug:
			print(msg)
			
		#Add Gain and Buffer
		MSB,LSB = self.dataConversionForTransfer(int(gain*10)) #multiplied by 10 to avoid decimals
		#msg.append(MSB) nothing should be in this bit gain is 0-1 (0-10 after *10)
		msg.append(LSB)	
		msg.append(bufferVal)
		if debug:
			print(msg)
			
		#Add Offset Sign, Offset Value, and Buffer
		if(offset<0):
			msg.append(1)	#offset is negative
		else:
			msg.append(0)
		MSB,LSB = self.dataConversionForTransfer(int(offset*10)) #multiplied by 10 to avoid decimals
		#msg.append(MSB) #nothing should be in this value offset is -1 <-> 1
		msg.append(LSB)	
		msg.append(bufferVal)
		if debug:
			print(msg)
			
		#Add Op_code and Buffer
		MSB,LSB = self.dataConversionForTransfer(op_code)
		#msg.append(MSB) Nothing in this bit
		msg.append(LSB)	
		msg.append(bufferVal)
		if debug:
			print(msg)
		
		#Add channel 1-3 shifts and Buffer
		msg.append(self.channelShiftToCode(self.channel_1_shift_value))
		msg.append(self.channelShiftToCode(self.channel_2_shift_value))
		msg.append(self.channelShiftToCode(self.channel_3_shift_value))
		msg.append(bufferVal)
		if debug:
			print(msg)
			print(len(msg))
		
		return msg
		
		
	def closeEvent(self,event):
		self.bus.close()
		

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
