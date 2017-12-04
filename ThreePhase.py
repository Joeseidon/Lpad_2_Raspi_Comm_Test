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
		self.freq = 3 #hz 
		self.freqStep = 1
		self.globalGain = 0.8003
		self.globalOffset = 0
		self.op_code = 1 #idle
		self.channel_1_offset_value = 0
		self.channel_2_offset_value = 0
		self.channel_3_offset_value = 0
		self.channel_1_gain_value = 1.0
		self.channel_2_gain_value = 1.0
		self.channel_3_gain_value = 1.0
		
		#Create necessary local vars
		self.update_freq = 1000 #time in msec for timer experation
		self.msg_data = []
		self.dataHasChanged = False
		
		#Connect GUI features to functionality
			#connect frequency input
		self.Increase_Freq_Btn.clicked.connect(self.increaseFreq)
		self.Decrease_Freq_Btn.clicked.connect(self.decreaseFreq)
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
			#connect channel offset 
		self.Channel1_offset.valueChanged.connect(self.channelOffsetUpdate)
		self.Channel2_offset.valueChanged.connect(self.channelOffsetUpdate)
		self.Channel3_offset.valueChanged.connect(self.channelOffsetUpdate)
			#connect channel multipliers
		self.Channel1_gain.valueChanged.connect(self.channelGainUpdate)
		self.Channel2_gain.valueChanged.connect(self.channelGainUpdate)
		self.Channel3_gain.valueChanged.connect(self.channelGainUpdate)
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
	
	def increaseFreq(self):
		#grab current dial value 
		self.freq = self.FreqDial.sliderPosition() + self.freqStep
		self.FreqDial.setValue(self.freq)
		#update LCD display to this value
		self.FreqLCD.display(self.freq)
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def decreaseFreq(self):
		#grab current dial value 
		self.freq = self.FreqDial.sliderPosition() - self.freqStep
		self.FreqDial.setValue(self.freq)
		#update LCD display to this value
		self.FreqLCD.display(self.freq)
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def resetGenData(self):
		#reset 3 phase generator settings
		self.FreqDial.setValue(3)
		self.FreqLCD.display(self.freq)
		self.OffsetInput.setValue(0.0)
		self.GainInput.setValue(0.8)
		self.Startbtn.setEnabled(True)
		self.Stopbtn.setEnabled(False)
		self.Channel1_offset.setValue(0.0)
		self.Channel2_offset.setValue(0.0)
		self.Channel3_offset.setValue(0.0)
		self.Channel1_gain.setValue(0.0)
		self.Channel2_gain.setValue(0.0)
		self.Channel3_gain.setValue(0.0)
		self.op_code = 3
		#set global update value
		self.dataHasChanged = True
		
	def channelOffsetUpdate(self):
		valid_change = True
		'''print('Channel 1: ', self.channel_1_offset_value)
		print('Channel 2: ', self.channel_2_offset_value)
		print('Channel 3: ', self.channel_3_offset_value)
		print('Global Offset: ', self.globalOffset)
		#update all channel offset values
		if ((self.channel_1_offset_value + self.globalOffset > 1) or (self.channel_1_offset_value + self.globalOffset < -1)):
				print("ch1 off error")
		else:
			valid_change = False
						
		if ((self.channel_2_offset_value + self.globalOffset > 1) or (self.channel_2_offset_value + self.globalOffset < -1)):
				print("ch2 off error")
		else:
			valid_change = False
			
		if ((self.channel_3_offset_value + self.globalOffset > 1) or (self.channel_2_offset_value + self.globalOffset < -1)):
				print("ch3 off error")
		else:
			valid_change = False'''
			
		if(valid_change):
			self.channel_1_offset_value = self.Channel1_offset.value()
			self.channel_2_offset_value = self.Channel2_offset.value()
			self.channel_3_offset_value = self.Channel3_offset.value()
			#set global update value
			self.dataHasChanged = True
	
	def channelGainUpdate(self):
		valid_change = True
		'''print('Channel 1: ', self.channel_1_gain_value)
		print('Channel 2: ', self.channel_2_gain_value)
		print('Channel 3: ', self.channel_3_gain_value)
		print('Global Gain: ', self.globalGain)
		#update all channel shift values
		if ((self.channel_1_gain_value + self.globalGain > 1) or (self.channel_1_gain_value + self.globalGain < 0)):
				print("ch1 gain error")
		else:
			valid_change=False
			
		if ((self.channel_2_gain_value + self.globalGain > 1) or (self.channel_2_gain_value + self.globalGain < 0)):
				print("ch2 gain error")
		else:
			valid_change=False
			
		if ((self.channel_3_gain_value + self.globalGain > 1) or (self.channel_3_gain_value + self.globalGain < 0)):
				print("ch3 gain error")
		else:
			valid_change=False'''
			
		if(valid_change):
			self.channel_1_gain_value = self.Channel1_gain.value()
			self.channel_2_gain_value = self.Channel2_gain.value()
			self.channel_3_gain_value = self.Channel3_gain.value()
			#set global update value
			self.dataHasChanged = True
		
		
	def outOfRangeMsgBox(self, msg):	
		msgBox = QtGui.QMessageBox()
		msgBox.setIcon(QtGui.QMessageBox.Warning)
		msgBox.setText(msg)
		msgBox.setInformativeText("The General Settings combine with the Channel Settings")
		msgBox.setWindowTitle("Value Beyond Excepted Range")
		msgBox.setStandardButtons(QtGui.QMessageBox.Ok)
	
	def updateSysStatus(self, msg="Normal"):
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
		self.globalOffset=self.OffsetInput.value()		
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def updateGain(self):
		#grab current spin box value
		self.globalGain=self.GainInput.value()		
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
			self.writeSettingsToGenerator(debug=True)
			
	def writeSettingsToGenerator(self,debug=False):
		self.command = 0
		
		self.msg_data = self.createMsg(debug=False)
		self.sendMsg(msg=self.msg_data)
	
	def sendMsg(self, msg, debug=False):
		if debug:
			print("Write Test Data: ",self.msg_data)
		try:
			self.bus.write_i2c_block_data(self.DEVICE_ADDR, self.command, self.msg_data)
			self.updateSysStatus()
		except:
			self.updateSysStatus("I2C Write Error. Please Check Connections")
			
	def dataConversionForTransfer(self,value):
		r = "{0:#0{1}x}".format(value,6)	
		topBits = int(r[:-2], 16)
		lowerBits = int("0x"+r[4:], 16)
		return topBits, lowerBits
		
	def determineChannelGain(self, channel):
		if(channel == 1):
			chGain = self.channel_1_gain_value
		elif(channel == 2):
			chGain = self.channel_2_gain_value
		elif(channel == 3):
			chGain = self.channel_3_gain_value
			
		total_gain = (self.globalGain + chGain)
		#Protect againts out of range issues 
		if(total_gain < 0):
			total_gain = 0
		elif(total_gain > 1):
			total_gain = 1
			
		gain_msb, gain_lsb = self.dataConversionForTransfer(int(total_gain*100)) #multiplied by 100 to transfer two decimal places
			
		return gain_msb, gain_lsb
		
	def determineChannelOffset(self, channel):
		if(channel == 1):
			chOffset = self.channel_1_offset_value
		elif(channel == 2):
			chOffset = self.channel_2_offset_value
		elif(channel == 3):
			chOffset = self.channel_3_offset_value
			
		total_offset = (self.globalOffset + chOffset)
		#Protect againts out of range issues 
		if(total_offset > 1):
			total_offset = 1
		elif(total_offset < -1):
			total_offset = -1
			
		if(total_offset >= 0):
			sign = 0
		else:
			sign = 1
			
		total_offset = abs(total_offset) * 10 #scaled to avoid passing deciaml values
		return sign, total_offset
	
	def getChannelData(self, channel):			
		#Combine general gain and channel modification to find
		#requried gain
		gain_msb, gain_lsb = self.determineChannelGain(channel)
		#Determine combine channel offset
		offset_sign, offset = self.determineChannelOffset(channel)
			
		return offset_sign, offset, gain_msb, gain_lsb
	
	def createMsg(self,close_Msg	= False,
					   debug		= False):
		#limited to 8bits transfered in each index(i.e: [0xFF,0x43 ....])			
		'''MSG structure:
			data = [
			0		: 	Command
			1		:	Op_code
			2		:	Freq MSB
			3		:	Freq LSB
			4		:	Channel 1 Offset Sign
			5		:	Channel 1 Offset
			6		:	Channel 1 Gain MSB
			7		:	Channel 1 Gain LSB
			8		:	Channel 2 Offset Sign
			9		:	Channel 2 Offset
			10		:	Channel 2 Gain MSB
			11		:	Channel 2 Gain LSB
			12		:	Channel 3 Offset Sign
			13		:	Channel 3 Offset
			14		:	Channel 3 Gain MSB
			15		: 	Channel 3 Gain LSB
			]
			'''
			
		if close_Msg:
			#send stop op_code and all other bits are zero
			msg = [3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		else:	
			msg = []
			
			#Add Op_code
			MSB,LSB = self.dataConversionForTransfer(self.op_code)
			msg.append(LSB)	
			if debug:
				print(msg)
				
			#Add Freq
			MSB,LSB = self.dataConversionForTransfer(self.freq)
			msg.append(MSB)
			msg.append(LSB)	
			if debug:
				print(msg)
				
			for channel in range(1,4):
				offset_sign, offset, gain_msb, gain_lsb = self.getChannelData(channel)
				print("Channel: ",channel)
				print("\t",offset_sign, offset, gain_msb, gain_lsb)
				msg.append(offset_sign)
				msg.append(offset)
				msg.append(gain_msb)
				msg.append(gain_lsb)
				if debug:
					print(msg)
					print(len(msg))
				
			
			'''#Add Buffer after Command
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
			if debug:
				print(msg)
				
			#Add Op_code and Buffer
			MSB,LSB = self.dataConversionForTransfer(op_code)
			#msg.append(MSB) Nothing in this bit
			msg.append(LSB)	
			if debug:
				print(msg)
			
			#Add channel 1-3 Offset
			msg.append(self.channelShiftToCode(self.channel_1_offset_value))
			msg.append(self.channelShiftToCode(self.channel_2_offset_value))
			msg.append(self.channelShiftToCode(self.channel_3_offset_value))
			if debug:
				print(msg)
				
			#Add channel 1-3 Multipliers and Buffer
			msg.append(int(self.channel_1_gain_value * 10))
			msg.append(int(self.channel_2_gain_value * 10))
			msg.append(int(self.channel_3_gain_value * 10))
			msg.append(bufferVal)
			if debug:
				print(msg)
				print(len(msg))'''
		print(msg)
		
		return msg
		
		
	def closeEvent(self,event):
		#send default msg to reset all values and stop the generator if it is running
		self.msg_data = self.createMsg(close_Msg = True)
		self.sendMsg(msg=self.msg_data)
		#close the I2C bus
		self.bus.close()
		

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
