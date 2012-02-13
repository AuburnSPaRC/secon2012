#!/usr/bin/env python
import pygtk
pygtk.require("2.0")
import sys
import gtk
import serial
import struct

currentStage	#The current stage we are working on
turntype=0;   #Turn type


class DebuggerGUI(object):       
	def __init__(self):
		builder=gtk.Builder();
		builder.add_from_file("debugger2.glade")
		builder.connect_signals(self)
		self.window=builder.get_object("MainWindow")
		self.turn_window=builder.get_object("TurnTypeWindow")
		self.P=builder.get_object("P")
		self.I=builder.get_object("I")
		self.D=builder.get_object("D")
		self.window.set_position(gtk.WIN_POS_CENTER)
		self.window.show_all()
		

	
	#Pressed the turntype window button	
	def callback_turntype(self, widget, callback_data=None):
		self.turn_window.show_all()

	#Pressed Ok in the turntype window
	def callback_turntype_ok(self,widget,callback_data=None):
		self.turn_window.hide_all();

	#Pressed Cancel in the turntype window
	def callback_turntype_cancel(self, widget, callback_data=None):
		self.turn_window.hide_all();
	
	#Pressed exit in MainWindow
	def callback_exit(self, widget, callback_data=None):
		gtk.main_quit()
	






	#Pressed send in MainWindow
	def callback_send(self, widget, callback_data=None):
		pVal=float(self.P.get_text())
		iVal=float(self.I.get_text())
		dVal=float(self.D.get_text())
		ser=serial.Serial('/dev/ttyACM2',baudrate=9600)
		info=struct.pack('=cfff','d',pVal,iVal,dVal)
		ser.write(info)
		#unp=struct.unpack('cfff',info
		ser.close()

if __name__ == "__main__":
	app=DebuggerGUI()
	gtk.main()
