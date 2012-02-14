#!/usr/bin/env python
import pygtk
pygtk.require("2.0")
import sys
import gtk
import serial
import struct

currentStage=-1	#The current stage we are working on
turntype=0   #Turn type


class DebuggerGUI(object):       
	def __init__(self):
		builder=gtk.Builder();
		builder.add_from_file("debugger2.glade")
		builder.connect_signals(self)
		self.window=builder.get_object("MainWindow")
		self.summary_window=builder.get_object("SummaryWindow")
		self.turn_window=builder.get_object("TerminationWindow")
		self.termination_combo=builder.get_object("termination_combo")


		#Add List To Termination Table
		self.termination_combo.set_active(0)
		cell = gtk.CellRendererText()
		self.termination_combo.pack_start(cell, True)
		self.termination_combo.add_attribute(cell, "text", 0)
		###############################################

		
		self.window.set_position(gtk.WIN_POS_CENTER)	#Center the window on-screen
		self.window.show_all()				#Show the window
		#self.summary_window.show_all()
		

	
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
	
	#New Position, load data
	def callback_pos_changed(self, widget, callback_data=None):
		f=open("settings.txt","r");
		line=f.readline();
		print line;
		f.close();






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
