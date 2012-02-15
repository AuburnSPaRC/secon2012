#!/usr/bin/env python
import pygtk
pygtk.require("2.0")
import sys
import gtk
import serial
import struct
import string

currentStage=-1	#The current stage we are working on
enc_fol=0	#Following = 0, encoding =1
termination=0
termination_action=0
left_amnt=0
right_amnt=0
start_pos=0


class DebuggerGUI(object):       
	def __init__(self):
		builder=gtk.Builder();
		builder.add_from_file("debugger2.glade")
		builder.connect_signals(self)
		#Get Handles to needed objects
		self.window=builder.get_object("MainWindow")				#Main Window
		self.summary_window=builder.get_object("SummaryWindow")			#Summary Window
		self.action_window=builder.get_object("ActionWindow")			#Action Window
		self.turn_window=builder.get_object("TerminationWindow")		#Termination Type Window
		self.termination_combo=builder.get_object("termination_combo")		#List Box of Termination Types
		self.action_list_box=builder.get_object("action_list_box")		#List Box of Action Types
		self.pos_select=builder.get_object("pos_select")			#Entry Box for current position	
		self.enc_fol_button=builder.get_object("follow_enc_main")		#Encoder/Following Toggle Button
		self.right_amount_action_box=builder.get_object("right_amount_box")	#Right Wheel Clicks Action Amount
		self.left_amount_action_box=builder.get_object("left_amount_box")	#Left Wheel Clicks Action Amount
		self.start_list_box=builder.get_object("start_list_box")		#List Box of Start Positions

		#Add List To Action Table
		self.start_list_box.set_active(start_pos)
		cell = gtk.CellRendererText()
		self.start_list_box.pack_start(cell, True)
		self.start_list_box.add_attribute(cell, "text", 0)
		###############################################

		#Add List To Action Table
		self.action_list_box.set_active(termination_action)
		cell = gtk.CellRendererText()
		self.action_list_box.pack_start(cell, True)
		self.action_list_box.add_attribute(cell, "text", 0)
		###############################################

		#Add List To Termination Table
		self.termination_combo.set_active(termination)
		cell = gtk.CellRendererText()
		self.termination_combo.pack_start(cell, True)
		self.termination_combo.add_attribute(cell, "text", 0)
		###############################################

		
		self.window.set_position(gtk.WIN_POS_CENTER)	#Center the window on-screen
		self.window.show_all()				#Show the window
		#self.summary_window.show_all()
		

	#Action Window Opened
	def callback_action(self, widget, callback_data=None):
		self.action_list_box.set_active(termination_action)
		self.left_amount_action_box.set_text(str(left_amnt))
		self.right_amount_action_box.set_text(str(right_amnt))
		self.action_window.show_all()
	#############################################################

	#Action Window OK
	def callback_end_ok(self, widget, callback_data=None):
		global termination_action
		global left_amnt
		global right_amnt
		if currentStage!=-1:
			left_amnt=int(self.left_amount_action_box.get_text())
			right_amnt=int(self.right_amount_action_box.get_text())
			termination_action=self.action_list_box.get_active()
		self.action_window.hide_all()
	#############################################################

	def callback_end_cancel(self, widget, callback_data=None):
		self.action_window.hide_all()
		
	#Toggled line following/encoders
	def callback_toggle_following(self, widget, callback_data=None):
		global enc_fol
		if currentStage!=-1:
			if enc_fol == 0:
				self.enc_fol_button.set_label("Encoders")
				enc_fol=1
			else:
				self.enc_fol_button.set_label("Following")
				enc_fol=0
			



	#Pressed the termination window button	
	def callback_termination(self, widget, callback_data=None):
		self.termination_combo.set_active(termination)
		self.turn_window.show_all()

	#Pressed Ok in the termination window
	def callback_termination_ok(self,widget,callback_data=None):
		global termination
		if currentStage!=-1:
			termination=self.termination_combo.get_active()
		self.turn_window.hide_all();

	#Pressed Cancel in the termination window
	def callback_termination_cancel(self, widget, callback_data=None):
		self.turn_window.hide_all();
	
	#Pressed exit in MainWindow
	def callback_exit(self, widget, callback_data=None):
		gtk.main_quit()
	

	#Save data
	def callback_save_data(self,widget,callback_data=None):
		if currentStage!=-1:
			l=0
			f=open("settings.txt","r")
			lines=f.readlines()
			f.close()	

			f=open("settings.txt","w")
			while l<=37:
				if l==currentStage:
					f.write(str(enc_fol)+" "+str(termination)+" "+str(termination_action)+" "+str(left_amnt)+" "+str(right_amnt)+"\n")
				else:
					f.write(lines[l])	
				l=l+1;
			f.flush()
			f.close()
		else:
			print "No state set."
	####################################################################3

	

	#New Position, load data
	def callback_pos_changed(self, widget, callback_data=None):
		global currentStage	#The current stage we are working on
		global enc_fol		#Following = 0, encoding =1
		global termination
		global termination_action
		global left_amnt
		global right_amnt

		l=0
		if(str.isdigit(self.pos_select.get_text())):
			currentStage=int(self.pos_select.get_text())
			if currentStage>37:
				self.pos_select.set_text("37")
				currentStage=37
			if currentStage<0:
				self.pos_select.set_text("0")
				currentStage=0

			f=open("settings.txt","r")
			line=f.readline()
			while l < currentStage:
				line=f.readline()
				l=l+1;
			
			linelist=string.split(line)
			f.close()
			
			#Set the values from saved data
			enc_fol=int(linelist[0])
			termination=int(linelist[1])
			termination_action=int(linelist[2])
			left_amnt=int(linelist[3])
			right_amnt=int(linelist[4])
			

			#Set the buttons and values correctly in the GUI
			if enc_fol == 0:
				self.enc_fol_button.set_label("Following")
			else:
				self.enc_fol_button.set_label("Encoders")
				

		






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
