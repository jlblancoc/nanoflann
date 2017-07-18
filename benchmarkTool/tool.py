#!/usr/bin/python
import os
import tkFont
try:
    from Tkinter import *
except ImportError:
    from tkinter import *

def realTests():
	os.system('realTests/./realTests.py')
	return

def randomTests():
	os.system('randomTests/./randomTests.py')
	return

def createButton(TextMessage, row_index, col_index):
	helv36 = tkFont.Font(family='Helvetica', size=18, weight=tkFont.BOLD)
	Grid.rowconfigure(frame, row_index, weight=1)
	Grid.columnconfigure(frame, col_index, weight=1)
	if(TextMessage == 'Random Dataset'):
		btn = Button(frame, font=helv36, text =TextMessage, command = randomTests, foreground='black', background='white', activeforeground='white', activebackground='green') #create a button inside frame 
	if(TextMessage == 'Real Dataset'):
		btn = Button(frame, font=helv36, text =TextMessage, command = realTests, foreground='black', background='white', activeforeground='white', activebackground='green') #create a button inside frame 
	btn.grid(row=row_index, column=col_index, sticky=N+S+E+W)

if __name__ == '__main__':
	#Create & Configure root 
	root = Tk()
	root.wm_title("Benchmarking Tool")
	height = 400
	width  = 250
	root.geometry('{}x{}'.format(height, width))
	root.minsize(height, width)
	root.maxsize(height*2, width*2)
	Grid.rowconfigure(root, 0, weight=1)
	Grid.columnconfigure(root, 0, weight=1)

	frame=Frame(root)
	frame.grid(row=0, column=0, sticky=N+S+E+W)
		
	createButton("Real Dataset", 0, 0)
	createButton("Random Dataset", 1, 0)

	root.mainloop()