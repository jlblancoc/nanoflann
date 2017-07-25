#!/usr/bin/python
import os
import tkFont
import Tkinter as tk

def realTests():
	os.system('realTests/./realTests.py')
	return

def randomTests(window, numPoints, CheckVar1, CheckVar2, CheckVar3, CheckVar4):
	libState = [CheckVar1.get(), CheckVar2.get(), CheckVar3.get(), CheckVar4.get()]
	os.system('randomTests/./randomTests.py ' + str(numPoints.get()) + ' ' + str(libState[0]) + ' ' + str(libState[1]) + ' ' + str(libState[2]) + ' ' + str(libState[3]))
	window.destroy()
	return

def DarkenLabel(C):
    if(C.cget('bg') == "green"):
    	C.config(bg = "white")
    else:
    	C.config(bg = "green")

def getParams():
	window = tk.Toplevel(root)
	window.wm_title("Select Parameters")
	window.resizable(width=False, height=False)
	
	L1 = tk.Label(window, text="Max Points")
	L1.grid(row=0, column=0)
	
	numPoints =  tk.StringVar(window, value='10000')
	E1 = tk.Entry(window, textvariable = numPoints, bd = 5, width = 20)
	E1.grid(row=0, column=1)

	CheckVar1 = tk.IntVar()
	CheckVar2 = tk.IntVar()
	CheckVar3 = tk.IntVar()
	CheckVar4 = tk.IntVar()
	C1 = tk.Checkbutton(window, text = "nanoflann", variable = CheckVar1, onvalue = 1, command = lambda: DarkenLabel(C1), background='white', offvalue = 0, height=5, width = 20)
	C1.grid(row=1, column=1)
	C2 = tk.Checkbutton(window, text = "flann", variable = CheckVar2, onvalue = 1, command = lambda: DarkenLabel(C2), background='white', offvalue = 0, height=5, width = 20)
	C2.grid(row=2, column=1)
	C3 = tk.Checkbutton(window, text = "fastann", variable = CheckVar3, onvalue = 1, command = lambda: DarkenLabel(C3), background='white', offvalue = 0, height=5, width = 20)
	C3.grid(row=3, column=1)
	C4 = tk.Checkbutton(window, text = "libkdtree", variable = CheckVar4, onvalue = 1, command = lambda: DarkenLabel(C4), background='white', offvalue = 0, height=5, width = 20)
	C4.grid(row=4, column=1)

	btn = tk.Button(window, text = 'submit', command = lambda: randomTests(window, numPoints, CheckVar1, CheckVar2, CheckVar3, CheckVar4))
	btn.grid(row=5, column=1, pady = 10)
	return

def createButton(TextMessage, row_index, col_index):
	helv36 = tkFont.Font(family='Helvetica', size=18, weight=tkFont.BOLD)
	tk.Grid.rowconfigure(frame, row_index, weight=1)
	tk.Grid.columnconfigure(frame, col_index, weight=1)
	if(TextMessage == 'Real Test'):
		btn = tk.Button(frame, font=helv36, text =TextMessage, command = realTests, foreground='black', background='white', activeforeground='white', activebackground='green') #create a button inside frame 
	if(TextMessage == 'Random Test'):
		btn = tk.Button(frame, font=helv36, text =TextMessage, command = getParams, foreground='black', background='white', activeforeground='white', activebackground='green') #create a button inside frame 
	btn.grid(row=row_index, column=col_index, sticky=tk.N+tk.S+tk.E+tk.W)

if __name__ == '__main__':
	#Create & Configure root 
	col = 'white'
	root = tk.Tk()
	root.wm_title("Benchmarking Tool")
	height = 400
	width  = 250
	root.geometry('{}x{}'.format(height, width))
	root.minsize(height, width)
	root.maxsize(height*2, width*2)
	tk.Grid.rowconfigure(root, 0, weight=1)
	tk.Grid.columnconfigure(root, 0, weight=1)

	frame=tk.Frame(root)
	frame.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)
		
	createButton("Real Test", 0, 0)
	createButton("Random Test", 1, 0)

	root.mainloop()