#!/usr/bin/python
import os
try:
    import tkinter.font as tkFont
    import tkinter as tk
    from tkinter import Tk
    from tkinter.filedialog import askopenfilename
except ImportError:
    import tkFont
    import Tkinter as tk
    from Tkinter import Tk
    from tkFileDialog import askopenfilename

filename1 = ''
filename2 = ''

def realTests(window, nanoflannFlag, flannFlag, fastannFlag, libkdtreeFlag):
    libState  = [nanoflannFlag.get(), flannFlag.get(), fastannFlag.get(), libkdtreeFlag.get()]
    os.system('realTests/./realTests.py ' + filename1 + ' ' + filename2 + ' ' + str(libState[0]) + ' ' + str(libState[1]) + ' ' + str(libState[2]) + ' ' + str(libState[3]))
    window.destroy()
    return

def selectFile(idx, btn):
    global filename1
    global filename2
    dir_path = os.path.dirname(os.path.realpath(__file__)) + '/realTests/dat_avz'
    if(idx == 0):
        filename1 = askopenfilename(initialdir = dir_path)
        btn.config(text = filename1)
    else:
        filename2 = askopenfilename(initialdir = dir_path)
        btn.config(text = filename2)

def DarkenLabel(C):
    if(C.cget('bg') == "green"):
        C.config(bg = "white")
    else:
        C.config(bg = "green")

def getParamsReal():
    
    window = tk.Toplevel(root)
    window.wm_title("Select Paramters")
    window.resizable(width=False, height=False)

    # select data file 1
    btn1 = tk.Button(window, text = 'Select first file', command = lambda: selectFile(0, btn1), width = 80, height = 3, pady = 5)
    btn1.grid(row=0, column=0)

    # select data file 2
    btn2 = tk.Button(window, text = 'Select second file', command = lambda: selectFile(1, btn2), width = 80, height = 3, pady = 5)
    btn2.grid(row=1, column=0)

    nanoflannFlag = tk.IntVar()
    flannFlag = tk.IntVar()
    fastannFlag = tk.IntVar()
    libkdtreeFlag = tk.IntVar()
    
    # 4 checkbuttons
    C1 = tk.Checkbutton(window, text = "nanoflann", variable = nanoflannFlag, onvalue = 1, command = lambda: DarkenLabel(C1), background='white', offvalue = 0, height=5, width = 20)
    C1.grid(row=0, column=1)
    C2 = tk.Checkbutton(window, text = "flann", variable = flannFlag, onvalue = 1, command = lambda: DarkenLabel(C2), background='white', offvalue = 0, height=5, width = 20)
    C2.grid(row=1, column=1)
    C3 = tk.Checkbutton(window, text = "fastann", variable = fastannFlag, onvalue = 1, command = lambda: DarkenLabel(C3), background='white', offvalue = 0, height=5, width = 20)
    C3.grid(row=2, column=1)
    C4 = tk.Checkbutton(window, text = "libkdtree", variable = libkdtreeFlag, onvalue = 1, command = lambda: DarkenLabel(C4), background='white', offvalue = 0, height=5, width = 20)
    C4.grid(row=3, column=1)

    # submit button
    btn3 = tk.Button(window, text = 'submit', command = lambda: realTests(window, nanoflannFlag, flannFlag, fastannFlag, libkdtreeFlag))
    btn3.grid(row=3, column=0, pady = 10)

    return

def randomTests(window, numPoints, nanoflannFlag, flannFlag, fastannFlag, libkdtreeFlag):
    libState  = [nanoflannFlag.get(), flannFlag.get(), fastannFlag.get(), libkdtreeFlag.get()]
    numPoints = numPoints.get()
    # pass parameters as command line arguments
    os.system('randomTests/./randomTests.py ' + str(numPoints) + ' ' + str(libState[0]) + ' ' + str(libState[1]) + ' ' + str(libState[2]) + ' ' + str(libState[3]))
    # close window
    window.destroy()
    return

def getParams():
    window = tk.Toplevel(root)
    window.wm_title("Select Parameters")
    window.resizable(width=False, height=False)
    
    # Label for Entry box
    L1 = tk.Label(window, text="Max Points")
    L1.grid(row=0, column=0)
    
    # Entry box
    numPoints =  tk.StringVar(window, value='10000')
    E1 = tk.Entry(window, textvariable = numPoints, bd = 5, width = 20)
    E1.grid(row=0, column=1)

    nanoflannFlag = tk.IntVar()
    flannFlag = tk.IntVar()
    fastannFlag = tk.IntVar()
    libkdtreeFlag = tk.IntVar()
    
    # 4 checkbuttons
    C1 = tk.Checkbutton(window, text = "nanoflann", variable = nanoflannFlag, onvalue = 1, command = lambda: DarkenLabel(C1), background='white', offvalue = 0, height=5, width = 20)
    C1.grid(row=1, column=1)
    C2 = tk.Checkbutton(window, text = "flann", variable = flannFlag, onvalue = 1, command = lambda: DarkenLabel(C2), background='white', offvalue = 0, height=5, width = 20)
    C2.grid(row=2, column=1)
    C3 = tk.Checkbutton(window, text = "fastann", variable = fastannFlag, onvalue = 1, command = lambda: DarkenLabel(C3), background='white', offvalue = 0, height=5, width = 20)
    C3.grid(row=3, column=1)
    C4 = tk.Checkbutton(window, text = "libkdtree", variable = libkdtreeFlag, onvalue = 1, command = lambda: DarkenLabel(C4), background='white', offvalue = 0, height=5, width = 20)
    C4.grid(row=4, column=1)

    # submit button
    btn = tk.Button(window, text = 'submit', command = lambda: randomTests(window, numPoints, nanoflannFlag, flannFlag, fastannFlag, libkdtreeFlag))
    btn.grid(row=5, column=1, pady = 10)
    
    return

def createButton(TextMessage, row_index, col_index):
    helv36 = tkFont.Font(family='Helvetica', size=18, weight=tkFont.BOLD)
    tk.Grid.rowconfigure(frame, row_index, weight=1)
    tk.Grid.columnconfigure(frame, col_index, weight=1)
    if(TextMessage == 'Real Test'):
        btn = tk.Button(frame, font=helv36, text =TextMessage, command = getParamsReal, foreground='black', background='white', activeforeground='white', activebackground='green') #create a button inside frame 
    if(TextMessage == 'Random Test'):
        btn = tk.Button(frame, font=helv36, text =TextMessage, command = getParams, foreground='black', background='white', activeforeground='white', activebackground='green') #create a button inside frame 
    btn.grid(row=row_index, column=col_index, sticky=tk.N+tk.S+tk.E+tk.W)
    return

if __name__ == '__main__':

    # configure window
    root = tk.Tk()
    root.wm_title("Nanoflann Benchmarking Tool")
    height = 600
    width  = 250
    root.geometry('{}x{}'.format(height, width))
    root.minsize(height, width)
    root.maxsize(height*2, width*2)
    tk.Grid.rowconfigure(root, 0, weight=1)
    tk.Grid.columnconfigure(root, 0, weight=1)

    frame=tk.Frame(root)
    frame.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)
    
    realLabel = tk.Label(root, text="Real Test: Compare Nanoflann performance with other kd-tree libraries on real dataset.\n Run Real Test and select two files of same object with different pose.\n", height = 5)
    randomLabel = tk.Label(root, text="Random Test: Compare Nanoflann performance with other kd-tree libraries on random dataset.\n Run Random Test and select parameters to run the test.\n", height = 5)
    # 2 buttons	
    createButton("Real Test", 0, 0)
    createButton("Random Test", 1, 0)
    realLabel.grid(row = 2, column = 0, sticky=tk.N+tk.S+tk.E+tk.W)
    randomLabel.grid(row = 3, column = 0, sticky=tk.N+tk.S+tk.E+tk.W)
    root.mainloop()