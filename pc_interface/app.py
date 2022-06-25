# Código adptado de https://www.instructables.com/UART-Controller-With-Tkinter-and-Python-GUI/

import tkinter
from tkinter import *
import tkinter.ttk as ttk

import threading
import os
import time
import serial

serial_data = ''
filter_data = ''
update_period = 5
serial_object = None
gui = Tk()
gui.title("UART Interface")

def connect():
    """The function initiates the Connection to the UART device with the Port and Buad fed through the Entry
    boxes in the application.
    The radio button selects the platform, as the serial object has different key phrases 
    for Linux and Windows. Some Exceptions have been made to prevent the app from crashing,
    such as blank entry fields and value errors, this is due to the state-less-ness of the 
    UART device, the device sends data at regular intervals irrespective of the master's state.
    The other Parts are self explanatory.
    """

    version_ = button_var.get()
    global serial_object
    port = port_entry.get()
    baud = baud_entry.get()
    

    try:
        if version_ == 2:
            try:
                serial_object = serial.Serial('/dev/tty' + str(port), int(baud))
            
            except:
                print("Cant Open Specified Port")
        elif version_ == 1:
            serial_object = serial.Serial('COM' + str(port), int(baud))

    except ValueError:
        print("Enter Baud and Port")
        return

    t1 = threading.Thread(target = get_data)
    t1.daemon = True
    t1.start()
    
def get_data():
    """This function serves the purpose of collecting data from the serial object and storing 
    the filtered data into a global variable.
    The function has been put into a thread since the serial event is a blocking function.
    """
    global serial_object
    global filter_data

    while(1):   
        #try:
        filter_data = str(serial_object.readline())
        
        print(filter_data)
        
        #except TypeError:
        #    print()
        #    pass
    
def update_gui():
    """" This function is an update function which is also threaded. The function assimilates the data
    and applies it to it corresponding progress bar. The text box is also updated every couple of seconds.
    A simple auto refresh function .after() could have been used, this has been avoid purposely due to 
    various performance issues.
    """
    global filter_data
    global update_period

    text.place(x = 15, y = 10)
    
    while(1):
        if filter_data:
            text.insert(END, filter_data)
            text.insert(END,"\n")

def send():
    """This function is for sending data from the computer to the host controller.
    
        The value entered in the the entry box is pushed to the UART. The data can be of any format, since
        the data is always converted into ASCII, the receiving device has to convert the data into the required f
        format.
    """
    command = 'R' if rw_var.get() == 1 else 'W'
    serial_object.write(command.encode())

    addr = address_entry.get()

    data = ''

    if(command == 'W'):
        send_data = data_entry.get()

        with open(send_data, 'r') as f:
            data = f.read()
    
    serial_object.write((addr+data).encode())

def disconnect():
    """ 
    This function is for disconnecting and quitting the application.
    Sometimes the application throws a couple of errors while it is being shut down, the fix isn't out yet
    but will be pushed to the repo once done.
    simple GUI.quit() calls.
    """
    try:
        serial_object.close() 
    
    except AttributeError:
        print("Closed without Using it -_-")

    gui.quit()

if __name__ == "__main__":

    """
    The main loop consists of all the GUI objects and its placement.
    The Main loop handles all the widget placements.
    """
    #frames
    frame_1 = Frame(height = 285, width = 480, bd = 3, relief = 'groove').place(x = 7, y = 5)
    frame_2 = Frame(height = 150, width = 480, bd = 3, relief = 'groove').place(x = 7, y = 300)
    text = Text(width = 57, height = 5)
    
    #threads
    t2 = threading.Thread(target = update_gui)
    t2.daemon = True
    t2.start()
    
    #Label

    baud   = Label(text = "Baud").place(x = 100, y = 348)
    port   = Label(text = "Port").place(x = 200, y = 348)
    address = Label(text = "Adress").place(x = 10, y = 100)
    datafile = Label(text = "Data File").place(x = 100, y = 100)

    #Entry
    address_entry = Entry()
    address_entry.place(x = 10, y = 117)
    data_entry = Entry()
    data_entry.place(x = 100, y = 117)
    
    baud_entry = Entry(width = 7)
    baud_entry.place(x = 100, y = 365)
    
    port_entry = Entry(width = 7)
    port_entry.place(x = 200, y = 365)

    rw_var = IntVar()
    r_1 = Radiobutton(text = "Read", variable = rw_var, value = 1).place(x = 300, y = 117)
    w_1 = Radiobutton(text = "Write", variable = rw_var, value = 2).place(x = 400, y = 117)

    #radio button
    button_var = IntVar()
    radio_1 = Radiobutton(text = "Windows", variable = button_var, value = 1).place(x = 10, y = 315)
    radio_2 = Radiobutton(text = "Linux", variable = button_var, value = 2).place(x = 110, y = 315)

    #button
    button1 = Button(text = "Send", command = send, width = 6).place(x = 15, y = 250)
    connect = Button(text = "Connect", command = connect).place(x = 15, y = 360)
    disconnect = Button(text = "Disconnect", command = disconnect).place(x =300, y = 360)
   
    #mainloop
    gui.geometry('500x500')
    gui.mainloop()