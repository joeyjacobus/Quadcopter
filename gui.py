from Tkinter import *
from functools import partial
import serial
import sys
import os
import time


class GUI():
	def __init__(self, serial):
                self.serial = serial

		self.master = Tk()


                self.reset = Button(self.master, text="RESET", command=self.reset)
                self.reset.grid(row=0, column=5)

                self.divider1 = Label(self.master, text="Roll vals")
                self.divider1.grid(row=0, column=2)

		self.l = Label(self.master, text="Roll Pval")
                self.l.grid(row=1, column=1)
                self.sb = Spinbox(self.master, from_=0.0, to=10.0, format="%.1f", increment=0.1)
                self.sb.config(command=partial(self.callback, self.sb , "RP"))
                self.sb.grid(row=1, column=2)
                self.sb.delete(0, "end")
                self.sb.insert(0, 2.0)

		self.l2 = Label(self.master, text="Roll Ival")
                self.l2.grid(row=2, column=1)
                self.sb2 = Spinbox(self.master, from_=0.0, to=10.0, format="%.1f", increment=0.1)
                self.sb2.config(command=partial(self.callback, self.sb2 , "RI"))
                self.sb2.grid(row=2, column=2)
                self.sb2.delete(0, "end")
                self.sb2.insert(0, 2.0)


		self.l3 = Label(self.master, text="Roll Dval")
                self.l3.grid(row=3, column=1)
                self.sb3 = Spinbox(self.master, from_=0.0, to=10.0, format="%.1f", increment=0.1)
                self.sb3.config(command=partial(self.callback, self.sb3 , "RD"))
                self.sb3.grid(row=3, column=2)
                self.sb3.delete(0, "end")
                self.sb3.insert(0, 0.4)

		self.l10 = Label(self.master, text="Target Roll")
                self.l10.grid(row=4, column=1)
                self.sb10 = Spinbox(self.master, from_=-15.0, to=15.0)
                self.sb10.config(command=partial(self.callback, self.sb10 , "RT"))
                self.sb10.grid(row=4, column=2)
                self.sb10.delete(0, "end")
                self.sb10.insert(0, 0)

                self.divider = Label(self.master, text="Pitch vals")
                self.divider.grid(row=5, column=2)

		self.l4 = Label(self.master, text="Pitch Pval")
                self.l4.grid(row=6, column=1)
                self.sb4 = Spinbox(self.master, from_=0.0, to=10.0, format="%.1f", increment=0.1)
                self.sb4.config(command=partial(self.callback, self.sb4 , "PP"))
                self.sb4.grid(row=6, column=2)
                self.sb4.delete(0, "end")
                self.sb4.insert(0, 0.6)

		self.l5 = Label(self.master, text="Pitch Ival")
                self.l5.grid(row=7, column=1)
                self.sb5 = Spinbox(self.master, from_=0.0, to=10.0, format="%.1f", increment=0.1)
                self.sb5.config(command=partial(self.callback, self.sb5 , "PI"))
                self.sb5.grid(row=7, column=2)
                self.sb5.delete(0, "end")
                self.sb5.insert(0, 0.6)


		self.l6 = Label(self.master, text="Pitch Dval")
                self.l6.grid(row=8, column=1)
                self.sb6 = Spinbox(self.master, from_=0.0, to=10.0, format="%.1f", increment=0.1)
                self.sb6.config(command=partial(self.callback, self.sb6 , "PD"))
                self.sb6.grid(row=8, column=2)
                self.sb6.delete(0, "end")
                self.sb6.insert(0, 0.3)

		self.l11 = Label(self.master, text="Target Pitch")
                self.l11.grid(row=9, column=1)
                self.sb11 = Spinbox(self.master, from_=-15.0, to=15.0)
                self.sb11.config(command=partial(self.callback, self.sb11 , "PT"))
                self.sb11.grid(row=9, column=2)
                self.sb11.delete(0, "end")
                self.sb11.insert(0, 0)
                self.bpitch = Button(self.master, text="Send", command=partial(self.callback, self.sb11, "PT"))
                self.bpitch.grid(row=9, column=3)



		self.l7 = Label(self.master, text="Base Thrust")
                self.l7.grid(row=10, column=1)
                self.sb7 = Spinbox(self.master, from_=1050.0, to=2000.0, increment=1)
                self.sb7.config(command=partial(self.callback, self.sb7 , "BT"))
                self.sb7.grid(row=10, column=2)
                self.sb7.delete(0, "end")
                self.sb7.insert(0, 1100)
                self.b = Button(self.master, text="Send", command=partial(self.callback, self.sb7, "BT"))
                self.b.grid(row=10, column=3)




		mainloop()
        def reset(self):
                self.serial.write("reset\n")
                print "reset!"

        def callback(self, e, which=None):
                print e.get()
                val = e.get()
                try:
                        val = float(val)
                except ValueError:
                        print("Not a double")
                        return
                self.serial.write(which + "=" + str(val) + "\n")
                print which + "=" + str(val) + "\n"
                #print self.serial.readline()




def findUsbDev():
	results = []
	for l in os.listdir("/dev"):
		if l.startswith("cu.usbmodem"):
			results.append(os.path.join("/dev",l))
	return results

def main():
	usbNames = findUsbDev()
	if len(usbNames) > 1 or len(usbNames) == 0:
		print "More than one device or no device connected"
		print usbNames
		return
	usbName = usbNames[0]
	ser = serial.Serial(usbName, 115200)
	g = GUI(ser)

main()
