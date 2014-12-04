from serial import Serial
import struct
import os
import time


vmsport = Serial('/dev/ttyACM0', 921600, timeout = 10)
vmsfile = open(time.strftime("%Y-%m-%d_%H-%M-%S") +  ".txt",'w');


vmsport.flushInput()
vmsport.flushOutput()

counter  = 0

while(1):
  data = vmsport.read(512)
  vmsfile.write(data)
  counter = counter + data.__len__()
  print "Counter= ", counter
vmsfile.close()
