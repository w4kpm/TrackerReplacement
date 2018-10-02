#! /usr/bin/env python

import serial
import time
def encode_pos(b):
    return b + 32

def buildText(row,col,text):
    buildstr = "@%c%c%s\r\n"%(encode_pos(row),encode_pos(col),text)
    return bytearray(buildstr,encoding='ascii')

def buildButton(button,state):
    buildstr = "@%c%c%c\r\n"%(encode_pos(4),encode_pos(button),state)
    return bytearray(buildstr,encoding='ascii')
s = serial.Serial('/dev/ttyUSB0',115200,timeout=2)

for y in range(4):
    for x in range(4):
        s.write(buildButton(x,'0'))
        s.write(buildText(x,0,'            '))
    s.flush()
    s.write(buildButton(y,1))
    s.write(buildText(y,0,"Light #%d "%y))
    s.flush()
    time.sleep(1)
   
for x in range(4):
    s.write(buildButton(x,'0'))
    s.write(buildText(x,0,'            '))
s.flush()
