#!/usr/bin/env python

from common import * 
from astral import Astral
import math
import operator
ast = Astral()
def isDark():
    lat = 37.0889
    long = -80.5052
    now = datetime.datetime.utcnow()
    return ast.solar_elevation(now,lat,long) < -5.0



def generic_tracker_update(modbusid,serialport,setpoint,register):
    x = 0
    read = False
    validread = True

    while read == False:
        lock = rl.lock("seriallock"+serialport,3000)
        lock2= rl.lock("seriallock"+serialport+scriptname,3000)
    
        if lock:
            read = True
            try:
                instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=9600)
                builder = BinaryPayloadBuilder(byteorder=Endian.Big)
                builder.add_16bit_int(setpoint)
                payload = builder.build()            
                print ord(payload[0][0]),ord(payload[0][1])
                pld = ord(payload[0][1])|(ord(payload[0][0])<<8)    
                instrument.connect()
                instrument.write_register(register,pld,unit=modbusid,timeout=.1)
                instrument.close()
            except Exception as e:
                print(e)
                validread = False
            rl.unlock(lock)
            rl.unlock(lock2)
        else:
            print('LockFail')




def change_tracker_setpoint(modbusid,serialport,setpoint):
    generic_tracker_update(modbusid,serialport,setpoint,3)


def reset_tracker_error(modbusid,serialport):
    generic_tracker_update(modbusid,serialport,0,7)


def reset_tracker_maxamps(modbusid,serialport):
    generic_tracker_update(modbusid,serialport,0,8)

def set_tracker_mode(modbusid,serialport,mode):
    """ 0 for normal, 1 for manual"""
    generic_tracker_update(modbusid,serialport,mode,4)

def set_tracker_anglemode(modbusid,serialport,mode):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,mode,5)


def set_tracker_stopangle(modbusid,serialport,timeframe):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,timeframe,1003)
    generic_tracker_update(modbusid,serialport,0x1234,1234)

def set_tracker_hysterisis(modbusid,serialport,timeframe):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,timeframe,1002)
    generic_tracker_update(modbusid,serialport,0x1234,1234)


def set_tracker_stoptime(modbusid,serialport,timeframe):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,timeframe,1004)
    generic_tracker_update(modbusid,serialport,0x1234,1234)


def set_tracker_slowamps(modbusid,serialport,timeframe):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,timeframe,1001)
    generic_tracker_update(modbusid,serialport,0x1234,1234)



def set_tracker_anglemode_perm(modbusid,serialport,timeframe):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,timeframe,1005)
    generic_tracker_update(modbusid,serialport,0x1234,1234)




def set_tracker_anglediff(modbusid,serialport,timeframe):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,timeframe,1006)
    generic_tracker_update(modbusid,serialport,0x1234,1234)


def set_tracker_maxamps(modbusid,serialport,timeframe):
    """ 0 for both 1 for sensor 1 2 for sensor 2"""
    generic_tracker_update(modbusid,serialport,timeframe,1000)
    generic_tracker_update(modbusid,serialport,0x1234,1234)


idno = 32

port = '/dev/ttyM0'
#for idno in [31]:
#      set_tracker_anglemode_perm(idno,port,1)
      #reset_tracker_maxamps(idno,port)


#      set_tracker_maxamps(idno,port,95)
#set_tracker_anglediff(idno,port,70)
#reset_tracker_error(idno,port)
#for idno in [31]:
for idno in [32]:
    #set_tracker_hysterisis(idno,port,20)
    #change_tracker_setpoint(idno,port,200)
    #set_tracker_anglediff(idno,port,50)
    #  for x in range(10):


    readmodbus(idno,1000,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1001,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1002,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1003,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1004,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1005,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1006,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1007,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1008,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)
    readmodbus(idno,1009,'int','modbusrtu',port,'0.0.0.0',9600,.2),
    time.sleep(.5)

    readmodbus(idno,13,'int','modbusrtu',port,'0.0.0.0',9600,.2)
    time.sleep(.5)
    readmodbus(idno,1,'sint','modbusrtu',port,'0.0.0.0',9600,.2)
    time.sleep(.5)
    readmodbus(idno,3,'sint','modbusrtu',port,'0.0.0.0',9600,.2)
    time.sleep(.5)
    readmodbus(idno,5,'sint','modbusrtu',port,'0.0.0.0',9600,.2)
    time.sleep(.5)
    readmodbus(idno,6,'sint','modbusrtu',port,'0.0.0.0',9600,.2)
    time.sleep(.5)
    readmodbus(idno,8,'int','modbusrtu',port,'0.0.0.0',9600,.2)
    time.sleep(.5)
    readmodbus(idno,7,'int','modbusrtu',port,'0.0.0.0',9600,.2)
    time.sleep(1)

