#! /usr/bin/env python
import os
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




arrAzimuth = 150
lat = 37.0877
long = -80.502



def nowZenith():
    return ast.solar_zenith(datetime.datetime.utcnow(),lat,long)

def getElev():
    return math.radians(ast.solar_elevation(datetime.datetime.utcnow(),lat,long))
def getAzimuth():
    return ast.solar_azimuth(datetime.datetime.utcnow(),lat,long)
def getZenith():
    return math.radians(nowZenith())

def optimalTilt():
    if getZenith() > 90.0:
        return 0
    tilt = math.degrees(math.atan(math.tan(getZenith())*math.sin(math.radians(getAzimuth()-arrAzimuth))))
    if tilt > 44.0:
        return -44.0
    if tilt < -44.0:
        return 44.0
    return -1.0 * tilt

    


def readGoodWind():
    valid = False
    while not valid:
        currentWind,valid = readmodbus(60,2,'int','modbusrtu','/home/sf/nports/annex','0.0.0.0',9600,.2)
        time.sleep(1)
        print("readResultWind: ",currentWind,valid)
    return currentWind/10.0

def readGoodSnow():
    valid = False
    while not valid:
        currentsnow,valid = readmodbus(60,8,'int','modbusrtu','/home/sf/nports/annex','0.0.0.0',9600,.2)
        time.sleep(1)
        print("readResultSnow: ",currentsnow,valid)

    return currentsnow



maxThrow = 3.000 # meters = 10 ft
panelWidth = 3.000
def safeSin(degValue):
    return (max(math.sin(math.radians(degValue)),.000000001))
def maxHeight():
    return ((maxThrow/safeSin(abs(getAzimuth()-arrAzimuth)))/safeSin(nowZenith()))*math.cos(math.radians(nowZenith()))

def maxRotation():
    if maxHeight() > 0.5*panelWidth:
        return 90.0
    return math.degrees(math.asin((maxHeight()-0.24)/(0.5*panelWidth)))

def determineMaxRotation(optTilt):
    if optTilt==0:
        return 0.0
    if optTilt > 0:
        return min(optTilt,max(0,maxRotation()))
    return max(optTilt,-1.0*max(0,maxRotation()))

def change_tracker_setpoint(modbusid,serialport,setpoint):
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
                instrument.write_register(3,pld,unit=modbusid,timeout=.1)
                instrument.close()
            except Exception as e:
                print(e)
                validread = False
            rl.unlock(lock)
            rl.unlock(lock2)
        else:
            print('LockFail')



#def moveOptimal(desiredAngle,tracker,serialPort):
#    print desiredAngle
#    setTrackerSpecial(0,serialPort,0,9600,.2,buildManualRequest)
#    currentAngle = readGoodAngle(tracker,serialPort)
#    print currentAngle
#    if currentAngle > desiredAngle:
#        print "moveWest"
#        moveOp = buildWestRequest
#        moveTest = operator.gt
#    else:
#        print "moveEast"
#        moveOp = buildEastRequest
#        moveTest = operator.lt
#    valid = False
#    
#    output,valid = setTrackerSpecial(0,serialPort,0,9600,0.2,moveOp)
#    while moveTest(currentAngle,desiredAngle):
#        currentAngle = readGoodAngle(tracker,serialPort)
#        time.sleep(1)
#    setTrackerSpecial(0,serialPort,0,9600,0.2,buildStopRequest)
#    setTrackerSpecial(0,serialPort,0,9600,0.2,buildStopRequest)
#    setTrackerSpecial(0,serialPort,0,9600,0.2,buildStopRequest)
#




# NB - Windspeeds are in meters/second - these approximate to 30 and 20 mph
maxWindSpeed = 25
resumeWindSpeed = 10
sustainTime = 30*60  # sustain time in seconds

windwarning = rd.get('TrackerWindWarning')
snowwarning = rd.get('TrackerSnowWarning')



currentWind = readGoodWind()
currentSnow = readGoodSnow()
if currentWind > maxWindSpeed:
    windwarning = True
    rd.set('TrackerWindWarning',True)
    rd.expire('TrackerWindWarning',sustainTime)

if windwarning and (currentWind > resumeWindSpeed):
    windwarning = True
    rd.set('TrackerWindWarning',True)
    rd.expire('TrackerWindWarning',sustainTime)

if os.path.exists('snow.alert'):
    print('previous snow alert')
    currentSnow =True

if currentSnow:
    f = open('snow.alert','a')
    f.close()
    rd.set('TrackerSnowWarning',True)
    rd.expire('TrackerSnowWarning',sustainTime)
    
    print("hey - we've got snow!")
    # I want to do some other stuff here



print (ast.solar_azimuth(datetime.datetime.utcnow(),lat,long))

def changeAngles(angle):
    port = '/dev/ttyM0'
    for id in [30,31,32,33]:
        print(math.floor(angle*10.0))
        change_tracker_setpoint(id,port,int(math.floor(angle*10.0)))
        print(readmodbus(id,1,'sint','modbusrtu',port,'0.0.0.0',9600,.2))
    port = '/dev/ttyM1'
    for id in [34,35,36,37,38,39,40,41,42]:
        change_tracker_setpoint(id,port,int(math.floor(angle*10.0)))
        print(readmodbus(id,1,'sint','modbusrtu',port,'0.0.0.0',9600,.2))


print currentWind
if windwarning:
    print ('wind alert - table top')
    changeAngles(0)
    exit()
if currentSnow:
    print('Snow alert - go to 45Deg')
    changeAngles(-42.0)
    exit()
if isDark():
    print "It is Dark!"
    # safe mode was doing something weird - just sent to 0.0 and see how it goes for now.
    changeAngles(30.0)
    exit()

#----------------------------------------------------

optimalAngle = determineMaxRotation(optimalTilt())
print optimalAngle
print "============"
if optimalAngle > 39:
    optimalAngle = 39
if optimalAngle < -42:
    optimalAngle = -42

print optimalAngle
changeAngles(optimalAngle)
