from pymodbus.client.sync import ModbusSerialClient 
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder



def readmodbus(modbusid,register,fieldtype,readtype,serialport):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=9600)
    instrument.connect()
    
    #print("Reading ModbusID %d register %d"%(modbusid,register)),
    x = None
    read = False
    validread = True
    while read == False:
        lock = True

        if lock:

            read = True
           # instrument.serial.reset_input_buffer()
            signed = False
            try:
                if fieldtype in ['int','sint']:
                    readlen = 1
                if fieldtype in ['long','slong','float']:
                    readlen = 2
                if fieldtype == 'string':
                    readlen = 10
                #print(".")
                if readtype == 3:
                    x = instrument.read_holding_registers(register,readlen,unit=modbusid,timeout=.5)
                if readtype ==4:
                    x = instrument.read_input_registers(register,readlen,unit=modbusid,timeout=.5)
                #print("..")
                instrument.close()
                #print(x)
                decoder = BinaryPayloadDecoder.fromRegisters(x.registers,byteorder=Endian.Big)
                if fieldtype == 'slong':
                    x = decoder.decode_32bit_int()
                if fieldtype == 'long':
                    x = decoder.decode_32bit_uint()
                if fieldtype == 'sint':
                    x = decoder.decode_16bit_int()
                if fieldtype == 'int':
                    x = decoder.decode_16bit_uint()
                if fieldtype == 'float':
                    x = decoder.decode_32bit_float()
                if fieldtype == 'string':
                    x = decoder.decode_string(10)

            except Exception as e:
                print(e)
                x=0
                validread = False
    #print(x,validread)
    #if fieldtype in ['sint','slong','int','long']:
    #    print("%X"%x)
    return x


def change_tracker_setpoint(modbusid,serialport,setpoint):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=9600)

    builder = BinaryPayloadBuilder(byteorder=Endian.Big)

    builder.add_16bit_int(setpoint)
    payload = builder.build()
    #print(payload)

    #print(payload[0][0])
    #print(payload[0][1])
    
    pld = payload[0][1]|(payload[0][0]<<8)
    
    instrument.connect()
    instrument.write_register(3,pld,unit=modbusid,timeout=.1)
    instrument.close()



def reset_tracker_error(modbusid,serialport,setpoint):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=9600)

    builder = BinaryPayloadBuilder(byteorder=Endian.Big)

    builder.add_16bit_int(setpoint)
    payload = builder.build()
    #print(payload)

    #print(payload[0][0])
    #print(payload[0][1])
    
    pld = payload[0][1]|(payload[0][0]<<8)
    
    instrument.connect()
    instrument.write_register(4,pld,unit=modbusid,timeout=.1)
    instrument.close()


def change_sensor_id(modbusid,serialport,id):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=9600)

    builder = BinaryPayloadBuilder(byteorder=Endian.Big)

    builder.add_16bit_int(id)
    payload = builder.build()
    #print(payload)

    #print(payload[0][0])
    #print(payload[0][1])
    
    pld = payload[0][1]|(payload[0][0]<<8)
    
    instrument.connect()
    instrument.write_register(7,pld,unit=modbusid,timeout=.1)
    instrument.close()


    

print(readmodbus(16,3,'sint',4,'/dev/ttyUSB1'))
reset_tracker_error(16,'/dev/ttyUSB1',0)
change_tracker_setpoint(16,'/dev/ttyUSB1',75)

#readmodbus(1,1,'sint',4,'/dev/ttyUSB0')
#change_sensor_id(1,'/dev/ttyUSB0',2)
#readmodbus(1,1,'sint',4,'/dev/ttyUSB0')
