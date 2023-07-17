# -*- coding: utf-8 -*-
import serial

_serial = serial.Serial(port = '/dev/ttyUSB1', 
                        baudrate = 115200,
                        bytesize = 8,
                        parity = serial.PARITY_NONE,
                        stopbits = 1,
                        timeout = 5)
        

at_cmd = "AT+A={:02x}{:02x}\r\n".format(1,2)
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))
print(at_cmd)


at_cmd = "AT+N=12\r\n"
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))



at_cmd = "AT+S=11\r\n"
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))

