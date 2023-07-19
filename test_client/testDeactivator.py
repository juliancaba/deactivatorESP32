# -*- coding: utf-8 -*-
import serial
import time

_serial = serial.Serial(port = '/dev/ttyUSB1', 
                        baudrate = 115200,
                        bytesize = 8,
                        parity = serial.PARITY_NONE,
                        stopbits = 1,
                        timeout = 5)



at_cmd = "AT+A={:02x}{:02x}\r\n".format(1,1)
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))

time.sleep(1)

at_cmd = "AT+A=1E{:02x}\r\n".format(2)
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))
print(at_cmd)

time.sleep(1)

at_cmd = "AT+A={:02x}{:02x}\r\n".format(6,3)
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))

time.sleep(2)

at_cmd = "AT+N=12\r\n"
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))

at_cmd = "AT+OK=0504\r\n"
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))

time.sleep(2)



at_cmd = "AT+S=05\r\n"
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))


at_cmd = "AT+A=1E{:02x}\r\n".format(6)
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))

time.sleep(1)

at_cmd = "AT+S=07\r\n"
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))


time.sleep(1)

at_cmd = "AT+A=2E{:02x}\r\n".format(8)
_serial.flushOutput()
_serial.write(bytearray(at_cmd.encode('utf-8')))

