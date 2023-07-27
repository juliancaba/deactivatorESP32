# -*- coding: utf-8 -*-
import serial
import os



class DeactivatorCli:
    def __init__(self, device, bauds):
        self._pqseq = 0
        self._serial = serial.Serial(port = device, 
                                      baudrate = bauds,
                                      bytesize = 8,
                                      parity = serial.PARITY_NONE,
                                      stopbits = 1,
                                      timeout = 5)

    def __del__(self):
        self._serial.close()
        
            
    
    def usage(self):
        os.system('clear')
        print("Deactivator Client (press q to exit)")
        print("  u: Send unknown AT command")
        print("  p: Send pulse configuration")
        print("  s: Send stop pulse")
        print("  o: Send ok AT command")
        print("  h: Print this message")

    def _serialWR(self, at_cmd):
        self._serial.flushInput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        self._pqseq += 1
        

        
    def _stdRD(self, msg, a, b):
        while(1):
            try:
                _ch = input('{}: '.format(msg))
                _ch_int = int(_ch)
                if _ch_int >= a and _ch_int <= b:
                    return _ch_int
                else:
                    print("[ERROR] Please enter a valid option")        
            except:
                print("[ERROR] Please enter a valid option")
                
    
    def options(self):
        self.usage()
        op = input('Enter an option: ')
        if (op == 'q'):
            print("[INFO] Bye bye!")
            self.__del__()
            return False
        elif (op == 'u'):
            at_cmd="AT+N=30\r\n"
            self._serialWR(at_cmd)
        elif (op == 'p'):
            pulse_ms = self._stdRD("Pulse in ms (100ms to 25500ms)",100,25500)
            at_cmd="AT+A={:02X}{:02X}\r\n".format(int(pulse_ms/100),self._pqseq)
            self._serialWR(at_cmd)
        elif (op == 's'):
            at_cmd="AT+S={:02X}\r\n".format(self._pqseq)
            self._serialWR(at_cmd)
        elif (op == 'o'):
            at_cmd = "AT+OK={:02X}{:02X}\r\n".format(self._pqseq+1, self._pqseq)
            self._serialWR(at_cmd)            
            
        return True
