import unittest
import serial

class DeactivatorESP32TestCase(unittest.TestCase):

    def setUp(self):        
       self. _serial = serial.Serial(port = '/dev/ttyUSB1', 
                                baudrate = 115200,
                                bytesize = 8,
                                parity = serial.PARITY_NONE,
                                stopbits = 1,
                                timeout = 5)
       

    def tearDown(self):
        del self._serial
        

    def test_pulse_100ms(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+A={:02x}{:02x}\r\n".format(1,1)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        init_msg = self._serial.read(12)
        end_msg = self._serial.read(12)
        keep_alive_end = self._serial.read(9)[5:7]
        
        self.assertEqual(init_msg, b'AT+OK=0101\r\n', "Wrong init msg")
        self.assertEqual(end_msg, b'AT+OK=0100\r\n', "Wrong end msg")
        self.assertEqual(int(keep_alive_init,16)+1, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

    def test_ongoing_pulse(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+A=0A{:02x}\r\n".format(2)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        init_msg = self._serial.read(12)
        at_cmd = "AT+A={:02x}{:02x}\r\n".format(1,3)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        err_msg = self._serial.read(13)        
        end_msg = self._serial.read(12)
        keep_alive_end = self._serial.read(9)[5:7]
        
        self.assertEqual(init_msg, b'AT+OK=0201\r\n', "Wrong init msg")
        self.assertEqual(err_msg, b'AT+ERR=0302\r\n', "Wrong err msg")
        self.assertEqual(end_msg, b'AT+OK=0200\r\n', "Wrong end msg")
        self.assertEqual(int(keep_alive_init,16)+2, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

        
    def test_pulse_2100ms(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+A=15{:02x}\r\n".format(4)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        init_msg = self._serial.read(12)
        keep_alive_end = self._serial.read(9)[5:7]
        end_msg = self._serial.read(12)
        #keep_alive_end = self._serial.read(9)[5:7]
        
        self.assertEqual(init_msg, b'AT+OK=0401\r\n', "Wrong init msg")
        self.assertEqual(end_msg, b'AT+OK=0400\r\n', "Wrong end msg")
        self.assertEqual(int(keep_alive_init,16)+1, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

        
    def test_at_not_found(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+N=15{:02x}\r\n".format(4)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        err_msg = self._serial.read(13)
        keep_alive_end = self._serial.read(9)[5:7]
        
        self.assertEqual(err_msg, b'AT+ERR=0403\r\n', "Wrong err msg")
        self.assertEqual(int(keep_alive_init,16)+1, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

        
    def test_ack_keep_alive(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+OK={:02x}{:02x}\r\n".format(int(keep_alive_init,16),int(keep_alive_init,16)-1)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        keep_alive_end = self._serial.read(9)[5:7]

        self.assertEqual(int(keep_alive_init,16)+1, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

        
    def test_stop_without_pulse(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+S=05\r\n"
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        ok_msg = self._serial.read(12)
        keep_alive_end = self._serial.read(9)[5:7]

        self.assertEqual(ok_msg, b'AT+OK=0500\r\n', "Wrong ok msg")
        self.assertEqual(int(keep_alive_init,16)+1, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

        
    def test_stop_ongoing_pulse(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+A=05{:02x}\r\n".format(6)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        init_msg = self._serial.read(12)
        
        at_cmd = "AT+S=06\r\n"
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        ok_msg = self._serial.read(12)
        
        keep_alive_end = self._serial.read(9)[5:7]

        
        self.assertEqual(init_msg, b'AT+OK=0601\r\n', "Wrong init msg")
        self.assertEqual(ok_msg, b'AT+OK=0601\r\n', "Wrong ok msg")
        self.assertEqual(int(keep_alive_init,16)+2, int(keep_alive_end,16), "Wrong Keep Alive after AT command")


    def test_long_pulse(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+A=20{:02x}\r\n".format(20)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        init_msg = self._serial.read(12)
        keep_alive_end = self._serial.read(9)[5:7]
                
        self.assertEqual(init_msg, b'AT+OK=1401\r\n', "Wrong init msg")
        self.assertEqual(int(keep_alive_init,16)+1, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

        
    def test_short_pulse(self):
        keep_alive_init = self._serial.read(9)[5:7]
        at_cmd = "AT+A=02{:02x}\r\n".format(21)
        self._serial.flushOutput()
        self._serial.write(bytearray(at_cmd.encode('utf-8')))
        init_msg = self._serial.read(12)
        keep_alive_end = self._serial.read(9)[5:7]
                
        self.assertEqual(init_msg, b'AT+OK=1501\r\n', "Wrong init msg")
        print(init_msg)
        print("KA1: {}  KA+1: {}".format(keep_alive_init, keep_alive_end))
        self.assertEqual(int(keep_alive_init,16)+1, int(keep_alive_end,16), "Wrong Keep Alive after AT command")

        

def suite():
    suite = unittest.TestSuite()
    suite.addTest(DeactivatorESP32TestCase('test_pulse_100ms'))
    suite.addTest(DeactivatorESP32TestCase('test_ongoing_pulse'))
    suite.addTest(DeactivatorESP32TestCase('test_pulse_2100ms'))
    suite.addTest(DeactivatorESP32TestCase('test_at_not_found'))
    suite.addTest(DeactivatorESP32TestCase('test_ack_keep_alive'))
    suite.addTest(DeactivatorESP32TestCase('test_stop_without_pulse'))
    suite.addTest(DeactivatorESP32TestCase('test_stop_ongoing_pulse'))
    return suite
        
        

if __name__ == '__main__':
    runner = unittest.TextTestRunner()
    runner.run(suite())


    
