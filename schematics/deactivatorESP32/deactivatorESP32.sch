EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:ESP32-WROOM-32 U?
U 1 1 64BBA1E0
P 5350 3500
F 0 "U?" H 5350 5081 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 5350 4990 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 5350 2000 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 5050 3550 50  0001 C CNN
	1    5350 3500
	1    0    0    -1  
$EndComp
Text GLabel 6500 3300 2    50   Input ~ 0
UART_RX
Text GLabel 6500 3400 2    50   Output ~ 0
UART_TX
Wire Wire Line
	5950 3300 6500 3300
Wire Wire Line
	5950 3400 6500 3400
Wire Wire Line
	5950 4200 6500 4200
Wire Wire Line
	5950 3100 6500 3100
Text GLabel 6500 4200 2    50   Output ~ 0
LED_STATUS_R
Text GLabel 6500 3100 2    50   Output ~ 0
LED_STATUS_G
Wire Wire Line
	5950 3000 6500 3000
Wire Wire Line
	5950 2900 6500 2900
Text GLabel 6500 3000 2    50   Output ~ 0
LED_MODE_G
Text GLabel 6500 2900 2    50   Output ~ 0
LED_MODE_R
Wire Wire Line
	5950 2800 6500 2800
Text GLabel 6500 2800 2    50   Output ~ 0
PULSE
Wire Wire Line
	5950 3900 6500 3900
Wire Wire Line
	5950 3600 6500 3600
Text GLabel 6500 3600 2    50   Input ~ 0
BUTTON_MODE
Text GLabel 6500 3900 2    50   Input ~ 0
BUTTON_PULSE
Wire Wire Line
	5950 3700 6500 3700
Wire Wire Line
	5950 3800 6500 3800
Text GLabel 6500 3800 2    50   Output ~ 0
I2C_SCL
Text GLabel 6500 3700 2    50   Output ~ 0
I2C_SDA
Wire Wire Line
	5950 4600 6500 4600
Text GLabel 6500 4600 2    50   Input ~ 0
ADC1_CH1
$EndSCHEMATC
