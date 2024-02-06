#ifndef __PINOUT_H__
#define __PINOUT_H__


// GPIO
#define pinPulse         GPIO_NUM_14  // GPIO14 (17 - D7)
#define pinJUMPER_MODE   GPIO_NUM_35  // GPI35  (11 - A2)
#define pinLED_MODE_B    GPIO_NUM_2   // GPIO2  (22 - A0)
#define pinLED_MODE_G    GPIO_NUM_4   // GPIO4  (24 - A1)
#define pinLED_MODE_R    GPIO_NUM_25  // GPIO25 (14 - D3)
#define pinBUTTON_PULSE  GPIO_NUM_5   // GPIO05 (34 - D10)
#define pinLED_STATUS_B  GPIO_NUM_27  // GPIO27 (16 - D6)
#define pinLED_STATUS_G  GPIO_NUM_12  // GPIO12 (18 - D8)
#define pinLED_STATUS_R  GPIO_NUM_23  // GPIO23 (36 - D11)
#define pinBUTTON_REACT  GPIO_NUM_18  // GPIO18 (35 - D13)
#define pinLED_REACT     GPIO_NUM_19  // GPIO19 (38 - D12)


// Temperature DS18B20
#define pinTemperature         GPIO_NUM_33  // GPIO33 (13)
#define ADCchannelTemperature  ADC1_CHANNEL_5


// UART 2
#define UART_PORT 2
#define pinUART_RX GPIO_NUM_16 // GPIO16 (25 - D5)
#define pinUART_TX GPIO_NUM_17 // GPIO17 (27 - D4)

// I2C (Reserved)
#define pinI2C_SCL GPIO_NUM_22 // GPIO22 (39 - D15)
#define pinI2C_SDA GPIO_NUM_21 // GPIO21 (42 - D14)

// ADC1 (Reserved) - ADC2 bussy for WiFi
#define pinADC1_CH6 GPIO_NUM_34 // GPI35 (10 - A3)

// DAC (Reserved)
#define pinDAC2 GPIO_NUM_26 // GPIO26 (15 - D2)


#endif