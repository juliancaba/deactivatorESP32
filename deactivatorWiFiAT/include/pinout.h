#ifndef __PINOUT_H__
#define __PINOUT_H__


// GPIO
#define pinPulse         GPIO_NUM_5  // GPIO5  (34 - D10)
#define pinBUTTON_MODE   GPIO_NUM_19 // GPIO19 (19 - D11)
#define pinBUTTON_PULSE  GPIO_NUM_23 // GPIO23 (36 - D12)
#define pinLED_STATUS_R  GPIO_NUM_27 // GPIO27 (16 - D6)
#define pinLED_STATUS_G  GPIO_NUM_14 // GPIO14 (17 - D7)
#define pinLED_MODE_G    GPIO_NUM_13 // GPIO13 (20 - D9)
#define pinLED_MODE_R    GPIO_NUM_12 // GPIO12 (18 - D8)

// UART 2
#define UART_PORT 2
#define pinUART_RX GPIO_NUM_16 // GPIO16 (25 - D5)
#define pinUART_TX GPIO_NUM_17 // GPIO17 (27 - D4)

// I2C (Reserved)
#define pinI2C_SCL GPIO_NUM_22 // GPIO22 (39 - D15)
#define pinI2C_SDA GPIO_NUM_21 // GPIO21 (42 - D14)

// ADC1 (Reserved) - ADC2 bussy for WiFi
#define pinADC1_CH1 GPIO_NUM_35 // GPI35 (11 - A2)


#endif