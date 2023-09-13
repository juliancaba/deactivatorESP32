#ifndef __CONFIG_H__
#define __CONFIG_H__


typedef enum {
    AT_NONE, 
    AT_ACTIVATE,
    AT_STOP,
    AT_KEEP_ALIVE_ACK,
    AT_ERROR} tOperation;


#define TO_DECIMAL(A) (A >= '0' && A <= '9') ? (A-'0') : (A-'A'+10)
#define TO_UINT8(A,B) (((TO_DECIMAL(A))*16) + (TO_DECIMAL(B)))


const char* ssid = "IoTnet";
const char* password = "darksecret";


#define KEEP_ALIVE_TIME 2000
#define WDG_SERVER_TIME 500


#define UART_BAUDRATE 115200
#define BUF_RX_SIZE 256
#define TX_SIZE_KEEPALIVE 9 
#define TX_SIZE_RESPONSE_OK 12 
#define TX_SIZE_RESPONSE_ERR 13 
#define TX_SIZE_MAX 16
#define TX_MUTEX_WAIT 2000


#define PULSE_SEND 0x01
#define PULSE_END 0x00
#define PULSE_ERR 0x02
#define PULSE_NONE 0x00
#define PULSE_STOP 0x01
#define AT_ERR 0x03
#define PULSE_RESOLUTION   100 // 100ms
#define PULSE_MANUAL_MS    2000 // 2s
#define PULSE_PERIOD_TICKS 10 // Pulse_Duration * PERIOD_TICKS

#define NOTIFY true
#define SILENT false

#define ADDR_EEPROM_PULSE     0x0
#define ADDR_EEPROM_PAUSE     0x4
#define ADDR_EEPROM_NUMPULSES 0x8
#define ADDR_EEPROM_START     0xC

#define PWM_CHANNEL 1
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_16_BIT // Set duty resolution to 16 bits
#define LEDC_DUTY               (32767) // Set duty to 50%. ((2 ** 16) - 1) * 50%
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

const char* PATTERN_KEEP_ALIVE="AT+K=%02X\r\n";
const char* PATTERN_OK="AT+OK=%02X%02X\r\n";
const char* PATTERN_ERR="AT+ERR=%02X%02X\r\n";
const char* PATTERN_PREFIX_ACTIVATE="AT+A=";
const char* PATTERN_PREFIX_STOP="AT+S=";
const char* PATTERN_PREFIX_KA_ACK="AT+OK";
const char* PATTERN_CR="\r";
const char* PATTERN_LF="\n";

#endif