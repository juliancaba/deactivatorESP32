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


#define KEEP_ALIVE_TIME 2000


#define UART_BAUDRATE 115200
#define BUF_RX_SIZE 256
#define TX_SIZE_KEEPALIVE 9*sizeof(char) 
#define TX_SIZE_RESPONSE_OK 12*sizeof(char) 
#define TX_SIZE_RESPONSE_ERR 13*sizeof(char) 
#define TX_SIZE_MAX 16
#define TX_MUTEX_WAIT 2000


#define PULSE_SEND 0x01
#define PULSE_END 0x00
#define PULSE_ERR 0x02
#define PULSE_NONE 0x00
#define PULSE_STOP 0x01
#define AT_ERR 0x03
#define PULSE_RESOLUTION 100 // 100ms

const char* PATTERN_KEEP_ALIVE="AT+K=%02X\r\n";
const char* PATTERN_OK="AT+OK=%02X%02X\r\n";
const char* PATTERN_ERR="AT+ERR=%02X%02X\r\n";
const char* PATTERN_PREFIX_ACTIVATE="AT+A=";
const char* PATTERN_PREFIX_STOP="AT+S=";
const char* PATTERN_PREFIX_KA_ACK="AT+OK";
const char* PATTERN_CR="\r";
const char* PATTERN_LF="\n";

#endif