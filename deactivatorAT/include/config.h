#ifndef __CONFIG_H__
#define __CONFIG_H__

#define _NONE 0
#define _ACTIVATE 1
#define _STATUS 2


#define KEEP_ALIVE_TIME 2000

#define UART_BAUDRATE 115200
#define BUF_RX_SIZE 256
#define BUF_TX_SIZE_KEEPALIVE 9 // pdseq 2 caracteres (legible)

#define MUTEXTX_WAIT 2000

const char* PATTERN_KEEP_ALIVE="AT+K=%02X\r\n";
const char* PATTERN_PREFIX_ACTIVATE="AT+A=";
const char* PATTERN_PREFIX_STATUS="AT+S=";
const char* PATTERN_CRLF="\r\n";

#endif