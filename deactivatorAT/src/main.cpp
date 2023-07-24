#include <Arduino.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <EEPROM.h>

#include "driver/uart.h"
#include "driver/ledc.h"

#include "pinout.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


QueueHandle_t sSemphrMode = NULL;
QueueHandle_t sSemphrPulse = NULL;
QueueHandle_t xMutexTX = NULL;
QueueHandle_t xMutexPulse = NULL;

TaskHandle_t tHandler_KeepAlive = NULL;
TaskHandle_t tHandler_AT = NULL;
TaskHandle_t tHandler_Pulse = NULL;
TaskHandle_t tHandler_ManualPulse = NULL;

//static volatile bool pulsePendingFlag;
static volatile bool modeUSB = false;

static uint8_t* dataRX = NULL;
static char *msgPULSE = NULL;
static char *dataKA = NULL;
static char *msgTX = NULL;

static volatile uint8_t pdseq;
static uint8_t pqseq;
static uint8_t pdseq_ack;
static uint8_t pqseq_ack;
static uint16_t pulse_size_ms;
static uint8_t pulse_pqseq;
static enum {stIDLE, stPLUSE_SIZE, stSEQ, stEQUAL, stPDSEQ_ACK, stPQSEQ_ACK, stCR, stLF} stateAT;
static tOperation operation = AT_NONE;

void app_main(void);
void stopPulse(bool notifyStop);
void vTaskPulse(void* pvParam);

void IRAM_ATTR mode_handleInterrupt(void* arg)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( sSemphrMode, &xHigherPriorityTaskWoken );
}

void IRAM_ATTR pulse_handleInterrupt(void* arg)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( sSemphrPulse, &xHigherPriorityTaskWoken );
}

/*
void IRAM_ATTR ledcEvent()
{

    printf("ENTRO LEDC\r\n");
  ledcWrite(PWM_CHANNEL, 0);
}
*/

void transmitTX(const char *dataTX, uint8_t size)
{
  xSemaphoreTake(xMutexTX, TX_MUTEX_WAIT/portTICK_PERIOD_MS);
  uart_write_bytes(UART_PORT, dataTX, size);
  xSemaphoreGive( xMutexTX );
}


static void vTaskManualPulse( void *pvParameters )
{
  for( ;; ){
    xSemaphoreTake( sSemphrPulse, portMAX_DELAY );
    printf("[INFO] Pulso Manual\r\n");
  }
  vTaskDelete(NULL);
}


static void vTaskMode( void *pvParameters )
{
  for( ;; ){
    xSemaphoreTake( sSemphrMode, portMAX_DELAY );
    if (!modeUSB){
      modeUSB = true;
      gpio_set_level(pinLED_STATUS_G, HIGH);
      gpio_set_level(pinLED_STATUS_R, LOW);
      gpio_set_level(pinLED_MODE_G, HIGH);
      gpio_set_level(pinLED_MODE_R, LOW);
      if(tHandler_KeepAlive != NULL)
        vTaskResume(tHandler_KeepAlive);
      if(tHandler_AT != NULL)
        vTaskResume(tHandler_AT);
      if(tHandler_ManualPulse != NULL)
        vTaskResume(tHandler_ManualPulse);
      printf("[INFO] USB Mode (on)\r\n");
    }
    else{
      modeUSB = false;
      gpio_set_level(pinLED_STATUS_G, LOW);
      gpio_set_level(pinLED_STATUS_R, HIGH);
      gpio_set_level(pinLED_MODE_G, LOW);
      gpio_set_level(pinLED_MODE_R, LOW);

      vTaskSuspend(tHandler_KeepAlive);
      vTaskSuspend(tHandler_AT);
      vTaskSuspend(tHandler_ManualPulse);
      if(tHandler_Pulse != NULL)
        stopPulse(SILENT);

      printf("[INFO] USB Mode (off)\r\n");
    }
  }
  vTaskDelete(NULL);
}

/*
void vTaskPulse(void* pvParam)
{
  for(;;) {
    //pulsePendingFlag = true;
    msgPULSE = (char *)pvPortMalloc(TX_SIZE_MAX);
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_SEND);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);
    gpio_set_level(pinPulse, HIGH);  
    vTaskDelay(pulse_size_ms/portTICK_PERIOD_MS);
    gpio_set_level(pinPulse, LOW);
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_END);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);
    //pulsePendingFlag = false;
    tHandler_Pulse = NULL;
    vTaskDelete(NULL);
    vPortFree(msgPULSE);
  }
}
*/

void vTaskPulse(void* pvParam)
{
  for(;;) {
    msgPULSE = (char *)pvPortMalloc(TX_SIZE_MAX);
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_SEND);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);
    gpio_set_level(pinPulse, HIGH);  
    vTaskDelay(pulse_size_ms/portTICK_PERIOD_MS);
    gpio_set_level(pinPulse, LOW);
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_END);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);
    tHandler_Pulse = NULL;
    vTaskDelete(NULL);
    vPortFree(msgPULSE);
  }
}


void stopPulse(bool notifyStop)
{
  gpio_set_level(pinPulse, LOW);
  if(notifyStop){
    sprintf(msgPULSE, PATTERN_OK, pqseq, PULSE_STOP);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);
  }
  vTaskDelete(tHandler_Pulse);
  tHandler_Pulse = NULL;
  vPortFree(msgPULSE);
}


void vTaskKeepAlive(void* pvParam)
{
  char *dataKA = (char *)pvPortMalloc(TX_SIZE_MAX);
  for(;;) {
    sprintf(dataKA, PATTERN_KEEP_ALIVE, pdseq);
    transmitTX((const char *) dataKA, TX_SIZE_KEEPALIVE);
    vTaskDelay(KEEP_ALIVE_TIME/portTICK_PERIOD_MS);
  }
  vPortFree(dataKA);
  vTaskDelete(NULL);
}


void vTaskAT(void* pvParam)
{
  int rxBytes;
  dataRX = (uint8_t*) pvPortMalloc(8);
  msgTX = (char *)pvPortMalloc(TX_SIZE_MAX);
  for(;;) {
      switch (stateAT)
      {
      case stIDLE:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 5, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          if(strncmp((const char*)dataRX, PATTERN_PREFIX_ACTIVATE, 5) == 0){
            stateAT = stPLUSE_SIZE;
            operation = AT_ACTIVATE;
          }
          else {
            if(strncmp((const char*)dataRX, PATTERN_PREFIX_STOP, 5) == 0){
              stateAT = stSEQ;
              operation = AT_STOP;
            }
            else{
              if(strncmp((const char*)dataRX, PATTERN_PREFIX_KA_ACK, 5) == 0){
                stateAT = stEQUAL;
                operation = AT_KEEP_ALIVE_ACK;
              }
              else{
                operation = AT_ERROR;
                stateAT = stCR;
              }
            }
          }
        }
        break;

      case stPLUSE_SIZE:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 2, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          //printf("[INFO] DATA %d %d\r\n", dataRX[0], dataRX[1]);
          pulse_size_ms = (uint32_t)TO_UINT8(dataRX[0], dataRX[1]) * PULSE_RESOLUTION;
          EEPROM.put(ADDR_EEPROM_PULSE, pulse_size_ms);
          stateAT = stSEQ;
        }
        break;

      case stSEQ:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 2, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          pqseq = TO_UINT8(dataRX[0], dataRX[1]);
          if (operation == AT_ACTIVATE && tHandler_Pulse == NULL)
            pulse_pqseq = pqseq;
          stateAT = stCR;
        }
        break;

      case stEQUAL:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 1, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {          
          if(strncmp((const char*)dataRX, "=", 1) == 0){
            stateAT = stPDSEQ_ACK;
          }
          else{
            operation = AT_ERROR;
            stateAT = stCR;
          }
        }
        break;

      case stPDSEQ_ACK:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 2, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          pdseq_ack = TO_UINT8(dataRX[0], dataRX[1]);
          stateAT = stPQSEQ_ACK;
        }
        break;

      case stPQSEQ_ACK:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 2, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          pqseq_ack = TO_UINT8(dataRX[0], dataRX[1]);
          stateAT = stCR;
        }
        break;

      case stCR:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 1, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          if(strncmp((const char*)dataRX, PATTERN_CR, 1) == 0){
            stateAT = stLF;
          }
          else{
            operation = AT_ERROR;
          }
        }
        break;

      case stLF:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 1, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          if(strncmp((const char*)dataRX, PATTERN_LF, 1) == 0){
            pdseq++;
            stateAT = stIDLE;
            switch (operation)
            {
            case AT_ERROR:
              //printf("[INFO] ERROR\r\n");
              sprintf(msgTX, PATTERN_ERR, pqseq, AT_ERR);
              transmitTX((const char *) msgTX, TX_SIZE_RESPONSE_ERR);
              break;

            case AT_ACTIVATE:
              //printf("[INFO] ACTIVATE %d %d\r\n", pulse_size_ms, pqseq);
              if (tHandler_Pulse == NULL)
                xTaskCreatePinnedToCore(vTaskPulse, "Task Pulse", 2000, NULL, 1, &tHandler_Pulse, 1);
              else{
                sprintf(msgTX, PATTERN_ERR, pqseq, PULSE_ERR);
                transmitTX((const char *) msgTX, TX_SIZE_RESPONSE_ERR);
              }
              break;

            case AT_STOP:
              //printf("[INFO] STOP %d\r\n", pqseq);
              if (tHandler_Pulse == NULL){
                sprintf(msgTX, PATTERN_OK, pqseq, PULSE_NONE);
                transmitTX((const char *) msgTX, TX_SIZE_RESPONSE_OK);
              }
              else{
                stopPulse(NOTIFY);
                // pulsePendingFlag = false;
              }
              break;

            case AT_KEEP_ALIVE_ACK:
              // ToDo: Further versions - Check ACK
              printf("[INFO] KEEP ALIVE ACK %d %d\r\n", pdseq_ack, pqseq_ack);
              break;
            
            default:
              break;
            }
            operation = AT_NONE;
          }

      
      default:
        break;
      }
    }    
  }
  vPortFree(dataRX);
  vPortFree(msgTX);
  vTaskDelete(NULL);
}



void setup() 
{
  gpio_set_direction(pinBUTTON_PULSE, GPIO_MODE_INPUT);
  gpio_pulldown_dis(pinBUTTON_PULSE);
  gpio_set_intr_type(pinBUTTON_PULSE, GPIO_INTR_POSEDGE);

  gpio_set_direction(pinBUTTON_MODE, GPIO_MODE_INPUT);
  gpio_pulldown_dis(pinBUTTON_MODE);
  gpio_set_intr_type(pinBUTTON_MODE, GPIO_INTR_POSEDGE);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(pinBUTTON_PULSE, pulse_handleInterrupt, (void*)pinBUTTON_PULSE);
  gpio_isr_handler_add(pinBUTTON_MODE, mode_handleInterrupt, (void*)pinBUTTON_MODE);

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = ((1ULL<<pinLED_STATUS_R) | (1ULL<<pinLED_STATUS_G) | (1ULL<<pinLED_MODE_G) | (1ULL<<pinLED_MODE_R));
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);
 

  uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

  uart_driver_install(UART_PORT, BUF_RX_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
  uart_param_config(UART_PORT, &uart_config);
  uart_set_pin(UART_PORT, pinUART_TX, pinUART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  //ledcAttachPin(pinPulse, PWM_CHANNEL);
  //ledcSetup(PWM_CHANNEL, 10, 8); // 10 Hz PWM (100ms), 8-bit resolution
  //attachInterrupt(digitalPinToInterrupt(pinPulse), ledcEvent, FALLING);

  gpio_set_level(pinLED_STATUS_G, LOW);  
  gpio_set_level(pinLED_STATUS_R, LOW);
  gpio_set_level(pinLED_MODE_G, LOW);
  gpio_set_level(pinLED_MODE_R, LOW);
  gpio_set_level(pinPulse, LOW);

  pdseq = 0;
  EEPROM.get(ADDR_EEPROM_PULSE, pulse_size_ms);

//  ledcWrite(PWM_CHANNEL, 128);

  app_main();
}


void app_main(void)
{
  if(xTaskGetSchedulerState()==taskSCHEDULER_RUNNING)
    printf("[INFO] Scheduler is running\n");

  vSemaphoreCreateBinary( sSemphrMode );
  if( sSemphrMode == NULL )
  {
    printf("[FAIL] Mode Semaphore has not been created\n");
    return;
  }

  vSemaphoreCreateBinary( sSemphrPulse );
  if( sSemphrPulse == NULL )
  {
    printf("[FAIL] Pulse Semaphore has not been created\n");
    return;
  }

  xMutexTX = xSemaphoreCreateMutex();
  if( xMutexTX == NULL )
  {
    printf("[FAIL] TX Mutex has not been created\n");
    return;
  }

  xTaskCreatePinnedToCore(vTaskMode, "Task Mode", 2500, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(vTaskManualPulse, "Task Manual Pulse", 2500, NULL, 1, &tHandler_ManualPulse, 0);
  xTaskCreatePinnedToCore(vTaskKeepAlive, "Task KA", 2500, NULL, 2, &tHandler_KeepAlive, 0);
  xTaskCreatePinnedToCore(vTaskAT, "Task AT", 5000, NULL, 1, &tHandler_AT, 0);
}


void loop() {
  // put your main code here, to run repeatedly:
}

