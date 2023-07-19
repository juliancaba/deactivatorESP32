#include <Arduino.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/uart.h"

#include "pinout.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


QueueHandle_t sSemphrEnable = NULL;
QueueHandle_t xMutexTX = NULL;

TaskHandle_t tHandler_KeepAlive = NULL;
TaskHandle_t tHandler_AT = NULL;
TaskHandle_t tHandler_Pulse = NULL;

uint8_t* dataRX = NULL;
char *msgPULSEStop = NULL;
char *msgPULSE = NULL;//(char *)malloc(TX_SIZE_RESPONSE_OK);
char *dataKA = NULL;//(char *)malloc(TX_SIZE_KEEPALIVE);
char *msgTX = NULL;//(char *)malloc(TX_SIZE_RESPONSE_ERR);

static volatile uint8_t pdseq;
static uint8_t pqseq;
static uint8_t pdseq_ack;
static uint8_t pqseq_ack;
static uint16_t pulse_size_ms;
static uint8_t pulse_pqseq;
static enum {stIDLE, stPLUSE_SIZE, stSEQ, stEQUAL, stPDSEQ_ACK, stPQSEQ_ACK, stCR, stLF} stateAT;
static tOperation operation = AT_NONE;

void app_main(void);
void createTasks(void);

void IRAM_ATTR en_handleInterrupt(void* arg)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( sSemphrEnable, &xHigherPriorityTaskWoken );
}


void transmitTX(const char *dataTX)
{
  xSemaphoreTake(xMutexTX, TX_MUTEX_WAIT/portTICK_PERIOD_MS);
  uart_write_bytes(UART_PORT, dataTX, strlen(dataTX));
  xSemaphoreGive( xMutexTX );
}


static void vTaskEnable( void *pvParameters )
{
  for( ;; ){
    xSemaphoreTake( sSemphrEnable, portMAX_DELAY );

    if (gpio_get_level(pinSwitch) == HIGH){
      gpio_set_level(pinLED_on, HIGH);
      gpio_set_level(pinLED_off, LOW);
      //vTaskResume(tHandler_KeepAlive);
      //vTaskResume(tHandler_AT);
      //createTasks();
    }
    else{
      gpio_set_level(pinLED_on, LOW);
      gpio_set_level(pinLED_off, HIGH);
      //vTaskDelete(tHandler_KeepAlive);
      //vTaskDelete(tHandler_AT);
      //tHandler_AT = NULL;
      //tHandler_KeepAlive = NULL;
    }
  }
  vTaskDelete(NULL);
}


void vTaskPulse(void* pvParam)
{
  //char *msgPULSE = (char *)malloc(TX_SIZE_RESPONSE_OK);
  //msgPULSE = (char *)malloc(TX_SIZE_RESPONSE_OK);
  for(;;) {
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_SEND);
    transmitTX((const char *) msgPULSE);
    gpio_set_level(pinPulse, HIGH);  
    vTaskDelay(pulse_size_ms/portTICK_PERIOD_MS);
    gpio_set_level(pinPulse, LOW);
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_END);
    transmitTX((const char *) msgPULSE);
    tHandler_Pulse = NULL;
    vTaskDelete(NULL);
  }
  //free(msgPULSE);
}

void stopPulse(void)
{
  //char *msgPULSE2 = (char *)malloc(TX_SIZE_RESPONSE_OK);
  //msgPULSE2 = (char *)malloc(TX_SIZE_RESPONSE_OK);
  gpio_set_level(pinPulse, LOW);
  sprintf(msgPULSEStop, PATTERN_OK, pqseq, PULSE_STOP);
  transmitTX((const char *) msgPULSEStop);
  //tHandler_Pulse = NULL;
  vTaskDelete(tHandler_Pulse);
  tHandler_Pulse = NULL;
  //free(msgPULSE2);
}


void vTaskKeepAlive(void* pvParam)
{
  //char *dataKA = (char *)malloc(TX_SIZE_KEEPALIVE);
  for(;;) {
    dataKA = (char *)pvPortMalloc(TX_SIZE_KEEPALIVE);
    sprintf(dataKA, PATTERN_KEEP_ALIVE, pdseq);
    transmitTX((const char *) dataKA);
    vTaskDelay(KEEP_ALIVE_TIME/portTICK_PERIOD_MS);
  }
  //free(dataKA);
  vTaskDelete(NULL);
}


void vTaskAT(void* pvParam)
{
  int rxBytes;
  //uint8_t* dataRX = (uint8_t*) malloc(8);
  //char *msgTX = (char *)malloc(TX_SIZE_RESPONSE_ERR);
  //msgTX = (char *)malloc(TX_SIZE_RESPONSE_ERR);
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
          printf("[INFO] DATA %d %d\r\n", dataRX[0], dataRX[1]);
          pulse_size_ms = (uint32_t)TO_UINT8(dataRX[0], dataRX[1]) * PULSE_RESOLUTION;
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
              printf("[INFO] ERROR\r\n");
              sprintf(msgTX, PATTERN_ERR, pqseq, AT_ERR);
              transmitTX((const char *) msgTX);
              break;

            case AT_ACTIVATE:
              printf("[INFO] ACTIVATE %d %d\r\n", pulse_size_ms, pqseq);
              if (tHandler_Pulse == NULL)
                xTaskCreatePinnedToCore(vTaskPulse, "Task Pulse", 2000, NULL, 1, &tHandler_Pulse, 1);
              else{
                sprintf(msgTX, PATTERN_ERR, pqseq, PULSE_ERR);
                transmitTX((const char *) msgTX);
              }
              break;

            case AT_STOP:
              printf("[INFO] STOP %d\r\n", pqseq);
              if (tHandler_Pulse == NULL){
                sprintf(msgTX, PATTERN_OK, pqseq, PULSE_NONE);
                transmitTX((const char *) msgTX);
              }
              else{
                stopPulse();
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
  //free(dataRX);
  //free(msgTX);
  vTaskDelete(NULL);
}



void setup() 
{
  gpio_set_direction(pinSwitch, GPIO_MODE_INPUT);
  gpio_pulldown_dis(pinSwitch);
  gpio_set_intr_type(pinSwitch, GPIO_INTR_ANYEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(pinSwitch, en_handleInterrupt, (void*)pinSwitch);

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = ((1ULL<<pinLED_off) | (1ULL<<pinLED_on) | (1ULL<<pinLED_USB_ok) | (1ULL<<pinLED_USB_err) | (1ULL<<pinPulse));
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


  gpio_set_level(pinLED_on, LOW);  
  gpio_set_level(pinLED_off, LOW);
  gpio_set_level(pinLED_USB_ok, LOW);
  gpio_set_level(pinLED_USB_err, LOW);
  gpio_set_level(pinPulse, LOW);

  pdseq = 0;

  // FreeRTOS-ESP32 uses heap_1 so free is not allowed
  dataRX = (uint8_t*) malloc(8);
  msgPULSEStop = (char *)malloc(TX_SIZE_RESPONSE_OK);
  msgPULSE = (char *)malloc(TX_SIZE_RESPONSE_OK);
  dataKA = (char *)malloc(TX_SIZE_KEEPALIVE);
  msgTX = (char *)malloc(TX_SIZE_RESPONSE_ERR);

  app_main();
}


void app_main(void)
{
  if(xTaskGetSchedulerState()==taskSCHEDULER_RUNNING)
    printf("[INFO] Scheduler is running\n");

  vSemaphoreCreateBinary( sSemphrEnable );
  if( sSemphrEnable == NULL )
  {
    printf("[FAIL] Enable Semaphore has not been created\n");
    return;
  }

  xMutexTX = xSemaphoreCreateMutex();
  if( xMutexTX == NULL )
  {
    printf("[FAIL] TX Mutex has not been created\n");
    return;
  }

  xTaskCreatePinnedToCore(vTaskEnable, "Task Enable", 2500, NULL, 10, NULL, 0);
  createTasks();

}

void createTasks(void)
{
  xTaskCreatePinnedToCore(vTaskKeepAlive, "Task KA", 5000, NULL, 2, &tHandler_KeepAlive, 0);
  xTaskCreatePinnedToCore(vTaskAT, "Task AT", 5000, NULL, 1, &tHandler_AT, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
}

