#include <Arduino.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <EEPROM.h>

#include "driver/uart.h"
#include "driver/timer.h"

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
static uint16_t pulse_num = 1;
static volatile uint16_t pulse_cnt;
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


void IRAM_ATTR timeout_handler(void *para)
{
  TIMERG0.int_clr_timers.t0 = 1;

  if (pulse_num >= pulse_cnt){
    if(gpio_get_level(pinPulse) == HIGH){
      gpio_set_level(pinPulse, LOW);
    }
    else{
      gpio_set_level(pinPulse, HIGH);
    }
    pulse_cnt++;
    timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);
  }
  //TIMERG0.hw_timer[0].config.alarm_en = 1;
}


void transmitTX(const char *dataTX, uint8_t size)
{
  xSemaphoreTake(xMutexTX, TX_MUTEX_WAIT/portTICK_PERIOD_MS);
  uart_write_bytes(UART_PORT, dataTX, size);
  xSemaphoreGive( xMutexTX );
}


void generatePulse(int pulseHigh_ms)
{
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, pulseHigh_ms*PULSE_PERIOD_TICKS);
  
  pulse_cnt=0;

  gpio_set_level(pinPulse, HIGH);
  timer_start(TIMER_GROUP_0, TIMER_0);

  while(pulse_cnt!=pulse_num);

  gpio_set_level(pinPulse, LOW);
  timer_pause(TIMER_GROUP_0, TIMER_0);
}


static void vTaskManualPulse( void *pvParameters )
{
  for( ;; ){
    xSemaphoreTake( sSemphrPulse, portMAX_DELAY );
    printf("[INFO] Pulso Manual\r\n");

    xSemaphoreTake(xMutexPulse, portMAX_DELAY);
    generatePulse(PULSE_MANUAL_MS);
    xSemaphoreGive( xMutexPulse );
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


void vTaskPulse(void* pvParam)
{
  for(;;) {
    xSemaphoreTake(xMutexPulse, portMAX_DELAY);

    msgPULSE = (char *)pvPortMalloc(TX_SIZE_MAX);
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_SEND);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);
    
    generatePulse(pulse_size_ms);
    
    sprintf(msgPULSE, PATTERN_OK, pulse_pqseq, PULSE_END);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);

    xSemaphoreGive( xMutexPulse );

    tHandler_Pulse = NULL;
    vTaskDelete(NULL);
    vPortFree(msgPULSE);
  }
}


void stopPulse(bool notifyStop)
{
  //pulse_cnt=pulse_num-1;
  //while(pulse_cnt!=pulse_num);
  //timer_pause(TIMER_GROUP_0, TIMER_0);
  gpio_set_level(pinPulse, LOW);
  timer_pause(TIMER_GROUP_0, TIMER_0);
  if(notifyStop){
    sprintf(msgPULSE, PATTERN_OK, pqseq, PULSE_STOP);
    transmitTX((const char *) msgPULSE, TX_SIZE_RESPONSE_OK);
  }
  xSemaphoreGive( xMutexPulse );
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
  io_conf.pin_bit_mask = ((1ULL<<pinLED_STATUS_R) | (1ULL<<pinLED_STATUS_G) | (1ULL<<pinLED_MODE_G) | (1ULL<<pinLED_MODE_R) | (1ULL<<pinPulse));
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
  
  timer_config_t config; 
  config.alarm_en = TIMER_ALARM_EN; 
  config.counter_en = TIMER_PAUSE; 
  config.intr_type = TIMER_INTR_LEVEL;
  config.counter_dir = TIMER_COUNT_UP; 
  config.auto_reload = TIMER_AUTORELOAD_EN; 
  config.divider = 8000; 
  
  timer_init(TIMER_GROUP_0,TIMER_0,&config);

  timer_isr_register(TIMER_GROUP_0, TIMER_0, timeout_handler,
                       (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

  gpio_set_level(pinLED_STATUS_G, LOW);  
  gpio_set_level(pinLED_STATUS_R, LOW);
  gpio_set_level(pinLED_MODE_G, LOW);
  gpio_set_level(pinLED_MODE_R, LOW);
  gpio_set_level(pinPulse, LOW);

  pdseq = 0;
  EEPROM.get(ADDR_EEPROM_PULSE, pulse_size_ms);

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

  //vSemaphoreCreateBinary(xMutexPulse);
  xMutexPulse = xSemaphoreCreateMutex();
  if( xMutexPulse == NULL )
  {
    printf("[FAIL] Pulse Mutex has not been created\n");
    return;
  }

  xTaskCreatePinnedToCore(vTaskMode, "Task Mode", 2500, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(vTaskManualPulse, "Task Manual Pulse", 5000, NULL, 1, &tHandler_ManualPulse, 0);
  xTaskCreatePinnedToCore(vTaskKeepAlive, "Task KA", 2500, NULL, 2, &tHandler_KeepAlive, 0);
  xTaskCreatePinnedToCore(vTaskAT, "Task AT", 5000, NULL, 1, &tHandler_AT, 0);
}


void loop() {
  // put your main code here, to run repeatedly:
}

