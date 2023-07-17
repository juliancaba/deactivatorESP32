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


QueueHandle_t sSemphrEnable;
QueueHandle_t xMutexTX;

TaskHandle_t tHandler_KeepAlive;
TaskHandle_t tHandler_AT;

static uint8_t pdseq;
static uint8_t pqseq;
static uint8_t pulse;
static enum {stIDLE, stPLUSE_SIZE, stSEQ, stCRLF} stateAT;
static uint8_t operation = _NONE;

void app_main(void);

void IRAM_ATTR en_handleInterrupt(void* arg)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( sSemphrEnable, &xHigherPriorityTaskWoken );
}


void transmitTX(const char *dataTX)
{
  xSemaphoreTake(xMutexTX, MUTEXTX_WAIT/portTICK_PERIOD_MS);
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
    }
    else{
      gpio_set_level(pinLED_on, LOW);
      gpio_set_level(pinLED_off, HIGH);
      //vTaskSuspend(tHandler_KeepAlive);
      //vTaskSuspend(tHandler_AT);
    }
  }
  vTaskDelete(NULL);
}




void vTaskKeepAlive(void* pvParam)
{
  char *dataKA = (char *)malloc(BUF_TX_SIZE_KEEPALIVE*sizeof(char));
  for(;;) {
    sprintf(dataKA, PATTERN_KEEP_ALIVE, pdseq);
    transmitTX((const char *) dataKA);
    //pdseq++;
    vTaskDelay(KEEP_ALIVE_TIME/portTICK_PERIOD_MS);
  }
  free(dataKA);
  vTaskDelete(NULL);
}


void vTaskAT(void* pvParam)
{
  int rxBytes;
  uint8_t* dataRX = (uint8_t*) malloc(8);
  //uint8_t dataRX;
  for(;;) {
      switch (stateAT)
      {
      case stIDLE:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 5, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          if(strncmp((const char*)dataRX, PATTERN_PREFIX_ACTIVATE, 5) == 0){
            stateAT = stPLUSE_SIZE;
            operation = _ACTIVATE;
          }
          else {
            if(strncmp((const char*)dataRX, PATTERN_PREFIX_STATUS, 5) == 0){
              stateAT = stSEQ;
              operation = _STATUS;
            }
            else{
              stateAT = stCRLF;
            }
          }
        }
        break;

      case stPLUSE_SIZE:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 2, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          pulse = (dataRX[0]-48)*16+(dataRX[1]-48);
          stateAT = stSEQ;
        }
        break;

      case stSEQ:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 2, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          pqseq = (dataRX[0]-48)*16+(dataRX[1]-48);
          stateAT = stCRLF;
        }
        break;

      case stCRLF:
        rxBytes = uart_read_bytes(UART_PORT, dataRX, 2, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
          if(strncmp((const char*)dataRX, PATTERN_CRLF, 2) == 0){
            pdseq++;
            stateAT = stIDLE;
            switch (operation)
            {
            case _NONE:
              printf("[INFO] ERROR\r\n");
              transmitTX("ERROR\r\n");
              break;

            case _ACTIVATE:
              printf("[INFO] ACTIVATE %d %d\r\n", pulse, pqseq);
              transmitTX("ACTIVATE\r\n");
              break;

            case _STATUS:
              printf("[INFO] STATUS %d\r\n", pqseq);
              transmitTX("STATUS\r\n");
              break;
            
            default:
              break;
            }
            operation = _NONE;
          }

      
      default:
        break;
      }
    }    
  }
  free(dataRX);
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
  io_conf.pin_bit_mask = ((1ULL<<pinLED_off) | (1ULL<<pinLED_on) | (1ULL<<pinLED_USB_ok) | (1ULL<<pinLED_USB_err));
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

  pdseq = 0;

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
  xTaskCreatePinnedToCore(vTaskKeepAlive, "Task KA", 5000, NULL, 2, &tHandler_KeepAlive, 0);
  xTaskCreatePinnedToCore(vTaskAT, "Task AT", 5000, NULL, 1, &tHandler_AT, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
}

