#include "radio.h"
#include "esp32-hal-gpio.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include <Arduino.h>
#include <RFM69.h>
#include <SPI.h>
#include <cstdint>
#include <cstring>

#include <drone_comms.h>

#define RFM69_RST 14
#define RFM69_CS 12
#define RFM69_INT 39
#define SPI_SCLK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define FREQUENCY RF69_433MHZ
#define NODEID 1
#define NETWORKID 100

static const char *TAG = "RADIO_TASK";

#define PACKET_QUEUE_CAP 10

void radio_rx_task(void *pvParameters) {
  ESP_LOGI(TAG, "Radio Task Started on Core %d", xPortGetCoreID());
  packet_queue = xQueueCreate(PACKET_QUEUE_CAP, MAX_PACKET_SIZE);
  controller_input_semaphore = xSemaphoreCreateMutex();

  pinMode(RFM69_CS, OUTPUT);
  pinMode(RFM69_INT, INPUT);

  // 1. Setup SPI for this task
  SPIClass vspi(VSPI);
  vspi.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, 34);

  // 3. Hardware Reset
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  digitalWrite(RFM69_RST, LOW);
  vTaskDelay(pdMS_TO_TICKS(50));

  RFM69 radio(RFM69_CS, RFM69_INT, true, &vspi);

  if (radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    radio.setHighPower(true);
    ESP_LOGI(TAG, "Radio Initialized. Version: 0x%02X", radio.readReg(0x10));
  } else {
    ESP_LOGE(TAG, "Radio Init FAILED! Deleting Task.");
    vTaskDelete(NULL);
  }

  // 4. Polling Loop
  while (1) {
    if (radio.receiveDone()) {
      // ESP_LOGD(TAG, "Packet [ID:%d] RSSI:%d LEN:%d", radio.SENDERID,
      // radio.RSSI,
      //          radio.DATALEN);

      memset(packet_data, '\0', sizeof(packet_data));
      memcpy(packet_data, radio.DATA, radio.DATALEN);

      PACKET_TYPE packet_type = *((PACKET_TYPE *)&packet_data[0]);
      if (packet_type == CONTROLLER_INPUT) {

        controller_input *inp =
            (controller_input *)(&packet_data[0] + sizeof(PACKET_TYPE));

        if (xSemaphoreTake(controller_input_semaphore, (TickType_t)50) ==
            pdTRUE) {
          current_controller_input = *inp;
          xSemaphoreGive(controller_input_semaphore);
        }

      } else {
        xQueueSend(packet_queue, &packet_data, portMAX_DELAY);
      }

      if (radio.ACKRequested()) {
        radio.sendACK();
      }
    }

    // Small delay to prevent Watchdog trigger and allow other tasks to run
    // 1ms is usually enough for high-speed polling
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
