#include "radio.h"
#include "Esp.h"
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
#define GROUNDID 99
#define NETWORKID 100

static const char *TAG = "RADIO_TASK";

#define PACKET_QUEUE_CAP 10

void radio_task(void *pvParameters) {
  ESP_LOGI(TAG, "Radio Task Started on Core %d", xPortGetCoreID());
  packet_rx_queue = xQueueCreate(PACKET_QUEUE_CAP, MAX_PACKET_SIZE);
  packet_tx_queue = xQueueCreate(PACKET_QUEUE_CAP, MAX_PACKET_SIZE);
  controller_input_semaphore = xSemaphoreCreateMutex();

  pinMode(RFM69_CS, OUTPUT);
  pinMode(RFM69_INT, INPUT);

  SPIClass vspi(VSPI);
  vspi.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, 34);

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
    ESP_LOGE(TAG, "Radio Init FAILED! Restarting.");
    ESP.restart();
  }

  while (1) {
    if (radio.receiveDone()) {
      // ESP_LOGD(TAG, "Packet [ID:%d] RSSI:%d LEN:%d", radio.SENDERID,
      // radio.RSSI,
      //          radio.DATALEN);

      memset(packet_data, '\0', sizeof(packet_data));
      memcpy(packet_data, radio.DATA, radio.DATALEN);

      PACKET_TYPE packet_type = *((PACKET_TYPE *)&packet_data[0]);
      if (packet_type == CONTROLLER_INPUT) {

        packet_controller_input *inp =
            (packet_controller_input *)(&packet_data[0] + sizeof(PACKET_TYPE));

        if (xSemaphoreTake(controller_input_semaphore, (TickType_t)50) ==
            pdTRUE) {
          current_controller_input = *inp;
          xSemaphoreGive(controller_input_semaphore);
        }

      } else {
        xQueueSend(packet_rx_queue, &packet_data[0], portMAX_DELAY);
      }

      if (radio.ACKRequested()) {
        radio.sendACK();
      }
    }
    if (xQueueReceive(packet_tx_queue, &packet_data[0], 1)) {
      PACKET_TYPE packet_type = *((PACKET_TYPE *)&packet_data[0]);
      radio.send(GROUNDID, &packet_data[0], get_packet_size(packet_type));
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
