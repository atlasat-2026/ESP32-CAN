#include "radio.h"

#include "Esp.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-spi.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "packet_handler.h"
#include <RFM69.h>
#include <SPI.h>
#include <cstdint>
#include <cstring>

#include <drone_comms.h>

// Right to left on hardware.

#define RFM69_INT GPIO_NUM_8 // DI0
#define RFM69_CS GPIO_NUM_9  // NSS
#define RFM69_RST GPIO_NUM_10

#define RFM69_SCLK GPIO_NUM_11 // SCK
#define RFM69_MOSI GPIO_NUM_12
#define RFM69_MISO GPIO_NUM_13

#define FREQUENCY RF69_433MHZ
#define NODEID 1
#define GROUNDID 99
#define NETWORKID 100

static const char *TAG = "RADIO_TASK";

#define PACKET_QUEUE_CAP 10

static bool is_pending_switch = false;
static uint32_t target_bitrate = 0;
static uint32_t switch_at_ms = 0;
static uint32_t rollback_at_ms = 0;
static bool confirmed_at_new_rate = false;

void radio_task(void *pvParameters) {
  ESP_LOGI(TAG, "Radio Task Started on Core %d", xPortGetCoreID());
  packet_rx_queue = xQueueCreate(PACKET_QUEUE_CAP, MAX_PACKET_SIZE);
  packet_tx_queue = xQueueCreate(PACKET_QUEUE_CAP, MAX_PACKET_SIZE);
  controller_input_semaphore = xSemaphoreCreateMutex();

  pinMode(RFM69_CS, OUTPUT);
  pinMode(RFM69_INT, INPUT);

  SPIClass hspi(HSPI);
  hspi.begin(RFM69_SCLK, RFM69_MISO, RFM69_MOSI, RFM69_CS);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  digitalWrite(RFM69_RST, LOW);
  vTaskDelay(pdMS_TO_TICKS(50));

  RFM69 radio(RFM69_CS, RFM69_INT, true, &hspi);

  if (radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    radio.setPowerLevel(31);
    radio.setHighPower(true);
    // radio.setCustomBitrate(DEFAULT_COMMS_BITRATE);
    ESP_LOGI(TAG, "Radio Initialized. Version: 0x%02X", radio.readReg(0x10));
    radio.readAllRegsCompact();

  } else {

    radio.readAllRegsCompact();
    ESP_LOGE(TAG, "Radio Init FAILED! Restarting.");
    ESP.restart();
  }

  while (1) {
    uint32_t now = millis();

    while (radio.receiveDone()) {

      // If we receive ANY valid packet while in probation, confirm the switch
      // (Bit-rate switching)
      if (is_pending_switch && now > switch_at_ms) {
        confirmed_at_new_rate = true;
        ESP_LOGI(TAG, "New bitrate confirmed by valid packet.");
      }

      ESP_LOGD(TAG, "Packet [ID:%d] RSSI:%d LEN:%d", radio.SENDERID, radio.RSSI,
               radio.DATALEN);

      memset(packet_data, '\0', sizeof(packet_data));
      memcpy(packet_data, radio.DATA, radio.DATALEN);

      PACKET_TYPE packet_type = *((PACKET_TYPE *)&packet_data[0]);
      if (packet_type == COMMAND_CHANGE_DATARATE) {
        packet_command_datarate *cmd =
            (packet_command_datarate *)(&packet_data[0] + sizeof(PACKET_TYPE));
        target_bitrate = cmd->target_bitrate;

        switch_at_ms = now + cmd->ms_delay;
        rollback_at_ms = now + cmd->ms_rollback;
        is_pending_switch = true;
        confirmed_at_new_rate = false;

        ESP_LOGI(TAG, "Datarate change requested: %d. Switching in 100ms...",
                 target_bitrate);
      } else {
        xQueueSend(packet_rx_queue, &packet_data[0], portMAX_DELAY);
      }

      if (radio.ACKRequested()) {
        radio.sendACK();
      }
    }

    // Send packets that were queued up for sending
    if (xQueueReceive(packet_tx_queue, &packet_data[0], 1)) {
      PACKET_TYPE packet_type = *((PACKET_TYPE *)&packet_data[0]);
      radio.send(GROUNDID, &packet_data[0], get_packet_size(packet_type));
    }

    // --- STATE MACHINE FOR BITRATE SWITCHING ---

    // 1. Execute the Switch
    if (is_pending_switch && now >= switch_at_ms && !confirmed_at_new_rate) {
      // We only want to trigger the register write once
      radio.setCustomBitrate(target_bitrate);
      switch_at_ms = 0xFFFFFFFF; // Prevent re-triggering
    }

    // 2. The Rollback (The Fail-safe)
    if (is_pending_switch && !confirmed_at_new_rate && now > rollback_at_ms) {
      ESP_LOGE(TAG,
               "ROLLBACK: No confirmation at new rate. Reverting to default.");
      radio.setCustomBitrate(DEFAULT_COMMS_BITRATE);
      is_pending_switch = false;
    }

    // 3. Clear pending flag once confirmed
    if (confirmed_at_new_rate) {
      is_pending_switch = false;
      confirmed_at_new_rate = false;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
