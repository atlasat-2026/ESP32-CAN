#include "packet_handler.h"

#include "drone.h"
#include "drone_comms.h"
#include "drone_controller.h"
#include "env_sens.h"
#include "esp32-hal.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "gps.h"
#include "imu.h"
#include "nav.h"
#include "portmacro.h"
#include "radio.h"
#include "sens_fus.h"
#include <atomic>
#include <cstdint>
#include <sys/unistd.h>

#ifdef PS
#undef PS
#endif

#ifdef F
#undef F
#endif

#include <Eigen/Dense>

void handle_waypoint_update(uint8_t *packet_addr, uint8_t index);
void handle_nav_update(uint8_t *packet_addr);

void handle_packet(uint8_t *packet_addr) {
  PACKET_TYPE packet_type = *((PACKET_TYPE *)packet_addr);
  if (packet_type == PACKET_TYPE::COMMAND_REQUEST) {
    packet_command_request *packet =
        (packet_command_request *)(packet_addr + sizeof(PACKET_TYPE));
    send_packet_getter(packet->packet_requested);
  }

  // NAV SETTERS
  else if (packet_type >= PACKET_TYPE::DRONE_NAV_WAYPOINT_0 &&
           packet_type <= PACKET_TYPE::DRONE_NAV_WAYPOINT_7) {
    uint8_t index = packet_type - PACKET_TYPE::DRONE_NAV_WAYPOINT_0;
    handle_waypoint_update(packet_addr, index);

  } else if (packet_type == PACKET_TYPE::DRONE_NAV) {
    handle_nav_update(packet_addr);

  } else if (packet_type == PACKET_TYPE::KILLSWITCH_TOGGLE) {
    packet_killswitch_toggle *packet =
        (packet_killswitch_toggle *)(packet_addr + sizeof(PACKET_TYPE));
    std::atomic_store(&killswitch_active, packet->killswitch_active);
    ESP_LOGI("RADIO_RX", "KILLSWITCH PAACKET");
    ESP_LOGE("switch_set", "killswitch set to: %d, packet: %d",
             atomic_load(&killswitch_active), packet->killswitch_active);

  } else if (packet_type == PACKET_TYPE::MODE_INPUT) {
    packet_mode_input *packet =
        (packet_mode_input *)(packet_addr + sizeof(PACKET_TYPE));
    atomic_store(&drone_cont->current_input_mode, packet->input_type);

  } else if (packet_type == PACKET_TYPE::CONTROLLER_INPUT) {
    packet_controller_input *packet =
        (packet_controller_input *)(packet_addr + sizeof(PACKET_TYPE));

    if (xSemaphoreTake(controller_input_semaphore, 10)) {
      current_controller_input = *packet;
      time_last_controller = millis();
      xSemaphoreGive(controller_input_semaphore);
    }
  }
}

void handle_waypoint_update(uint8_t *packet_addr, uint8_t index) {
  packet_drone_nav_waypoint *packet =
      (packet_drone_nav_waypoint *)(packet_addr + sizeof(PACKET_TYPE));
  float lon, lat, alt;
  lon = packet->coord[0];
  lat = packet->coord[1];
  alt = packet->coord[2];
  Eigen::Vector3f coords(lon, lat, alt);

  if (xSemaphoreTake(nav_mutex, portMAX_DELAY)) {
    nav_man.waypoints[index].coords = coords;
    xSemaphoreGive(nav_mutex);
  }
}

void handle_nav_update(uint8_t *packet_addr) {
  packet_drone_nav *packet =
      (packet_drone_nav *)(packet_addr + sizeof(PACKET_TYPE));

  if (xSemaphoreTake(nav_mutex, portMAX_DELAY)) {
    nav_man.set_active_mask(packet->active_mask);
    nav_man.current_waypoint = packet->current_waypoint;
  }
}

void send_packet_getter(PACKET_TYPE requested_type) {

  std::pair<uint8_t *, size_t> resp_packet = {nullptr, 0};

  if (requested_type == PACKET_TYPE::INFO_DRONE_POSITION) {

    Eigen::Vector3f local_vel = imu_state_var.rot.inverse() * sens_fus.velocity;

    resp_packet = create_packet_pooled(
        PACKET_TYPE::INFO_DRONE_POSITION,
        packet_info_drone_position{
            .trans = {sens_fus.position.x(), sens_fus.position.y(),
                      sens_fus.position.z()},
            .vel = {local_vel.x(), local_vel.y(), local_vel.z()},
            .rot = {imu_state_var.rot.w(), imu_state_var.rot.x(),
                    imu_state_var.rot.y(), imu_state_var.rot.z()},
            .angvel = {imu_state_var.angvel.x(), imu_state_var.angvel.y(),
                       imu_state_var.angvel.z()}});
  }

  if (requested_type == PACKET_TYPE::INFO_DRONE_STATUS) {
    if (gps_mutex && xSemaphoreTake(gps_mutex, portMAX_DELAY)) {
      resp_packet = create_packet_pooled(
          PACKET_TYPE::INFO_DRONE_STATUS,
          packet_info_drone_status{
              .origin = {gps->origin_long, gps->origin_lat},
              .time_since_boot = millis(),
              .unix_timestamp_millis = gps->unix_timestamp_millis(),
              .gps_fix = gps->gps_avaliable()});
      xSemaphoreGive(gps_mutex);
    }
  }

  // Navigation
  if (requested_type == PACKET_TYPE::DRONE_NAV) {
    uint8_t active_mask, current;
    if (xSemaphoreTake(nav_mutex, portMAX_DELAY)) {
      active_mask = nav_man.get_active_mask();
      current = nav_man.current_waypoint;
      xSemaphoreGive(nav_mutex);

      resp_packet = create_packet_pooled(
          PACKET_TYPE::DRONE_NAV, packet_drone_nav{active_mask, current});
    }
  }

  if (requested_type >= PACKET_TYPE::DRONE_NAV_WAYPOINT_0 &&
      requested_type <= PACKET_TYPE::DRONE_NAV_WAYPOINT_7) {
    uint8_t index = requested_type - PACKET_TYPE::DRONE_NAV_WAYPOINT_0;

    Eigen::Vector3f coords;
    bool land = false;
    if (xSemaphoreTake(nav_mutex, portMAX_DELAY)) {
      coords = nav_man.waypoints[index].coords;
      land = nav_man.waypoints[index].landing;
      xSemaphoreGive(nav_mutex);
    }

    resp_packet = create_packet_pooled(
        requested_type,
        packet_drone_nav_waypoint{{coords[0], coords[1], coords[2]}, land});
  }

  if (resp_packet.first != nullptr) {
    xQueueSend(packet_tx_queue, resp_packet.first, portMAX_DELAY);
  }
}
