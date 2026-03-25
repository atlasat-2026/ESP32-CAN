#pragma once

#include "HardwareSerial.h"
#include "Wire.h"
#include <Adafruit_GPS.h>

struct GPS {
  Adafruit_GPS gps;
  GPS(TwoWire *theWire) {
    Serial2.begin(9600);
    gps = Adafruit_GPS(&Serial2);
    gps.begin(0x10);
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  }

  void poll() {
    this->gps.read();
    if (this->gps.newNMEAreceived()) {
      if (!this->gps.parse(this->gps.lastNMEA()))
        return;
    }
  }
};
