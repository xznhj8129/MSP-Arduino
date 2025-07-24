#include <Arduino.h>
#include <HardwareSerial.h>

#include "inav_api.h"

// --- Serial ports & baud ---
HardwareSerial &fcSerial = Serial2;
const long FC_BAUD_RATE    = 115200;
const long DEBUG_BAUD_RATE = 115200;

// --- Timing ---
unsigned long lastStatusRequest = 0;
unsigned long lastGpsRequest    = 0;
unsigned long lastNavRequest    = 0;
unsigned long lastRC            = 0;

const unsigned long STATUS_INTERVAL = 500;   // ms
const unsigned long GPS_INTERVAL    = 1000;  // ms
const unsigned long NAV_INTERVAL    = 1000;  // ms
const unsigned long RC_INTERVAL     = 100;   // ms

InavAPI inav;

// --- Helpers ---
static void printWaypoint(const inav_nav_waypoint &wp) {
    Serial.printf("  WP Index: %u\n", wp.waypointIndex);
    Serial.printf("    Action: %u\n", wp.action);
    Serial.printf("    Lat: %.7f\n", wp.latitude);
    Serial.printf("    Lon: %.7f\n", wp.longitude);
    Serial.printf("    Alt (m): %.2f\n", wp.altitude);
    Serial.printf("    P1: %u, P2: %u, P3: %u\n", wp.param1, wp.param2, wp.param3);
    Serial.printf("    Flags: 0x%02X\n", wp.flag);
}

void setup() {
    Serial.begin(DEBUG_BAUD_RATE);
    while (!Serial) {}

    Serial.println("\n--- INAV MSP Example (via InavAPI, bool-returns) ---");

    fcSerial.begin(FC_BAUD_RATE);
    if (!&fcSerial) {
        Serial.println("Error initializing FC Serial!");
        while (1);
    }

    inav.Init(fcSerial, 500);

    if (!inav.isConnected()) {
        Serial.println("No MSP response. Check wiring/baud.");
        while (1) delay(1000);
    }

    Serial.println("Requesting FC Info...");
    msp_api_version_t api{};
    if (inav.get_api_version(api)) {
        Serial.printf("MSP Proto: %u, API: %u.%u\n", api.mspProtocolVersion, api.apiVersionMajor, api.apiVersionMinor);
    } else Serial.println("  API version failed");

    msp_fc_variant_t variant{};
    if (inav.get_fc_variant(variant)) {
        Serial.printf("FC Variant: %.4s\n", variant.fcVariantIdentifier);
    }

    msp_fc_version_t ver{};
    if (inav.get_fc_version(ver)) {
        Serial.printf("FC Version: %u.%u.%u\n", ver.fcVersionMajor, ver.fcVersionMinor, ver.fcVersionPatch);
    }

    char targetName[48];
    msp_board_info_t board{};
    if (inav.get_board_info(board, targetName, sizeof(targetName))) {
        Serial.printf("Board: %.4s  HW: %u  Target: %s\n",
                      board.boardIdentifier, board.hardwareRevision, targetName);
    }

    // Mode ranges
    inav_mode_ranges ranges{};
    if (inav.get_mode_ranges(ranges)) {
        Serial.printf("Received %u Mode Ranges\n", ranges.count);
        for (uint16_t i = 0; i < ranges.count; ++i) {
            if (ranges.ranges[i].modePermanentId == 0) continue;
            uint16_t start = 900 + ranges.ranges[i].rangeStartStep * 25;
            uint16_t end   = 900 + ranges.ranges[i].rangeEndStep   * 25;
            Serial.printf("  Range[%u]: ModeID=%u, Aux=%u, PWM=[%u,%u]\n",
                          i,
                          ranges.ranges[i].modePermanentId,
                          ranges.ranges[i].auxChannelIndex + 1,
                          start, end);
        }
    } else {
        Serial.println("Failed to get Mode Ranges");
    }

    // Waypoint 1 check
    inav_nav_waypoint wp1{};
    if (inav.get_wp(1, wp1)) {
        Serial.println("--- Waypoint 1 (if any) ---");
        printWaypoint(wp1);
    }

    // Upload a sample mission
    Serial.println("--- Uploading Sample Mission ---");
    inav_nav_waypoint w1{};
    w1.waypointIndex = 1;
    w1.action        = NAV_WP_ACTION_WAYPOINT;
    w1.latitude      = 40.7128000;
    w1.longitude     = -74.0060000;
    w1.altitude      = 50.0f;
    w1.param1        = 500; // cm/s
    w1.param2 = w1.param3 = 0;
    w1.flag          = 0;

    if (inav.set_waypoint(w1)) {
        Serial.println("  Waypoint 1 SET OK");
        printWaypoint(w1);
    } else Serial.println("  Failed to set Waypoint 1");

    delay(50);

    inav_nav_waypoint w2{};
    w2.waypointIndex = 2;
    w2.action        = NAV_WP_ACTION_RTH;
    w2.flag          = NAV_WP_FLAG_LAST;
    if (inav.set_waypoint(w2)) {
        Serial.println("  Waypoint 2 SET OK");
        printWaypoint(w2);
    } else Serial.println("  Failed to set Waypoint 2");

    Serial.println("\n--- Setup Complete - Starting Loop ---");
    lastStatusRequest = millis();
    lastGpsRequest    = millis();
    lastNavRequest    = millis();
}

void loop() {
    unsigned long now = millis();

    if (now - lastStatusRequest >= STATUS_INTERVAL) {
        lastStatusRequest = now;

        inav_status st{};
        if (inav.get_status(st)) {
            Serial.printf("Cycle: %u us  CPU: %u%%  Armed:%d\n",
                        st.cycleTime_us, st.cpuLoad_pct, st.armed);
            Serial.printf("Profile:%u Batt:%u Mixer:%u\n",
                        st.profile, st.battProfile, st.mixerProfile);
            Serial.printf("Sensors ACC:%d BARO:%d MAG:%d GPS:%d RNG:%d OPF:%d PIT:%d TMP:%d HWFAIL:%d\n",
                        st.sensors.acc, st.sensors.baro, st.sensors.mag, st.sensors.gps,
                        st.sensors.rangefinder, st.sensors.opflow, st.sensors.pitot,
                        st.sensors.temp, st.sensors.hwFailure);
        }

        inav_analog an{};
        if (inav.get_analog(an)) {
            Serial.printf("VBat: %.2fV  I: %.2fA  mAh:%lu  %u%%  RSSI:%u\n",
                        an.vbat_V, an.amps_A,
                        (unsigned long)an.mAhDrawn,
                        an.percentRemaining,
                        an.rssi);
        }


        Serial.println("Attitude:");
        inav_attitude att{};
        if (inav.get_attitude(att)) {
            Serial.printf("  Roll: %.1f, Pitch: %.1f, Yaw: %d\n",
                          att.roll, att.pitch, att.yaw);
        } else Serial.println("  Attitude request failed");

        Serial.println("Altitude:");
        inav_altitude alt{};
        if (inav.get_altitude(alt)) {
            Serial.printf("  Alt: %.2f m, Vario: %.2f m/s, Baro: %.2f m\n",
                          alt.estimatedAltitude, alt.variometer, alt.baroAltitude);
        } else Serial.println("  Altitude request failed");
    }

    if (now - lastGpsRequest >= GPS_INTERVAL) {
        lastGpsRequest = now;

        Serial.println("\nGPS:");
        inav_gps_data gps{};
        if (inav.get_gps(gps)) {
            Serial.printf("  Fix: %u, Sats: %u\n", gps.fixType, gps.sats);
            if (gps.fixType > 0) {
                Serial.printf("  Lat: %.7f, Lon: %.7f, Alt: %.1f m\n",
                              gps.latitude, gps.longitude, gps.altitude);
                Serial.printf("  Speed: %.2f m/s, Course: %.1f deg, HDOP: %.2f\n",
                              gps.speed, gps.course, gps.hdop);
            }
        } else {
            Serial.println("  GPS request failed");
        }

        inav_comp_gps cg{};
        if (inav.get_compgps(cg)) {
            Serial.printf("  DistHome: %u m, DirHome: %u deg\n",
                          cg.distanceToHome, cg.directionToHome);
        } else {
            Serial.println("  CompGPS request failed");
        }
    }

    if (now - lastNavRequest >= NAV_INTERVAL) {
        lastNavRequest = now;

        Serial.println("\nNav Status:");
        inav_nav_data ns{};
        if (inav.get_nav_status(ns)) {
            Serial.printf("  Mode: %d, State: %d, Target WP: %d, Err: %d, Target Head: %d\n",
                          ns.navMode, ns.navState, ns.activeWpNumber, ns.navError, ns.targetHeading);
        } else {
            Serial.println("  Nav Status failed");
        }
    }

    if (now - lastRC >= RC_INTERVAL) {
        lastRC = now;
        msp_rc_channels_t rc{};
        for (int i = 0; i < MAX_RC_CHANNELS; ++i) rc.channel[i] = 1500;
        rc.channel[3] = 1000; // throttle low
        inav.set_rc(rc);
        delay(20);
    }

    delay(10);
}
