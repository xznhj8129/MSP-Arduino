#include <Arduino.h>
#include <HardwareSerial.h> // Or SoftwareSerial if needed
#include "MSP.h"

// --- Configuration ---
HardwareSerial& fcSerial = Serial2; // Serial port connected to FC (e.g., Serial2 on ESP32)
const long FC_BAUD_RATE = 115200;  // Baud rate for FC connection
const long DEBUG_BAUD_RATE = 115200; // Baud rate for debug output (Serial Monitor)

// --- MSP Library Instance ---
MSP msp;

// --- Timing ---
unsigned long lastStatusRequest = 0;
unsigned long lastGpsRequest = 0;
unsigned long lastNavRequest = 0;
const unsigned long STATUS_INTERVAL = 500; // ms
const unsigned long GPS_INTERVAL = 1000; // ms
const unsigned long NAV_INTERVAL = 1000; // ms

// --- Helper Function to print waypoint info ---
void printWaypoint(const msp_nav_waypoint_t *wp) {
    if (!wp) return;
    Serial.printf("  WP Index: %d\n", wp->waypointIndex);
    Serial.printf("    Action: %d\n", wp->action);
    Serial.printf("    Lat: %.7f\n", (double)wp->latitude / 1e7);
    Serial.printf("    Lon: %.7f\n", (double)wp->longitude / 1e7);
    Serial.printf("    Alt (cm): %ld\n", wp->altitude);
    Serial.printf("    P1: %d, P2: %d, P3: %d\n", wp->param1, wp->param2, wp->param3);
    Serial.printf("    Flags: 0x%02X\n", wp->flag);
}

void setup() {
    Serial.begin(DEBUG_BAUD_RATE);
    while (!Serial) { ; } // Wait for serial monitor connection

    Serial.println("\n--- INAV MSP Example ---");

    fcSerial.begin(FC_BAUD_RATE);
    if (!fcSerial) {
        Serial.println("Error initializing Flight Controller Serial!");
        while (1); // Halt
    }

    msp.begin(fcSerial); // Initialize MSP library with the FC serial port

    Serial.println("Requesting FC Info...");
    msp_api_version_t api;
    if (msp.requestApiVersion(&api)) {
        Serial.printf("MSP Protocol: v%d, API Version: %d.%d\n",
            api.mspProtocolVersion, api.apiVersionMajor, api.apiVersionMinor);
    } else {
        Serial.println("Failed to get API Version.");
    }

    msp_fc_variant_t variant;
    if (msp.requestFcVariant(&variant)) {
        Serial.printf("FC Variant: %.4s\n", variant.fcVariantIdentifier);
    } else {
        Serial.println("Failed to get FC Variant.");
    }

    msp_fc_version_t fc_version;
    if (msp.requestFcVersion(&fc_version)) {
        Serial.printf("FC Version: %d.%d.%d\n",
            fc_version.fcVersionMajor, fc_version.fcVersionMinor, fc_version.fcVersionPatch);
    } else {
        Serial.println("Failed to get FC Version.");
    }

    // Example: Board Info
    msp_board_info_t boardInfo;
    if (msp.requestBoardInfo(&boardInfo)) {
         Serial.printf("boardIdentifier: %s\n", boardInfo.boardIdentifier);
         Serial.printf("hardwareRevision: %d\n", boardInfo.hardwareRevision);
    } else {
         Serial.println("Failed to get Board Info.");
    }

    // Example: Get craft name
    msp_name_t craftName;
    if (msp.requestCraftName(&craftName)) {
         Serial.printf("Craft Name: %s\n", craftName.craftName);
    } else {
         Serial.println("Failed to get Craft Name.");
    }

    // Example: BoxIDs
    msp_boxids_t boxIds;   // Holds the raw ID list
    uint16_t      boxId_count;   // Number of IDs actually returned

    if (msp.requestBoxIDs(&boxIds, &boxId_count)) {
        Serial.printf("Received %u Box IDs:\n", boxId_count);

        for (uint16_t i = 0; i < boxId_count; ++i) {
            Serial.printf("  ID[%u] = %u\n", i, boxIds.ids[i]);
        }
    }
        else {
        Serial.println("Failed to get Box IDs.");
    }

    // Example: Mode Ranges
    msp_mode_ranges_t modeRanges;
    uint16_t          modeRangeCount;

    if (msp.requestModeRanges(&modeRanges, &modeRangeCount)) {
        Serial.printf("Received %u Mode Ranges:\n", modeRangeCount);

        for (uint16_t i = 0; i < modeRangeCount; ++i) {
            // Per documentation, modePermanentId = 0 means the slot is unused.
            if (modeRanges.ranges[i].modePermanentId == 0) continue;

            // Convert steps (0-48) to PWM values (900-2100). Formula: 900 + step * 25
            uint16_t range_start = 900 + (modeRanges.ranges[i].rangeStartStep * 25);
            uint16_t range_end   = 900 + (modeRanges.ranges[i].rangeEndStep * 25);

            Serial.printf("  Range[%u]: ModeID=%u, Aux=%u, Range=[%u, %u]\n",
                i,
                modeRanges.ranges[i].modePermanentId,
                modeRanges.ranges[i].auxChannelIndex + 1, // Print as 1-based (AUX1, AUX2...)
                range_start,
                range_end);
        }
    } else {
        Serial.println("Failed to get Mode Ranges.");
    }


    // Example: Get Waypoint Info
    msp_wp_getinfo_t wpInfo;
    if (msp.requestWaypointInfo(&wpInfo)) {
        Serial.println("--- Waypoint Info ---");
        Serial.printf("  Max WPs: %d\n", wpInfo.maxWaypoints);
        Serial.printf("  Mission Valid: %s\n", wpInfo.missionValid ? "Yes" : "No");
        Serial.printf("  Current WP Count: %d\n", wpInfo.waypointCount);
    } else {
         Serial.println("Failed to get Waypoint Info.");
    }

    // Example: Upload a simple 2-waypoint mission (WP 1: Fly to point, WP 2: RTH)
    // IMPORTANT: Make sure coordinates and altitudes are reasonable for your location!
    // IMPORTANT: This overwrites the existing mission in RAM! Use MSP_WP_MISSION_SAVE to make persistent.
    Serial.println("--- Uploading Sample Mission ---");
    msp_nav_waypoint_t wp;

    // Waypoint 1: Fly to location
    wp.waypointIndex = 1; // Index 1 (0 is Home/RTH position)
    wp.action = 1; // NAV_WP_ACTION_WAYPOINT
    wp.latitude = 407128000; // Example Lat (Times Square * 1e7)
    wp.longitude = -740060000; // Example Lon (Times Square * 1e7)
    wp.altitude = 5000; // 50 meters altitude (cm) relative to home (default flag)
    wp.param1 = 500; // Target speed cm/s (example)
    wp.param2 = 0; // Loiter radius/time (unused for basic WP)
    wp.param3 = 0; // Heading override (0 = auto)
    wp.flag = 0;   // Altitude relative to home, not last WP

    Serial.println("Setting Waypoint 1...");
    if (msp.setWaypoint(&wp)) {
        Serial.println("  Waypoint 1 SET OK");
        printWaypoint(&wp);
    } else {
        Serial.println("  Failed to set Waypoint 1.");
    }
    delay(50); // Small delay between WP sets

     // Waypoint 2: Return To Home (and Land if enabled in RTH settings)
    wp.waypointIndex = 2;
    wp.action = 4; // NAV_WP_ACTION_RTH
    // Lat/Lon/Alt are ignored for RTH action usually
    wp.latitude = 0;
    wp.longitude = 0;
    wp.altitude = 0;
    wp.param1 = 0; // Not used for RTH
    wp.param2 = 0;
    wp.param3 = 0;
    wp.flag = 165; // NAV_WP_FLAG_LAST - Marks the end of the mission

    Serial.println("Setting Waypoint 2 (RTH)...");
     if (msp.setWaypoint(&wp)) {
        Serial.println("  Waypoint 2 SET OK");
        printWaypoint(&wp);
    } else {
        Serial.println("  Failed to set Waypoint 2.");
    }
    delay(50);

    // Optionally save the mission to EEPROM (requires NAV_NON_VOLATILE_WAYPOINT_STORAGE)
    /*
    Serial.println("Saving mission to EEPROM...");
    if (msp.commandMissionSave()) {
        Serial.println("  Mission Save command sent.");
    } else {
        Serial.println("  Failed to send Mission Save command.");
    }
    */

    Serial.println("\n--- Setup Complete - Starting Loop ---");
    lastStatusRequest = millis(); // Request status immediately
    lastGpsRequest = millis();
    lastNavRequest = millis();
}

void loop() {
    unsigned long now = millis();

    // Request Status Periodically
    if (now - lastStatusRequest >= STATUS_INTERVAL) {
        lastStatusRequest = now;
        Serial.println("\nRequesting INAV Status...");
        msp2_inav_status_t status;
        if (msp.requestInavStatus(&status)) {
             Serial.printf("  CycleTime: %d us, CPULoad: %d%%\n", status.cycleTime, status.cpuLoad);
             Serial.printf("  SensorStatus: 0x%04X\n", status.sensorStatus);
             Serial.printf("  Profile: %d, BattProfile: %d, MixerProfile: %d\n",
                           status.profileAndBattProfile & 0x0F, (status.profileAndBattProfile >> 4) & 0x0F, status.mixerProfile);
             Serial.printf("  Arming Flags: 0x%08lX\n", status.armingFlags);
             Serial.printf("  Active Modes: 0x%016llX\n", status.activeModes); // Print full 64-bit mask
             Serial.printf("  Armed: %s\n", msp.isArmed() ? "YES" : "NO");
        } else {
            Serial.println("  Failed to get INAV Status.");
        }

        // Also request analog data
         Serial.println("Requesting Analog Data...");
         msp2_inav_analog_t analog_data;
         if (msp.requestAnalog(&analog_data)) {
             Serial.printf("  VBat: %.2f V, Amps: %.2f A, mAh: %ld, RSSI: %d\n",
                           (float)analog_data.vbat / 100.0f,
                           (float)analog_data.amperage / 100.0f,
                           analog_data.mAhDrawn,
                           analog_data.rssi);
         } else {
             Serial.println("  Failed to get Analog Data.");
         }

        // And attitude
        Serial.println("Requesting Attitude...");
        msp_attitude_t attitude;
        if (msp.requestAttitude(&attitude)) {
            Serial.printf("  Roll: %.1f, Pitch: %.1f, Yaw: %d\n",
                          (float)attitude.roll / 10.0f,
                          (float)attitude.pitch / 10.0f,
                          attitude.yaw);
        } else {
            Serial.println("  Failed to get Attitude.");
        }

        // And altitude
        Serial.println("Requesting Altitude...");
        msp_altitude_t altitude;
        if (msp.requestAltitude(&altitude)) {
             Serial.printf("  Alt: %.2f m, Vario: %.2f m/s\n",
                           (float)altitude.estimatedAltitude / 100.0f,
                           (float)altitude.variometer / 100.0f);//,
                           //(float)altitude.baroAltitude / 100.0f);
        } else {
            Serial.println("  Failed to get Altitude.");
        }
    }

    // Request GPS Periodically
    if (now - lastGpsRequest >= GPS_INTERVAL) {
        lastGpsRequest = now;
        Serial.println("\nRequesting GPS Data...");
        msp_raw_gps_t gps;
        if (msp.requestRawGPS(&gps)) {
            Serial.printf("  Fix: %d, Sats: %d\n", gps.fixType, gps.numSat);
            if (gps.fixType > 0) { // 2D or 3D fix
                Serial.printf("  Lat: %.7f, Lon: %.7f, Alt: %d m\n",
                              (double)gps.latitude / 1e7,
                              (double)gps.longitude / 1e7,
                              gps.altitude); // Alt is meters in MSP_RAW_GPS
                Serial.printf("  Speed: %.2f m/s, Course: %.1f deg, HDOP: %.2f\n",
                              (float)gps.speed / 100.0f,
                              (float)gps.groundCourse / 10.0f,
                              (float)gps.hdop / 100.0f);
            }
        } else {
            Serial.println("  Failed to get GPS Data.");
        }

        // Also request Comp GPS (Distance/Dir to Home)
        msp_comp_gps_t compGps;
        if (msp.requestCompGPS(&compGps)) {
             Serial.printf("  DistHome: %d m, DirHome: %d deg\n",
                           compGps.distanceToHome, compGps.directionToHome);
        } else {
             Serial.println("  Failed to get Comp GPS Data.");
        }
    }

     // Request Navigation Status Periodically
    if (now - lastNavRequest >= NAV_INTERVAL) {
        lastNavRequest = now;
        Serial.println("\nRequesting Nav Status...");
        msp_nav_status_t navStatus;
        if (msp.requestNavStatus(&navStatus)) {
             Serial.printf("  NavMode: %d, NavState: %d, Target WP: %d, NavError: %d, Target Head: %d\n",
                           navStatus.navMode,
                           navStatus.navState,
                           navStatus.activeWpNumber,
                           navStatus.navError,
                           navStatus.targetHeading);
        } else {
             Serial.println("  Failed to get Nav Status.");
        }
    }

    // Example: Send RC Override (Use with EXTREME caution - potential flyaway!)
    // This continuously sends RC channel values. Only uncomment if you know what you are doing
    // and have a way to disable it (e.g., a physical switch checked in the loop).
    // trust me i'm an expert
    
    const bool enable_rc_override = true; // SET TO TRUE ONLY FOR TESTING WITH SAFETY MEASURES
    if (enable_rc_override) {
        uint16_t rc_data[MSP_MAX_RC_CHANNELS] = { 1500 }; // Default all channels to mid-stick

        // Set basic AETR channels (adjust indices based on your RX_MAP setting)
        rc_data[0] = 1500; // Roll
        rc_data[1] = 1500; // Pitch
        rc_data[2] = 1500; // Yaw
        rc_data[3] = 1000; // Throttle (Low)

        // Set an AUX channel (e.g., AUX1 = channel index 4)
        // rc_data[4] = 2000; // High value

        // Send override for the first 8 channels (adjust count as needed)
        uint8_t channels_to_send = 18;
        if (!msp.commandRawRC(rc_data, channels_to_send)) {
            // Serial.println("RC Override Send Failed"); // Avoid flooding serial
        }
         // Need a small delay when sending overrides frequently
        delay(20); // Adjust based on flight controller performance and link speed
    }
    

    // Let the flight controller breathe between requests if loop runs very fast
    delay(1000); // Optional small delay
}