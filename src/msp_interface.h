/*
  MSP.h - Arduino library for MSP V2 protocol (focused on INAV)

  Based on original work by Fabrizio Di Vittorio (fdivitto2013@gmail.com)
  Extended and refactored for wider INAV MSP support.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include <Arduino.h>
#include <Stream.h>
#include "inav_defs.h"
#include "inav_enums.h"
#include "msp_structs.h"
#include "msp_protocol.h"
#include "msp_protocol_v2_common.h"
#include "msp_protocol_v2_inav.h"
#include "msp_protocol_v2_sensor.h"
#include "msp_protocol_v2_sensor_msg.h"


class MSPIntf {
  public:
    // --- Constructor & Basic Setup ---
    MSPIntf();
    void begin(Stream &stream, uint32_t timeout = 500);
    void reset(); // Clears serial buffer

    // --- Low Level MSP Communication ---
    void send(uint16_t messageID, const void *payload = nullptr, uint16_t size = 0);
    bool recv(uint16_t *messageID, void *payload, uint16_t maxSize, uint16_t *recvSize = nullptr);
    bool waitFor(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize = nullptr);
    bool request(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize = nullptr);
    bool command(uint16_t messageID, const void *payload = nullptr, uint16_t size = 0, bool waitACK = true);

    // --- High Level Functions ---

    // Version & Board Info
    bool requestApiVersion(msp_api_version_t *reply);
    bool requestFcVariant(msp_fc_variant_t *reply);
    bool requestFcVersion(msp_fc_version_t *reply);
    bool requestBoardInfo(msp_board_info_t *reply, char *targetNameBuf = nullptr, uint8_t targetNameBufLen = 0); // Handles variable length name
    bool requestBuildInfo(msp_build_info_t *reply);
    bool requestUID(msp_uid_t *reply);
    bool requestCraftName(msp_name_t *reply);
    bool setCraftName(const char *name);

    // Status
    bool requestStatus(msp_status_t *reply); // Legacy status
    bool requestStatusEx(msp_status_ex_t *reply); // Extended V1 status
    bool requestInavStatus(msp2_inav_status_t *reply); // Recommended modern status
    bool isArmed(); // Checks armed status based on last requestInavStatus

    // Sensor Data
    bool requestRawIMU(msp_raw_imu_t *reply);
    bool requestAttitude(msp_attitude_t *reply);
    bool requestAltitude(msp_altitude_t *reply);
    bool requestSonarAltitude(msp_sonar_altitude_t *reply); // Rangefinder
    bool requestAirspeed(msp2_inav_air_speed_t *reply); // Pitot/Estimated

    // GPS & Navigation
    bool requestRawGPS(msp_raw_gps_t *reply);
    bool requestCompGPS(msp_comp_gps_t *reply); // Distance/Direction to home
    bool requestNavStatus(msp_nav_status_t *reply);
    bool setHeading(int16_t headingDeg); // Sets MAGHOLD target

    // Waypoints & Missions
    bool requestWaypointInfo(msp_wp_getinfo_t *reply);
    bool requestWaypoint(uint8_t index, msp_nav_waypoint_t *reply);
    bool setWaypoint(const msp_nav_waypoint_t *wp);
    bool commandMissionLoad(uint8_t missionId = 0);
    bool commandMissionSave(uint8_t missionId = 0);

    // Modes
    bool requestBoxIDs(msp_boxids_t *reply, uint16_t *count); // Get permanent IDs
    // getActiveModes is complex due to bitmask size, better handled in sketch using requestInavStatus
    bool requestModeRanges(msp_mode_ranges_t *reply, uint16_t *count); // Get mode activation ranges

    // RC & Motors
    bool requestRcChannels(msp_rc_channels_t *reply, uint8_t expectedChannels); // Specify max channels expected
    bool commandRawRC(const uint16_t *channels, uint8_t channelCount);
    bool requestMotorOutputs(msp_motor_outputs_t *reply);
    bool commandMotorOutputs(const uint16_t *motorValues); // For motor testing

    // Configuration
    // We will *not* implement configuration messages
    bool requestNavPosholdConfig(msp_nav_poshold_config_t *reply);
    //bool setNavPosholdConfig(const msp_nav_poshold_config_t *config);
    bool requestVoltageMeterConfig(msp_voltage_meter_config_t *reply); // Legacy
    //bool setVoltageMeterConfig(const msp_voltage_meter_config_t *config); // Legacy
    bool requestBatteryConfig(msp2_inav_battery_config_t *reply); // Modern
    //bool setBatteryConfig(const msp2_inav_battery_config_t *config); // Modern
    bool requestSensorConfig(msp_sensor_config_t *reply);
    //bool setSensorConfig(const msp_sensor_config_t *config);

    // Battery & Power
    bool requestBatteryState(msp_battery_state_t *reply);
    bool requestAnalog(msp2_inav_analog_t *reply); // Modern analog data

    // Calibration
    /*bool commandAccCalibration();
    bool commandMagCalibration();
    bool commandResetConfig(); // Reset to defaults
    bool commandEepromWrite(); // Save current config*/
    // Hell no

    // Programming Framework (Example)
    bool requestGvarStatus(msp2_inav_gvar_status_t *reply);
    bool requestLogicConditionsStatus(msp2_inav_logic_conditions_status_t *reply);


  private:
    Stream *_stream = nullptr;
    uint32_t _timeout = 500;
    uint32_t _last_status_arming_flags = 0; // Cache for isArmed()

    static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
};