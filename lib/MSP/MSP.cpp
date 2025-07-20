/*
  MSP.cpp - Arduino library for MSP V2 protocol (focused on INAV)

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

#include <Arduino.h>
#include "MSP.h"

// --- CRC8 DVB-S2 Calculation ---
// (Copied from Betaflight/INAV source)
uint8_t MSP::crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// --- Constructor ---
MSP::MSP() {}

// --- Basic Setup ---
void MSP::begin(Stream &stream, uint32_t timeout)
{
    _stream = &stream;
    _timeout = timeout;
    _last_status_arming_flags = 0; // Reset cached armed status
}

void MSP::reset()
{
    if (!_stream) return;
    _stream->flush(); // Wait for outgoing data to be sent
    // Clear incoming buffer
    unsigned long start = millis();
    while (_stream->available() > 0 && (millis() - start < 100)) { // Timeout failsafe
        _stream->read();
    }
}

// --- Low Level MSP Communication ---

// Send MSP V2 frame ($X<)
void MSP::send(uint16_t messageID, const void *payload, uint16_t size)
{
    if (!_stream) return;

    const uint8_t header[] = {'$', 'X', '<'};
    _stream->write(header, 3);

    uint8_t flag = 0; // Flag is always 0 for requests from GCS/companion
    uint8_t crc = 0;
    uint8_t tmp_buf[2];

    // Flag
    crc = crc8_dvb_s2(crc, flag);
    _stream->write(flag);

    // Message ID (LSB first)
    tmp_buf[0] = messageID & 0xFF;
    tmp_buf[1] = (messageID >> 8) & 0xFF;
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    // Payload Size (LSB first)
    tmp_buf[0] = size & 0xFF;
    tmp_buf[1] = (size >> 8) & 0xFF;
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    // Payload
    if (payload && size > 0) {
        const uint8_t *payloadPtr = (const uint8_t *)payload;
        for (uint16_t i = 0; i < size; ++i) {
            crc = crc8_dvb_s2(crc, payloadPtr[i]);
        }
        _stream->write(payloadPtr, size);
    }

    // Checksum
    _stream->write(crc);
    _stream->flush(); // Ensure data is sent
}

// Receive MSP V2 frame ($X>)
bool MSP::recv(uint16_t *messageID, void *payload, uint16_t maxSize, uint16_t *recvSize)
{
    if (!_stream) return false;

    uint32_t t0 = millis();
    uint8_t state = 0; // 0: Wait $, 1: Wait X, 2: Wait >, 3: Read Flag, 4: Read ID, 5: Read Size, 6: Read Payload, 7: Read CRC
    uint8_t crcCalc = 0;
    uint8_t crcRecv = 0;
    uint16_t id = 0;
    uint16_t payloadSize = 0;
    uint16_t payloadRead = 0;
    uint8_t *payloadPtr = (uint8_t *)payload;

    if (recvSize) *recvSize = 0; // Initialize received size

    while (millis() - t0 < _timeout) {
        if (_stream->available() > 0) {
            uint8_t c = _stream->read();
            // Serial.print((char)c); // Debugging

            switch (state) {
                case 0: // Wait $
                    if (c == '$') state++;
                    break;
                case 1: // Wait X
                    state = (c == 'X') ? state + 1 : 0;
                    break;
                case 2: // Wait >
                    state = (c == '>') ? state + 1 : 0;
                    break;
                case 3: // Read Flag
                    crcCalc = crc8_dvb_s2(0, c); // Initialize CRC with flag
                    state++;
                    break;
                case 4: // Read ID (LSB, MSB)
                    id = c; // LSB
                    crcCalc = crc8_dvb_s2(crcCalc, c);
                    while (_stream->available() == 0 && millis() - t0 < _timeout) {} // Wait for next byte
                    if (_stream->available() == 0) return false; // Timeout
                    c = _stream->read();
                    id |= (uint16_t)c << 8; // MSB
                    crcCalc = crc8_dvb_s2(crcCalc, c);
                    if (messageID) *messageID = id;
                    state++;
                    break;
                case 5: // Read Size (LSB, MSB)
                    payloadSize = c; // LSB
                    crcCalc = crc8_dvb_s2(crcCalc, c);
                     while (_stream->available() == 0 && millis() - t0 < _timeout) {} // Wait for next byte
                    if (_stream->available() == 0) return false; // Timeout
                    c = _stream->read();
                    payloadSize |= (uint16_t)c << 8; // MSB
                    crcCalc = crc8_dvb_s2(crcCalc, c);
                    if (recvSize) *recvSize = payloadSize;
                    payloadRead = 0;
                    state = (payloadSize > 0) ? state + 1 : state + 2; // Skip to CRC if no payload
                     // Reset payload pointer for each message
                    payloadPtr = (uint8_t *)payload;
                    break;
                case 6: // Read Payload
                    if (payloadRead < maxSize && payload) {
                         *(payloadPtr++) = c;
                    } // else discard byte if buffer too small
                    crcCalc = crc8_dvb_s2(crcCalc, c);
                    payloadRead++;
                    if (payloadRead >= payloadSize) {
                        // Zero out remaining buffer if payload was smaller than max size
                        if (payload && payloadRead < maxSize) {
                             memset(payloadPtr, 0, maxSize - payloadRead);
                        }
                        state++;
                    }
                    break;
                case 7: // Read CRC
                    crcRecv = c;
                    // Serial.printf(" | ID: %d, Size: %d, CRC_Recv: %02X, CRC_Calc: %02X\n", id, payloadSize, crcRecv, crcCalc); // Debugging
                    return (crcRecv == crcCalc); // Success if CRC matches
            } // switch(state)
        } // if available
    } // while timeout

    // Serial.println(" | Timeout or Error"); // Debugging
    return false; // Timeout
}


// Wait for a specific message ID
bool MSP::waitFor(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize)
{
    uint16_t recvMessageID;
    uint16_t currentRecvSize;
    uint32_t t0 = millis();

    // Use local recvSize if caller doesn't provide one
    uint16_t *pRecvSize = recvSize ? recvSize : &currentRecvSize;

    while (millis() - t0 < _timeout) {
        if (recv(&recvMessageID, payload, maxSize, pRecvSize)) {
            if (messageID == recvMessageID) {
                return true; // Found the message
            }
            // Received a different message, ignore and continue waiting
        }
         // Allow other tasks to run briefly
        // delay(1); // Consider adding a small delay if needed in cooperative multitasking
    }

    return false; // Timeout
}

// Send a request and wait for the reply with the same message ID
bool MSP::request(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize)
{
    // Clear any stale data before sending
    //reset(); // Optional: Consider if clearing buffer before request is needed

    send(messageID, nullptr, 0); // Requests typically have no payload
    return waitFor(messageID, payload, maxSize, recvSize);
}

// Send a command, optionally wait for ACK (reply with same message ID, zero payload)
bool MSP::command(uint16_t messageID, const void *payload, uint16_t size, bool waitACK)
{
    // Clear any stale data before sending
    //reset(); // Optional: Consider if clearing buffer before command is needed

    send(messageID, payload, size);

    if (waitACK) {
        // For ACK, we expect a reply with the same ID and 0 payload size
        uint16_t ackRecvSize = 0;
        if (waitFor(messageID, nullptr, 0, &ackRecvSize)) {
             // Check if payload size is indeed 0 for a true ACK
             return (ackRecvSize == 0);
        } else {
            return false; // Timeout waiting for ACK
        }
    }

    return true; // Command sent, no ACK requested
}


// --- High Level Functions ---

// Define message IDs if not provided by external includes
#ifndef MSP_API_VERSION
#define MSP_API_VERSION                 1
#define MSP_FC_VARIANT                  2
#define MSP_FC_VERSION                  3
#define MSP_BOARD_INFO                  4
#define MSP_BUILD_INFO                  5
#define MSP_INAV_PID                    6 // Use MSP2_PID instead
#define MSP_SET_INAV_PID                7 // Use MSP2_SET_PID instead
#define MSP_NAME                        10
#define MSP_SET_NAME                    11
#define MSP_NAV_POSHOLD                 12
#define MSP_SET_NAV_POSHOLD             13
#define MSP_CALIBRATION_DATA            14
#define MSP_SET_CALIBRATION_DATA        15
#define MSP_WP_GETINFO                  20
#define MSP_MODE_RANGES                 34
#define MSP_SET_MODE_RANGE              35
#define MSP_FEATURE                     36
#define MSP_SET_FEATURE                 37
#define MSP_BOARD_ALIGNMENT             38
#define MSP_SET_BOARD_ALIGNMENT         39
#define MSP_CURRENT_METER_CONFIG        40
#define MSP_SET_CURRENT_METER_CONFIG    41
#define MSP_MIXER                       42 // Use MSP2_INAV_MIXER
#define MSP_SET_MIXER                   43 // Use MSP2_INAV_SET_MIXER
#define MSP_RX_CONFIG                   44
#define MSP_SET_RX_CONFIG               45
#define MSP_RSSI_CONFIG                 50
#define MSP_SET_RSSI_CONFIG             51
#define MSP_CF_SERIAL_CONFIG            54 // Use MSP2_COMMON_SERIAL_CONFIG
#define MSP_SET_CF_SERIAL_CONFIG        55 // Use MSP2_COMMON_SET_SERIAL_CONFIG
#define MSP_VOLTAGE_METER_CONFIG        56 // Use MSP2_INAV_BATTERY_CONFIG
#define MSP_SET_VOLTAGE_METER_CONFIG    57 // Use MSP2_INAV_SET_BATTERY_CONFIG
#define MSP_SONAR_ALTITUDE              58
#define MSP_RX_MAP                      64
#define MSP_SET_RX_MAP                  65
#define MSP_REBOOT                      68
#define MSP_LOOP_TIME                   73
#define MSP_SET_LOOP_TIME               74
#define MSP_FAILSAFE_CONFIG             75
#define MSP_SET_FAILSAFE_CONFIG         76
#define MSP_SENSOR_CONFIG               96
#define MSP_SET_SENSOR_CONFIG           97
#define MSP_STATUS                      101
#define MSP_RAW_IMU                     102
#define MSP_SERVO                       103
#define MSP_MOTOR                       104
#define MSP_RC                          105
#define MSP_RAW_GPS                     106
#define MSP_COMP_GPS                    107
#define MSP_ATTITUDE                    108
#define MSP_ALTITUDE                    109
#define MSP_ANALOG                      110 // Use MSP2_INAV_ANALOG
#define MSP_RC_TUNING                   111 // Use MSP2_INAV_RATE_PROFILE
#define MSP_PID                         112 // Use MSP2_PID
#define MSP_BOXIDS                      119
#define MSP_SERVO_CONFIGURATIONS        120 // Use MSP2_INAV_SERVO_CONFIG
#define MSP_NAV_STATUS                  121
#define MSP_SET_HEAD                    211
#define MSP_ACC_CALIBRATION             205
#define MSP_MAG_CALIBRATION             206
#define MSP_RESET_CONF                  208
#define MSP_SET_WP                      209
#define MSP_SELECT_SETTING              210
#define MSP_SET_SERVO_CONFIGURATION     212 // Use MSP2_INAV_SET_SERVO_CONFIG
#define MSP_SET_MOTOR                   214
#define MSP_EEPROM_WRITE                250
#define MSP_ACC_TRIM                    240 // Not in INAV fc_msp.c
#define MSP_SET_ACC_TRIM                239 // Not in INAV fc_msp.c
#define MSP_STATUS_EX                   150
#define MSP_SENSOR_STATUS               151
#define MSP_UID                         160
#define MSP_BATTERY_STATE               130
#define MSP_WP_MISSION_LOAD             18
#define MSP_WP_MISSION_SAVE             19
#endif // MSP_API_VERSION

#ifndef MSP2_COMMON_SETTING_INFO
#define MSP2_COMMON_SETTING_INFO        0x1007
#define MSP2_COMMON_SERIAL_CONFIG       0x1009
#define MSP2_COMMON_SET_SERIAL_CONFIG   0x100A
#define MSP2_INAV_STATUS                0x2000
#define MSP2_INAV_ANALOG                0x2002
#define MSP2_INAV_BATTERY_CONFIG        0x2005
#define MSP2_INAV_SET_BATTERY_CONFIG    0x2006
#define MSP2_INAV_RATE_PROFILE          0x2007
#define MSP2_INAV_SET_RATE_PROFILE      0x2008
#define MSP2_INAV_AIR_SPEED             0x2009
#define MSP2_INAV_MIXER                 0x2010
#define MSP2_INAV_SET_MIXER             0x2011
#define MSP2_INAV_GVAR_STATUS           0x2027
#define MSP2_INAV_LOGIC_CONDITIONS_STATUS 0x2026
#define MSP2_PID                        0x2030
#define MSP2_SET_PID                    0x2031
#define MSP2_INAV_SERVO_CONFIG          0x2200
#define MSP2_INAV_SET_SERVO_CONFIG      0x2201
#endif // MSP2_COMMON_SETTING_INFO


// --- Version & Board Info ---
bool MSP::requestApiVersion(msp_api_version_t *reply) {
    return request(MSP_API_VERSION, reply, sizeof(*reply));
}
bool MSP::requestFcVariant(msp_fc_variant_t *reply) {
    return request(MSP_FC_VARIANT, reply, sizeof(*reply));
}
bool MSP::requestFcVersion(msp_fc_version_t *reply) {
    return request(MSP_FC_VERSION, reply, sizeof(*reply));
}
bool MSP::requestBoardInfo(msp_board_info_t *reply, char *targetNameBuf, uint8_t targetNameBufLen) {
     uint16_t recvSize;
     // Request fixed part first
     if (request(MSP_BOARD_INFO, reply, MSP_BOARD_INFO_FIXED_SIZE, &recvSize)) {
         if (recvSize >= MSP_BOARD_INFO_FIXED_SIZE && reply->targetNameLength > 0 && targetNameBuf && targetNameBufLen > 0) {
             uint16_t nameBytesToRead = reply->targetNameLength;
             uint16_t bufferSpace = targetNameBufLen - 1; // Leave space for null terminator
             uint16_t bytesRead = 0;
             uint32_t t0 = millis();

             // Read variable part (target name) directly
             while (bytesRead < nameBytesToRead && millis() - t0 < _timeout) {
                 if (_stream->available()) {
                     char c = _stream->read();
                     if (bytesRead < bufferSpace) {
                         targetNameBuf[bytesRead] = c;
                     }
                     bytesRead++;
                 }
             }
             targetNameBuf[min(bytesRead, bufferSpace)] = '\0'; // Null terminate

             // Now read the final CRC byte which followed the name
             uint8_t dummy_crc;
             uint32_t t1 = millis();
              while (_stream->available() < 1 && millis() - t1 < _timeout) {}
             if (_stream->available() >= 1) {
                _stream->read(); // Consume the CRC byte (validation happened in recv via request->waitFor)
                return true; // Assume success if we got here
             } else {
                return false; // Timeout reading name CRC
             }
         } else if (recvSize >= MSP_BOARD_INFO_FIXED_SIZE) {
             // No target name sent or no buffer provided, but fixed part OK
             if (targetNameBuf && targetNameBufLen > 0) targetNameBuf[0] = '\0';
             return true;
         }
     }
     return false; // Request failed
}
bool MSP::requestBuildInfo(msp_build_info_t *reply) {
    return request(MSP_BUILD_INFO, reply, sizeof(*reply));
}
bool MSP::requestUID(msp_uid_t *reply) {
    return request(MSP_UID, reply, sizeof(*reply));
}
bool MSP::requestCraftName(msp_name_t *reply) {
    uint16_t recvSize;
    // Request name, max buffer size MSP_MAX_NAME_LENGTH
    if (request(MSP_NAME, reply->craftName, MSP_MAX_NAME_LENGTH, &recvSize)) {
        // Null terminate the received string within buffer bounds
        reply->craftName[min((uint16_t)MSP_MAX_NAME_LENGTH, recvSize)] = '\0';
        return true;
    }
    return false;
}
bool MSP::setCraftName(const char *name) {
    if (!name) return false;
    uint16_t len = strlen(name);
    if (len > MSP_MAX_NAME_LENGTH) len = MSP_MAX_NAME_LENGTH;
    return command(MSP_SET_NAME, name, len);
}

// --- Status ---
bool MSP::requestStatus(msp_status_t *reply) {
    return request(MSP_STATUS, reply, sizeof(*reply));
}
bool MSP::requestStatusEx(msp_status_ex_t *reply) {
    return request(MSP_STATUS_EX, reply, sizeof(*reply));
}
bool MSP::requestInavStatus(msp2_inav_status_t *reply) {
    bool success = request(MSP2_INAV_STATUS, reply, sizeof(*reply));
    if (success) {
        _last_status_arming_flags = reply->armingFlags; // Cache for isArmed()
    }
    return success;
}

// Check ARMFLAG_ARMED bit (bit 2) from last cached status
#ifndef ARMFLAG_ARMED
#define ARMFLAG_ARMED 2
#endif
bool MSP::isArmed() {
    return (_last_status_arming_flags & (1UL << ARMFLAG_ARMED)) != 0;
}


// --- Sensor Data ---
bool MSP::requestRawIMU(msp_raw_imu_t *reply) {
    return request(MSP_RAW_IMU, reply, sizeof(*reply));
}
bool MSP::requestAttitude(msp_attitude_t *reply) {
    return request(MSP_ATTITUDE, reply, sizeof(*reply));
}
bool MSP::requestAltitude(msp_altitude_t *reply) {
    return request(MSP_ALTITUDE, reply, sizeof(*reply));
}
bool MSP::requestSonarAltitude(msp_sonar_altitude_t *reply) {
    return request(MSP_SONAR_ALTITUDE, reply, sizeof(*reply));
}
bool MSP::requestAirspeed(msp2_inav_air_speed_t *reply) {
    return request(MSP2_INAV_AIR_SPEED, reply, sizeof(*reply));
}

// --- GPS & Navigation ---
bool MSP::requestRawGPS(msp_raw_gps_t *reply) {
    return request(MSP_RAW_GPS, reply, sizeof(*reply));
}
bool MSP::requestCompGPS(msp_comp_gps_t *reply) {
    return request(MSP_COMP_GPS, reply, sizeof(*reply));
}
bool MSP::requestNavStatus(msp_nav_status_t *reply) {
    return request(MSP_NAV_STATUS, reply, sizeof(*reply));
}
bool MSP::setHeading(int16_t headingDeg) {
    // Ensure heading is within 0-359 range if necessary, though FC might handle wrap-around.
    uint16_t heading = (uint16_t)constrain(headingDeg, 0, 359);
    msp_set_head_t payload;
    payload.heading = heading;
    return command(MSP_SET_HEAD, &payload, sizeof(payload));
}

// --- Waypoints & Missions ---
bool MSP::requestWaypointInfo(msp_wp_getinfo_t *reply) {
    // WP_GETINFO is MSPv1 Out only, no payload needed for request
    return request(MSP_WP_GETINFO, reply, sizeof(*reply));
}
bool MSP::requestWaypoint(uint8_t index, msp_nav_waypoint_t *reply) {
    // Request is index, reply is full waypoint struct
    uint8_t payload = index;
    // Note: MSP_WP uses same ID for request and reply
    if (waitFor(MSP_WP, reply, sizeof(*reply))) {
        // Check if the returned index matches the requested one
        return (reply->waypointIndex == index);
    }
    // Send request if waitFor failed (e.g., first request)
    send(MSP_WP, &payload, sizeof(payload));
    if (waitFor(MSP_WP, reply, sizeof(*reply))) {
         return (reply->waypointIndex == index);
    }
    return false;
}

bool MSP::setWaypoint(const msp_nav_waypoint_t *wp) {
    if (!wp) return false;
    // Check if index is valid (optional, FC validates too)
    // if (wp->waypointIndex >= MSP_MAX_WAYPOINTS) return false; // Assuming MSP_MAX_WAYPOINTS is defined
    return command(MSP_SET_WP, wp, sizeof(*wp));
}

bool MSP::commandMissionLoad(uint8_t missionId) {
    msp_mission_id_t payload;
    payload.missionID = missionId;
    // This command might take time, ACK might not be immediate?
    // Check INAV source if ACK behavior is standard. Assuming standard ACK.
    return command(MSP_WP_MISSION_LOAD, &payload, sizeof(payload));
}

bool MSP::commandMissionSave(uint8_t missionId) {
    msp_mission_id_t payload;
    payload.missionID = missionId;
     // This command might take time (EEPROM write). Increase timeout if needed?
    return command(MSP_WP_MISSION_SAVE, &payload, sizeof(payload));
}


// --- Modes ---
bool MSP::requestBoxIDs(msp_boxids_t *reply, uint16_t *count) {
     if (!reply || !count) return false;
     uint16_t recvSize;
     if (request(MSP_BOXIDS, reply->ids, sizeof(reply->ids), &recvSize)) {
         *count = recvSize; // Number of IDs received
         return true;
     }
     *count = 0;
     return false;
}


// --- RC & Motors ---
bool MSP::requestRcChannels(msp_rc_channels_t *reply, uint8_t expectedChannels) {
    uint16_t recvSize;
    // Request RC data
    if (request(MSP_RC, reply->channel, sizeof(uint16_t) * expectedChannels, &recvSize)) {
        // Verify we received at least some channels (size = num_channels * 2)
        return (recvSize > 0 && (recvSize % 2 == 0));
    }
    return false;
}

bool MSP::commandRawRC(const uint16_t *channels, uint8_t channelCount) {
    if (!channels || channelCount == 0 || channelCount > MSP_MAX_RC_CHANNELS) return false;
    // MSP_SET_RAW_RC typically doesn't expect an ACK in high-frequency use cases
    // Set waitACK to false if needed. Check INAV source for actual behavior.
    return command(200 /*MSP_SET_RAW_RC*/, channels, sizeof(uint16_t) * channelCount, false); // Send without waiting for ACK
}

bool MSP::requestMotorOutputs(msp_motor_outputs_t *reply) {
    return request(MSP_MOTOR, reply, sizeof(*reply));
}

bool MSP::commandMotorOutputs(const uint16_t *motorValues) {
     if (!motorValues) return false;
     // Ensure we send exactly 8 motor values as expected by MSP_SET_MOTOR
     uint16_t payload[MSP_MAX_SUPPORTED_MOTORS];
     memcpy(payload, motorValues, sizeof(payload)); // Assumes input has at least 8 values
     // MSP_SET_MOTOR doesn't usually have an ACK
     return command(MSP_SET_MOTOR, payload, sizeof(payload), false);
}

// --- Configuration ---
bool MSP::requestNavPosholdConfig(msp_nav_poshold_config_t *reply) {
    return request(MSP_NAV_POSHOLD, reply, sizeof(*reply));
}
bool MSP::setNavPosholdConfig(const msp_nav_poshold_config_t *config) {
    if (!config) return false;
    // Size check: INAV expects exactly 13 bytes for V1 SET_NAV_POSHOLD
    if (sizeof(*config) != 13) { /* Handle error or assert */ return false; }
    return command(MSP_SET_NAV_POSHOLD, config, sizeof(*config));
}
bool MSP::requestVoltageMeterConfig(msp_voltage_meter_config_t *reply){
    return request(MSP_VOLTAGE_METER_CONFIG, reply, sizeof(*reply));
}
bool MSP::setVoltageMeterConfig(const msp_voltage_meter_config_t *config){
    if (!config) return false;
    // Size check: INAV expects exactly 4 bytes for V1 SET_VOLTAGE_METER_CONFIG
    if (sizeof(*config) != 4) { /* Handle error or assert */ return false; }
    return command(MSP_SET_VOLTAGE_METER_CONFIG, config, sizeof(*config));
}
bool MSP::requestBatteryConfig(msp2_inav_battery_config_t *reply){
    return request(MSP2_INAV_BATTERY_CONFIG, reply, sizeof(*reply));
}
bool MSP::setBatteryConfig(const msp2_inav_battery_config_t *config){
     if (!config) return false;
     // Size check: INAV expects exactly 29 bytes for V2 SET_BATTERY_CONFIG
    if (sizeof(*config) != 29) { /* Handle error or assert */ return false; }
    return command(MSP2_INAV_SET_BATTERY_CONFIG, config, sizeof(*config));
}
bool MSP::requestSensorConfig(msp_sensor_config_t *reply){
     return request(MSP_SENSOR_CONFIG, reply, sizeof(*reply));
}
bool MSP::setSensorConfig(const msp_sensor_config_t *config){
    if (!config) return false;
    // Size check: INAV expects exactly 6 bytes for SET_SENSOR_CONFIG
    if (sizeof(*config) != 6) { /* Handle error or assert */ return false; }
    return command(MSP_SET_SENSOR_CONFIG, config, sizeof(*config));
}


// --- Battery & Power ---
bool MSP::requestBatteryState(msp_battery_state_t *reply){
    return request(MSP_BATTERY_STATE, reply, sizeof(*reply));
}
bool MSP::requestAnalog(msp2_inav_analog_t *reply){
    return request(MSP2_INAV_ANALOG, reply, sizeof(*reply));
}


// --- Calibration & System ---
/*bool MSP::commandAccCalibration() {
    // No payload, expect ACK
    return command(MSP_ACC_CALIBRATION, nullptr, 0);
}
bool MSP::commandMagCalibration() {
    // No payload, expect ACK
    return command(MSP_MAG_CALIBRATION, nullptr, 0);
}
bool MSP::commandResetConfig() {
    // No payload, potentially long operation, ACK might be delayed or sent before reset?
    // Test required. Assuming standard ACK for now.
    return command(MSP_RESET_CONF, nullptr, 0);
}
bool MSP::commandEepromWrite() {
    // No payload, potentially long operation (EEPROM write).
    return command(MSP_EEPROM_WRITE, nullptr, 0);
} 
//Hell no.
*/


// --- Programming Framework ---
bool MSP::requestGvarStatus(msp2_inav_gvar_status_t *reply){
    return request(MSP2_INAV_GVAR_STATUS, reply, sizeof(*reply));
}
bool MSP::requestLogicConditionsStatus(msp2_inav_logic_conditions_status_t *reply){
    return request(MSP2_INAV_LOGIC_CONDITIONS_STATUS, reply, sizeof(*reply));
}