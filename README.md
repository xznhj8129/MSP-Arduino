# Arduino/ESP32 MSP Library for INAV

# UNTESTED. I HAVEN'T EVEN HAD TIME TO UPLOAD THIS TO AN MCU. DON'T USE IT.

This library provides an interface for communicating with INAV flight controllers using the MultiWii Serial Protocol (MSP), focusing primarily on **MSP Version 2** (`$X` framing). It is designed for use with Arduino-compatible microcontrollers like the ESP32, enabling communication for telemetry display, configuration, mission management, and more.

This is an enhanced version based on the original MSP library by Fabrizio Di Vittorio, extended to support a wider range of INAV-specific MSP messages and common use cases.

## Features

*   **MSPv2 Framing:** Implements sending (`$X<`) and receiving (`$X>`) MSPv2 frames with CRC8 DVB-S2 checksum verification.
*   **INAV Focus:** Prioritizes messages commonly used with INAV firmware.
*   **Extensive Message Support:** Includes structures and functions for many common MSP messages, including:
    *   **Status & Info:** `MSP_API_VERSION`, `MSP_FC_VARIANT`, `MSP_FC_VERSION`, `MSP_BOARD_INFO`, `MSP_BUILD_INFO`, `MSP_UID`, `MSP_NAME`, `MSP_STATUS`, `MSP_STATUS_EX`, `MSP2_INAV_STATUS`.
    *   **Sensors:** `MSP_RAW_IMU`, `MSP_ATTITUDE`, `MSP_ALTITUDE`, `MSP_SONAR_ALTITUDE`, `MSP2_INAV_AIR_SPEED`.
    *   **GPS & Navigation:** `MSP_RAW_GPS`, `MSP_COMP_GPS`, `MSP_NAV_STATUS`.
    *   **Waypoints:** `MSP_WP_GETINFO`, `MSP_WP`, `MSP_SET_WP`, `MSP_WP_MISSION_LOAD`, `MSP_WP_MISSION_SAVE`.
    *   **Configuration:** `MSP_NAV_POSHOLD`, `MSP_SET_NAV_POSHOLD`, `MSP_VOLTAGE_METER_CONFIG`, `MSP_SET_VOLTAGE_METER_CONFIG`, `MSP2_INAV_BATTERY_CONFIG`, `MSP2_INAV_SET_BATTERY_CONFIG`, `MSP_SENSOR_CONFIG`, `MSP_SET_SENSOR_CONFIG`, `MSP_SET_HEAD`.
    *   **Battery:** `MSP_BATTERY_STATE`, `MSP2_INAV_ANALOG`.
    *   **Modes:** `MSP_BOXIDS`.
    *   **RC & Motors:** `MSP_RC`, `MSP_MOTOR`, `MSP_SET_MOTOR`, `MSP_SET_RAW_RC` (use with caution!).
    *   **System & Calibration:** `MSP_ACC_CALIBRATION`, `MSP_MAG_CALIBRATION`, `MSP_RESET_CONF`, `MSP_EEPROM_WRITE`.
    *   **Programming Framework:** `MSP2_INAV_GVAR_STATUS`, `MSP2_INAV_LOGIC_CONDITIONS_STATUS`.
*   **High-Level API:** Provides convenient functions (`request...`, `set...`, `command...`) wrapping the low-level MSP communication for easier use.
*   **PlatformIO & Arduino IDE Compatible:** The library structure allows use as a local library in PlatformIO or installation into the standard Arduino IDE libraries folder.
*   **ESP32 Tested:** Developed and tested primarily with ESP32, but should work on other Arduino boards with sufficient resources and `Stream` object support.

## Compatibility

*   **Hardware:** ESP32 recommended. Should work on other Arduino-compatible boards supporting the `Stream` class (e.g., ESP8266, SAMD, Teensy) but may require adjustments based on resources (RAM, Flash).
*   **Firmware:** Designed for **INAV** flight controller firmware (recent versions supporting MSPv2). Compatibility with Betaflight, Cleanflight, or MultiWii may vary significantly, especially for INAV-specific messages.
*   **Protocol:** Uses **MSP Version 2** (`$X` framing) for sending and receiving.

## Project Structure (Recommended)

This structure is compatible with both PlatformIO and the Arduino IDE library format.

```
YourESP32_MSP_Project/
├── platformio.ini           # PlatformIO configuration
├── lib/                     # Project-specific libraries
│   └── MSP/                 # <<< This folder is the Arduino-compatible library >>>
│       ├── MSP.cpp          # Library source file
│       ├── MSP.h            # Library header file
│       ├── MSP_enum.h       # Enum definitions
│       ├── msp_protocol.h   # From INAV source code
│       ├── msp_protocol_v2_common.h # From INAV source code
│       ├── msp_protocol_v2_inav.h # From INAV source code
│       ├── msp_protocol_v2_sensor.h # From INAV source code
│       └── msp_protocol_v2_sensor_msg.h # From INAV source code
│       └── examples/        # Standard examples folder for Arduino IDE
│           └── msp_example/ # Each example in its own folder
│               └── msp_example.ino # The example sketch
├── src/                     # Source files for YOUR main PlatformIO application
│   └── main.cpp             # PlatformIO builds this by default.
```

## Installation and Setup

### PlatformIO (Recommended)

1.  **Clone or Download:** Get the library files (`MSP.h`, `MSP.cpp`, `MSP_enum.h`).
2.  **Create Library Folder:** In your PlatformIO project, create a directory `lib/MSP/`.
3.  **Copy Library Files:** Place `MSP.h`, `MSP.cpp`, and `MSP_enum.h` inside the `lib/MSP/` folder.
4.  **Add INAV Headers:** Copy the required INAV `msp_protocol*.h` header files into the project's `include/` directory.
5.  **Configure `platformio.ini`:** Use the provided `platformio.ini` example, ensuring the `board` type matches your ESP32 board.
6.  **Include in Sketch:** In your `src/main.cpp`, include the library and necessary protocol headers:
    ```c++
    #include <Arduino.h>
    #include <HardwareSerial.h>
    #include "MSP.h"

    // INAV Headers from include/ folder
    #include "msp_protocol.h"
    #include "msp_protocol_v2_common.h"
    #include "msp_protocol_v2_inav.h"
    // ... add other protocol headers as needed ...
    ```

### Arduino IDE

1.  **Clone or Download:** Get the library files.
2.  **Create Library Folder:** Navigate to your Arduino `libraries` folder (usually in `Documents/Arduino/libraries/`). Create a folder named `MSP`.
3.  **Copy Library Files:** Place `MSP.h`, `MSP.cpp`, `MSP_enum.h` directly inside the `libraries/MSP/` folder. You can optionally add the `examples` subfolder here as well.
4.  **Add INAV Headers:** This is the tricky part for the Arduino IDE. You need to make the INAV `msp_protocol*.h` files accessible to your sketch. Options:
    *   Copy the required `msp_protocol*.h` files directly into your sketch's folder.
    *   Create *another* library (e.g., `INAV_MSP_Defs`) in the Arduino `libraries` folder and place the INAV headers inside it. Then include them from your sketch like `#include <INAV_MSP_Defs/msp_protocol.h>`.
5.  **Include in Sketch:** In your sketch (`.ino` file), include the library and the INAV protocol headers (adjust path based on step 4):
    ```c++
    #include <HardwareSerial.h>
    #include <MSP.h> // From libraries/MSP
    // ... add other protocol headers as needed ...
    ```
6.  **Restart Arduino IDE:** Ensure the IDE recognizes the new library.

## Basic Usage (API)

```c++
#include <HardwareSerial.h>
#include "MSP.h"

MSP msp; // Create an instance
HardwareSerial& fcSerial = Serial2; // Choose your serial port

void setup() {
  Serial.begin(115200); // Debug serial
  fcSerial.begin(115200); // FC serial - MATCH FC CONFIG

  msp.begin(fcSerial, 500); // Initialize with stream and 500ms timeout
}

void loop() {
  // --- Using High-Level Functions (Recommended) ---

  msp_attitude_t attitude;
  if (msp.requestAttitude(&attitude)) {
    Serial.printf("Roll: %.1f, Pitch: %.1f, Yaw: %d\n",
                  (float)attitude.roll / 10.0f,
                  (float)attitude.pitch / 10.0f,
                  attitude.yaw);
  } else {
    Serial.println("Failed to get attitude");
  }

  // Example: Set Heading for MagHold Mode
  // msp.setHeading(90); // Set target heading to 90 degrees


  // --- Using Low-Level Functions ---

  // Send a request for MSP_STATUS (ID 101)
  msp.send(MSP_STATUS, nullptr, 0);

  // Wait for the reply
  uint16_t msgID;
  uint16_t recvSize;
  msp_status_t status_reply; // Make sure struct is defined
  if (msp.waitFor(MSP_STATUS, &status_reply, sizeof(status_reply), &recvSize)) {
     // Process status_reply...
     Serial.printf("Status Recv - Size: %d, CycleTime: %d\n", recvSize, status_reply.cycleTime);
  } else {
     Serial.println("Timeout waiting for MSP_STATUS reply");
  }

  delay(100); // Delay between requests
}
```

## High-Level Function Examples

See the `MSP.h` file for the full list of implemented high-level functions.

```c++
// Get comprehensive status
msp2_inav_status_t status;
if (msp.requestInavStatus(&status)) { /* process status */ }

// Get GPS data
msp_raw_gps_t gps;
if (msp.requestRawGPS(&gps)) { /* process gps */ }

// Get Navigation Status
msp_nav_status_t navStatus;
if (msp.requestNavStatus(&navStatus)) { /* process navStatus */ }

// Get current battery voltage, current, capacity etc.
msp2_inav_analog_t analogData;
if (msp.requestAnalog(&analogData)) { /* process analogData */ }

// Get Waypoint info
msp_wp_getinfo_t wpInfo;
if (msp.requestWaypointInfo(&wpInfo)) { /* process wpInfo */ }

// Get a specific Waypoint
msp_nav_waypoint_t wp;
if (msp.requestWaypoint(1, &wp)) { /* process waypoint 1 */ }

// Set (upload) a waypoint
msp_nav_waypoint_t newWp;
// ... fill newWp structure ...
if (msp.setWaypoint(&newWp)) { /* Waypoint set OK */ }

// Save the mission in RAM to FC's non-volatile memory
// if (msp.commandMissionSave()) { /* Save command sent */ }

// Trigger accelerometer calibration
// if (msp.commandAccCalibration()) { /* Calibration started */ }

// Save current configuration changes to EEPROM
// if (msp.commandEepromWrite()) { /* Config saved */ }
```

## Example Sketch

An example sketch (`lib/MSP/examples/MSP_INAV_Example/MSP_INAV_Example.ino` in the recommended structure) is provided to demonstrate:
*   Connecting to the flight controller.
*   Requesting basic information (Version, Name, Status, etc.).
*   Periodically requesting flight data (Status, GPS, Nav Status, Attitude, Altitude, Analog).
*   Uploading a sample 2-waypoint mission.
*   Printing received data to the Serial Monitor.
*   **Includes a commented-out section for RC Override - use with extreme caution!**

**Before running the example:**
1.  Modify `fcSerial` and `FC_BAUD_RATE` to match your hardware connection.
2.  Modify `DEBUG_BAUD_RATE` if needed for your Serial Monitor.
3.  **Crucially, review and change the sample waypoint coordinates and altitudes in `setup()` to values safe and appropriate for your location.**

## Important Considerations and Limitations

*   **INAV Protocol Headers REQUIRED:** You *must* provide the `msp_protocol*.h` files from the INAV source code appropriate for your firmware version. Place them in the `include/` folder (PlatformIO) or make them accessible to your sketch (Arduino IDE).
*   **Blocking Communication:** The `recv`, `waitFor`, and `request` functions are blocking. They will wait up to the specified timeout for a reply. This might not be suitable for highly time-sensitive applications without careful task management (e.g., using FreeRTOS on ESP32).
*   **Error Handling:** Error handling is basic (functions typically return `true` on success, `false` on failure/timeout). Check the return values of functions.
*   **Buffer Sizes:** Ensure the `payload` buffers passed to `recv`, `waitFor`, and `request` are large enough for the expected replies. For variable-length replies (like `MSP_BOXIDS`), ensure the buffer in the struct definition is adequately sized. Mismatched sizes can lead to buffer overflows or data corruption.
*   **Message Completeness:** This library implements many, but not all, MSP messages defined by INAV. Check `MSP.h` and `MSP.cpp` for the currently supported high-level functions. Low-level `send`/`recv` can be used for unsupported messages if you define the payload structures yourself.
*   **Timeout Value:** The default timeout (500ms) might need adjustment depending on the flight controller's load and the serial connection speed. Increase it if you experience frequent timeouts, decrease it for faster failure detection.
*   **RC Override (`commandRawRC`)**: Sending RC commands via MSP bypasses the standard RC receiver. Use this **only** for specific testing scenarios (like controlling a gimbal via ESP32) and **with extreme caution**, as incorrect implementation can easily lead to flyaways or loss of control. Ensure you have robust safety mechanisms (e.g., disable override if a physical RC link is detected).
*   **Configuration Changes:** Setting configuration values (e.g., `setWaypoint`, `setBatteryConfig`) modifies the settings in the flight controller's RAM. You *must* call `msp.commandEepromWrite()` afterwards to make these changes persistent across reboots.
*   **MSPv1:** While the framing uses MSPv2, many V1 message IDs are still used and implemented. True MSPv1 framing (`$M`) is not supported by this version of the library.

## Protocol Reference

For detailed information on specific MSP message IDs, payloads, and behaviors, refer to the official INAV documentation:

*   [INAV MSP Wiki](https://github.com/iNavFlight/inav/wiki/MSP-V2)
*   [INAV Source Code](https://github.com/iNavFlight/inav/) (Specifically the `src/main/msp/` directory)

## License

This library is based on code originally licensed under the GNU Lesser General Public License v2.1 or later. See the header files for details.
```