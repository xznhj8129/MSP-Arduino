# Arduino/ESP32 MSP Library for INAV

# Extremely WIP, use at your own risk!

This library provides an interface for communicating with INAV flight controllers using the MultiWii Serial Protocol (MSP). It is designed for use with Arduino-compatible microcontrollers like the ESP32, enabling communication for telemetry display, configuration, mission management, and more.

main.cpp provides the basic example of usage, will change significantly as the proper API between MSP communication and user script is figured out.

This is an enhanced version based on the original MSP library by Fabrizio Di Vittorio. It is meant to use INAV source code directly for enums and defines.

## Features

*   **Extensive Message Support:** Includes structures and functions for many common MSP messages, including:
    *   **Status & Info:** `MSP_API_VERSION`, `MSP_FC_VARIANT`, `MSP_FC_VERSION`, `MSP_BOARD_INFO`, `MSP_BUILD_INFO`, `MSP_UID`, `MSP_NAME`, `MSP_STATUS`, `MSP_STATUS_EX`, `MSP2_INAV_STATUS`.
    *   **Sensors:** `MSP_RAW_IMU`, `MSP_ATTITUDE`, `MSP_ALTITUDE`, `MSP_SONAR_ALTITUDE`, `MSP2_INAV_AIR_SPEED`.
    *   **GPS & Navigation:** `MSP_RAW_GPS`, `MSP_COMP_GPS`, `MSP_NAV_STATUS`.
    *   **Waypoints:** `MSP_WP_GETINFO`, `MSP_WP`, `MSP_SET_WP`, `MSP_WP_MISSION_LOAD`, `MSP_WP_MISSION_SAVE`.
    *   **Configuration:** `MSP_NAV_POSHOLD`, `MSP_SET_NAV_POSHOLD`, `MSP_VOLTAGE_METER_CONFIG`, `MSP_SET_VOLTAGE_METER_CONFIG`, `MSP2_INAV_BATTERY_CONFIG`, `MSP2_INAV_SET_BATTERY_CONFIG`, `MSP_SENSOR_CONFIG`, `MSP_SET_SENSOR_CONFIG`, `MSP_SET_HEAD`.
    *   **Battery:** `MSP_BATTERY_STATE`, `MSP2_INAV_ANALOG`.
    *   **Modes:** `MSP_BOXIDS`,`MSP_MODE_RANGES`.
    *   **RC & Motors:** `MSP_RC`, `MSP_MOTOR`, `MSP_SET_MOTOR`, `MSP_SET_RAW_RC` (use with caution!).
    *   **Programming Framework:** `MSP2_INAV_GVAR_STATUS`, `MSP2_INAV_LOGIC_CONDITIONS_STATUS`.
*   **High-Level API:** Provides convenient functions (`request...`, `set...`, `command...`) wrapping the low-level MSP communication for easier use.
*   **PlatformIO & Arduino IDE Compatible:** The library structure allows use as a local library in PlatformIO or installation into the standard Arduino IDE libraries folder.
*   **ESP32 Tested:** Developed and tested primarily with ESP32, but should work on other Arduino boards with sufficient resources and `Stream` object support.

## Compatibility

*   **Hardware:** ESP32 recommended. Should work on other Arduino-compatible boards supporting the `Stream` class (e.g., ESP8266, SAMD, Teensy) but may require adjustments based on resources (RAM, Flash).
*   **Firmware:** Designed for **INAV** flight controller firmware (recent versions supporting MSPv2). Compatibility with Betaflight, Cleanflight, or MultiWii may vary significantly, especially for INAV-specific messages.

## Project Structure

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

## Important Considerations and Limitations

*   **INAV Protocol Headers REQUIRED:** You *must* provide the `msp_protocol*.h` files from the INAV source code appropriate for your firmware version. Place them in the `include/` folder (PlatformIO) or make them accessible to your sketch (Arduino IDE).
*   **Blocking Communication:** The `recv`, `waitFor`, and `request` functions are blocking. They will wait up to the specified timeout for a reply. This might not be suitable for highly time-sensitive applications without careful task management (e.g., using FreeRTOS on ESP32).
*   **Error Handling:** Error handling is basic (functions typically return `true` on success, `false` on failure/timeout). Check the return values of functions.
*   **Message Completeness:** This library implements many, but not all, MSP messages defined by INAV. Check `MSP.h` and `MSP.cpp` for the currently supported high-level functions. Low-level `send`/`recv` can be used for unsupported messages if you define the payload structures yourself.
*   **Timeout Value:** The default timeout (500ms) might need adjustment depending on the flight controller's load and the serial connection speed. Increase it if you experience frequent timeouts, decrease it for faster failure detection.
*   **RC Override (`commandRawRC`)**: Sending RC commands via MSP bypasses the standard RC receiver. Use this **only** for specific testing scenarios (like controlling a gimbal via ESP32) and **with extreme caution**, as incorrect implementation can easily lead to flyaways or loss of control. Ensure you have robust safety mechanisms (e.g., disable override if a physical RC link is detected).
*   **Configuration Changes:** Setting configuration values (e.g., `setWaypoint`, `setBatteryConfig`) modifies the settings in the flight controller's RAM. You *must* call `msp.commandEepromWrite()` afterwards to make these changes persistent across reboots.

## Protocol Reference

For detailed information on specific MSP message IDs, payloads, and behaviors, refer to the official INAV documentation:

*   [INAV MSP Wiki](https://github.com/iNavFlight/inav/wiki/MSP-V2)
*   [INAV Source Code](https://github.com/iNavFlight/inav/) (Specifically the `src/main/msp/` directory)
*   [INAV MSP Messages reference](https://github.com/iNavFlight/inav/wiki/MSP-Messages-reference)

## License

This library is based on code originally licensed under the GNU Lesser General Public License v2.1 or later. See the header files for details.
```