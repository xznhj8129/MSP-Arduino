#pragma once
#include "inav_defs.h"

// --- Core MSP Structures (Additions/Refinements based on reference) ---

// Generic reply structure for commands that only return success/failure or basic info
struct msp_ack_t {
    // Often empty, sometimes contains a status byte or the original command ID.
    // Use specific structs when detailed replies are expected.
};

// MSP_API_VERSION reply
struct msp_api_version_t {
  uint8_t mspProtocolVersion;
  uint8_t apiVersionMajor;
  uint8_t apiVersionMinor;
} __attribute__ ((packed));

// MSP_FC_VARIANT reply
struct msp_fc_variant_t {
  char fcVariantIdentifier[4];
} __attribute__ ((packed));

// MSP_FC_VERSION reply
struct msp_fc_version_t {
  uint8_t fcVersionMajor;
  uint8_t fcVersionMinor;
  uint8_t fcVersionPatch;
} __attribute__ ((packed));

// MSP_BOARD_INFO reply
// Note: Variable length targetName needs special handling in receiver function if used
struct msp_board_info_t {
  char     boardIdentifier[4];
  uint16_t hardwareRevision;
  uint8_t  osdSupport; // 0=None, 2=Onboard
  uint8_t  commCapabilities; // Bit 0=VCP, Bit 1=SoftSerial
  uint8_t  targetNameLength;
  // char targetName[]; // Follows immediately - requires custom read
} __attribute__ ((packed));

// MSP_BUILD_INFO reply
struct msp_build_info_t {
  char buildDate[11];
  char buildTime[8];
  char gitRevision[7];
} __attribute__ ((packed));

// MSP_NAME reply / MSP_SET_NAME command
// Note: Variable length, handle with care
struct msp_name_t {
    char craftName[MAX_NAME_LENGTH + 1]; // Buffer to receive/send name
};

// MSP_STATUS reply (Legacy - less info than EX/MSP2)
struct msp_status_t {
  uint16_t cycleTime;
  uint16_t i2cErrors;
  uint16_t sensorStatus; // Bitmask: 0:ACC, 1:BARO, 2:MAG, 3:GPS, 4:RANGEFINDER, 5:GYRO
  uint32_t activeModesLow; // First 32 modes bitmask
  uint8_t  profile;
} __attribute__ ((packed));

// MSP_STATUS_EX reply (Extended V1 status)
struct msp_status_ex_t {
  uint16_t cycleTime;
  uint16_t i2cErrors;
  uint16_t sensorStatus;
  uint32_t activeModesLow;
  uint8_t  profile;
  uint16_t cpuLoad;
  uint16_t armingFlags; // Note: Truncated to 16 bits here
  uint8_t  accCalibAxisFlags;
} __attribute__ ((packed));

// MSP2_INAV_STATUS reply
typedef uint64_t boxBitmask_t; // Assuming 64 modes max for safety, check firmware if > 64 needed
struct msp2_inav_status_t {
  uint16_t cycleTime;
  uint16_t i2cErrors;
  uint16_t sensorStatus;
  uint16_t cpuLoad;
  uint8_t  profileAndBattProfile; // Bits 0-3: Config profile, Bits 4-7: Battery profile
  uint32_t armingFlags; // Full 32-bit arming flags
  boxBitmask_t activeModes; // Full mode bitmask
  uint8_t  mixerProfile;
} __attribute__ ((packed));

// MSP_RAW_IMU reply
struct msp_raw_imu_t {
  int16_t accX, accY, accZ; // Scaled ~1/512 G
  int16_t gyroX, gyroY, gyroZ;
  int16_t magX, magY, magZ; // Raw units
} __attribute__ ((packed));

// MSP_ATTITUDE reply
struct msp_attitude_t {
  int16_t roll;  // deci-degrees
  int16_t pitch; // deci-degrees
  int16_t yaw;   // degrees (converted in FC)
} __attribute__ ((packed));

// MSP_ALTITUDE reply
struct msp_altitude_t {
  int32_t estimatedAltitude; // cm
  int16_t variometer;        // cm/s
  int32_t baroAltitude;      // cm 
} __attribute__ ((packed));

// MSP_SONAR_ALTITUDE reply
struct msp_sonar_altitude_t {
  uint32_t rangefinderAltitude; // cm
} __attribute__ ((packed));

// MSP2_INAV_AIR_SPEED reply
struct msp2_inav_air_speed_t {
    uint32_t airspeed; // cm/s
} __attribute__ ((packed));

// MSP_RAW_GPS reply
struct msp_raw_gps_t {
  uint8_t  fixType;       // enum: 0=NoFix, 1=2D, 2=3D, etc.
  uint8_t  numSat;
  int32_t latitude;      // deg * 1e7
  int32_t longitude;     // deg * 1e7
  int16_t altitude;      // meters (MSPv1 specific)
  uint16_t speed;         // cm/s
  uint16_t groundCourse;  // deci-degrees (0-3599)
  uint16_t hdop;          // HDOP * 100
} __attribute__ ((packed));

// MSP_COMP_GPS reply
struct msp_comp_gps_t {
  uint16_t distanceToHome;  // meters
  uint16_t directionToHome; 
  uint8_t  gpsHeartbeat;    // Toggles
} __attribute__ ((packed));

// MSP_NAV_STATUS reply
struct msp_nav_status_t {
  uint8_t navMode;        // enum NAV_MODE_*
  uint8_t navState;       // enum NAV_STATE_*
  uint8_t activeWpAction; // enum navWaypointAction_e
  uint8_t activeWpNumber; // Index
  uint8_t navError;       // enum NAV_ERROR_*
  int16_t targetHeading; // degrees
} __attribute__ ((packed));

// MSP_WP_GETINFO reply
struct msp_wp_getinfo_t {
  uint8_t wpCapabilities; // Reserved, usually 0
  uint8_t maxWaypoints;   // NAV_MAX_WAYPOINTS
  uint8_t missionValid;   // Boolean
  uint8_t waypointCount;  // Current count
} __attribute__ ((packed));

// MSP_WP / MSP_SET_WP structure
struct msp_nav_waypoint_t {
  uint8_t  waypointIndex; // Index (0 to NAV_MAX_WAYPOINTS - 1)
  uint8_t  action;        // enum navWaypointAction_e
  int32_t latitude;      // deg * 1e7
  int32_t longitude;     // deg * 1e7
  int32_t altitude;      // cm (ref depends on flag)
  uint16_t param1;        // Varies with action
  uint16_t param2;        // Varies with action
  uint16_t param3;        // Varies with action
  uint8_t  flag;          // Bitmask NAV_WP_FLAG_* (e.g., altitude ref, last WP)
} __attribute__ ((packed));

// MSP_WP_MISSION_LOAD / MSP_WP_MISSION_SAVE command payload
struct msp_mission_id_t {
  uint8_t missionID; // Currently ignored, set to 0
} __attribute__ ((packed));

// MSP_SET_HEAD command payload
struct msp_set_head_t {
  int16_t heading;
} __attribute__ ((packed));

// MSP_NAV_POSHOLD / MSP_SET_NAV_POSHOLD structure
struct msp_nav_poshold_config_t {
    uint8_t  userControlMode;    // enum
    uint16_t maxAutoSpeed;       // cm/s
    uint16_t maxAutoClimbRate;   // cm/s
    uint16_t maxManualSpeed;     // cm/s
    uint16_t maxManualClimbRate; // cm/s
    uint8_t  mcMaxBankAngle;     // degrees
    uint8_t  mcAltHoldThrottleType; // enum
    uint16_t mcHoverThrottle;    // PWM
} __attribute__ ((packed));

// MSP_UID reply
struct msp_uid_t {
  uint32_t uid0;
  uint32_t uid1;
  uint32_t uid2;
} __attribute__ ((packed));

// MSP_ACC_TRIM / MSP_SET_ACC_TRIM (Note: Not implemented in INAV fc_msp.c)
struct msp_acc_trim_t {
    int16_t pitch; // Usually 0.1 deg units in older MWii
    int16_t roll;  // Usually 0.1 deg units in older MWii
} __attribute__ ((packed));

// MSP_BATTERY_STATE reply
struct msp_battery_state_t {
  uint8_t  cellCount;
  uint16_t capacity;     // mAh
  uint8_t  vbatScaled;   // 0.1V units
  uint16_t mAhDrawn;     // mAh
  int16_t  amperage;     // 0.01A units
  uint8_t  batteryState; // enum BATTERY_STATE_*
  uint16_t vbatActual;   // 0.01V units
} __attribute__ ((packed));

// MSP2_INAV_ANALOG reply (Comprehensive analog data)
struct msp2_inav_analog_t {
    uint8_t  batteryFlags; // Packed: state, cell count, etc.
    uint16_t vbat;         // 0.01V
    uint16_t amperage;     // 0.01A
    uint32_t powerDraw;    // mW
    uint32_t mAhDrawn;     // mAh
    uint32_t mWhDrawn;     // mWh
    uint32_t remainingCapacity; // mAh or mWh
    uint8_t  percentageRemaining; // %
    uint16_t rssi;         // 0-1023 or %
} __attribute__ ((packed));

// MSP_VOLTAGE_METER_CONFIG / MSP_SET_VOLTAGE_METER_CONFIG (Legacy)
struct msp_voltage_meter_config_t {
    uint8_t vbatScale;       // Scale / 10
    uint8_t vbatMinCell;     // 0.1V
    uint8_t vbatMaxCell;     // 0.1V
    uint8_t vbatWarningCell; // 0.1V
} __attribute__ ((packed));

// MSP2_INAV_BATTERY_CONFIG / MSP2_INAV_SET_BATTERY_CONFIG (Modern)
struct msp2_inav_battery_config_t {
    uint16_t vbatScale;
    uint8_t  vbatSource; // enum
    uint8_t  cellCount;
    uint16_t vbatCellDetect; // 0.01V
    uint16_t vbatMinCell;    // 0.01V
    uint16_t vbatMaxCell;    // 0.01V
    uint16_t vbatWarningCell;// 0.01V
    uint16_t currentOffset;  // mV
    uint16_t currentScale;   // Scale
    uint32_t capacityValue;  // mAh/mWh
    uint32_t capacityWarning;// mAh/mWh
    uint32_t capacityCritical;// mAh/mWh
    uint8_t  capacityUnit;   // enum
} __attribute__ ((packed));

// MSP_SENSOR_CONFIG / MSP_SET_SENSOR_CONFIG reply/command
struct msp_sensor_config_t {
    uint8_t accHardware;
    uint8_t baroHardware;
    uint8_t magHardware;
    uint8_t pitotHardware;
    uint8_t rangefinderHardware;
    uint8_t opflowHardware;
} __attribute__ ((packed));

// MSP_RC reply / MSP_SET_RAW_RC command
struct msp_rc_channels_t {
    uint16_t channel[MAX_RC_CHANNELS]; // Use actual channel count from RX config if known
} __attribute__ ((packed));

// MSP_MOTOR reply / MSP_SET_MOTOR command
struct msp_motor_outputs_t {
    uint16_t motor[MAX_SUPPORTED_MOTORS]; // Reports first 8 motors
} __attribute__ ((packed));

// MSP_BOXIDS reply
// Note: Variable length, max depends on features. Use a reasonably large buffer.
struct msp_boxids_t {
    uint8_t ids[MAX_BOX_IDS];
};

// MSP2_INAV_GVAR_STATUS reply
struct msp2_inav_gvar_status_t {
    uint32_t gvarValues[MAX_GVARS];
} __attribute__ ((packed));

// MSP2_INAV_LOGIC_CONDITIONS_STATUS reply
struct msp2_inav_logic_conditions_status_t {
    uint32_t conditionValues[MAX_LOGIC_CONDITIONS];
} __attribute__ ((packed));

// Note: MSP2_INAV_GLOBAL_FUNCTIONS is not implemented in INAV source, skip for now

struct msp_mode_range_t {
    uint8_t modePermanentId; // Permanent ID of the flight mode (from MSP_BOXIDS)
    uint8_t auxChannelIndex; // 0-based index of the AUX channel
    uint8_t rangeStartStep;  // Start step (0-48), converts to 900-2100 PWM
    uint8_t rangeEndStep;    // End step (0-48), converts to 900-2100 PWM
} __attribute__ ((packed));

struct msp_mode_ranges_t {
    msp_mode_range_t ranges[MAX_MODE_RANGES];
};