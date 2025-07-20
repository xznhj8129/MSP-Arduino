// bits of getActiveModes() return value 
#define MSP_MODE_ARM                0
#define MSP_MODE_ANGLE              1
#define MSP_MODE_HORIZON            2
#define MSP_MODE_NAVALTHOLD         3
#define MSP_MODE_HEADINGHOLD        5
#define MSP_MODE_HEADFREE           6
#define MSP_MODE_HEADADJ            7
#define MSP_MODE_CAMSTAB            8
#define MSP_MODE_NAVRTH            10
#define MSP_MODE_NAVPOSHOLD        11
#define MSP_MODE_MANUAL            12
#define MSP_MODE_BEEPER            13
#define MSP_MODE_LEDSOFF           15
#define MSP_MODE_LIGHTS            16
#define MSP_MODE_OSDOFF            19
#define MSP_MODE_TELEMETRY         20
#define MSP_MODE_AUTOTUNE          21
#define MSP_MODE_BLACKBOX          26
#define MSP_MODE_FAILSAFE          27
#define MSP_MODE_NAVWP             28
#define MSP_MODE_AIRMODE           29
#define MSP_MODE_HOMERESET         30
#define MSP_MODE_GCSNAV            31
#define MSP_MODE_FPVANGLEMIX       32
#define MSP_MODE_SURFACE           33
#define MSP_MODE_FLAPERON          34
#define MSP_MODE_TURNASSIST        35
#define MSP_MODE_NAVLAUNCH         36
#define MSP_MODE_SERVOAUTOTRIM     37
#define MSP_MODE_KILLSWITCH        38
#define MSP_MODE_CAMCONTROL1       39
#define MSP_MODE_CAMCONTROL2       40
#define MSP_MODE_CAMCONTROL3       41
#define MSP_MODE_OSDALT1           42
#define MSP_MODE_OSDALT2           43
#define MSP_MODE_OSDALT3           44
#define MSP_MODE_NAVCOURSEHOLD     45
#define MSP_MODE_MCBRAKING         46
#define MSP_MODE_USER1             47
#define MSP_MODE_USER2             48
#define MSP_MODE_LOITERCHANGE      49
#define MSP_MODE_MSPRCOVERRIDE     50
#define MSP_MODE_PREARM            51
#define MSP_MODE_TURTLE            52
#define MSP_MODE_NAVCRUISE         53
#define MSP_MODE_AUTOLEVEL         54
#define MSP_MODE_WPPLANNER         55
#define MSP_MODE_MISSIONCHANGE     59
#define MSP_MODE_BEEPERMUTE        60
#define MSP_MODE_MULTIFUNCTION     61
#define MSP_MODE_MIXERPROFILE2     62
#define MSP_MODE_MIXERTRANSITION   63
#define MSP_MODE_ANGHOLD           64


#define MSP_MAX_WAYPOINTS 60 // Example typical value, check NAV_MAX_WAYPOINTS for target
#define MSP_MAX_RC_CHANNELS 18 // Common RX channel limit


//accelerationSensor
#define ACC_NONE         0
#define ACC_AUTODETECT   1
#define ACC_MPU6000      2
#define ACC_MPU6500      3
#define ACC_MPU9250      4
#define ACC_BMI160       5
#define ACC_ICM20689     6
#define ACC_BMI088       7
#define ACC_ICM42605     8
#define ACC_BMI270       9
#define ACC_LSM6DXX      10
#define ACC_FAKE         11
#define ACC_MAX  11

//armingDisableFlagNames_BF
#define ARMDISABLE_NOGYRO   0
#define ARMDISABLE_FAILSAFE         1
#define ARMDISABLE_RXLOSS   2
#define ARMDISABLE_BADRX    3
#define ARMDISABLE_BOXFAILSAFE      4
#define ARMDISABLE_RUNAWAY  5
#define ARMDISABLE_CRASH    6
#define ARMDISABLE_THROTTLE         7
#define ARMDISABLE_ANGLE    8
#define ARMDISABLE_BOOTGRACE        9
#define ARMDISABLE_NOPREARM         10
#define ARMDISABLE_LOAD     11
#define ARMDISABLE_CALIB    12
#define ARMDISABLE_CLI      13
#define ARMDISABLE_CMS      14
#define ARMDISABLE_BST      15
#define ARMDISABLE_MSP      16
#define ARMDISABLE_PARALYZE         17
#define ARMDISABLE_GPS      18
#define ARMDISABLE_RESCUE SW        19
#define ARMDISABLE_RPMFILTER        20
#define ARMDISABLE_ARMSWITCH        21

//armingDisableFlagNames_INAV
#define ARMFLAG_OK_TO_ARM        0
#define ARMFLAG_PREVENT_ARMING   1
#define ARMFLAG_ARMED    2
#define ARMFLAG_WAS_EVER_ARMED   3
#define ARMFLAG_SIMULATOR_MODE   4
#define ARMFLAG_BLOCKED_FAILSAFE_SYSTEM  7
#define ARMFLAG_BLOCKED_UAV_NOT_LEVEL    8
#define ARMFLAG_BLOCKED_SENSORS_CALIBRATING      9
#define ARMFLAG_BLOCKED_SYSTEM_OVERLOADED        10
#define ARMFLAG_BLOCKED_NAVIGATION_SAFETY        11
#define ARMFLAG_BLOCKED_COMPASS_NOT_CALIBRATED   12
#define ARMFLAG_BLOCKED_ACCELEROMETER_NOT_CALIBRATED     13
#define ARMFLAG_BLOCKED_ARMING_DISABLED_ARM_SWITCH       14
#define ARMFLAG_BLOCKED_HARDWARE_FAILURE         15
#define ARMFLAG_BLOCKED_ARMING_DISABLED_BOXFAILSAFE      16
#define ARMFLAG_BLOCKED_ARMING_DISABLED_BOXKILLSWITCH    17
#define ARMFLAG_BLOCKED_ARMING_DISABLED_RC_LINK  18
#define ARMFLAG_BLOCKED_ARMING_DISABLED_THROTTLE         19
#define ARMFLAG_BLOCKED_ARMING_DISABLED_CLI      20
#define ARMFLAG_BLOCKED_ARMING_DISABLED_CMS_MENU         21
#define ARMFLAG_BLOCKED_ARMING_DISABLED_OSD_MENU         22
#define ARMFLAG_BLOCKED_ARMING_DISABLED_ROLLPITCH_NOT_CENTERED   23
#define ARMFLAG_BLOCKED_ARMING_DISABLED_SERVO_AUTOTRIM   24
#define ARMFLAG_BLOCKED_ARMING_DISABLED_OOM      25
#define ARMFLAG_BLOCKED_INVALID_SETTING  26
#define ARMFLAG_BLOCKED_ARMING_DISABLED_PWM_OUTPUT_ERROR         27
#define ARMFLAG_BLOCKED_ARMING_DISABLED_NO_PREARM        28
#define ARMFLAG_BLOCKED_ARMING_DISABLED_DSHOT_BEEPER     29
#define ARMFLAG_BLOCKED_ARMING_DISABLED_LANDING_DETECTED         30

//baroSensor
#define BARO_NONE        0
#define BARO_AUTODETECT  1
#define BARO_BMP085      2
#define BARO_MS5611      3
#define BARO_BMP280      4
#define BARO_MS5607      5
#define BARO_LPS25H      6
#define BARO_SPL06       7
#define BARO_BMP388      8
#define BARO_DPS310      9
#define BARO_B2SMPB      10
#define BARO_MSP         11
#define BARO_FAKE        12
#define BARO_MAX         12

//batCapacityUnit
#define BAT_CAPACITY_UNIT_MAH    0
#define BAT_CAPACITY_UNIT_MWH    1

//batVoltageSource
#define BAT_VOLTAGE_RAW  0
#define BAT_VOLTAGE_SAG_COMP     1

//batteryState
#define BATTERY_OK       0
#define BATTERY_WARNING  1
#define BATTERY_CRITICAL         2
#define BATTERY_NOT_PRESENT      3

//climbRateToAltitudeControllerMode
#define ROC_TO_ALT_RESET         0
#define ROC_TO_ALT_CONSTANT      1
#define ROC_TO_ALT_TARGET        2

//currentSensor
#define CURRENT_SENSOR_NONE      0
#define CURRENT_SENSOR_ADC       1
#define CURRENT_SENSOR_VIRTUAL   2
#define CURRENT_SENSOR_FAKE      3
#define CURRENT_SENSOR_ESC       4
#define CURRENT_SENSOR_MAX       3

//dynamicGyroNotchMode
#define DYNAMIC_NOTCH_MODE_2D    0
#define DYNAMIC_NOTCH_MODE_R     1
#define DYNAMIC_NOTCH_MODE_P     2
#define DYNAMIC_NOTCH_MODE_Y     3
#define DYNAMIC_NOTCH_MODE_RP    4
#define DYNAMIC_NOTCH_MODE_RY    5
#define DYNAMIC_NOTCH_MODE_PY    6
#define DYNAMIC_NOTCH_MODE_3D    7

//flyingPlatformType
#define PLATFORM_MULTIROTOR      0
#define PLATFORM_AIRPLANE        1
#define PLATFORM_HELICOPTER      2
#define PLATFORM_TRICOPTER       3
#define PLATFORM_ROVER   4
#define PLATFORM_BOAT    5
#define PLATFORM_OTHER   6

//fwAutolandState
#define FW_AUTOLAND_STATE_IDLE   0
#define FW_AUTOLAND_STATE_LOITER         1
#define FW_AUTOLAND_STATE_DOWNWIND       2
#define FW_AUTOLAND_STATE_BASE_LEG       3
#define FW_AUTOLAND_STATE_FINAL_APPROACH         4
#define FW_AUTOLAND_STATE_GLIDE  5
#define FW_AUTOLAND_STATE_FLARE  6

//fwAutolandWaypoint
#define FW_AUTOLAND_WP_TURN      0
#define FW_AUTOLAND_WP_FINAL_APPROACH    1
#define FW_AUTOLAND_WP_LAND      2
#define FW_AUTOLAND_WP_COUNT     3

//geoAltitudeConversionMode
#define GEO_ALT_ABSOLUTE         0
#define GEO_ALT_RELATIVE         1

//geoAltitudeDatumFlag
#define NAV_WP_TAKEOFF_DATUM     0
#define NAV_WP_MSL_DATUM         1

//geoOriginResetMode
#define GEO_ORIGIN_SET   0
#define GEO_ORIGIN_RESET_ALTITUDE        1

//gyroSensor
#define GYRO_NONE        0
#define GYRO_AUTODETECT  1
#define GYRO_MPU6000     2
#define GYRO_MPU6500     3
#define GYRO_MPU9250     4
#define GYRO_BMI160      5
#define GYRO_ICM20689    6
#define GYRO_BMI088      7
#define GYRO_ICM42605    8
#define GYRO_BMI270      9
#define GYRO_LSM6DXX     10
#define GYRO_FAKE        11

//hardwareSensorStatus
#define HW_SENSOR_NONE   0
#define HW_SENSOR_OK     1
#define HW_SENSOR_UNAVAILABLE    2
#define HW_SENSOR_UNHEALTHY      3

//magSensor
#define MAG_NONE         0
#define MAG_AUTODETECT   1
#define MAG_HMC5883      2
#define MAG_AK8975       3
#define MAG_MAG3110      4
#define MAG_AK8963       5
#define MAG_IST8310      6
#define MAG_QMC5883      7
#define MAG_MPU9250      8
#define MAG_IST8308      9
#define MAG_LIS3MDL      10
#define MAG_MSP  11
#define MAG_RM3100       12
#define MAG_VCM5883      13
#define MAG_MLX90393     14
#define MAG_FAKE         15
#define MAG_MAX  15

//navAGLEstimateQuality
#define SURFACE_QUAL_LOW         0
#define SURFACE_QUAL_MID         1
#define SURFACE_QUAL_HIGH        2

//navArmingBlocker
#define NAV_ARMING_BLOCKER_NONE  0
#define NAV_ARMING_BLOCKER_MISSING_GPS_FIX       1
#define NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE         2
#define NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR        3
#define NAV_ARMING_BLOCKER_JUMP_WAYPOINT_ERROR   4

//navExtraArmingSafety
#define NAV_EXTRA_ARMING_SAFETY_ON       0
#define NAV_EXTRA_ARMING_SAFETY_ALLOW_BYPASS     1

//navFwLaunchStatus
#define FW_LAUNCH_DETECTED       4
#define FW_LAUNCH_ABORTED        9
#define FW_LAUNCH_FLYING         10

//navMcAltHoldThrottle
#define MC_ALT_HOLD_STICK        0
#define MC_ALT_HOLD_MID  1
#define MC_ALT_HOLD_HOVER        2

//navMissionRestart
#define WP_MISSION_START         0
#define WP_MISSION_RESUME        1
#define WP_MISSION_SWITCH        2

//navOverridesMotorStop
#define NOMS_OFF_ALWAYS  0
#define NOMS_OFF         1
#define NOMS_AUTO_ONLY   2
#define NOMS_ALL_NAV     3

//navPositionEstimationFlags
#define EST_GPS_XY_VALID         (1 << 0)
#define EST_GPS_Z_VALID  (1 << 1)
#define EST_BARO_VALID   (1 << 2)
#define EST_SURFACE_VALID        (1 << 3)
#define EST_FLOW_VALID   (1 << 4)
#define EST_XY_VALID     (1 << 5)
#define EST_Z_VALID      (1 << 6)

//navRTHAllowLanding
#define NAV_RTH_ALLOW_LANDING_NEVER      0
#define NAV_RTH_ALLOW_LANDING_ALWAYS     1
#define NAV_RTH_ALLOW_LANDING_FS_ONLY    2

//navRTHClimbFirst
#define RTH_CLIMB_OFF    0
#define RTH_CLIMB_ON     1
#define RTH_CLIMB_ON_FW_SPIRAL   2

//navSetWaypointFlags
#define NAV_POS_UPDATE_NONE      0
#define NAV_POS_UPDATE_Z         1 << 1
#define NAV_POS_UPDATE_XY        1 << 0
#define NAV_POS_UPDATE_HEADING   1 << 2
#define NAV_POS_UPDATE_BEARING   1 << 3
#define NAV_POS_UPDATE_BEARING_TAIL_FIRST        1 << 4

//navSystemStatus_Error
#define MW_NAV_ERROR_NONE        0
#define MW_NAV_ERROR_TOOFAR      1
#define MW_NAV_ERROR_SPOILED_GPS         2
#define MW_NAV_ERROR_WP_CRC      3
#define MW_NAV_ERROR_FINISH      4
#define MW_NAV_ERROR_TIMEWAIT    5
#define MW_NAV_ERROR_INVALID_JUMP        6
#define MW_NAV_ERROR_INVALID_DATA        7
#define MW_NAV_ERROR_WAIT_FOR_RTH_ALT    8
#define MW_NAV_ERROR_GPS_FIX_LOST        9
#define MW_NAV_ERROR_DISARMED    10
#define MW_NAV_ERROR_LANDING     11

//navSystemStatus_Flags
#define MW_NAV_FLAG_ADJUSTING_POSITION   1 << 0
#define MW_NAV_FLAG_ADJUSTING_ALTITUDE   1 << 1

//navSystemStatus_Mode
#define MW_GPS_MODE_NONE         0
#define MW_GPS_MODE_HOLD         1
#define MW_GPS_MODE_RTH  2
#define MW_GPS_MODE_NAV  3
#define MW_GPS_MODE_EMERG        15

//navSystemStatus_State
#define MW_NAV_STATE_NONE        0
#define MW_NAV_STATE_RTH_START   1
#define MW_NAV_STATE_RTH_ENROUTE         2
#define MW_NAV_STATE_HOLD_INFINIT        3
#define MW_NAV_STATE_HOLD_TIMED  4
#define MW_NAV_STATE_WP_ENROUTE  5
#define MW_NAV_STATE_PROCESS_NEXT        6
#define MW_NAV_STATE_DO_JUMP     7
#define MW_NAV_STATE_LAND_START  8
#define MW_NAV_STATE_LAND_IN_PROGRESS    9
#define MW_NAV_STATE_LANDED      10
#define MW_NAV_STATE_LAND_SETTLE         11
#define MW_NAV_STATE_LAND_START_DESCENT  12
#define MW_NAV_STATE_HOVER_ABOVE_HOME    13
#define MW_NAV_STATE_EMERGENCY_LANDING   14
#define MW_NAV_STATE_RTH_CLIMB   15

//navWaypointActions
#define NAV_WP_ACTION_WAYPOINT   1
#define NAV_WP_ACTION_HOLD_TIME  3
#define NAV_WP_ACTION_RTH        4
#define NAV_WP_ACTION_SET_POI    5
#define NAV_WP_ACTION_JUMP       6
#define NAV_WP_ACTION_SET_HEAD   7
#define NAV_WP_ACTION_LAND       8

//navWaypointFlags
#define NAV_WP_FLAG_HOME         72
#define NAV_WP_FLAG_LAST         165

//navWaypointHeadings
#define NAV_WP_HEAD_MODE_NONE    0
#define NAV_WP_HEAD_MODE_POI     1
#define NAV_WP_HEAD_MODE_FIXED   2

//navWaypointP3Flags
#define NAV_WP_ALTMODE   (1<<0)
#define NAV_WP_USER1     (1<<1)
#define NAV_WP_USER2     (1<<2)
#define NAV_WP_USER3     (1<<3)
#define NAV_WP_USER4     (1<<4)

//nav_resetype
#define NAV_RESET_NEVER  0
#define NAV_RESET_ON_FIRST_ARM   1
#define NAV_RESET_ON_EACH_ARM    2

//navigationEstimateStatus
#define EST_NONE         0
#define EST_USABLE       1
#define EST_TRUSTED      2

//navigationFSMEvent
#define NAV_FSM_EVENT_NONE       0
#define NAV_FSM_EVENT_TIMEOUT    1
#define NAV_FSM_EVENT_SUCCESS    2
#define NAV_FSM_EVENT_ERROR      3
#define NAV_FSM_EVENT_SWITCH_TO_IDLE     4
#define NAV_FSM_EVENT_SWITCH_TO_ALTHOLD  5
#define NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D       6
#define NAV_FSM_EVENT_SWITCH_TO_RTH      7
#define NAV_FSM_EVENT_SWITCH_TO_WAYPOINT         8
#define NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING        9
#define NAV_FSM_EVENT_SWITCH_TO_LAUNCH   10
#define NAV_FSM_EVENT_SWITCH_TO_COURSE_HOLD      11
#define NAV_FSM_EVENT_SWITCH_TO_CRUISE   12
#define NAV_FSM_EVENT_SWITCH_TO_COURSE_ADJ       13
#define NAV_FSM_EVENT_SWITCH_TO_MIXERAT  14
#define NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING     15
#define NAV_FSM_EVENT_STATE_SPECIFIC_1   16
#define NAV_FSM_EVENT_STATE_SPECIFIC_2   17
#define NAV_FSM_EVENT_STATE_SPECIFIC_3   18
#define NAV_FSM_EVENT_STATE_SPECIFIC_4   19
#define NAV_FSM_EVENT_STATE_SPECIFIC_5   20
#define NAV_FSM_EVENT_STATE_SPECIFIC_6   21
#define NAV_FSM_EVENT_SWITCH_TO_RTH_HEAD_HOME    18
#define NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING      16
#define NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND        16
#define NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED        17
#define NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_HOLD_TIME       18
#define NAV_FSM_EVENT_SWITCH_TO_RTH_HOVER_ABOVE_HOME     19
#define NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_TRACKBACK  20
#define NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_INITIALIZE         21
#define NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING_ABORT       21
#define NAV_FSM_EVENT_COUNT      31

//navigationFSMState
#define NAV_STATE_UNDEFINED      0
#define NAV_STATE_IDLE   1
#define NAV_STATE_ALTHOLD_INITIALIZE     2
#define NAV_STATE_ALTHOLD_IN_PROGRESS    3
#define NAV_STATE_POSHOLD_3D_INITIALIZE  4
#define NAV_STATE_POSHOLD_3D_IN_PROGRESS         5
#define NAV_STATE_RTH_INITIALIZE         6
#define NAV_STATE_RTH_CLIMB_TO_SAFE_ALT  7
#define NAV_STATE_RTH_TRACKBACK  8
#define NAV_STATE_RTH_HEAD_HOME  9
#define NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING     10
#define NAV_STATE_RTH_HOVER_ABOVE_HOME   11
#define NAV_STATE_RTH_LANDING    12
#define NAV_STATE_RTH_FINISHING  13
#define NAV_STATE_RTH_FINISHED   14
#define NAV_STATE_WAYPOINT_INITIALIZE    15
#define NAV_STATE_WAYPOINT_PRE_ACTION    16
#define NAV_STATE_WAYPOINT_IN_PROGRESS   17
#define NAV_STATE_WAYPOINT_REACHED       18
#define NAV_STATE_WAYPOINT_HOLD_TIME     19
#define NAV_STATE_WAYPOINT_NEXT  20
#define NAV_STATE_WAYPOINT_FINISHED      21
#define NAV_STATE_WAYPOINT_RTH_LAND      22
#define NAV_STATE_EMERGENCY_LANDING_INITIALIZE   23
#define NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS  24
#define NAV_STATE_EMERGENCY_LANDING_FINISHED     25
#define NAV_STATE_LAUNCH_INITIALIZE      26
#define NAV_STATE_LAUNCH_WAIT    27
#define NAV_STATE_LAUNCH_IN_PROGRESS     28
#define NAV_STATE_COURSE_HOLD_INITIALIZE         29
#define NAV_STATE_COURSE_HOLD_IN_PROGRESS        30
#define NAV_STATE_COURSE_HOLD_ADJUSTING  31
#define NAV_STATE_CRUISE_INITIALIZE      32
#define NAV_STATE_CRUISE_IN_PROGRESS     33
#define NAV_STATE_CRUISE_ADJUSTING       34
#define NAV_STATE_FW_LANDING_CLIMB_TO_LOITER     35
#define NAV_STATE_FW_LANDING_LOITER      36
#define NAV_STATE_FW_LANDING_APPROACH    37
#define NAV_STATE_FW_LANDING_GLIDE       38
#define NAV_STATE_FW_LANDING_FLARE       39
#define NAV_STATE_FW_LANDING_ABORT       40
#define NAV_STATE_MIXERAT_INITIALIZE     41
#define NAV_STATE_MIXERAT_IN_PROGRESS    42
#define NAV_STATE_MIXERAT_ABORT  43
#define NAV_STATE_COUNT  44

//navigationFSMStateFlags
#define NAV_CTL_ALT      (1 << 0)
#define NAV_CTL_POS      (1 << 1)
#define NAV_CTL_YAW      (1 << 2)
#define NAV_CTL_EMERG    (1 << 3)
#define NAV_CTL_LAUNCH   (1 << 4)
#define NAV_REQUIRE_ANGLE        (1 << 5)
#define NAV_REQUIRE_ANGLE_FW     (1 << 6)
#define NAV_REQUIRE_MAGHOLD      (1 << 7)
#define NAV_REQUIRE_THRTILT      (1 << 8)
#define NAV_AUTO_RTH     (1 << 9)
#define NAV_AUTO_WP      (1 << 10)
#define NAV_RC_ALT       (1 << 11)
#define NAV_RC_POS       (1 << 12)
#define NAV_RC_YAW       (1 << 13)
#define NAV_CTL_LAND     (1 << 14)
#define NAV_AUTO_WP_DONE         (1 << 15)
#define NAV_MIXERAT      (1 << 16)

//navigationHomeFlags
#define NAV_HOME_INVALID         0
#define NAV_HOME_VALID_XY        1 << 0
#define NAV_HOME_VALID_Z         1 << 1
#define NAV_HOME_VALID_HEADING   1 << 2
#define NAV_HOME_VALID_ALL       NAV_HOME_VALID_XY | NAV_HOME_VALID_Z | NAV_HOME_VALID_HEADING

//navigationPersistentId
#define NAV_PERSISTENT_ID_UNDEFINED      0
#define NAV_PERSISTENT_ID_IDLE   1
#define NAV_PERSISTENT_ID_ALTHOLD_INITIALIZE     2
#define NAV_PERSISTENT_ID_ALTHOLD_IN_PROGRESS    3
#define NAV_PERSISTENT_ID_UNUSED_1       4
#define NAV_PERSISTENT_ID_UNUSED_2       5
#define NAV_PERSISTENT_ID_POSHOLD_3D_INITIALIZE  6
#define NAV_PERSISTENT_ID_POSHOLD_3D_IN_PROGRESS         7
#define NAV_PERSISTENT_ID_RTH_INITIALIZE         8
#define NAV_PERSISTENT_ID_RTH_CLIMB_TO_SAFE_ALT  9
#define NAV_PERSISTENT_ID_RTH_HEAD_HOME  10
#define NAV_PERSISTENT_ID_RTH_HOVER_PRIOR_TO_LANDING     11
#define NAV_PERSISTENT_ID_RTH_LANDING    12
#define NAV_PERSISTENT_ID_RTH_FINISHING  13
#define NAV_PERSISTENT_ID_RTH_FINISHED   14
#define NAV_PERSISTENT_ID_WAYPOINT_INITIALIZE    15
#define NAV_PERSISTENT_ID_WAYPOINT_PRE_ACTION    16
#define NAV_PERSISTENT_ID_WAYPOINT_IN_PROGRESS   17
#define NAV_PERSISTENT_ID_WAYPOINT_REACHED       18
#define NAV_PERSISTENT_ID_WAYPOINT_NEXT  19
#define NAV_PERSISTENT_ID_WAYPOINT_FINISHED      20
#define NAV_PERSISTENT_ID_WAYPOINT_RTH_LAND      21
#define NAV_PERSISTENT_ID_EMERGENCY_LANDING_INITIALIZE   22
#define NAV_PERSISTENT_ID_EMERGENCY_LANDING_IN_PROGRESS  23
#define NAV_PERSISTENT_ID_EMERGENCY_LANDING_FINISHED     24
#define NAV_PERSISTENT_ID_LAUNCH_INITIALIZE      25
#define NAV_PERSISTENT_ID_LAUNCH_WAIT    26
#define NAV_PERSISTENT_ID_UNUSED_3       27
#define NAV_PERSISTENT_ID_LAUNCH_IN_PROGRESS     28
#define NAV_PERSISTENT_ID_COURSE_HOLD_INITIALIZE         29
#define NAV_PERSISTENT_ID_COURSE_HOLD_IN_PROGRESS        30
#define NAV_PERSISTENT_ID_COURSE_HOLD_ADJUSTING  31
#define NAV_PERSISTENT_ID_CRUISE_INITIALIZE      32
#define NAV_PERSISTENT_ID_CRUISE_IN_PROGRESS     33
#define NAV_PERSISTENT_ID_CRUISE_ADJUSTING       34
#define NAV_PERSISTENT_ID_WAYPOINT_HOLD_TIME     35
#define NAV_PERSISTENT_ID_RTH_HOVER_ABOVE_HOME   36
#define NAV_PERSISTENT_ID_UNUSED_4       37
#define NAV_PERSISTENT_ID_RTH_TRACKBACK  38
#define NAV_PERSISTENT_ID_MIXERAT_INITIALIZE     39
#define NAV_PERSISTENT_ID_MIXERAT_IN_PROGRESS    40
#define NAV_PERSISTENT_ID_MIXERAT_ABORT  41
#define NAV_PERSISTENT_ID_FW_LANDING_CLIMB_TO_LOITER     42
#define NAV_PERSISTENT_ID_FW_LANDING_LOITER      43
#define NAV_PERSISTENT_ID_FW_LANDING_APPROACH    44
#define NAV_PERSISTENT_ID_FW_LANDING_GLIDE       45
#define NAV_PERSISTENT_ID_FW_LANDING_FLARE       46
#define NAV_PERSISTENT_ID_FW_LANDING_ABORT       47

//opflowQuality
#define OPFLOW_QUALITY_INVALID   0
#define OPFLOW_QUALITY_VALID     1

//opticalFlowSensor
#define OPFLOW_NONE      0
#define OPFLOW_CXOF      1
#define OPFLOW_MSP       2
#define OPFLOW_FAKE      3

//pitotSensor
#define PITOT_NONE       0
#define PITOT_AUTODETECT         1
#define PITOT_MS4525     2
#define PITOT_ADC        3
#define PITOT_VIRTUAL    4
#define PITOT_FAKE       5
#define PITOT_MSP        6
#define PITOT_DLVR       7

//rangefinderType
#define RANGEFINDER_NONE         0
#define RANGEFINDER_SRF10        1
#define RANGEFINDER_VL53L0X      2
#define RANGEFINDER_MSP  3
#define RANGEFINDER_BENEWAKE     4
#define RANGEFINDER_VL53L1X      5
#define RANGEFINDER_US42         6
#define RANGEFINDER_TOF10102I2C  7
#define RANGEFINDER_FAKE         8

//rthTargetMode
#define RTH_HOME_ENROUTE_INITIAL         0
#define RTH_HOME_ENROUTE_PROPORTIONAL    1
#define RTH_HOME_ENROUTE_FINAL   2
#define RTH_HOME_FINAL_HOVER     3
#define RTH_HOME_FINAL_LAND      4

//rthTrackbackMode
#define RTH_TRACKBACK_OFF        0
#define RTH_TRACKBACK_ON         1
#define RTH_TRACKBACK_FS         2

//safehomeUsageMode
#define SAFEHOME_USAGE_OFF       0
#define SAFEHOME_USAGE_RTH       1
#define SAFEHOME_USAGE_RTH_FS    2

//sensorIndex
#define SENSOR_INDEX_GYRO        0
#define SENSOR_INDEX_ACC         1
#define SENSOR_INDEX_BARO        2
#define SENSOR_INDEX_MAG         3
#define SENSOR_INDEX_RANGEFINDER         4
#define SENSOR_INDEX_PITOT       5
#define SENSOR_INDEX_OPFLOW      6
#define SENSOR_INDEX_COUNT       7

//sensors
#define SENSOR_GYRO      1 << 0
#define SENSOR_ACC       1 << 1
#define SENSOR_BARO      1 << 2
#define SENSOR_MAG       1 << 3
#define SENSOR_RANGEFINDER       1 << 4
#define SENSOR_PITOT     1 << 5
#define SENSOR_OPFLOW    1 << 6
#define SENSOR_GPS       1 << 7
#define SENSOR_GPSMAG    1 << 8
#define SENSOR_TEMP      1 << 9

//tempSensorType
#define TEMP_SENSOR_NONE         0
#define TEMP_SENSOR_LM75         1
#define TEMP_SENSOR_DS18B20      2

//voltageSensor
#define VOLTAGE_SENSOR_NONE      0
#define VOLTAGE_SENSOR_ADC       1
#define VOLTAGE_SENSOR_ESC       2
#define VOLTAGE_SENSOR_FAKE      3

//wpFwTurnSmoothing
#define WP_TURN_SMOOTHING_OFF    0
#define WP_TURN_SMOOTHING_ON     1
#define WP_TURN_SMOOTHING_CUT    2

//wpMissionPlannerStatus
#define WP_PLAN_WAIT     0
#define WP_PLAN_SAVE     1
#define WP_PLAN_OK       2
#define WP_PLAN_FULL     3
// flags for msp_status_ex_t.sensor and msp_status_t.sensor
#define MSP_STATUS_SENSOR_ACC    1
#define MSP_STATUS_SENSOR_BARO   2
#define MSP_STATUS_SENSOR_MAG    4
#define MSP_STATUS_SENSOR_GPS    8
#define MSP_STATUS_SENSOR_SONAR 16
#define MSP_MAX_SUPPORTED_CHANNELS 16
// values for msp_raw_gps_t.fixType
#define MSP_GPS_NO_FIX 0
#define MSP_GPS_FIX_2D 1
#define MSP_GPS_FIX_3D 2

// values for msp_nav_status_t.mode
#define MSP_NAV_STATUS_MODE_NONE   0
#define MSP_NAV_STATUS_MODE_HOLD   1
#define MSP_NAV_STATUS_MODE_RTH    2
#define MSP_NAV_STATUS_MODE_NAV    3
#define MSP_NAV_STATUS_MODE_EMERG 15

// values for msp_nav_status_t.state
#define MSP_NAV_STATUS_STATE_NONE                0  // None
#define MSP_NAV_STATUS_STATE_RTH_START           1  // RTH Start
#define MSP_NAV_STATUS_STATE_RTH_ENROUTE         2  // RTH Enroute
#define MSP_NAV_STATUS_STATE_HOLD_INFINIT        3  // PosHold infinit
#define MSP_NAV_STATUS_STATE_HOLD_TIMED          4  // PosHold timed
#define MSP_NAV_STATUS_STATE_WP_ENROUTE          5  // WP Enroute
#define MSP_NAV_STATUS_STATE_PROCESS_NEXT        6  // Process next
#define MSP_NAV_STATUS_STATE_DO_JUMP             7  // Jump
#define MSP_NAV_STATUS_STATE_LAND_START          8  // Start Land
#define MSP_NAV_STATUS_STATE_LAND_IN_PROGRESS    9  // Land in Progress
#define MSP_NAV_STATUS_STATE_LANDED             10  // Landed
#define MSP_NAV_STATUS_STATE_LAND_SETTLE        11  // Settling before land
#define MSP_NAV_STATUS_STATE_LAND_START_DESCENT 12  // Start descent

// values for msp_nav_status_t.activeWpAction, msp_set_wp_t.action
#define MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT 0x01
#define MSP_NAV_STATUS_WAYPOINT_ACTION_RTH      0x04

// values for msp_nav_status_t.error
#define MSP_NAV_STATUS_ERROR_NONE               0   // All systems clear
#define MSP_NAV_STATUS_ERROR_TOOFAR             1   // Next waypoint distance is more than safety distance
#define MSP_NAV_STATUS_ERROR_SPOILED_GPS        2   // GPS reception is compromised - Nav paused - copter is adrift !
#define MSP_NAV_STATUS_ERROR_WP_CRC             3   // CRC error reading WP data from EEPROM - Nav stopped
#define MSP_NAV_STATUS_ERROR_FINISH             4   // End flag detected, navigation finished
#define MSP_NAV_STATUS_ERROR_TIMEWAIT           5   // Waiting for poshold timer
#define MSP_NAV_STATUS_ERROR_INVALID_JUMP       6   // Invalid jump target detected, aborting
#define MSP_NAV_STATUS_ERROR_INVALID_DATA       7   // Invalid mission step action code, aborting, copter is adrift
#define MSP_NAV_STATUS_ERROR_WAIT_FOR_RTH_ALT   8   // Waiting to reach RTH Altitude
#define MSP_NAV_STATUS_ERROR_GPS_FIX_LOST       9   // Gps fix lost, aborting mission
#define MSP_NAV_STATUS_ERROR_DISARMED          10   // NAV engine disabled due disarm
#define MSP_NAV_STATUS_ERROR_LANDING           11   // Landing


// MSP_FEATURE mask
#define MSP_FEATURE_RX_PPM              (1 <<  0)
#define MSP_FEATURE_VBAT                (1 <<  1)
#define MSP_FEATURE_UNUSED_1            (1 <<  2)
#define MSP_FEATURE_RX_SERIAL           (1 <<  3)
#define MSP_FEATURE_MOTOR_STOP          (1 <<  4)
#define MSP_FEATURE_SERVO_TILT          (1 <<  5)
#define MSP_FEATURE_SOFTSERIAL          (1 <<  6)
#define MSP_FEATURE_GPS                 (1 <<  7)
#define MSP_FEATURE_UNUSED_3            (1 <<  8)         // was FEATURE_FAILSAFE
#define MSP_FEATURE_UNUSED_4            (1 <<  9)         // was FEATURE_SONAR
#define MSP_FEATURE_TELEMETRY           (1 << 10)
#define MSP_FEATURE_CURRENT_METER       (1 << 11)
#define MSP_FEATURE_3D                  (1 << 12)
#define MSP_FEATURE_RX_PARALLEL_PWM     (1 << 13)
#define MSP_FEATURE_RX_MSP              (1 << 14)
#define MSP_FEATURE_RSSI_ADC            (1 << 15)
#define MSP_FEATURE_LED_STRIP           (1 << 16)
#define MSP_FEATURE_DASHBOARD           (1 << 17)
#define MSP_FEATURE_UNUSED_2            (1 << 18)
#define MSP_FEATURE_BLACKBOX            (1 << 19)
#define MSP_FEATURE_CHANNEL_FORWARDING  (1 << 20)
#define MSP_FEATURE_TRANSPONDER         (1 << 21)
#define MSP_FEATURE_AIRMODE             (1 << 22)
#define MSP_FEATURE_SUPEREXPO_RATES     (1 << 23)
#define MSP_FEATURE_VTX                 (1 << 24)
#define MSP_FEATURE_RX_SPI              (1 << 25)
#define MSP_FEATURE_SOFTSPI             (1 << 26)
#define MSP_FEATURE_PWM_SERVO_DRIVER    (1 << 27)
#define MSP_FEATURE_PWM_OUTPUT_ENABLE   (1 << 28)
#define MSP_FEATURE_OSD                 (1 << 29)

// values for msp_current_meter_config_t.currentMeterType
#define MSP_CURRENT_SENSOR_NONE    0
#define MSP_CURRENT_SENSOR_ADC     1
#define MSP_CURRENT_SENSOR_VIRTUAL 2
#define MSP_CURRENT_SENSOR_MAX     CURRENT_SENSOR_VIRTUAL
 

#define MSP_MAX_SUPPORTED_SERVOS 8
#define MSP_MAX_SERVO_RULES (2 * MSP_MAX_SUPPORTED_SERVOS)
#define MSP_MAX_SUPPORTED_MOTORS 8

// msp_rx_config_t.serialrx_provider
#define MSP_SERIALRX_SPEKTRUM1024      0
#define MSP_SERIALRX_SPEKTRUM2048      1
#define MSP_SERIALRX_SBUS              2
#define MSP_SERIALRX_SUMD              3
#define MSP_SERIALRX_SUMH              4
#define MSP_SERIALRX_XBUS_MODE_B       5
#define MSP_SERIALRX_XBUS_MODE_B_RJ01  6
#define MSP_SERIALRX_IBUS              7
#define MSP_SERIALRX_JETIEXBUS         8
#define MSP_SERIALRX_CRSF              9


// msp_rx_config_t.rx_spi_protocol values
#define MSP_SPI_PROT_NRF24RX_V202_250K 0
#define MSP_SPI_PROT_NRF24RX_V202_1M   1
#define MSP_SPI_PROT_NRF24RX_SYMA_X    2
#define MSP_SPI_PROT_NRF24RX_SYMA_X5C  3
#define MSP_SPI_PROT_NRF24RX_CX10      4
#define MSP_SPI_PROT_NRF24RX_CX10A     5
#define MSP_SPI_PROT_NRF24RX_H8_3D     6
#define MSP_SPI_PROT_NRF24RX_INAV      7
#define MSP_MAX_MAPPABLE_RX_INPUTS     8

// values for msp_sensor_alignment_t.gyro_align, acc_align, mag_align
#define MSP_SENSOR_ALIGN_CW0_DEG        1
#define MSP_SENSOR_ALIGN_CW90_DEG       2
#define MSP_SENSOR_ALIGN_CW180_DEG      3
#define MSP_SENSOR_ALIGN_CW270_DEG      4
#define MSP_SENSOR_ALIGN_CW0_DEG_FLIP   5
#define MSP_SENSOR_ALIGN_CW90_DEG_FLIP  6
#define MSP_SENSOR_ALIGN_CW180_DEG_FLIP 7
#define MSP_SENSOR_ALIGN_CW270_DEG_FLIP 8
