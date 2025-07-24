#include "inav_api.h"


// ---------- ctor ----------
InavAPI::InavAPI() {}

// ---------- init ----------
void InavAPI::Init(Stream &serial, uint32_t timeoutMs) {
    msp.begin(serial, timeoutMs);
    _inited = true;
    _haveStatus = false;
}

// ---------- helpers ----------
int InavAPI::float_to_pwm(float in) {
    if (in < 0.0f) in = 0.0f;
    if (in > 1.0f) in = 1.0f;
    return PWM_RANGE_MIN + (int)((PWM_RANGE_MAX - PWM_RANGE_MIN) * in);
}

bool InavAPI::isConnected() {
    if (!_inited) return false;
    msp_api_version_t v{};
    return msp.requestApiVersion(&v);
}

void InavAPI::_updateStatus() {
    if (msp.requestInavStatus(&_lastStatus)) {
        _haveStatus = true;
    } else {
        _haveStatus = false;
    }
}

// ---------- pass-through basics ----------
bool InavAPI::get_api_version(msp_api_version_t &o) {
    return msp.requestApiVersion(&o);
}
bool InavAPI::get_fc_variant(msp_fc_variant_t &o) {
    return msp.requestFcVariant(&o);
}
bool InavAPI::get_fc_version(msp_fc_version_t &o) {
    return msp.requestFcVersion(&o);
}
bool InavAPI::get_board_info(msp_board_info_t &o, char *buf, uint8_t len) {
    return msp.requestBoardInfo(&o, buf, len);
}

// ---------- GETs ----------
bool InavAPI::get_altitude(inav_altitude &out) {
    msp_altitude_t a{};
    if (!msp.requestAltitude(&a)) return false;
    out.estimatedAltitude = _cm_to_m(a.estimatedAltitude);
    out.variometer        = _cms_to_ms(a.variometer);
    out.baroAltitude      = _cm_to_m(a.baroAltitude);
    return true;
}

bool InavAPI::get_gps(inav_gps_data &out) {
    msp_raw_gps_t g{};
    if (!msp.requestRawGPS(&g)) return false;
    out.fixType   = g.fixType;
    out.sats      = g.numSat;
    out.latitude  = _i32_to_deg(g.latitude);
    out.longitude = _i32_to_deg(g.longitude);
    out.altitude  = (float)g.altitude;
    out.speed     = _cms_to_ms_u16(g.speed);
    out.course    = _ddeg_to_deg(g.groundCourse);
    out.hdop      = (float)g.hdop / 100.0f;
    return true;
}

bool InavAPI::get_compgps(inav_comp_gps &out) {
    msp_comp_gps_t c{};
    if (!msp.requestCompGPS(&c)) return false;
    out.distanceToHome  = c.distanceToHome;
    out.directionToHome = c.directionToHome;
    return true;
}

bool InavAPI::get_nav_status(inav_nav_data &nd) {
    msp_nav_status_t ns{};
    if (!msp.requestNavStatus(&ns)) return false;
    nd.navMode        = ns.navMode;
    nd.navState       = ns.navState;
    nd.activeWpAction = ns.activeWpAction;
    nd.activeWpNumber = ns.activeWpNumber;
    nd.navError       = ns.navError;
    nd.targetHeading  = ns.targetHeading;
    return true;
}

bool InavAPI::get_attitude(inav_attitude &out) {
    msp_attitude_t a{};
    if (!msp.requestAttitude(&a)) return false;
    out.roll  = _ddeg_to_deg(a.roll);
    out.pitch = _ddeg_to_deg(a.pitch);
    out.yaw   = a.yaw;
    return true;
}

bool InavAPI::get_imu(msp_raw_imu_t &out) {
    return msp.requestRawIMU(&out);
}

bool InavAPI::get_rc(msp_rc_channels_t &out) {
    return msp.requestRcChannels(&out, MAX_RC_CHANNELS);
}

bool InavAPI::get_rc_ch(int ch, int &value) {
    if (ch < 0 || ch >= MAX_RC_CHANNELS) return false;
    msp_rc_channels_t rc{};
    if (!msp.requestRcChannels(&rc, MAX_RC_CHANNELS)) return false;
    value = rc.channel[ch];
    return true;
}

bool InavAPI::get_sensor_config(msp_sensor_config_t &out) {
    return msp.requestSensorConfig(&out);
}

bool InavAPI::get_wp(int index, inav_nav_waypoint &out) {
    if (index < 0 || index >= NAV_MAX_WAYPOINTS) return false;
    msp_nav_waypoint_t raw{};
    if (!msp.requestWaypoint((uint8_t)index, &raw)) return false;

    out.waypointIndex = raw.waypointIndex;
    out.action        = raw.action;
    out.latitude      = _i32_to_deg(raw.latitude);
    out.longitude     = _i32_to_deg(raw.longitude);
    out.altitude      = _cm_to_m(raw.altitude);
    out.param1        = raw.param1;
    out.param2        = raw.param2;
    out.param3        = raw.param3;
    out.flag          = raw.flag;
    return true;
}

bool InavAPI::get_mode_ranges(inav_mode_ranges &out) {
    uint16_t count = 0;
    if (!msp.requestModeRanges((msp_mode_ranges_t*)&out, &count)) {
        out.count = 0;
        return false;
    }
    out.count = count;
    return true;
}

bool InavAPI::get_active_modes(std::vector<uint8_t> &out) {
    _updateStatus();
    if (!_haveStatus) return false;
    out.clear();
    for (uint8_t bit = 0; bit < 64; ++bit) {
        if (_lastStatus.activeModes & ((boxBitmask_t)1 << bit)) {
            out.push_back(bit); // still bit index; translate if you cached BOXIDs
        }
    }
    return true;
}

static void decodeSensorBits(uint16_t bits, inav_sensors &s) {
    s.acc         = bits & (1 << 0);
    s.baro        = bits & (1 << 1);
    s.mag         = bits & (1 << 2);
    s.gps         = bits & (1 << 3);
    s.rangefinder = bits & (1 << 4);
    s.opflow      = bits & (1 << 5);
    s.pitot       = bits & (1 << 6);
    s.temp        = bits & (1 << 7);
    s.hwFailure   = bits & (1 << 15);
}

bool InavAPI::get_status(inav_status &out) {
    msp2_inav_status_t raw{};
    if (!msp.requestInavStatus(&raw)) return false;

    _lastStatus = raw;
    _haveStatus = true;

    out.cycleTime_us  = raw.cycleTime;
    out.cpuLoad_pct   = raw.cpuLoad;
    out.profile       =  raw.profileAndBattProfile & 0x0F;
    out.battProfile   = (raw.profileAndBattProfile >> 4) & 0x0F;
    out.mixerProfile  = raw.mixerProfile;
    out.armed         = msp.isArmed();
    out.armingFlags   = raw.armingFlags;
    out.activeModes   = raw.activeModes;

    decodeSensorBits(raw.sensorStatus, out.sensors);
    return true;
}

bool InavAPI::get_analog(inav_analog &out) {
    msp2_inav_analog_t a{};
    if (!msp.requestAnalog(&a)) return false;

    out.batteryFlags      = a.batteryFlags;
    out.vbat_V            = (float)a.vbat / 100.0f;
    out.amps_A            = (float)a.amperage / 100.0f;
    out.power_mW          = a.powerDraw;
    out.mAhDrawn          = a.mAhDrawn;
    out.mWhDrawn          = a.mWhDrawn;
    out.remainingCapacity = a.remainingCapacity;
    out.percentRemaining  = a.percentageRemaining;
    out.rssi              = a.rssi;
    return true;
}


// ---------- SETs / Cmd ----------
bool InavAPI::set_waypoint(const inav_nav_waypoint &wp) {
    msp_nav_waypoint_t raw{};
    raw.waypointIndex = (uint8_t)wp.waypointIndex;
    raw.action        = (uint8_t)wp.action;
    raw.latitude      = (int32_t)(wp.latitude  * 1e7);
    raw.longitude     = (int32_t)(wp.longitude * 1e7);
    raw.altitude      = (int32_t)(wp.altitude  * 100.0f);
    raw.param1        = (uint16_t)wp.param1;
    raw.param2        = (uint16_t)wp.param2;
    raw.param3        = (uint16_t)wp.param3;
    raw.flag          = (uint8_t)wp.flag;
    return msp.setWaypoint(&raw);
}

bool InavAPI::set_rc(const msp_rc_channels_t &rcin) {
    return msp.commandRawRC(rcin.channel, MAX_RC_CHANNELS);
}

bool InavAPI::command_mission_save(uint8_t id) {
    return msp.commandMissionSave(id);
}

bool InavAPI::command_mission_load(uint8_t id) {
    return msp.commandMissionLoad(id);
}

bool InavAPI::command_heading(int16_t headingDeg) {
    return msp.setHeading(headingDeg);
}

// ---------- CHECKs ----------
bool InavAPI::check_can_arm() {
    _updateStatus();
    if (!_haveStatus) return false;
    return !check_is_armed();
}

bool InavAPI::check_is_armed() {
    _updateStatus();
    if (!_haveStatus) return false;
    return msp.isArmed();
}

bool InavAPI::check_override_enabled() { return true; }
bool InavAPI::check_override_active()  { return true; }
