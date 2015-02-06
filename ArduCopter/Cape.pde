/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Format: ["CAPE", longitude (int32_t), latitude (int32_t), altitude (float), arm (uint8_t), misc (uint8_t), checksum (uint16_t)]
#define CAPE_MESSAGE_LENGTH 20            // (4 + sizeof(int32_t) + sizeof(int32_t) + sizeof(float) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t))
#define CAPE_MESSAGE_CHKSUM_POS (CAPE_MESSAGE_LENGTH - 2)
#define CAPE_MESSAGE_ARM_ARM 0xaa
#define CAPE_MESSAGE_ARM_DISARM 0x55
#define early_wp_index 3
static uint8_t _cape_rx_buffer[CAPE_MESSAGE_LENGTH];
static uint8_t _cape_prefix[] = "CAPE";
static uint8_t _cape_bytes_received = 0;

static int32_t _cape_wearable_longitude;
static int32_t _cape_wearable_latitude;
static float _cape_wearable_altitude;
static bool _cape_wearable_arm;
static uint8_t _cape_wearable_misc;

// #define CAPE_RAIL_DISTANCE_THRESHOLD 1000.f // Distance in cm

static AP_Mission::Mission_Command _cape_prev_nav_cmd;
static AP_Mission::Mission_Command _cape_curr_nav_cmd;
static bool _cape_nav_cmds_remaining;
static bool _cape_armed_once = false;
static bool _cape_waiting_for_takeoff = true;

void Cape_init() {
    // Set up Serial 4
    if(hal.uartE) {
        hal.uartE->begin(9600, 32, 32);
        hal.uartE->printf("There are %d commands\n", mission.num_commands());
    }

    // Load first navigation commands
    _cape_nav_cmds_remaining = false;
    if(mission.get_next_nav_cmd(1, _cape_prev_nav_cmd)) {
        _cape_nav_cmds_remaining = mission.get_next_nav_cmd(_cape_prev_nav_cmd.index + 1, _cape_curr_nav_cmd);
        hal.uartE->printf("Prev index %d, cur index %d\n", _cape_prev_nav_cmd.index, _cape_curr_nav_cmd.index);
    }
}

void Cape_FastLoop() {
    if(Cape_ReadFromWearable()) {
        // New position received!
        if(!_cape_armed_once) {
            set_mode(GUIDED);
            pre_arm_checks(true);
            if(ap.pre_arm_check && arm_checks(true)  && _cape_wearable_arm) {
                if (init_arm_motors()) {
                    set_auto_armed(true);
                    guided_takeoff_start(_cape_prev_nav_cmd.content.location.alt);
                    _cape_armed_once = true;
                    _cape_waiting_for_takeoff = true;
                }
            }
        }

        if(_cape_armed_once && _cape_waiting_for_takeoff) {
            if(inertial_nav.position_ok()) {
                if(fabsf(_cape_prev_nav_cmd.content.location.alt - inertial_nav.get_altitude()) < 100.f) {
                    set_mode(AUTO);
                    _cape_waiting_for_takeoff = false;
                }
            }
        }

        if(_cape_armed_once && !_cape_waiting_for_takeoff) {
            Cape_UpdateFollowPosition();
            // Cape_SetROI();
        }

        if (_cape_armed_once) {
            Cape_SetROI(); // Set ROI soon as we take-off (so we get pitch tracking on take-off too). Does not cause drone to yaw during takeoff
        }

    }
}

int Cape_ReadFromWearable() {
    int message_received = 0;

    if(hal.uartE) {
        int16_t new_byte = hal.uartE->read();
        while(new_byte != -1) {
            if(Cape_ValidateMessage(new_byte))
                message_received = 1;

            // Try to read another byte
            new_byte = hal.uartE->read();
        }
    }

    return message_received;
}

int Cape_ValidateMessage(uint8_t byte) {
    // Check for invalid prefix
    if(_cape_bytes_received < 4) {
        if(byte != _cape_prefix[_cape_bytes_received]) {
            _cape_bytes_received = 0;
            return 0;
        }
    }

    // Add new byte to buffer
    _cape_rx_buffer[_cape_bytes_received] = byte;
    _cape_bytes_received++;

    // Check for complete message
    int valid_message = 0;
    if(_cape_bytes_received == CAPE_MESSAGE_LENGTH) {
        uint16_t rx_checksum = *(uint16_t*)(&(_cape_rx_buffer[CAPE_MESSAGE_CHKSUM_POS]));
        *(uint16_t*)(&(_cape_rx_buffer[CAPE_MESSAGE_CHKSUM_POS])) = 0;

        if(rx_checksum == crc_calculate(_cape_rx_buffer, CAPE_MESSAGE_LENGTH)) {
            // Valid message
            valid_message = 1;

            _cape_wearable_longitude = *(int32_t*)(&(_cape_rx_buffer[4]));
            _cape_wearable_latitude = *(int32_t*)(&(_cape_rx_buffer[8]));
            _cape_wearable_altitude = *(float*)(&(_cape_rx_buffer[12]));
            _cape_wearable_misc = *(uint8_t*)(&(_cape_rx_buffer[17])); // unused

            uint8_t arm_state = *(uint8_t*)(&(_cape_rx_buffer[16]));
            if(!_cape_wearable_arm && (arm_state == CAPE_MESSAGE_ARM_ARM))
                _cape_wearable_arm = true;
            else if(_cape_wearable_arm && (arm_state == CAPE_MESSAGE_ARM_DISARM))
                _cape_wearable_arm = false;
        }

        _cape_bytes_received = 0;
    }

    return valid_message;
}

void Cape_UpdateFollowPosition() {

    // If no waypoints remaining, abort
    if(!_cape_nav_cmds_remaining)
      return;

    // Drone location
    if(!inertial_nav.position_ok()) {
        return;
    }
    int32_t longitude = inertial_nav.get_longitude();
    int32_t latitude = inertial_nav.get_latitude();
    float altitude = inertial_nav.get_altitude();

    // Calculate distances in cm
    float lon_to_cm_scaling = longitude_scale(ahrs.get_home()) * LATLON_TO_CM;

    // Curr waypoint to prev waypoint
    int32_t wp_dlng = _cape_curr_nav_cmd.content.location.lng - _cape_prev_nav_cmd.content.location.lng;
    float wp_dlng_f = wp_dlng * lon_to_cm_scaling;
    int32_t wp_dlat = _cape_curr_nav_cmd.content.location.lat - _cape_prev_nav_cmd.content.location.lat;
    float wp_dlat_f = wp_dlat * LATLON_TO_CM;
    float wp_dalt_f = _cape_curr_nav_cmd.content.location.alt - _cape_prev_nav_cmd.content.location.alt;

    // Normalize waypoint-to-prev-waypoint
    float norm_wp = sqrtf(powf(wp_dlng_f, 2) + powf(wp_dlat_f, 2) + powf(wp_dalt_f, 2));
    wp_dlng_f /= norm_wp;
    wp_dlat_f /= norm_wp;
    wp_dalt_f /= norm_wp;

    // Curr waypoint to wearable
    int32_t ww_dlng = _cape_curr_nav_cmd.content.location.lng - _cape_wearable_longitude;
    float ww_dlng_f = ww_dlng * lon_to_cm_scaling;
    int32_t ww_dlat = _cape_curr_nav_cmd.content.location.lat - _cape_wearable_latitude;
    float ww_dlat_f = ww_dlat * LATLON_TO_CM;
    float ww_dalt_f = _cape_curr_nav_cmd.content.location.alt - _cape_wearable_altitude;

    // Dot product of norm(waypoint to prev waypoint) and (waypoint to wearable)
    distance_to_plane = (ww_dlng_f * wp_dlng_f) + (ww_dlat_f * wp_dlat_f) + (ww_dalt_f * wp_dalt_f);
    hal.uartE->printf("Distance %f\n", distance_to_plane);

    if((distance_to_plane <= g.rail_distance_threshold && _cape_curr_nav_cmd.index>early_wp_index) || 
        (distance_to_plane <= g.early_dist_thres && _cape_curr_nav_cmd.index<=early_wp_index)) { // temporary shitty code, will fix if it works
        // Move to next waypoint
        _cape_prev_nav_cmd = _cape_curr_nav_cmd;
        _cape_nav_cmds_remaining = mission.get_next_nav_cmd(_cape_prev_nav_cmd.index + 1, _cape_curr_nav_cmd);
        if(_cape_nav_cmds_remaining) {
            mission.set_current_cmd(_cape_curr_nav_cmd.index);
        }
    }
}

void Cape_SetROI() {
    Location roi_loc;
    roi_loc.lat = _cape_wearable_latitude;
    roi_loc.lng = _cape_wearable_longitude;
    roi_loc.alt = _cape_wearable_altitude;
    roi_gps_coords=roi_loc; // gabe added. If this doesn't work, maybe assign lat/lng/alt separately
    set_auto_yaw_roi(roi_loc);
}






//// Checksum from mavlink checksum.h

#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8

/**
* @brief Accumulate the X.25 CRC by adding one char at a time.
*
* The checksum function adds the hash of one char at a time to the
* 16 bit checksum (uint16_t).
*
* @param data new char to hash
* @param crcAccum the already accumulated checksum
**/
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
    /*Accumulate one byte of data into the CRC*/
    uint8_t tmp;

    tmp = data ^ (uint8_t)(*crcAccum &0xff);
    tmp ^= (tmp<<4);
    *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

/**
* @brief Initiliaze the buffer for the X.25 CRC
*
* @param crcAccum the 16 bit X.25 CRC
*/
static inline void crc_init(uint16_t* crcAccum)
{
    *crcAccum = X25_INIT_CRC;
}


/**
* @brief Calculates the X.25 checksum on a byte buffer
*
* @param  pBuffer buffer containing the byte array to hash
* @param  length  length of the byte array
* @return the checksum over the buffer bytes
**/
static inline uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
    uint16_t crcTmp;
    crc_init(&crcTmp);
    while (length--) {
        crc_accumulate(*pBuffer++, &crcTmp);
    }
    return crcTmp;
}

/**
* @brief Accumulate the X.25 CRC by adding an array of bytes
*
* The checksum function adds the hash of one char at a time to the
* 16 bit checksum (uint16_t).
*
* @param data new bytes to hash
* @param crcAccum the already accumulated checksum
**/
static inline void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint8_t length)
{
    const uint8_t *p = (const uint8_t *)pBuffer;
    while (length--) {
        crc_accumulate(*p++, crcAccum);
    }
}