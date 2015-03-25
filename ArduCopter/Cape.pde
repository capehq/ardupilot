/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Format: ["CAPE", longitude (int32_t), latitude (int32_t), altitude (float), arm (uint8_t), misc (uint8_t), checksum (uint16_t)]
#define CAPE_MESSAGE_LENGTH 20            // (4 + sizeof(int32_t) + sizeof(int32_t) + sizeof(float) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t))
#define CAPE_MESSAGE_CHKSUM_POS (CAPE_MESSAGE_LENGTH - 2)
#define CAPE_MESSAGE_ARM_ARM 0xaa
#define CAPE_MESSAGE_ARM_DISARM 0x55
#define early_wp_index 3
#define sizeOfAltArray 100
#define HEARTBEAT_INTERVAL 5000 // in milliseconds
#define SEND_HEARTBEAT 0xF
#define HEARTBEAT_LENGTH 1 // length of heartbeat in bytes

static int currIndexAltAvg = 0;
static float currInterpAlt = 0;

static uint8_t _cape_rx_buffer[CAPE_MESSAGE_LENGTH];
static uint8_t _cape_prefix[] = "CAPE";
static uint8_t _cape_bytes_received = 0;

static int32_t _cape_wearable_longitude;
static int32_t _cape_wearable_latitude;
static float _cape_wearable_altitude;
static bool _cape_wearable_arm;
static uint8_t _cape_wearable_misc;
static float _cape_wearable_altitude_vec[sizeOfAltArray];
static float _cape_alt_average_sum = 0;

// #define CAPE_RAIL_DISTANCE_THRESHOLD 1000.f // Distance in cm

static AP_Mission::Mission_Command _cape_drone_prev_nav_cmd;
static AP_Mission::Mission_Command _cape_drone_curr_nav_cmd;
static bool _cape_nav_cmds_remaining;
static bool _cape_armed_once = false;
static bool _cape_waiting_for_takeoff = true;

static AP_Mission::Mission_Command _cape_wearable_prev_nav_cmd; 
static AP_Mission::Mission_Command _cape_wearable_curr_nav_cmd; //current waypoint ahead of user

void Cape_init() {
    // Set up Serial 4
    if(hal.uartE) {
        hal.uartE->begin(9600, 32, 32);
        hal.uartE->printf("There are %d commands\n", mission.num_commands());
    }

    // Load first navigation commands
    _cape_nav_cmds_remaining = false;
    if(mission.get_next_nav_cmd(1, _cape_drone_prev_nav_cmd)) {
        _cape_nav_cmds_remaining = mission.get_next_nav_cmd(_cape_drone_prev_nav_cmd.index + 1, _cape_drone_curr_nav_cmd);
        hal.uartE->printf("Prev index %d, cur index %d\n", _cape_drone_prev_nav_cmd.index, _cape_drone_curr_nav_cmd.index);
    }

    mission.get_next_nav_cmd(1, _cape_wearable_prev_nav_cmd);
    mission.get_next_nav_cmd(_cape_wearable_prev_nav_cmd.index + 1, _cape_wearable_curr_nav_cmd);
}

void Cape_FastLoop() {
    if(Cape_ReadFromWearable()) {
        time_cape_ROI_update = hal.scheduler->millis();
        // New position received!
        if(!_cape_armed_once) {
            set_mode(GUIDED);
            pre_arm_checks(true);
            if(ap.pre_arm_check && arm_checks(true)  && _cape_wearable_arm) {
                if (init_arm_motors()) {
                    set_auto_armed(true);
                    guided_takeoff_start(_cape_drone_prev_nav_cmd.content.location.alt+g.drone_height_off);
                    _cape_armed_once = true;
                    _cape_waiting_for_takeoff = true;
                }
            }
        }

        if(_cape_armed_once && _cape_waiting_for_takeoff) {
            if(inertial_nav.position_ok()) {
                if(fabsf((_cape_drone_prev_nav_cmd.content.location.alt+g.drone_height_off) - inertial_nav.get_altitude()) < 100.f) {
                    set_mode(AUTO);
                    _cape_waiting_for_takeoff = false;
                }
            }
        }

        if(_cape_armed_once && !_cape_waiting_for_takeoff) { //reached target altitude
            Cape_UpdateFollowPosition();
            currInterpAlt = getSkierAltitude();
            // Cape_SetROI();
        }
    }

    if (_cape_armed_once) {
        if (_cape_waiting_for_takeoff) currInterpAlt = 0; //ensure cam points at ground when taking off
        _cape_alt_average_sum = _cape_alt_average_sum - _cape_wearable_altitude_vec[currIndexAltAvg];
        _cape_alt_average_sum = _cape_alt_average_sum + currInterpAlt;
        _cape_wearable_altitude_vec[currIndexAltAvg] = currInterpAlt;
        currIndexAltAvg = (currIndexAltAvg + 1) % sizeOfAltArray;
        Cape_SetROI(); 
    }
}

//10Hz loop
void Cape_MediumLoop() {
    //move ROI updates here if 100hz is too fast...
    PulseGen(); // even though it is calling this function at 10 Hz, the pulse is emitted more slowly
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
            // _cape_wearable_altitude = *(float*)(&(_cape_rx_buffer[12]));
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

    distance_to_plane = getDistanceToPlane(_cape_drone_curr_nav_cmd.index);

    hal.uartE->printf("Distance %f\n", distance_to_plane);

    if((distance_to_plane <= g.rail_distance_threshold && _cape_drone_curr_nav_cmd.index>early_wp_index) || 
        (distance_to_plane <= g.early_dist_thres && _cape_drone_curr_nav_cmd.index<=early_wp_index)) { 
        // Move to next waypoint
        _cape_drone_prev_nav_cmd = _cape_drone_curr_nav_cmd;
        _cape_nav_cmds_remaining = mission.get_next_nav_cmd(_cape_drone_prev_nav_cmd.index + 1, _cape_drone_curr_nav_cmd);
        if(_cape_nav_cmds_remaining) {
            mission.set_current_cmd(_cape_drone_curr_nav_cmd.index);
        }
    }


    // To update which waypoint the skier is approaching
    dist_skier_to_plane=getDistanceToPlane(_cape_wearable_curr_nav_cmd.index);
    if (dist_skier_to_plane < 0) {
        if (_cape_wearable_curr_nav_cmd.index <= mission.num_commands() - 3) { //don't advance if next waypoint is RTL (last wp)
            _cape_wearable_prev_nav_cmd = _cape_wearable_curr_nav_cmd;
            mission.get_next_nav_cmd(_cape_wearable_prev_nav_cmd.index + 1, _cape_wearable_curr_nav_cmd);
        }
    }
}

// 2D distance of user from plane that intersects waypoint_number
float getDistanceToPlane(int waypoint_number) {
    static AP_Mission::Mission_Command _temp_prev_nav_cmd; 
    static AP_Mission::Mission_Command _temp_curr_nav_cmd; 

    mission.get_next_nav_cmd(waypoint_number-1, _temp_prev_nav_cmd); 
    mission.get_next_nav_cmd(waypoint_number, _temp_curr_nav_cmd);

    // Calculate distances in cm
    float lon_to_cm_scaling = longitude_scale(ahrs.get_home()) * LATLON_TO_CM;

    // Curr waypoint to prev waypoint
    int32_t wp_dlng = _temp_curr_nav_cmd.content.location.lng - _temp_prev_nav_cmd.content.location.lng;
    float wp_dlng_f = wp_dlng * lon_to_cm_scaling;
    int32_t wp_dlat = _temp_curr_nav_cmd.content.location.lat - _temp_prev_nav_cmd.content.location.lat;
    float wp_dlat_f = wp_dlat * LATLON_TO_CM;
    // float wp_dalt_f = _cape_drone_curr_nav_cmd.content.location.alt - _cape_drone_prev_nav_cmd.content.location.alt; //2d distance now

    // Normalize waypoint-to-prev-waypoint
    float norm_wp = sqrtf(powf(wp_dlng_f, 2) + powf(wp_dlat_f, 2));// + powf(wp_dalt_f, 2));
    wp_dlng_f /= norm_wp;
    wp_dlat_f /= norm_wp;
    // wp_dalt_f /= norm_wp;

    // Curr waypoint to wearable
    int32_t ww_dlng = _temp_curr_nav_cmd.content.location.lng - _cape_wearable_longitude;
    float ww_dlng_f = ww_dlng * lon_to_cm_scaling;
    int32_t ww_dlat = _temp_curr_nav_cmd.content.location.lat - _cape_wearable_latitude;
    float ww_dlat_f = ww_dlat * LATLON_TO_CM;
    // float ww_dalt_f = _cape_drone_curr_nav_cmd.content.location.alt - _cape_wearable_altitude;

    // Dot product of norm(waypoint to prev waypoint) and (waypoint to wearable)
    return (ww_dlng_f * wp_dlng_f) + (ww_dlat_f * wp_dlat_f); // + (ww_dalt_f * wp_dalt_f);    
}

// Returns altitude of skier
float getSkierAltitude() {
    float distanceWearableToPlane = getDistanceToPlane(_cape_wearable_curr_nav_cmd.index);
    float distanceBetweenWaypoints = abs(get_distance_cm(_cape_wearable_prev_nav_cmd.content.location,_cape_wearable_curr_nav_cmd.content.location)); // function defined in ../libraries/AP_Math/location.cpp. 2D distance between current and previous waypoints
    float wpAltDif = _cape_wearable_prev_nav_cmd.content.location.alt-_cape_wearable_curr_nav_cmd.content.location.alt;
    return wpAltDif * distanceWearableToPlane / distanceBetweenWaypoints + _cape_wearable_curr_nav_cmd.content.location.alt;
}



void Cape_SetROI() {
    Location roi_loc;
    roi_loc.lat = _cape_wearable_latitude;
    roi_loc.lng = _cape_wearable_longitude;
    // roi_loc.alt = _cape_wearable_altitude; // needs to be changed
    roi_loc.alt = computeSkierAltAvg();
    roi_gps_coords=roi_loc; // for logging purposes
    set_auto_yaw_roi(roi_loc);
}

float computeSkierAltAvg() {
    return _cape_alt_average_sum/sizeOfAltArray;
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

// 2013-03-25 Addition for watchdog timer functionality
// Added by Alex Loo

/***********************************************************************
Function
    PulseGen
Parameters
    none
Returns
    none
Description
    This function generates a "heart beat" message over the XBee radio.
Notes
    none yet
Author
    Alex Loo
***********************************************************************/
void PulseGen() {
    static uint16_t lastTime = 0;
    uint16_t currentTime;

    currentTime = hal.scheduler->millis();

    // Check if drone has passed the last follow waypoint
    if (_cape_drone_curr_nav_cmd.index == mission.num_commands()-1) {
        // Drone is returning to base, so no need to send pulse. Immediately exit
        return;
    }
    // Else, we are still doing the follow mission, so send out the heartbeat at the interval
    else if (currentTime - lastTime > HEARTBEAT_INTERVAL) {
        // interval has ellapsed, send heartbeat message
        if (hal.uartE) {
            hal.uartE->write(SEND_HEARTBEAT, HEARTBEAT_LENGTH);
        }
        lastTime = currentTime; // update last time
    }
    return;
}