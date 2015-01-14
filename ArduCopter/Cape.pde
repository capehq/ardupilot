/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CAPE_UPDATE_INTERVAL 10
static uint32_t _cape_update_counter = 0;
#define CAPE_ARM_DELAY 50 // 500ms delay
static uint32_t _cape_arm_counter = 0;
static bool _cape_arm_state;
static bool _cape_armed_once=false;

// Format: ["CAPE", longitude (int32_t), latitude (int32_t), altitude (float), arm (uint8_t), misc (uint8_t), checksum (uint16_t)]
#define CAPE_MESSAGE_LENGTH 20            // (4 + sizeof(int32_t) + sizeof(int32_t) + sizeof(float) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t))
#define CAPE_MESSAGE_CHKSUM_POS (CAPE_MESSAGE_LENGTH - 2)
#define CAPE_MESSAGE_ARM_ARM 0xaa
#define CAPE_MESSAGE_ARM_DISARM 0x55
static uint8_t _cape_tx_buffer[CAPE_MESSAGE_LENGTH] = "CAPE";

#define PX4_WEARABLE_LED                130
#define PX4_WEARABLE_BTN                131

void Cape_init() {
    // Set up Serial 4
    if(hal.uartE) {
        hal.uartE->begin(9600, 32, 32);
    }
}

void Cape_FastLoop() {
    // Check and update button state
    if(hal.gpio->read(PX4_WEARABLE_BTN) == HIGH) {
        if(_cape_arm_counter > CAPE_ARM_DELAY) {
            // Do nothing
        }
        if(_cape_arm_counter == CAPE_ARM_DELAY) {
            _cape_arm_state = !_cape_arm_state;
            _cape_arm_counter++;
        }
        else
        {
            _cape_arm_counter++;
        }
    }
    else {
        _cape_arm_counter = 0;
    }

    if (!_cape_armed_once && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        set_mode(GUIDED);
        pre_arm_checks(true);
        if(ap.pre_arm_check && arm_checks(true) ) { // && _cape_wearable_arm) {
            if (init_arm_motors()) {
                set_auto_armed(true);
                _cape_armed_once=true;
            }
        }
    }

    hal.gpio->write(PX4_WEARABLE_LED, _cape_arm_state ? HIGH : LOW);

    // Send update to drone
    if(_cape_update_counter <= 0) {
        if( inertial_nav.position_ok() ) {
            // pull position from interial nav library
            int32_t longitude = inertial_nav.get_longitude();
            int32_t latitude = inertial_nav.get_latitude();
            float altitude = inertial_nav.get_altitude();

            *(int32_t*)(&(_cape_tx_buffer[4])) = longitude;
            *(int32_t*)(&(_cape_tx_buffer[8])) = latitude;
            *(float*)(&(_cape_tx_buffer[12])) = altitude;
            *(uint8_t*)(&(_cape_tx_buffer[16])) = _cape_arm_state ? CAPE_MESSAGE_ARM_ARM : CAPE_MESSAGE_ARM_DISARM;
            *(uint8_t*)(&(_cape_tx_buffer[17])) = 0; // misc, unused
            *(uint16_t*)(&(_cape_tx_buffer[CAPE_MESSAGE_CHKSUM_POS])) = 0;

            uint16_t checksum = crc_calculate(_cape_tx_buffer, CAPE_MESSAGE_LENGTH);

            *(uint16_t*)(&(_cape_tx_buffer[CAPE_MESSAGE_CHKSUM_POS])) = checksum;

            if(hal.uartE) {
                hal.uartE->write(_cape_tx_buffer, CAPE_MESSAGE_LENGTH);
            }

            _cape_update_counter = CAPE_UPDATE_INTERVAL;
        }
    }
    else
        _cape_update_counter--;
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