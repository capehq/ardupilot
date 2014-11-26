/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


// Format: ["CAPE", longitude (int32_t), latitude (int32_t), altitude (float), checksum (uint16_t)]
#define CAPE_MESSAGE_LENGTH 18            // (4 + sizeof(int32_t) + sizeof(int32_t) + sizeof(float) + sizeof(uint16_t))
static uint8_t _cape_rx_buffer[CAPE_MESSAGE_LENGTH];
static uint8_t _cape_prefix[] = "CAPE";
static uint8_t _cape_bytes_received = 0;

static int16_t _cape_wearable_longitude;
static int16_t _cape_wearable_latitude;
static float _cape_wearable_altitude;


void Cape_init() {
    // Set up Serial 4
    if(hal.uartE) {
        hal.uartE->begin(9600, 32, 32);
    }
}

void Cape_FastLoop() {
    if(Cape_ReadFromWearable()) {
        // New position received!


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
        uint16_t rx_checksum = *(uint16_t*)(&(_cape_rx_buffer[16]));
        *(uint16_t*)(&(_cape_rx_buffer[16])) = 0;

        if(rx_checksum == crc_calculate(_cape_rx_buffer, CAPE_MESSAGE_LENGTH)) {
            // Valid message
            valid_message = 1;

            _cape_wearable_longitude = *(int32_t*)(&(_cape_rx_buffer[4]));
            _cape_wearable_latitude = *(int32_t*)(&(_cape_rx_buffer[8]));
            _cape_wearable_altitude = *(float*)(&(_cape_rx_buffer[12]));
        }

        _cape_bytes_received = 0;
    }

    return valid_message;
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
