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

#define HEARTBEAT_MESSAGE 0xFE
#define HEARTBEAT_TIMEOUT 15000 // in milliseconds
//#define DEBUG

void Cape_init() {
    // Set up Serial 4
    if(hal.uartE) {
        hal.uartE->begin(9600, 2, 32);
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

    // arm motors (initialize stuff and start logging)
    if (!_cape_armed_once && gps.status() >= AP_GPS::GPS_OK_FIX_3D && _cape_arm_state==true) {
        set_mode(GUIDED);
        pre_arm_checks(true);
        if(ap.pre_arm_check && arm_checks(true) ) {
            if (init_arm_motors()) {
                set_auto_armed(true);
                _cape_armed_once=true;
            }
        }
    }

    hal.gpio->write(PX4_WEARABLE_LED, _cape_arm_state ? HIGH : LOW);

    // Send update to drone
    if(_cape_update_counter <= 0 ) {
        if( inertial_nav.position_ok() && _cape_arm_state==true) { // if we have an inertial fix and have pressed the button
            // pull position from interial nav library
            // int32_t longitude = inertial_nav.get_longitude();
            // int32_t latitude = inertial_nav.get_latitude();
            int32_t longitude = gps.location().lng;
            int32_t latitude = gps.location().lat;
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

// 2013-03-25 Addition for watchdog timer functionality
// Added by Alex Loo

/* 
I had to add this function to print formatted debug messages. I tried to use 
gcs_send_text_fmt, but I would always get an error. I traced this error to be that
the prototype for gcs_send_text_fmt was always after my code in ArduCopter.cpp. I
could not make it move up, so I just copied the function as one of my own to force
it to work.
*/
void gcs_send_cape_debug(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
    gcs[0].send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            gcs[i].send_message(MSG_STATUSTEXT);
        }
    }
}

/***********************************************************************
Function
    Cape_PulseCheck
Parameters
    none
Returns
    none
Description
    This function disarms the wearable if a heartbeat has not been received in a predetermined time
Notes
    none yet
Author
    Alex Loo
***********************************************************************/

void Cape_PulseCheck() {
    static uint32_t lastTime = 0;
    uint32_t currentTime;
    uint32_t diffTime;

    //gcs_send_text_P(SEVERITY_LOW, PSTR("Starting pulse check routine.\n"));
    //delay(500);
    currentTime = hal.scheduler->millis();
    diffTime = currentTime - lastTime;
    //gcs_send_cape_debug(PSTR("The difference is %u ms.\n"), (unsigned)(diffTime));
    //delay(500);


    if (!_cape_arm_state) {
        // Wearable is not armed, so just update the counter
        //gcs_send_text_P(SEVERITY_LOW, PSTR("Not armed. Updating last time.\n"));
        //delay(500);
        lastTime = currentTime;
        flush_Rx_buffer(); // clear the XBee Rx buffer.
    } 
    else {
        // Wearable is armed
        // First check if the XBee has received a heartbeat from the drone.
        if (hal.uartE-> available() > 0) {
            //gcs_send_text_P(SEVERITY_LOW, PSTR("UART has data."));
            //delay(500);
            uint8_t new_byte = hal.uartE->read(); // check XBee for new message

            #ifdef DEBUG
            // spoof a received message
            if (diffTime >= 10000)
            {
                new_byte = 0xFE;
                gcs_send_text_P(SEVERITY_MED, PSTR("Spoofing a received hearbeat."));
                //delay(500);
            }
            #endif

            if (new_byte == HEARTBEAT_MESSAGE) {
                // there was a new message and it was the correct heartbeat --> reset the timer
                gcs_send_text_P(SEVERITY_HIGH, PSTR("Heartbeat received.\n"));
                //delay(500);
                lastTime = currentTime;
            }
        }
        // Second check if enough time has elapsed without a received heartbeat to disarm the wearable
        if (diffTime >= HEARTBEAT_TIMEOUT) {
            //gcs_send_text_P(SEVERITY_HIGH, PSTR("No heartbeat received in a while. Disarming the wearable.\n"));
            //delay(500);
            _cape_arm_state = false; // no heartbeat received in a while, disarm the wearable
            init_disarm_motors();
            // added the following 4 lines on 2015.07.08 (aloo)
            pre_arm_checks(false);
            set_auto_armed(false);
            _cape_armed_once = false;
            _cape_arm_counter = 0;
        }
    }
    //gcs_send_text_P(SEVERITY_LOW, PSTR("Exiting pulse check routine."));
    //delay(500);
}



/*
void Cape_PulseCheck() {
    static uint32_t lastTime = 0;
    uint32_t currentTime;
    uint32_t diffTime;

    gcs_send_text_P(SEVERITY_LOW, PSTR("Running pulse check.\n"));

    // If the wearable is not currently armed update lastTime and exit out
    if (_cape_arm_state == false){
        gcs_send_text_P(SEVERITY_LOW, PSTR("Not armed. Updating last time.\n"));
        lastTime = hal.scheduler->millis();
        return;
    }
    
    //gcs_send_text_P(SEVERITY_LOW, PSTR("Currently armed. Further checks needed."));

    currentTime = hal.scheduler->millis();
    gcs_send_cape_debug(PSTR("The current time is %u ms.\n"), (unsigned)currentTime);
    delay(100);
    gcs_send_cape_debug(PSTR("The last time is %u ms.\n"), (unsigned)lastTime);
    delay(100);
    diffTime = currentTime - lastTime;
    gcs_send_cape_debug(PSTR("The difference is %u ms.\n"), (unsigned)(diffTime));
    delay(50);
    
    if (diffTime >= HEARTBEAT_TIMEOUT)
    {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Time difference exceeds debug threshold.\n"));
        delay(50);
        _cape_arm_state = false; // no heartbeat received in a while, disarm the wearable
        init_disarm_motors();
        gcs_send_text_P(SEVERITY_LOW, PSTR("No heartbeat received in a while. Disarming the wearable.\n"));
    }

    // Check if a pulse has been received
    if (hal.uartE) {
        int8_t new_byte = hal.uartE->read(); // check XBee for new message
        if (new_byte == HEARTBEAT_MESSAGE) {
            // there was a new message and it was the correct heartbeat --> reset the timer
            gcs_send_text_P(SEVERITY_LOW, PSTR("Heartbeat received.\n"));
            lastTime = currentTime;
            return; // can exit function as no further checks needed
        }
    }
    // if no new pulse, check to see if the last one was received longer ago than timeout allows
    else if (currentTime - lastTime > 10000) {
        _cape_arm_state = false; // no heartbeat received in a while, disarm the wearable
        init_disarm_motors();
        gcs_send_text_P(SEVERITY_LOW, PSTR("No heartbeat received in a while. Disarming the wearable.\n"));
    }
    return;
}
*/

// 2013-07-16 Addition for watchdog timer functionality
// Added by Alex Loo
/***********************************************************************
Function
    flush_Rx_buffer
Parameters
    none
Returns
    none
Description
    This function keeps reading the UART E's Rx buffer until it is clear.
Notes
    none yet
Author
    Alex Loo
***********************************************************************/
void flush_Rx_buffer() {
    uint8_t garbageByte;

    while (hal.uartE->available() > 0) {
        garbageByte = hal.uartE->read();
    }

}