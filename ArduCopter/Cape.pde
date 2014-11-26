/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CAPE_UPDATE_INTERVAL 100
static uint32_t _cape_update_counter = 0;

void Cape_init() {
    // Set up Serial 4
    if(hal.uartE) {
        hal.uartE->begin(9600, 32, 32);
    }
}

void Cape_FastLoop() {
    if(!_cape_update_counter) {
        if( inertial_nav.position_ok() ) {
            // pull position from interial nav library
            int32_t longitude = inertial_nav.get_longitude();
            int32_t latitude = inertial_nav.get_latitude();
            float altitude = inertial_nav.get_altitude();

            if(hal.uartE) {
                hal.uartE->printf("hello, world\n");
            }

            _cape_update_counter = CAPE_UPDATE_INTERVAL;
        }
    }
    else
        _cape_update_counter--;
}
