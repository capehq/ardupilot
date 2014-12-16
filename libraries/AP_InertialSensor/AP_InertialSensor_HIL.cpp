/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor_HIL.h"

const extern AP_HAL::HAL& hal;

AP_InertialSensor_HIL::AP_InertialSensor_HIL(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
<<<<<<< HEAD
=======
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        uint32_t tnow = hal.scheduler->micros();
        uint32_t tdelay = (_last_sample_usec + _sample_period_usec) - tnow;
        if (tdelay < 100000) {
            hal.scheduler->delay_microseconds(tdelay);
        }
        if (_sample_available()) {
            return true;
        }
    }
    return false;
>>>>>>> 3.2-ben-drone-gabe-adding-stuff
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_HIL::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_HIL *sensor = new AP_InertialSensor_HIL(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_HIL::_init_sensor(void) 
{
    // grab the used instances
    _imu.register_gyro();
    _imu.register_accel();

    _product_id = AP_PRODUCT_ID_NONE;
    _imu.set_hil_mode();

    return true;
}

bool AP_InertialSensor_HIL::update(void) 
{
    // the data is stored directly in the frontend, so update()
    // doesn't need to do anything
    return true;
}
