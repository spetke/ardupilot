#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_VECTOR_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>
#include "AP_Airspeed_Backend.h"

class AP_Airspeed_VECTOR : public AP_Airspeed_Backend
{
public:

    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    // probe and initialise the sensor
    bool init(void) override;

    // this reads airspeed directly
    bool has_airspeed() override {return true;}

    // read the from the sensor
    bool get_airspeed(float &airspeed) override;

private:
    void timer();
    bool confirm_sensor_id(void);
    float temp_sum;
    float press_sum;
    float last_pressure;
    float last_temperature;
    uint32_t press_count;
    uint32_t temp_count;
    uint32_t last_sample_ms;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    // store last sent speed and temp as update rate is slow
    float _last_temp;
    float _last_speed;

    // time last message was received
    uint32_t _last_update_ms;
};

#endif  // AP_AIRSPEED_VECTOR_ENABLED
