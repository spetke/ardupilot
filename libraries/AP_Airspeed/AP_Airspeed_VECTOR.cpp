/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Airspeed_VECTOR.h"

#if AP_AIRSPEED_VECTOR_ENABLED

#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define VECTOR_I2C_ADDR_1 0x28
#define REG_SPEED_DATA7BIT 0x28
#define REG_SPEED_DATA8BIT 0x50

bool AP_Airspeed_VECTOR::init()
{
    // probe the sensor, supporting multiple possible I2C addresses
    const uint8_t addresses[] = {VECTOR_I2C_ADDR_1};
    for (uint8_t address : addresses)
    {
        dev = hal.i2c_mgr->get_device(get_bus(), address);
        if (!dev)
        {
            continue;
        }

        WITH_SEMAPHORE(dev->get_semaphore());
        dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        dev->set_retries(2);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VECTOR[%u]: Found on bus %u ", get_instance(), get_bus());
        dev->set_device_type(uint8_t(DevType::VECTOR));
        set_bus_id(dev->get_bus_id());

        dev->register_periodic_callback(10000,
                                        FUNCTOR_BIND_MEMBER(&AP_Airspeed_VECTOR::timer, void));
        return true;
    }

    // not found
    return false;
}
// read the data from the sensor
void AP_Airspeed_VECTOR::timer()
{
    // uint8_t status;
    //  if (!dev->read_registers(REG_SPE, &status, 1) ||
    //      (status & REG_SENSOR_READY) == 0)
    // {
    // no data ready
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ei dataa");
    //     return;
    // }

    // read data
    uint8_t data[2];
    if (!dev->read_registers(REG_SPEED_DATA7BIT, data, sizeof(data)))
    {
        return;
    }
    // speed  is signed 16 bit
    int16_t speed = (data[0] << 8) | data[1];
    WITH_SEMAPHORE(sem);
/*     speed_sum += speed;
    speed_count++;
    temp_sum += temp;
    temp_count++;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VECTOR nopeus näyttää olevan %u ", speed); */
    last_speed = (float)speed;
    last_sample_ms = AP_HAL::millis();
}

bool AP_Airspeed_VECTOR::get_airspeed(float &airspeed)
{
    WITH_SEMAPHORE(sem);

/*     if (AP_HAL::millis() - last_sample_ms > 100)
    {
        return false;
    } */
    airspeed = last_speed;
    return true;

}
// This has to be implemnted to fullfill  interface requirements for some reason
bool AP_Airspeed_VECTOR::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    return false;
}

#endif
