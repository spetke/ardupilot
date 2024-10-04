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
/*
Driver for VectorSave Airspeed sensor   
 */

#include "AP_Airspeed_VECTOR.h"

#if AP_AIRSPEED_VECTOR_ENABLED

#include "AP_Airspeed.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#define TIMEOUT_MS 2000
extern const AP_HAL::HAL &hal;

#define VECTOR_I2C_ADDR_8B 0x50
#define VECTOR_I2C_ADDR_7B 0x28
#define REG_CMD 0x30
#define REG_PRESS_DATA 0x06
#define REG_TEMP_DATA 0x09
#define REG_PART_ID 0x01
#define REG_PART_ID_SET 0xa4
#define REG_SENSOR_READY 0x08
#define REG_WHOAMI_DEFAULT_ID 0X00
#define REG_WHOAMI_RECHECK_ID 0X66
#define CMD_MEASURE 0x0A

bool AP_Airspeed_VECTOR::init()
{
    // probe the sensor, supporting multiple possible I2C addresses
    const uint8_t addresses[] = {VECTOR_I2C_ADDR_7B, VECTOR_I2C_ADDR_8B};
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

        if (!confirm_sensor_id())
        {
            continue;
        }

        dev->set_device_type(uint8_t(DevType::VECTOR));
        set_bus_id(dev->get_bus_id());

        dev->register_periodic_callback(1000000UL / 80U,
                                        FUNCTOR_BIND_MEMBER(&AP_Airspeed_VECTOR::timer, void));
        return true;
    }

    // not found
    return false;
}
// read the from the sensor
bool AP_Airspeed_VECTOR::get_airspeed(float &airspeed)
{


    return (now - _last_update_ms) < TIMEOUT_MS;
}

#endif  // AP_AIRSPEED_VECTOR_ENABLED
