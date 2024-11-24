-- Load the i2c library
local sensor = i2c:get_device(0, 40, 400000)
sensor:set_retries(10)



function update() -- this is the loop which periodically runs
    local lsb = sensor:read_registers(0)
    local fsb = sensor:read_registers(40)
    local msb = sensor:read_registers(80)




    gcs:send_text(0, "sensor " .. tostring(sensor))
    gcs:send_text(0, "sensor  data rekisterissä 0 " .. tostring(lsb))
    gcs:send_text(0, "sensor  data rekisterissä 40 " .. tostring(fsb))
    gcs:send_text(0, "sensor  data rekisterissä 80 " .. tostring(msb))


    return update, 100 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
