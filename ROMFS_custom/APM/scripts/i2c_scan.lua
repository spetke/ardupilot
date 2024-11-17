-- This script scans for devices on the i2c bus

---@diagnostic disable: need-check-nil

local address = 0
local found = 0

local sensor = i2c:get_device(0, 40)
sensor:set_retries(10)


local function readRegister16(addr)
    local lsb = sensor:read_registers(addr + 0)
    local msb = sensor:read_registers(addr + 1)

    return lsb
end
function update() -- this is the loop which periodically runs
    local test = readRegister16(80)
    gcs:send_text(0, "sensor " .. tostring(sensor))
    gcs:send_text(0, "lukema " .. tostring(test))

    return update, 100 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
