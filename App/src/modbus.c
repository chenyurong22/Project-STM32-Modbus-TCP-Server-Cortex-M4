#include "modbus.h"

#include <string.h>

static uint8_t coils[MB_MAX_COILS];
static uint8_t discrete_inputs[MB_MAX_DISCRETE_INPUTS];
static uint16_t holding_registers[MB_MAX_HREGS];
static uint16_t input_registers[MB_MAX_IREGS];

void mb_init(void)
{
    mb_lock();
    memset(coils, 0, sizeof(coils));
    memset(discrete_inputs, 0, sizeof(discrete_inputs));
    memset(holding_registers, 0, sizeof(holding_registers));
    memset(input_registers, 0, sizeof(input_registers));
    mb_unlock();
}

uint8_t mb_get_coil(uint16_t address)
{
    return address < MB_MAX_COILS ? (uint8_t)(coils[address] & 1u) : 0u;
}

void mb_set_coil(uint16_t address, uint8_t value)
{
    uint8_t normalized;
    if (address >= MB_MAX_COILS) {
        return;
    }
    normalized = value != 0u ? 1u : 0u;
    mb_lock();
    coils[address] = normalized;
    mb_unlock();
    mb_on_write_coil(address, normalized);
}

uint8_t mb_get_dinput(uint16_t address)
{
    return address < MB_MAX_DISCRETE_INPUTS ? (uint8_t)(discrete_inputs[address] & 1u) : 0u;
}

void mb_set_dinput(uint16_t address, uint8_t value)
{
    if (address >= MB_MAX_DISCRETE_INPUTS) {
        return;
    }
    mb_lock();
    discrete_inputs[address] = value != 0u ? 1u : 0u;
    mb_unlock();
}

uint16_t mb_get_hreg(uint16_t address)
{
    return address < MB_MAX_HREGS ? holding_registers[address] : 0u;
}

void mb_set_hreg(uint16_t address, uint16_t value)
{
    if (address >= MB_MAX_HREGS) {
        return;
    }
    mb_lock();
    holding_registers[address] = value;
    mb_unlock();
    mb_on_write_hreg(address, value);
}

uint16_t mb_get_ireg(uint16_t address)
{
    return address < MB_MAX_IREGS ? input_registers[address] : 0u;
}

void mb_set_ireg(uint16_t address, uint16_t value)
{
    if (address >= MB_MAX_IREGS) {
        return;
    }
    mb_lock();
    input_registers[address] = value;
    mb_unlock();
}

void __attribute__((weak)) mb_on_write_hreg(uint16_t address, uint16_t value)
{
    (void)address;
    (void)value;
}

void __attribute__((weak)) mb_on_write_coil(uint16_t address, uint8_t value)
{
    (void)address;
    (void)value;
}
