#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"

// #include "DaughterBoard.h"

/*
    Note: library does not currently support the following features:
        - programmable gain (default: 1x)
        - programmable output-GND resistors (none)
        - voltage buffering (no buffering)
        - high-speed mode (one-shot default)
        - non-volatile memory
        - different address variants (default: 0x60 / A0 ordering code)
        - preventing of writing values to register that are already programmed
        - multiple devices on same I2C bus (using different addresses)
        - generic chip type (MCP47X6)
            - 8 / 10 / 12-bit variants
                - output I2C commands
                - conversion to / from voltage
*/

/*
    Next Steps:
        - Ready
        - Power-on-Reset
        - Power-Down (Normal Operation / 1k-GND / 125k-GND / 640k-GND)
        - Gain (x1 / x2) - check whether using Vref as VDD
        - Read / Write / get / set output

*/
#define MCP47X6_I2C_PORT_DEFAULT I2C_NUM_0

#define MCP47X6_ADDR_I2C_BASE 0x60
#define MCP47X6_ADDR_I2C_DEFAULT MCP47X6_ADDR_I2C_BASE

// Time Constants
#define MCP47X6_I2C_WAIT_TIME_MS 1000
#define MCP47X6_I2C_WAIT_TIME_TICKS pdMS_TO_TICKS(MCP47X6_I2C_WAIT_TIME_MS)

enum MCP47X6_OUTPUT
{
    MCP47X6_OUTPUT_MIN = 0,
    MCP47X6_OUTPUT_MAX = 0xFFF,
    MCP47X6_OUTPUT_DEFAULT = MCP47X6_OUTPUT_MIN,
    MC47X6_OUTPUT_UNINITIALISED = 0xFFFF,
};

enum MCP47X6_COMMAND_CODE
{
    MCP47X6_COMMAND_CODE_WRITE_VOLATILE_DAC = 0b000,
    MCP47X6_COMMAND_CODE_WRITE_VOLATILE_MEMORY = 0b001,
    MCP47X6_COMMAND_CODE_WRITE_ALL_MEMORY = 0b011,
    MCP47X6_COMMAND_CODE_WRITE_VOLATILE_CONFIG = 0b100,
    // READ... ?
};

enum MCP47X6_MASK
{
    MCP47X6_MASK_READY = 0b10000000,
    MCP47X6_MASK_POWER_ON_RESET = 0b01000000,
    MCP47X6_MASK_COMMAND_CODE = 0b00100000,
    MCP47X6_MASK_VOLTAGE_REFERENCE = 0b00011000,
    MCP47X6_MASK_POWER_DOWN = 0b00000110,
    MCP47X6_MASK_GAIN = 0b00000001,
};

enum MCP47X6_SHIFT
{
    MCP47X6_SHIFT_READY = 7,
    MCP47X6_SHIFT_POWER_ON_RESET = 6,
    MCP47X6_SHIFT_COMMAND_CODE = 5,
    MCP47X6_SHIFT_VOLTAGE_REFERENCE = 3,
    MCP47X6_SHIFT_POWER_DOWN = 1,
    MCP47X6_SHIFT_GAIN = 0,
};

enum MCP47X6_VREF
{
    MCP47X6_VREF_VDD = 0,
    MCP47X6_VREF_EXTERNAL_UNBUFFERED = 1,
    MCP47X6_VREF_EXTERNAL_BUFFERED = 2,
    MCP47X6_VREF_MIN = MCP47X6_VREF_VDD,
    MCP47X6_VREF_MAX = MCP47X6_VREF_EXTERNAL_BUFFERED,
    MCP47X6_VREF_DEFAULT = MCP47X6_VREF_VDD,
    MCP47X6_VREF_UNINITIALISED = 0xFF,
};

enum MCP47X6_POWER_DOWN
{
    MCP47X6_POWER_DOWN_NORMAL_OPERATION = 0,
    MCP47X6_POWER_DOWN_1K_TO_GND = 1,
    MCP47X6_POWER_DOWN_100K_TO_GND = 2,
    MCP47X6_POWER_DOWN_500K_TO_GND = 3,
    MCP47X6_POWER_DOWN_MIN = MCP47X6_POWER_DOWN_NORMAL_OPERATION,
    MCP47X6_POWER_DOWN_MAX = MCP47X6_POWER_DOWN_500K_TO_GND,
    MCP47X6_POWER_DOWN_DEFAULT = MCP47X6_POWER_DOWN_NORMAL_OPERATION,
    MCP47X6_POWER_DOWN_UNINITIALISED = 0xFF,
};

enum MCP47X6_GAIN
{
    MCP47X6_GAIN_1X = 0,
    MCP47X6_GAIN_2X = 1,
    MCP47X6_GAIN_MIN = MCP47X6_GAIN_1X,
    MCP47X6_GAIN_MAX = MCP47X6_GAIN_2X,
    MCP47X6_GAIN_DEFAULT = MCP47X6_GAIN_1X,
    MCP47X6_GAIN_UNINITIALISED = 0xFF,
};

typedef struct MCP47X6
{
    i2c_port_t port;
    uint8_t address;
    uint8_t vref;
    uint8_t powerDown;
    uint8_t gain;
    uint16_t output;
    double externalVref;
} MCP47X6_t;

extern MCP47X6_t MCP47X6;

double MCP47X6_getVrefVoltage();
bool MCP47X6_setVrefVoltage(double voltage);

double MCP47X6_getOutputVoltage();
bool MCP47X6_setOutputVoltage(double voltage);

uint16_t MCP47X6_getOutput();
bool MCP47X6_setOutput(uint16_t output);

// bool MCP47X6_isReady();
// bool MCP47X6_readConfig();
bool MCP47X6_writeConfig(uint8_t vref, uint8_t powerDown, uint8_t gain);

uint8_t MCP47X6_getVoltageReference();
bool MCP47X6_setVoltageReference(uint8_t vref);

uint8_t MCP47X6_getPowerDown();
bool MCP47X6_setPowerDown(uint8_t powerDown);

uint8_t MCP47X6_getGain();
bool MCP47X6_setGain(uint8_t gain);

uint8_t MCP47X6_getAddress();
void MCP47X6_setAddress(uint8_t address);
void MCP47X6_setAddressBits(bool A2, bool A1, bool A0);

i2c_port_t MCP47X6_getPort();
void MCP47X6_setPort(i2c_port_t port);

bool MCP47X6_deinit();
bool MCP47X6_init(i2c_port_t port, uint8_t address, uint8_t vref, uint8_t powerDown, uint8_t gain, uint16_t output);
bool MCP47X6_initDefault();