#include "MCP47X6.h"

static char const *TAG = "MCP47X6";

MCP47X6_t MCP47X6 =
{
    .port = MCP47X6_I2C_PORT_DEFAULT,
    .address = MCP47X6_ADDR_I2C_BASE,
    .vref = MCP47X6_VREF_UNINITIALISED,
    .powerDown = MCP47X6_POWER_DOWN_UNINITIALISED,
    .gain = MCP47X6_GAIN_UNINITIALISED,
    .output = 0,
    .externalVref = -1.0,
};

// Helpers (Private)
uint8_t MCP47X6_organiseRegisterStatus(uint8_t command, uint8_t vref, uint8_t powerDown, uint8_t gain)
{
    uint8_t out = 0;

    out |= (command << MCP47X6_SHIFT_COMMAND_CODE);
    out |= (vref << MCP47X6_SHIFT_VOLTAGE_REFERENCE);
    out |= (powerDown << MCP47X6_SHIFT_POWER_DOWN);
    out |= (gain << MCP47X6_SHIFT_GAIN);

    return out;
}

double MCP47X6_scaleOutputToFSR()
{
    // TODO: handle gains etc.
    // TODO: account for 8/10/12-bit modes

    uint16_t output = MCP47X6_getOutput();
    uint16_t maxOutput = MCP47X6_OUTPUT_MAX;

    return (double)output / (double)maxOutput;
}


// Functions (Public)
double MCP47X6_getVrefVoltage()
{
    // TODO: handle gains etc.

    return MCP47X6.externalVref;
}
bool MCP47X6_setVrefVoltage(double voltage)
{
    // TODO: handle gains etc.
    
    if (voltage < 0.0)
    {
        ESP_LOGE(TAG, "External Vref must be above 0 V");

        return false;
    }

    MCP47X6.externalVref = voltage;

    return true;
}

double MCP47X6_getOutputVoltage()
{
    // TODO: handle gains etc.

    double voltage = MCP47X6_getVrefVoltage() * MCP47X6_scaleOutputToFSR();

    return voltage;
}
bool MCP47X6_setOutputVoltage(double voltage)
{
    // TODO: handle gains etc.

    bool success = true;

    //esp_err_t ret = ESP_OK;

    if (MCP47X6_getVrefVoltage() < 0.0)
    {
        ESP_LOGE(TAG, "External VREF not set");

        return false;
    }

    if (voltage == MCP47X6_getOutputVoltage())
    {
        return true;
    }

    if (voltage > MCP47X6_getVrefVoltage())
    {
        ESP_LOGW(TAG, "Voltage overrange");

        voltage = MCP47X6_getVrefVoltage();
    }
    else if (voltage < 0.0)
    {
        ESP_LOGW(TAG, "Voltage underrange");

        voltage = 0.0;
    }
    
    uint16_t output = (uint16_t)(voltage * (double)MCP47X6_OUTPUT_MAX / MCP47X6_getVrefVoltage());

    if (!MCP47X6_setOutput(output))
    {
        success = false;
    }

    return success;
}

uint16_t MCP47X6_getOutput()
{
    return MCP47X6.output;
}
bool MCP47X6_setOutput(uint16_t output)
{
    bool success = true;

    esp_err_t ret = ESP_OK;

    if (output == MCP47X6_getOutput())
    {
        return true;
    }
    
    if (output == (MCP47X6_OUTPUT_MAX + 1))
    {
        ESP_LOGV(TAG, "Level technically overrange");

        output = MCP47X6_OUTPUT_MAX;
    }
    else if (output > (MCP47X6_OUTPUT_MAX + 1))
    {
        ESP_LOGW(TAG, "Level overrange");

        output = MCP47X6_OUTPUT_MAX;
    }

    uint8_t byte0 = (uint8_t)((output >> 8) & 0xFF);
    uint8_t byte1 = (uint8_t)output;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP47X6.address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, byte0, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, byte1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(MCP47X6.port, cmd, MCP47X6_I2C_WAIT_TIME_TICKS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write I2C output value: %s", esp_err_to_name(ret));

        success = false;
    }
    else
    {
        MCP47X6.output = output;
    }

    i2c_cmd_link_delete(cmd);

    return success;
}

// bool MCP47X6_isReady();
// bool MCP47X6_readConfig();
bool MCP47X6_writeConfig(uint8_t vref, uint8_t powerDown, uint8_t gain)
{
    // TODO: support volatile and non-volatile memory type
    // for now, only support non-volatile memory type

    if (vref == MCP47X6.vref && powerDown == MCP47X6.powerDown && gain == MCP47X6.gain)
    {
        ESP_LOGI(TAG, "No change in MCP47X6 settings");

        return true;
    }

    if (vref > MCP47X6_VREF_MAX)
    {
        ESP_LOGW(TAG, "Invalid VREF");

        vref = MCP47X6_VREF_DEFAULT;
    }

    if (powerDown > MCP47X6_POWER_DOWN_MAX)
    {
        ESP_LOGW(TAG, "Invalid power down");

        powerDown = MCP47X6_POWER_DOWN_DEFAULT;
    }

    if (gain > MCP47X6_GAIN_MAX)
    {
        ESP_LOGW(TAG, "Invalid gain");

        gain = MCP47X6_GAIN_DEFAULT;
    }

    uint8_t out = MCP47X6_organiseRegisterStatus(MCP47X6_COMMAND_CODE_WRITE_VOLATILE_CONFIG,
        vref, powerDown, gain);
    
    bool success = true;

    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP47X6.address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, out, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(MCP47X6.port, cmd, pdMS_TO_TICKS(MCP47X6_I2C_WAIT_TIME_MS));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write I2C output value: %s", esp_err_to_name(ret));

        success = false;
    }
    else
    {
        MCP47X6.vref = vref;
        MCP47X6.powerDown = powerDown;
        MCP47X6.gain = gain;
    }

    return success;
}

uint8_t MCP47X6_getVoltageReference()
{
    return MCP47X6.vref;
}
bool MCP47X6_setVoltageReference(uint8_t vref)
{
    return MCP47X6_writeConfig(vref, MCP47X6.powerDown, MCP47X6.gain);
}

uint8_t MCP47X6_getPowerDown()
{
    return MCP47X6.powerDown;
}
bool MCP47X6_setPowerDown(uint8_t powerDown)
{
    return MCP47X6_writeConfig(MCP47X6.vref, powerDown, MCP47X6.gain);
}

uint8_t MCP47X6_getGain()
{
    return MCP47X6.gain;
}
bool MCP47X6_setGain(uint8_t gain)
{
    return MCP47X6_writeConfig(MCP47X6.vref, MCP47X6.powerDown, gain);
}

uint8_t MCP47X6_getAddress()
{
    return MCP47X6.address;
}
void MCP47X6_setAddress(uint8_t address)
{
    //ESP_LOGI(TAG, "TODO: validate address between range");

    MCP47X6.address = address;

    ESP_LOGI(TAG, "MCP47X6 I2C Address: 0x%2X", MCP47X6.address);
}
void MCP47X6_setAddressBits(bool A2, bool A1, bool A0)
{
    uint8_t address = MCP47X6_ADDR_I2C_BASE;

    if (A2)
        address |= (1 << 2);
    
    if (A1)
        address |= (1 << 1);
    
    if (A0)
        address |= (1 << 0);

    MCP47X6_setAddress(address);
    
    
}

i2c_port_t MCP47X6_getPort()
{
    return MCP47X6.port;
}
void MCP47X6_setPort(i2c_port_t port)
{
    if (port > I2C_NUM_MAX)
    {
        ESP_LOGW(TAG, "Invalid port");

        port = I2C_NUM_MAX;
    }

    MCP47X6.port = port;
}

bool MCP47X6_deinit()
{
    //ESP_LOGI(TAG, "Nothing currently in deinit()");

    MCP47X6.port = MCP47X6_I2C_PORT_DEFAULT;
    MCP47X6.address = MCP47X6_ADDR_I2C_BASE;
    MCP47X6.vref = MCP47X6_VREF_UNINITIALISED;
    MCP47X6.powerDown = MCP47X6_POWER_DOWN_UNINITIALISED;
    MCP47X6.gain = MCP47X6_GAIN_UNINITIALISED;
    MCP47X6.output = MC47X6_OUTPUT_UNINITIALISED;
    MCP47X6.externalVref = -1.0;
    
    return true;
}
bool MCP47X6_init(i2c_port_t port, uint8_t address, uint8_t vref, uint8_t powerDown, uint8_t gain, uint16_t output)
{
    bool success = true;

    //esp_err_t ret = ESP_OK;

    if (!MCP47X6_deinit())
    {
        ESP_LOGE(TAG, "Failed to de-initialise MCP47X6");

        success = false;
    }
    
    MCP47X6_setPort(port);
    MCP47X6_setAddress(address);

    if (!MCP47X6_writeConfig(vref, powerDown, gain))
    {
        ESP_LOGE(TAG, "Failed to write volatile config");

        success = false;
    }

    if (!MCP47X6_setOutput(output))
    {
        ESP_LOGE(TAG, "Failed to set output");

        success = false;
    }
    
    return success;
}
bool MCP47X6_initDefault()
{
    return MCP47X6_init(
        MCP47X6_I2C_PORT_DEFAULT,
        MCP47X6_ADDR_I2C_DEFAULT,
        MCP47X6_VREF_DEFAULT,
        MCP47X6_POWER_DOWN_DEFAULT,
        MCP47X6_GAIN_DEFAULT,
        0);
}