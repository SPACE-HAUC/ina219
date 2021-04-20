#include "ina219.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define eprintf(str, ...)                                                    \
    fprintf(stderr, "%s, %d: " str "\n", __func__, __LINE__, ##__VA_ARGS__); \
    fflush(stderr)

typedef union
{
    uint8_t buf[3];
    struct __attribute__((packed))
    {
        uint16_t val;
        uint8_t addr;
    };
} ina219_xfer;

static inline void invert_arr(uint8_t *dest, uint8_t *src, ssize_t len)
{
    for (ssize_t i = 0; i < len; i++)
        dest[i] = src[len - 1 - i];
}

int ina219_init(ina219 *dev, int bus, int addr, int ctx)
{
    if (dev == NULL)
    {
        eprintf("Memory not allocated for device");
        return -1;
    }
    if (dev->bus == NULL)
    {
        eprintf("Memory not allocated for bus");
        return -1;
    }
    int status = i2cbus_open(dev->bus, bus, addr);
    if (status < 0)
    {
        eprintf("Could not open device %d on bus %d", addr, bus);
        return -1;
    }
    dev->bus->ctx = ctx;
    status = ina219_set_calib(dev, INA219_CALIB_32V_1A); // set moderate calibration
    if (status < 0)
    {
        eprintf("Could not set initial calibration mode");
        return -1;
    }
    return 1;
}

int ina219_set_calib(ina219 *dev, int calib_mode)
{
    uint16_t config = 0x0;
    static ina219_xfer reg[1];
    static uint8_t buf[sizeof(ina219_xfer)];
    memset(reg->buf, 0x0, sizeof(ina219_xfer));
    switch (calib_mode)
    {
    case INA219_CALIB_32V_2A:
        // By default we use a pretty huge range for the input voltage,
        // which probably isn't the most appropriate choice for system
        // that don't use a lot of power.  But all of the calculations
        // are shown below if you want to change the settings.  You will
        // also need to change any relevant register settings, such as
        // setting the VBUS_MAX to 16V instead of 32V, etc.

        // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
        // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08,
        // 0.04) RSHUNT = 0.1               (Resistor value in ohms)

        // 1. Determine max possible current
        // MaxPossible_I = VSHUNT_MAX / RSHUNT
        // MaxPossible_I = 3.2A

        // 2. Determine max expected current
        // MaxExpected_I = 2.0A

        // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        // MinimumLSB = MaxExpected_I/32767
        // MinimumLSB = 0.000061              (61uA per bit)
        // MaximumLSB = MaxExpected_I/4096
        // MaximumLSB = 0,000488              (488uA per bit)

        // 4. Choose an LSB between the min and max values
        //    (Preferrably a roundish number close to MinLSB)
        // CurrentLSB = 0.0001 (100uA per bit)

        // 5. Compute the calibration register
        // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        // Cal = 4096 (0x1000)

        dev->calValue = 4096;

        // 6. Calculate the power LSB
        // PowerLSB = 20 * CurrentLSB
        // PowerLSB = 0.002 (2mW per bit)

        // 7. Compute the maximum current and shunt voltage values before overflow
        //
        // Max_Current = Current_LSB * 32767
        // Max_Current = 3.2767A before overflow
        //
        // If Max_Current > Max_Possible_I then
        //    Max_Current_Before_Overflow = MaxPossible_I
        // Else
        //    Max_Current_Before_Overflow = Max_Current
        // End If
        //
        // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        // Max_ShuntVoltage = 0.32V
        //
        // If Max_ShuntVoltage >= VSHUNT_MAX
        //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        // Else
        //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        // End If

        // 8. Compute the Maximum Power
        // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        // MaximumPower = 3.2 * 32V
        // MaximumPower = 102.4W

        // Set multipliers to convert raw current/power values
        dev->currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
        dev->powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

        config = INA219_CONFIG_BVOLTAGERANGE_32V |
                 INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                 INA219_CONFIG_SADCRES_12BIT_1S_532US |
                 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
        break;

    case INA219_CALIB_32V_1A:
        // By default we use a pretty huge range for the input voltage,
        // which probably isn't the most appropriate choice for system
        // that don't use a lot of power.  But all of the calculations
        // are shown below if you want to change the settings.  You will
        // also need to change any relevant register settings, such as
        // setting the VBUS_MAX to 16V instead of 32V, etc.

        // VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
        // VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
        // RSHUNT = 0.1			(Resistor value in ohms)

        // 1. Determine max possible current
        // MaxPossible_I = VSHUNT_MAX / RSHUNT
        // MaxPossible_I = 3.2A

        // 2. Determine max expected current
        // MaxExpected_I = 1.0A

        // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        // MinimumLSB = MaxExpected_I/32767
        // MinimumLSB = 0.0000305             (30.5uA per bit)
        // MaximumLSB = MaxExpected_I/4096
        // MaximumLSB = 0.000244              (244uA per bit)

        // 4. Choose an LSB between the min and max values
        //    (Preferrably a roundish number close to MinLSB)
        // CurrentLSB = 0.0000400 (40uA per bit)

        // 5. Compute the calibration register
        // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        // Cal = 10240 (0x2800)

        dev->calValue = 10240;

        // 6. Calculate the power LSB
        // PowerLSB = 20 * CurrentLSB
        // PowerLSB = 0.0008 (800uW per bit)

        // 7. Compute the maximum current and shunt voltage values before overflow
        //
        // Max_Current = Current_LSB * 32767
        // Max_Current = 1.31068A before overflow
        //
        // If Max_Current > Max_Possible_I then
        //    Max_Current_Before_Overflow = MaxPossible_I
        // Else
        //    Max_Current_Before_Overflow = Max_Current
        // End If
        //
        // ... In this case, we're good though since Max_Current is less than
        // MaxPossible_I
        //
        // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        // Max_ShuntVoltage = 0.131068V
        //
        // If Max_ShuntVoltage >= VSHUNT_MAX
        //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        // Else
        //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        // End If

        // 8. Compute the Maximum Power
        // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        // MaximumPower = 1.31068 * 32V
        // MaximumPower = 41.94176W

        // Set multipliers to convert raw current/power values
        dev->currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
        dev->powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

        // Set Config register to take into account the settings above
        config = INA219_CONFIG_BVOLTAGERANGE_32V |
                 INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                 INA219_CONFIG_SADCRES_12BIT_1S_532US |
                 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
        break;

    case INA219_CALIB_16V_400mA:
        // Calibration which uses the highest precision for
        // current measurement (0.1mA), at the expense of
        // only supporting 16V at 400mA max.

        // VBUS_MAX = 16V
        // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
        // RSHUNT = 0.1               (Resistor value in ohms)

        // 1. Determine max possible current
        // MaxPossible_I = VSHUNT_MAX / RSHUNT
        // MaxPossible_I = 0.4A

        // 2. Determine max expected current
        // MaxExpected_I = 0.4A

        // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        // MinimumLSB = MaxExpected_I/32767
        // MinimumLSB = 0.0000122              (12uA per bit)
        // MaximumLSB = MaxExpected_I/4096
        // MaximumLSB = 0.0000977              (98uA per bit)

        // 4. Choose an LSB between the min and max values
        //    (Preferrably a roundish number close to MinLSB)
        // CurrentLSB = 0.00005 (50uA per bit)

        // 5. Compute the calibration register
        // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        // Cal = 8192 (0x2000)

        dev->calValue = 8192;

        // 6. Calculate the power LSB
        // PowerLSB = 20 * CurrentLSB
        // PowerLSB = 0.001 (1mW per bit)

        // 7. Compute the maximum current and shunt voltage values before overflow
        //
        // Max_Current = Current_LSB * 32767
        // Max_Current = 1.63835A before overflow
        //
        // If Max_Current > Max_Possible_I then
        //    Max_Current_Before_Overflow = MaxPossible_I
        // Else
        //    Max_Current_Before_Overflow = Max_Current
        // End If
        //
        // Max_Current_Before_Overflow = MaxPossible_I
        // Max_Current_Before_Overflow = 0.4
        //
        // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        // Max_ShuntVoltage = 0.04V
        //
        // If Max_ShuntVoltage >= VSHUNT_MAX
        //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        // Else
        //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        // End If
        //
        // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        // Max_ShuntVoltage_Before_Overflow = 0.04V

        // 8. Compute the Maximum Power
        // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        // MaximumPower = 0.4 * 16V
        // MaximumPower = 6.4W

        // Set multipliers to convert raw current/power values
        dev->currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
        dev->powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

        // Set Config register to take into account the settings above
        config = INA219_CONFIG_BVOLTAGERANGE_16V |
                 INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                 INA219_CONFIG_SADCRES_12BIT_1S_532US |
                 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
        break;

    default:
        eprintf("Requested calib mode not valid, error");
        return -1;
        break;
    }

    // Set Calibration register to 'Cal' calculated above
    reg->addr = INA219_REG_CALIBRATION;
    reg->val = dev->calValue;
    invert_arr(buf, reg->buf, sizeof(ina219_xfer));
    int status = i2cbus_write(dev->bus, buf, sizeof(ina219_xfer));
    if (status < 0)
    {
        eprintf("Could not write calibration 0x%04x, register 0x%02x %02x%02x", dev->calValue, buf[0], buf[1], buf[2]);
        return -1;
    }
    reg->addr = INA219_REG_CONFIG;
    reg->val = config;
    invert_arr(buf, reg->buf, sizeof(ina219_xfer));
    status = i2cbus_write(dev->bus, buf, sizeof(ina219_xfer));
    if (status < 0)
    {
        eprintf("Could not write config 0x%04x, register 0x%02x %02x%02x", config, buf[0], buf[1], buf[2]);
        return -1;
    }
    return 1;
}

int ina219_get_bus_voltage(ina219 *dev, float *val)
{
    int status = INA219_SUCCESS;
    uint8_t buf[3] = {INA219_REG_BUSVOLTAGE, 0x0, 0x0};
    status = i2cbus_xfer(dev->bus, buf, 1, buf + 1, 2, 0);
    if (status < 0)
    {
        eprintf("Error reading bus volatage");
        return INA219_FAILURE;
    }
    int16_t raw_v = buf[1];
    raw_v <<= 8;
    raw_v |= buf[2];
    if ((raw_v & 0x2) == 0)
    {
        eprintf("Conversion not ready: 0x%04x", raw_v);
        status |= INA219_BUS_V_RDY;
    }
    if (raw_v & 0x1)
    {
        eprintf("Overflow in power or current calculation: 0x%04x", raw_v);
        status |= INA219_BUS_OV;
    }
    *val = 0.001f * raw_v;
    return status;
}

int ina219_get_shunt_voltage(ina219 *dev, float *val)
{
    int status = INA219_SUCCESS;
    uint8_t buf[3] = {INA219_REG_SHUNTVOLTAGE, 0x0, 0x0};
    status = i2cbus_xfer(dev->bus, buf, 1, buf + 1, 2, 0);
    int16_t raw_v = buf[1];
    raw_v <<= 8;
    raw_v |= buf[2];
    *val = raw_v * 0.00001f;
    return status;
}

int ina219_get_current(ina219 *dev, float *val)
{
    uint8_t buf[3] = {INA219_REG_CALIBRATION, 0x0, 0x0};
    invert_arr(buf + 1, (uint8_t *)&(dev->calValue), 2);
    if (i2cbus_write(dev->bus, buf, 3) < 0)
    {
        eprintf("Could not reset calibration");
        return INA219_FAILURE;
    }
    buf[0] = INA219_REG_CURRENT;
    if (i2cbus_xfer(dev->bus, buf, 1, buf + 1, 2, 0) < 0)
    {
        eprintf("Could not read current");
        return INA219_FAILURE;
    }
    int16_t raw_v = buf[1];
    raw_v <<= 8;
    raw_v |= buf[2];
    *val = raw_v * 1.0f / (dev->currentDivider_mA);
    return INA219_SUCCESS;
}

int ina219_get_power(ina219 *dev, float *val)
{
    uint8_t buf[3] = {INA219_REG_CALIBRATION, 0x0, 0x0};
    invert_arr(buf + 1, (uint8_t *)&(dev->calValue), 2);
    if (i2cbus_write(dev->bus, buf, 3) < 0)
    {
        eprintf("Could not reset calibration");
        return INA219_FAILURE;
    }
    buf[0] = INA219_REG_POWER;
    if (i2cbus_xfer(dev->bus, buf, 1, buf + 1, 2, 0) < 0)
    {
        eprintf("Could not read power");
        return INA219_FAILURE;
    }
    int16_t raw_v = buf[1];
    raw_v <<= 8;
    raw_v |= buf[2];
    *val = raw_v * (dev->powerMultiplier_mW) * 0.001f;
    return INA219_SUCCESS;
}

int ina219_powersave(ina219 *dev, bool on)
{
    uint8_t buf[3] = {INA219_REG_CONFIG, 0x0, on ? INA219_CONFIG_MODE_POWERDOWN : INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS};
    return i2cbus_write(dev->bus, buf, 3);
}

void ina219_destroy(ina219 *dev)
{
    if (dev == NULL)
        return;
    if (dev->bus == NULL)
        return;
    uint8_t buf[3] = {INA219_REG_CONFIG, 0x80, 0x00};
    i2cbus_write(dev->bus, buf, 3); // generate reset
    i2cbus_close(dev->bus); // close I2C bus
}

#ifdef UNIT_TEST_INA219
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

volatile sig_atomic_t done = 0;

void sighandler(int sig)
{
    done = 1;
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Usage: %s <I2C Bus ID> <I2C Bus Address>\n\n", argv[0]);
        return 0;
    }
    int bus = atoi(argv[1]);
    int addr = atoi(argv[2]);
    ina219 dev[1];
    if (ina219_init(dev, bus, addr, -1) < 0)
    {
        printf("Failed to initialize INA219 on bus %d, address 0x%02x\n", bus, addr);
        return 0;
    }
    signal(SIGINT, &sighandler);
    while (!done)
    {
        int print_char = 0;
        float val; int ret;
        char buf[512];
        ret = ina219_get_bus_voltage(dev, &val);
        print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "Bus: %.3e V ", val);
        if (ret < INA219_FAILURE && (ret & INA219_BUS_OV))
            print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "+ Math overflow ");
        else if (ret < INA219_FAILURE && (ret & INA219_BUS_V_RDY))
            print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "+ Conversion Rdy ");
        else if (ret == INA219_FAILURE)
            print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "+ General failure ");
        print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "| ");
        ret = ina219_get_shunt_voltage(dev, &val);
        print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "Shunt: %.3e V ", val);
        ret = ina219_get_current(dev, &val);
        print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "Current: %.3e A ", val);
        ret = ina219_get_power(dev, &val);
        print_char += snprintf(buf + print_char, sizeof(buf) - print_char, "Power: %.3e W ", val);
        printf("%s", buf);
        fflush(stdout);
        sleep(1);
        printf("\r");
        while (print_char--)
            printf(" ");
        printf("\r");
    }
    ina219_destroy(dev);
    return 0;
}

#endif