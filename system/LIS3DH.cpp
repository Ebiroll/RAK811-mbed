/*
 * mbed library program
 *  LIS3DH MEMS motion sensor: 3-axis "nano" accelerometer, made by STMicroelectronics
 *      http://www.st-japan.co.jp/web/jp/catalog/sense_power/FM89/SC444/PF250725
 *
 * Copyright (c) 2014,'15,'17 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: July       14th, 2014
 *      Revised: August     23rd, 2017
 */

#include "LIS3DH.h"

LIS3DH::LIS3DH (PinName p_sda, PinName p_scl,
                uint8_t addr, uint8_t data_rate, uint8_t fullscale)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    initialize (addr, data_rate, fullscale);
}

LIS3DH::LIS3DH (PinName p_sda, PinName p_scl, uint8_t addr)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    initialize (addr, LIS3DH_DR_NR_LP_50HZ, LIS3DH_FS_8G);
}

LIS3DH::LIS3DH (I2C& p_i2c,
                uint8_t addr, uint8_t data_rate, uint8_t fullscale)
 : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    initialize (addr, data_rate, fullscale);
}

LIS3DH::LIS3DH (I2C& p_i2c, uint8_t addr)
 : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    initialize (addr, LIS3DH_DR_NR_LP_50HZ, LIS3DH_FS_8G);
}

void LIS3DH::initialize (uint8_t addr, uint8_t data_rate, uint8_t fullscale)
{
    // Check acc is available of not
    acc_addr = addr;
    dt[0] = LIS3DH_WHO_AM_I;
    _i2c.write(acc_addr, dt, 1, true);
    _i2c.read(acc_addr, dt, 1, false);
    if (dt[0] == I_AM_LIS3DH) {
        acc_ready = 1;
    } else {
        acc_ready = 0;
        return;     // acc chip is NOT on I2C line then terminate
    }
    //  Reg.1
    dt[0] = LIS3DH_CTRL_REG1;
    dt[1] = 0x07;
    dt[1] |= data_rate << 4;
    _i2c.write(acc_addr, dt, 2, false);
    //  Reg.4
    dt[0] = LIS3DH_CTRL_REG4;
    dt[1] = 0x08;  // High resolution
    dt[1] |= fullscale << 4;
    _i2c.write(acc_addr, dt, 2, false);
    switch (fullscale) {
        case LIS3DH_FS_2G:
            fs_factor = LIS3DH_SENSITIVITY_2G;
            break;
        case LIS3DH_FS_4G:
            fs_factor = LIS3DH_SENSITIVITY_4G;
            break;
        case LIS3DH_FS_8G:
            fs_factor = LIS3DH_SENSITIVITY_8G;
            break;
        case LIS3DH_FS_16G:
            fs_factor = LIS3DH_SENSITIVITY_16G;
            break;
        default:
            ;
    }
}

void LIS3DH::read_reg_data(char *data)
{
    // X,Y & Z
    // manual said that
    // In order to read multiple bytes, it is necessary to assert the most significant bit
    // of the subaddress field.
    // In other words, SUB(7) must be equal to ‘1’ while SUB(6-0) represents the address
    // of the first register to be read.
    dt[0] = LIS3DH_OUT_X_L | 0x80;
    _i2c.write(acc_addr, dt, 1, true);
    _i2c.read(acc_addr, data, 6, false);
}

void LIS3DH::read_mg_data(float *dt_usr)
{
    char data[6];

    if (acc_ready == 0) {
        dt_usr[0] = 0;
        dt_usr[1] = 0;
        dt_usr[2] = 0;
        return;
    }
    read_reg_data(data);
    // change data type
#if OLD_REV // Fixed bugs -> (1) unit is not mg but g (2) shift right 4bit = /16
    dt_usr[0] = float(short((data[1] << 8) | data[0])) * fs_factor / 15;
    dt_usr[1] = float(short((data[3] << 8) | data[2])) * fs_factor / 15;
    dt_usr[2] = float(short((data[5] << 8) | data[4])) * fs_factor / 15;
#else
    dt_usr[0] = float(short((data[1] << 8) | data[0]) >> 4) * fs_factor;
    dt_usr[1] = float(short((data[3] << 8) | data[2]) >> 4) * fs_factor;
    dt_usr[2] = float(short((data[5] << 8) | data[4]) >> 4) * fs_factor;
#endif
    mbed_printf(0, "x val : %f", dt_usr[0]);

}

void LIS3DH::read_data(float *dt_usr)
{
    char data[6];

    if (acc_ready == 0) {
        dt_usr[0] = 0;
        dt_usr[1] = 0;
        dt_usr[2] = 0;
        return;
    }
    read_reg_data(data);
    // change data type
#if OLD_REV // Fixed bugs -> shift right 4bit = /16 (not /15)
    dt_usr[0] = float(short((data[1] << 8) | data[0])) * fs_factor / 15 * GRAVITY;
    dt_usr[1] = float(short((data[3] << 8) | data[2])) * fs_factor / 15 * GRAVITY;
    dt_usr[2] = float(short((data[5] << 8) | data[4])) * fs_factor / 15 * GRAVITY;
#else
    dt_usr[0] = float(short((data[1] << 8) | data[0]) >> 4) * fs_factor * GRAVITY;
    dt_usr[1] = float(short((data[3] << 8) | data[2]) >> 4) * fs_factor * GRAVITY;
    dt_usr[2] = float(short((data[5] << 8) | data[4]) >> 4) * fs_factor * GRAVITY;
#endif
    mbed_printf(0, "x val : %d \n", (short((data[1] << 8) | data[0]) >> 4));
}

uint8_t LIS3DH::read_id()
{
    dt[0] = LIS3DH_WHO_AM_I;
    _i2c.write(acc_addr, dt, 1, true);
    _i2c.read(acc_addr, dt, 1, false);
    return (uint8_t)dt[0];
}

uint8_t LIS3DH::data_ready()
{
    if (acc_ready == 1) {
        dt[0] = LIS3DH_STATUS_REG_AUX;
        _i2c.write(acc_addr, dt, 1, true);
        _i2c.read(acc_addr, dt, 1, false);
        if (!(dt[0] & 0x01)) {
            return 0;
        }
    }
    return 1;
}

void LIS3DH::frequency(int hz)
{
    _i2c.frequency(hz);
}

uint8_t LIS3DH::read_reg(uint8_t addr)
{
    if (acc_ready == 1) {
        dt[0] = addr;
        _i2c.write(acc_addr, dt, 1, true);
        _i2c.read(acc_addr, dt, 1, false);
    } else {
        dt[0] = 0xff;
    }
    return (uint8_t)dt[0];
}

void LIS3DH::write_reg(uint8_t addr, uint8_t data)
{
    if (acc_ready == 1) {
        dt[0] = addr;
        dt[1] = data;
        _i2c.write(acc_addr, dt, 2, false);
    }
}


