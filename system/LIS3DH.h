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

#ifndef LIS3DH_H
#define LIS3DH_H

#include "mbed.h"
#include <mbed_printf.h>


//  revision 6 have two bugs, (1) read_mg_data, (2) divided by 15 (16 is coorect value)
#define OLD_REV             0       // KEEP 0!! (If you set 1, work as old revision)

//  LIS3DH Address
//  7bit address = 0b001100x(0x18 or 0x19 depends on SA0/SDO)
#define LIS3DH_G_CHIP_ADDR  (0x18 << 1)    // SA0(=SDO pin) = Ground
#define LIS3DH_V_CHIP_ADDR  (0x19 << 1)    // SA0(=SDO pin) = Vdd This is the one for itracker module


//   LIS3DH ID
#define I_AM_LIS3DH            0x33

//  Register's definition
#define LIS3DH_STATUS_REG_AUX  0x07
#define LIS3DH_OUT_ADC1_L      0x08
#define LIS3DH_OUT_ADC1_H      0x09
#define LIS3DH_OUT_ADC2_L      0x0a
#define LIS3DH_OUT_ADC2_H      0x0b
#define LIS3DH_OUT_ADC3_L      0x0c
#define LIS3DH_OUT_ADC3_H      0x0d
#define LIS3DH_INT_COUNTER_REG 0x0e
#define LIS3DH_WHO_AM_I        0x0f
#define LIS3DH_TEMP_CFG_REG    0x1f
#define LIS3DH_CTRL_REG1       0x20
#define LIS3DH_CTRL_REG2       0x21
#define LIS3DH_CTRL_REG3       0x22
#define LIS3DH_CTRL_REG4       0x23
#define LIS3DH_CTRL_REG5       0x24
#define LIS3DH_CTRL_REG6       0x25
#define LIS3DH_REFERENCE       0x26
#define LIS3DH_STATUS_REG      0x27
#define LIS3DH_OUT_X_L         0x28
#define LIS3DH_OUT_X_H         0x29
#define LIS3DH_OUT_Y_L         0x2a
#define LIS3DH_OUT_Y_H         0x2b
#define LIS3DH_OUT_Z_L         0x2c
#define LIS3DH_OUT_Z_H         0x2d
#define LIS3DH_FIFO_CTRL_REG   0x2e
#define LIS3DH_FIFO_SRC_REG    0x2f
#define LIS3DH_INT1_CFG        0x30
#define LIS3DH_INT1_SOURCE     0x31
#define LIS3DH_INT1_THS        0x32
#define LIS3DH_INT1_DURATION   0x33
#define LIS3DH_CLICK_CFG       0x38
#define LIS3DH_CLICK_SRC       0x39
#define LIS3DH_CLICK_THS       0x3a
#define LIS3DH_TIME_LIMIT      0x3b
#define LIS3DH_TIME_LATENCY    0x3c
#define LIS3DH_TIME_WINDOW     0x3d

// Output Data Rate (ODR)
#define LIS3DH_DR_PWRDWN       0
#define LIS3DH_DR_NR_LP_1HZ    1
#define LIS3DH_DR_NR_LP_10HZ   2
#define LIS3DH_DR_NR_LP_25HZ   3
#define LIS3DH_DR_NR_LP_50HZ   4
#define LIS3DH_DR_NR_LP_100HZ  5
#define LIS3DH_DR_NR_LP_200HZ  6
#define LIS3DH_DR_NR_LP_400HZ  7
#define LIS3DH_DR_LP_1R6KHZ    8
#define LIS3DH_DR_NR_1R25KHZ   9

// Bandwidth (Low pass)
#define LIS3DH_BW_LOW          0
#define LIS3DH_BW_M_LOW        1
#define LIS3DH_BW_M_HI         2
#define LIS3DH_BW_HI           3

// Low power mode enable/disable
#define LIS3DH_LP_EN           0
#define LIS3DH_LP_DIS          1

// Axis control
#define LIS3DH_X_EN            1
#define LIS3DH_X_DIS           0
#define LIS3DH_Y_EN            1
#define LIS3DH_Y_DIS           0
#define LIS3DH_Z_EN            1
#define LIS3DH_Z_DIS           0

// Full Scale
#define LIS3DH_FS_2G           0
#define LIS3DH_FS_4G           1
#define LIS3DH_FS_8G           2
#define LIS3DH_FS_16G          3

// definition for Nomalization
#if OLD_REV
#define LIS3DH_SENSITIVITY_2G  (0.001F)
#define LIS3DH_SENSITIVITY_4G  (0.002F)
#define LIS3DH_SENSITIVITY_8G  (0.004F)
#define LIS3DH_SENSITIVITY_16G (0.012F)
#else
#define LIS3DH_SENSITIVITY_2G  1
#define LIS3DH_SENSITIVITY_4G  2
#define LIS3DH_SENSITIVITY_8G  4
#define LIS3DH_SENSITIVITY_16G 12
#endif

//Gravity at Earth's surface in m/s/s
#if OLD_REV
#define GRAVITY                (9.80665F)
#else
#define GRAVITY                (9.80665F / 1000)
#endif

/** Interface for STMicronics MEMS motion sensor: 3-axis "nano" accelerometer
 *      Chip: LIS3DH
 *
 * @code
 * #include "mbed.h"
 *
 * // I2C Communication
 * LIS3DH acc(p_sda, p_scl, chip_addr, datarate, bandwidth, fullscale);
 * // If you connected I2C line not only this device but also other devices,
 * //     you need to declare following method.
 * I2C i2c(dp5,dp27);              // SDA, SCL
 * LIS3DH acc(i2c, chip_addr, datarate, bandwidth, fullscale);
 *
 * int main() {
 * float f[3];
 *
 *   if (acc.read_id() == I_AM_LIS3DH){
 *      acc.read_data(f);
 *   }
 * }
 * @endcode
 */

class LIS3DH
{
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param device address LIS3DH(SA0=0 or 1), LIS3DH_G_CHIP_ADDR or LIS3DH_V_CHIP_ADDR
      * @param output data rate selection, power down mode, 1Hz to 5KHz
      * @param full scale selection, +/-2g to +/-16g
      */
    LIS3DH(PinName p_sda, PinName p_scl,
           uint8_t addr, uint8_t data_rate, uint8_t fullscale);

    /** Configure data pin
      * @param data SDA and SCL pins
      * @param device address LIS3DH(SA0=0 or 1), LIS3DH_G_CHIP_ADDR or LIS3DH_V_CHIP_ADDR
      * @default output data rate selection = 50Hz
      * @default full scale selection = +/-8g
      */
    LIS3DH(PinName p_sda, PinName p_scl, uint8_t addr);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param other parameters -> please see LIS3DH(PinName p_sda, PinName p_scl,...)
      */
    LIS3DH(I2C& p_i2c,
           uint8_t addr, uint8_t data_rate, uint8_t fullscale);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param other parameters -> please see LIS3DH(PinName p_sda, PinName p_scl,...)
      * @default output data rate selection = 50Hz
      * @default full scale selection = +/-8g
      */
    LIS3DH(I2C& p_i2c, uint8_t addr);

    /** Read a float type data from acc
      * @param float type of three arry's address, e.g. float dt_usr[3];
      * @return acc motion data unit: m/s/s(m/s2)
      * @return dt_usr[0]->x, dt_usr[1]->y, dt_usr[2]->z
      */
    void read_data(float *dt_usr);

    /** Read a float type data from acc
      * @param float type of three arry's address, e.g. float dt_usr[3];
      * @return acc motion data unit: mg
      * @return dt_usr[0]->x, dt_usr[1]->y, dt_usr[2]->z
      */
    void read_mg_data(float *dt_usr);

    /** Read a acc ID number
      * @param none
      * @return if STM MEMS acc, it should be I_AM_ LIS3DH(0x33)
      */
    uint8_t read_id();

    /** Read Data Ready flag
      * @param none
      * @return 1 = Ready
      */
    uint8_t data_ready();

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

    /** Read register (general purpose)
      * @param register's address
      * @return register data
      */
    uint8_t read_reg(uint8_t addr);

    /** Write register (general purpose)
      * @param register's address
      * @param data
      * @return none
      */
    void write_reg(uint8_t addr, uint8_t data);

protected:
    void initialize(uint8_t, uint8_t, uint8_t);
    void read_reg_data(char *data);

    I2C *_i2c_p;
    I2C &_i2c;

private:
#if OLD_REV
    float   fs_factor;  // full scale factor
#else
    uint8_t fs_factor;  // full scale factor
#endif
    char    dt[2];      // working buffer
    uint8_t acc_addr;   // acc sensor address
    uint8_t acc_id;     // acc ID
    uint8_t acc_ready;  // acc is on I2C line = 1, not = 0
};

#endif      // LIS3DH_H

