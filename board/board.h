/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "mbed.h"
#include "system/timer.h"
#include "debug.h"
#include "system/utilities.h"
#include "gps.h"
#include "LIS3DH.h"
#include "Adafruit_SSD1306.h"


/*!
 * Board IO pins definitions
 */

#define LED_1                                       PA_12
#define LED_2                                       PB_4

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PB_13
#define RADIO_XTAL_EN                               PH_1

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5
#define RADIO_NSS                                   PB_0

#define RADIO_DIO_0                                 PA_11
#define RADIO_DIO_1                                 PB_1
#define RADIO_DIO_2                                 PA_3
#define RADIO_DIO_3                                 PH_0
#define RADIO_DIO_4                                 PC_13

#define RADIO_RF_CRX_RX                             PB_6  //CRF3
#define RADIO_RF_CBT_HF                             PB_7  //CRF2 HF 
#define RADIO_RF_CTX_PA                             PA_4  //CRF1 PA 

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

//#define USB_DM                                      PA_11
//#define USB_DP                                      PA_12

#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9
#define LIS3DH_INT1_PIN		                    	PB_14
#define LIS3DH_INT2_PIN		                    	PB_15

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define GPS_UART        		            		UART_3 
#define GPS_POWER_ON_PIN		            		PA_15
#define GPS_UART_TX                                 PB_10
#define GPS_UART_RX                                 PB_11
#define GPS_PPS_PIN                                 PA_0

#define BAT_LEVEL_PIN                               PA_2
#define BAT_LEVEL_CHANNEL                           ADC_CHANNEL_2

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14


#define LOW_BAT_THRESHOLD   3450 // mV

extern DigitalOut RedLed;
extern DigitalOut GreenLed;
extern GPS Gps;
extern LIS3DH acc;
extern Adafruit_SSD1306_I2c display;

//extern MPL3115A2 Mpl3115a2;
//extern MMA8451Q Mma8451q;

//extern DigitalIn PC0; // Used for Push button application demo

//extern SX1272MB2xAS Radio;

/*!
 * Board versions
 */
typedef enum
{
    BOARD_VERSION_NONE = 0,
    BOARD_VERSION_2,
    BOARD_VERSION_3,
}BoardVersion_t;

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInit( void );

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value  battery voltage in millivolts
 */
uint32_t BoardGetBatteryVoltage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board version
 *
 * \retval value  Version [0: MOTE_VERSION_NONE, 
 *                         1: MOTE_VERSION_2,
 *                         2: MOTE_VERSION_3]
 */
BoardVersion_t BoardGetVersion( void );

/*!
 * \brief Generates Lower 32 bits of DEVEUI using 96 bits unique device ID 
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetDevEUI( uint8_t *id );


void BoardInitMcu( void );


#endif // __BOARD_H__
