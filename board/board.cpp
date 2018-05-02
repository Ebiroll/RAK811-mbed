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
#include "mbed.h"
#include "board.h"
#include "Adafruit_SSD1306.h"

/*!
 * Unique Devices IDs register set ( STM32 )
 */
#define         ID1                                 ( 0x1FF800D0 )
#define         ID2                                 ( 0x1FF800D4 )
#define         ID3                                 ( 0x1FF800E4 )



DigitalOut RedLed( LED_1 );     // Active Low
DigitalOut GreenLed( LED_2 );   // Active Low


GPS Gps( GPS_UART_TX, GPS_UART_RX, GPS_POWER_ON_PIN ); // Gps(tx, rx, en);

DigitalIn I2cInterrupt( LIS3DH_INT1_PIN );
I2C I2c(I2C_SDA, I2C_SCL);


LIS3DH acc(I2c, LIS3DH_V_CHIP_ADDR);  //  LIS3DH_DR_NR_LP_50HZ, LIS3DH_FS_8G

AnalogIn Battery(BAT_LEVEL_PIN);

//DigitalOut Pc7( PC_7 );
//DigitalIn Pc1( PC_1 );

// Used for Push button application demo
//DigitalIn PC0( PC_0, PullUp ); 

//AnalogIn *Battery;
//I2C NI2c(PB_3, PB_5);


#define AIN_VREF            3300    // STM32 internal refernce
#define AIN_VBAT_DIV        2       // Resistor divider

//SX1272MB2xAS Radio( NULL );

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInit( void )
{
    // Initalize LEDs
    RedLed = 1;     // Active Low
    GreenLed = 1;   // Active Low

    TimerTimeCounterInit( );


    Adafruit_SSD1306_I2c display(I2c,PB_12,SSD_I2C_ADDRESS,64,128);

    display.splash();
    wait(2.0);
    //display.clearDisplay();




    Gps.init( );
    Gps.enable( 1 );

}


uint8_t BoardGetBatteryLevel( void ) 
{
    // Per LoRaWAN spec; 0 = Charging; 1...254 = level, 255 = N/A
    return ( Battery.read_u16( ) >> 8 ) + ( Battery.read_u16( ) >> 9 );
}

uint32_t BoardGetBatteryVoltage( void ) 
{
    return ( Battery.read( ) * AIN_VREF * AIN_VBAT_DIV );
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetDevEUI( uint8_t *id )
{
    uint32_t *pDevEuiHWord = ( uint32_t* )&id[4];

    if( *pDevEuiHWord == 0 )
    {        
        *pDevEuiHWord = BoardGetRandomSeed( );
    }
    
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

BoardVersion_t BoardGetVersion( void )
{
   return BOARD_VERSION_3;

/*
    Pc7 = 1;
    char first = Pc1;
    Pc7 = 0;
    
    if( first && !Pc1 )
    {
        return BOARD_VERSION_2;
    }
    else
    {
        return BOARD_VERSION_3;
    }
*/
}

#if 0
// Original RAK McuInit()

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
#if defined( USE_BOOTLOADER )
        // Set the Vector Table base location at 0x3000
        SCB->VTOR = FLASH_BASE | 0x3000;
#endif
        HAL_Init( );

        SystemClockConfig( );
			
				UartMcuInit(&Uart1, UART_1, UART_TX, UART_RX);
				UartMcuConfig(&Uart1, RX_TX, 115200, 
																		 UART_8_BIT,
																		 UART_1_STOP_BIT,
																		 NO_PARITY,
																		 NO_FLOW_CTRL);
        RtcInit( );

        BoardUnusedIoInit( );	
    }
    else
    {
        SystemClockReConfig( );
    }

    AdcInit( &Adc, BAT_LEVEL_PIN );

    SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1276IoInit( );
    GpioInit( &SX1276.Xtal, RADIO_XTAL_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
		
    if( McuInitialized == false )
    {
        McuInitialized = true;
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            CalibrateSystemWakeupTime( );
        }
    }
}

void BoardDeInitMcu( void )
{
    //Gpio_t ioPin;

    AdcDeInit( &Adc );

    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );

    //GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	
	  GpioWrite( &SX1276.Xtal, 0 );
}

#endif