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

/*!
 * Unique Devices IDs register set ( STM32 )
 */
#define         ID1                                 ( 0x1FF800D0 )
#define         ID2                                 ( 0x1FF800D4 )
#define         ID3                                 ( 0x1FF800E4 )



DigitalOut RedLed( LED_1 );     // Active Low
DigitalOut GreenLed( LED_2 );   // Active Low

/*
PA_0    PPS pin

+const PinMap PinMap_UART_TX[] = {
+    {PA_2,  UART_2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2)},
+    {PA_9,  UART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)},
+    {PB_10, UART_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)},
+    {NC,    NC,     0}
+};
+
+const PinMap PinMap_UART_RX[] = {
+    {PA_10, UART_1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1)},
+    {PB_11, UART_3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART3)},
+    {NC,    NC,     0}
+};

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define GPS_UART        		            		UART_3 
#define GPS_POWER_ON_PIN		            		PA_15
#define GPS_UART_TX                                 PB_10
#define GPS_UART_RX                                 PB_11
#define GPS_PPS_PIN                                 PA_0

*/

GPS Gps( GPS_UART_TX , GPS_UART_RX,  GPS_POWER_ON_PIN ); // Gps(tx, rx, en);

DigitalIn I2cInterrupt( LIS3DH_INT1_PIN );
I2C I2c(I2C_SDA, I2C_SCL);


LIS3DH acc(I2c, LIS3DH_V_CHIP_ADDR);  //  LIS3DH_DR_NR_LP_50HZ, LIS3DH_FS_8G

AnalogIn Battery(BAT_LEVEL_PIN);

Adafruit_SSD1306_I2c display(I2c,PB_12,SSD_I2C_ADDRESS,64,128);


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
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;


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

    // Old version
    //BoardInitMcu();

    // Initalize LEDs
    RedLed = 1;     // Active Low
    GreenLed = 1;   // Active Low

    TimerTimeCounterInit( );


    display.splash();
    display.display();
    wait(0.5);
    display.clearDisplay();

    Gps.enable( 1 );
    Gps.init( );

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


/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;


/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void )
{
    SystemWakeupTimeCalibrated = true;
}


static void BoardUnusedIoInit( void )
{

#if defined( USE_DEBUGGER )
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );

#endif
}

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    __HAL_RCC_PWR_CLK_ENABLE( );

    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE2 );

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
//    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
//    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    HAL_RCC_OscConfig( &RCC_OscInitStruct );

    RCC_ClkInitStruct.ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 );
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 );

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit );

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    /* HAL_NVIC_GetPriorityGrouping*/
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void CalibrateSystemWakeupTime( void )
{
    int breakout=0;
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( (SystemWakeupTimeCalibrated == false) && breakout++<100000 )
        {
            //TimerLowPowerHandler( );
        }
    }
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE2 );

    /* Enable HSI */
    __HAL_RCC_HSI_ENABLE();
    /* Wait till HSE is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    /* Select HSI as system clock source */
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_HSI );

    /* Wait till HSI is used as system clock source */
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_HSI )
    {
    }
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

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
			
		//UartMcuInit(&Uart1, UART_1, UART_TX, UART_RX);
		//UartMcuConfig(&Uart1, RX_TX, 115200, 
		//																 UART_8_BIT,
		//																 UART_1_STOP_BIT,
		//																 NO_PARITY,
		//																 NO_FLOW_CTRL);
        //RtcInit( );

        BoardUnusedIoInit( );	
    }
    else
    {
        SystemClockReConfig( );
    }

    //AdcInit( &Adc, BAT_LEVEL_PIN );

    //SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    //SX1276IoInit( );
    //GpioInit( &SX1276.Xtal, RADIO_XTAL_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
		
    if( McuInitialized == false )
    {
        McuInitialized = true;
        //if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            CalibrateSystemWakeupTime( );
        }
    }
}

void BoardDeInitMcu( void )
{

	
	//  GpioWrite( &SX1276.Xtal, 0 );
}

#endif