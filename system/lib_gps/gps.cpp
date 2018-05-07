#include "mbed.h"
#include "gps.h"

#define LAT_UNFIXED     0xaaaaa9
#define LONG_UNFIXED    0x355554e

const char NmeaDataTypeGPGGA[] = "GPGGA";
//const char NmeaDataTypeGPGSA[] = "GPGSA";
//const char NmeaDataTypeGPGSV[] = "GPGSV";
const char NmeaDataTypeGPRMC[] = "GPRMC";

/* Value used for the conversion of the position from DMS to decimal */
const int32_t MaxNorthPosition = 8388607;       // 2^23 - 1
const int32_t MaxSouthPosition = 8388608;       // -2^23
const int32_t MaxEastPosition = 8388607;        // 2^23 - 1    
const int32_t MaxWestPosition = 8388608;        // -2^23

//InterruptIn pps_pin(PC_5);


GPS::GPS(PinName uart_tx, PinName uart_rx, PinName en) : m_uart(uart_tx, uart_rx), m_en_pin(en)
{
    have_fix = false;
 }
    
GPS::~GPS()
{
}

/*void GPS::pps()
{
    pps_occurred = true;
}*/

void GPS::on_uart_rx()
{
    //uint8_t *len = &rx_buf_lens[rx_bufs_in_idx];
    uint8_t len = rx_buf_lens[rx_bufs_in_idx];
    char ch;
    //printf(".");

    do {
        ch = m_uart.getc();

        if (ch != '$') {
            if (len < RX_BUF_SIZE) {
                rx_bufs[rx_bufs_in_idx][len] = ch;
                rx_buf_lens[rx_bufs_in_idx]++;
            } /*else
                printf("S");*/
        } /*else if (len != 0) {
            printf("$ at %d\r\n", len);
        }*/
        
        if (ch == '\n') {
            rx_bufs[rx_bufs_in_idx][len] = 0;  // null terminate
            if (++rx_bufs_in_idx == NUM_RX_BUFS)
                rx_bufs_in_idx = 0;
            /* just let it overflow -- if (rx_bufs_in_idx == rx_bufs_out_idx)
                printf("gps overflow\r\n");*/
            rx_buf_lens[rx_bufs_in_idx] = 0;
        }
    } while (m_uart.readable());
}

void GPS::init()
{
    rx_bufs_in_idx = 0;
    rx_buf_lens[rx_bufs_in_idx] = 0;
    rx_bufs_out_idx = 0;

    //gps_uart.baud(57600);
    m_uart.attach( mbed::callback( this, &GPS::on_uart_rx ) );

    //pps_pin.rise(this, &GPS::pps);
}

void GPS::enable(bool en)
{
    if (en) {
        if (en_invert)
            m_en_pin = 0;
        else
            m_en_pin = 1;
    } else {
        if (en_invert)
            m_en_pin = 1;
        else
            m_en_pin = 0;
    }
    
    // reset current string index
    rx_buf_lens[rx_bufs_in_idx] = 0;
}

bool GPS::enabled()
{
    if (en_invert)
        return !m_en_pin.read();
    else
        return m_en_pin.read();
}

/*char GPS::Nibble2HexChar( char a );
{
    if( a < 10 )
    {
        return '0' + a;
    }
    else if( a < 16 )
    {
        return 'A' + ( a - 10 );
    }
    else
    {
        return '?';
    }
}*/

int GPS::NmeaChecksum( char *nmeaStr, uint8_t nmeaStrSize, char * checksum )
{
    int i = 0;
    uint8_t checkNum = 0;

    // Check input parameters
    if( ( nmeaStr == NULL ) || ( checksum == NULL ) || ( nmeaStrSize <= 1 ) )
    {
        return -1;
    }

    // Skip the first '$' if necessary
    if( nmeaStr[i] == '$' )
    {
        i += 1;
    }

    // XOR until '*' or max length is reached
    while( nmeaStr[i] != '*' )
    {
        checkNum ^= nmeaStr[i];
        i += 1;
        if( i >= nmeaStrSize )
        {
            return -1;
        }
    }

    // Convert checksum value to 2 hexadecimal characters
    /*checksum[0] = Nibble2HexChar( checkNum / 16 ); // upper nibble
    checksum[1] = Nibble2HexChar( checkNum % 16 ); // lower nibble
    */
    sprintf(checksum, "%02X", checkNum);

    return i + 1;
}

bool GPS::NmeaValidateChecksum(int idx)
{
    int checksumIndex;
    char checksum[4];
    char *NmeaString = rx_bufs[idx];

    checksumIndex = NmeaChecksum( NmeaString, rx_buf_lens[idx], checksum );

    // could we calculate a verification checksum ?
    if( checksumIndex < 0 )
    {
        //printf("gps:checksumIndex:%d\r\n", checksumIndex);
        return false;
    }

    // check if there are enough char in the serial buffer to read checksum
    if( checksumIndex >= ( rx_buf_lens[idx] - 2 ) )
    {
        //printf("gps:checksumIndex:%d\r\n", checksumIndex);
        return false;
    }

    // check the checksum
    if( ( NmeaString[checksumIndex] == checksum[0] ) && ( NmeaString[checksumIndex + 1] == checksum[1] ) )
    {
        return true;
    }
    else
    {
        if (verbose)
            printf("gps:checksum fail idx:%d %c%c %c%c\r\n", checksumIndex, checksum[0], checksum[1], NmeaString[checksumIndex], NmeaString[checksumIndex+1]);
        return false;
    }
}

void GPS::ConvertPositionFromStringToNumerical( )
{
    int i;

    double valueTmp1;
    double valueTmp2;
    double valueTmp3;
    double valueTmp4;

    // Convert the latitude from ASCII to uint8_t values
    for( i = 0 ; i < 10 ; i++ )
    {
        NmeaGpsData.NmeaLatitude[i] = NmeaGpsData.NmeaLatitude[i] & 0xF;  
    }
    // Convert latitude from degree/minute/second (DMS) format into decimal
    valueTmp1 = ( double )NmeaGpsData.NmeaLatitude[0] * 10.0 + ( double )NmeaGpsData.NmeaLatitude[1];
    valueTmp2 = ( double )NmeaGpsData.NmeaLatitude[2] * 10.0 + ( double )NmeaGpsData.NmeaLatitude[3];
    valueTmp3 = ( double )NmeaGpsData.NmeaLatitude[5] * 1000.0 + ( double )NmeaGpsData.NmeaLatitude[6] * 100.0 + 
                ( double )NmeaGpsData.NmeaLatitude[7] * 10.0 + ( double )NmeaGpsData.NmeaLatitude[8];
                
    Latitude = valueTmp1 + ( ( valueTmp2 + ( valueTmp3 * 0.0001 ) ) / 60.0 ); 
    
    if( NmeaGpsData.NmeaLatitudePole[0] == 'S' )
    {
        Latitude *= -1;
    }
 
    // Convert the longitude from ASCII to uint8_t values
    for( i = 0 ; i < 10 ; i++ )
    {
        NmeaGpsData.NmeaLongitude[i] = NmeaGpsData.NmeaLongitude[i] & 0xF;  
    }
    // Convert longitude from degree/minute/second (DMS) format into decimal
    valueTmp1 = ( double )NmeaGpsData.NmeaLongitude[0] * 100.0 + ( double )NmeaGpsData.NmeaLongitude[1] * 10.0 + ( double )NmeaGpsData.NmeaLongitude[2];
    valueTmp2 = ( double )NmeaGpsData.NmeaLongitude[3] * 10.0 + ( double )NmeaGpsData.NmeaLongitude[4];
    valueTmp3 = ( double )NmeaGpsData.NmeaLongitude[6] * 1000.0 + ( double )NmeaGpsData.NmeaLongitude[7] * 100;
    valueTmp4 = ( double )NmeaGpsData.NmeaLongitude[8] * 10.0 + ( double )NmeaGpsData.NmeaLongitude[9];
    
    Longitude = valueTmp1 + ( valueTmp2 / 60.0 ) + ( ( ( valueTmp3 + valueTmp4 ) * 0.0001 ) / 60.0 );
    
    if( NmeaGpsData.NmeaLongitudePole[0] == 'W' )
    {
        Longitude *= -1;
    }

    //printf("DD %f, %f\r\n", Latitude, Longitude);
}

void GPS::ConvertPositionIntoBinary( )
{
    long double temp;

    if( Latitude >= 0 ) // North
    {    
        temp = Latitude * MaxNorthPosition;
        LatitudeBinary = temp / 90;
    }
    else                // South
    {    
        temp = Latitude * MaxSouthPosition;
        LatitudeBinary = temp / 90;
    }

    if( Longitude >= 0 ) // East
    {    
        temp = Longitude * MaxEastPosition;
        LongitudeBinary = temp / 180;
    }
    else                // West
    {    
        temp = Longitude * MaxWestPosition;
        LongitudeBinary = temp / 180;
    }
    
    //printf("binary: %x %x\r\n", LatitudeBinary, LongitudeBinary);
    if (LatitudeBinary == LAT_UNFIXED && LongitudeBinary == LONG_UNFIXED)
        have_fix = false;
    else
        have_fix = true;
}

int GPS::ParseGPSData(int idx)
{
    uint8_t i, j, fieldSize;
    char *NmeaString;

    if (NmeaValidateChecksum(idx) == false) {
        if (verbose)
            printf("gps:bad nmea checksum:%s\r\n", rx_bufs[rx_bufs_out_idx]);
        return FAIL;
    }

    NmeaString = rx_bufs[idx];
    
    if (NmeaString[0] == 'P' && NmeaString[1] == 'M' && NmeaString[2] == 'T' && NmeaString[3] == 'K') {
        /* anything useful from PMTK packets? */
        return SUCCESS;
    }
    
    fieldSize = 0;
    i = 0;
    while( NmeaString[i + fieldSize++] != ',' )
    {
        if( fieldSize > 6 )
        {
            printf("gps parse:%s\r\n", NmeaString);
            return FAIL;
        }
    }

    for( j = 0; j < fieldSize; j++, i++ )
    {
        NmeaGpsData.NmeaDataType[j] = NmeaString[i];
    }
    // Parse the GPGGA data 
    if( strncmp( ( const char* )NmeaGpsData.NmeaDataType, ( const char* )NmeaDataTypeGPGGA, 5 ) == 0 )
    {  
        // NmeaUtcTime
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaUtcTime[j] = NmeaString[i];
        }
        // NmeaLatitude
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 10 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitude[j] = NmeaString[i];
        }
        // NmeaLatitudePole
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitudePole[j] = NmeaString[i];
        }
        // NmeaLongitude
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitude[j] = NmeaString[i];
        }
        // NmeaLongitudePole
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitudePole[j] = NmeaString[i];
        }
        // NmeaFixQuality
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaFixQuality[j] = NmeaString[i];
        }
        // NmeaSatelliteTracked
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 3 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaSatelliteTracked[j] = NmeaString[i];
        }
        // NmeaHorizontalDilution
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 6 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaHorizontalDilution[j] = NmeaString[i];
        }
        // NmeaAltitude
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaAltitude[j] = NmeaString[i];
        }
        // NmeaAltitudeUnit
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaAltitudeUnit[j] = NmeaString[i];
        }
        // NmeaHeightGeoid
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaHeightGeoid[j] = NmeaString[i];
        }
        // NmeaHeightGeoidUnit
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaHeightGeoidUnit[j] = NmeaString[i];
        }

        //FormatGpsData( );
        ConvertPositionFromStringToNumerical( );
        ConvertPositionIntoBinary( );
        return SUCCESS;
    }
    else if ( strncmp( ( const char* )NmeaGpsData.NmeaDataType, ( const char* )NmeaDataTypeGPRMC, 5 ) == 0 )
    {    
        // NmeaUtcTime
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaUtcTime[j] = NmeaString[i];
        }
        // NmeaDataStatus
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaDataStatus[j] = NmeaString[i];
        }
        // NmeaLatitude
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 10 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitude[j] = NmeaString[i];
        }
        // NmeaLatitudePole
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitudePole[j] = NmeaString[i];
        }
        // NmeaLongitude
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitude[j] = NmeaString[i];
        }
        // NmeaLongitudePole
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitudePole[j] = NmeaString[i];
        }
        // NmeaSpeed
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaSpeed[j] = NmeaString[i];
        }
        // NmeaDetectionAngle
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaDetectionAngle[j] = NmeaString[i];
        }
        // NmeaDate
        fieldSize = 0;
        while( NmeaString[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return FAIL;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaDate[j] = NmeaString[i];
        }

        //FormatGpsData( );
        ConvertPositionFromStringToNumerical( );
        ConvertPositionIntoBinary( );
        return SUCCESS;
    }
    else
    {
        return FAIL;
    }
}

void GPS::service()
{
    while (rx_bufs_in_idx != rx_bufs_out_idx) {
        //printf("%02d:%s\r\n", rx_buf_lens[rx_bufs_out_idx], rx_bufs[rx_bufs_out_idx]);
        if (verbose)
            printf("gps:%s\r\n", rx_bufs[rx_bufs_out_idx]);
        ParseGPSData(rx_bufs_out_idx);
        if (++rx_bufs_out_idx == NUM_RX_BUFS)
            rx_bufs_out_idx = 0;
    }

    /*if (pps_occurred) {
        pps_occurred = false;
        if (verbose)
            printf("gps:pps\r\n");
    }*/
}
