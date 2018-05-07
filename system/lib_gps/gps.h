
/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0  
#endif

#define NUM_RX_BUFS         6
#define RX_BUF_SIZE         96

/* Structure to handle the GPS parsed data in ASCII */
typedef struct
{
    char NmeaDataType[6];
    char NmeaUtcTime[11];
    char NmeaDataStatus[2];
    char NmeaLatitude[10];
    char NmeaLatitudePole[2];
    char NmeaLongitude[11];
    char NmeaLongitudePole[2];
    char NmeaFixQuality[2];
    char NmeaSatelliteTracked[3];
    char NmeaHorizontalDilution[6];
    char NmeaAltitude[8];
    char NmeaAltitudeUnit[2];
    char NmeaHeightGeoid[8];
    char NmeaHeightGeoidUnit[2];
    char NmeaSpeed[8];
    char NmeaDetectionAngle[8];
    char NmeaDate[8];
}tNmeaGpsData;

class GPS {
    public:
        GPS(PinName uart_tx, PinName uart_rx, PinName en);
        ~GPS();
        void enable(bool);
        bool enabled(void);
        void service(void);
        void init(void);

        double Latitude, Longitude;
        char verbose;   // flag
        tNmeaGpsData NmeaGpsData;
        int32_t LatitudeBinary, LongitudeBinary;
        bool have_fix;
        bool en_invert;

        Serial m_uart;
        
    private:
        int ParseGPSData(int idx);
        bool NmeaValidateChecksum(int idx);
        int NmeaChecksum( char *nmeaStr, uint8_t nmeaStrSize, char * checksum );
        void on_uart_rx(void);
        uint8_t rx_buf_lens[NUM_RX_BUFS];
        uint8_t rx_bufs_in_idx;
        uint8_t rx_bufs_out_idx;
        char rx_bufs[NUM_RX_BUFS][RX_BUF_SIZE];
        void ConvertPositionFromStringToNumerical( );
        void ConvertPositionIntoBinary( );
        /*void pps(void);
        bool pps_occurred;*/
        
        DigitalOut m_en_pin;
};

