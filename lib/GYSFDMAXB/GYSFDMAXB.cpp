#include <stdint.h>
#include "GYSFDMAXB.h"

namespace greysound {

    GYSFDMAXB::GYSFDMAXB(PinName txd, PinName rxd, PinName pps, uint32_t baud) {
        // init vars/objects
        rxInPointer = 0;
        rxOutPointer = 0;
        longitude = 0.0f;
        latitude = 0.0f;
        altitude = 0.0f;
        modeIndicator = '*';
        memset(&currentTime, 0, sizeof(time_t));
        memset(rxBuffer, 0, GPS_DATA_BUFFER_SIZE);
        memset(rxLineBuffer, 0, GPS_LINE_BUFFER_SIZE);

        // init gps communication
        serialGps = new RawSerial(txd, rxd, baud);
//        serialGps->printf("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"); //TODO: 意味を調べる

        // init thread
        eventThread = new Thread(osPriorityAboveNormal);
    }

    void GYSFDMAXB::start() {
        // start thread
        eventThread->start(callback(this, &GYSFDMAXB::interruptRxThread));

        // wait data from GPS module (for every single byte)
        serialGps->attach(callback(this, &GYSFDMAXB::interruptRx), RawSerial::RxIrq);
    }

    void GYSFDMAXB::stop() {
        serialGps->attach(NULL, RawSerial::RxIrq);
        eventThread->terminate();
    }

    void GYSFDMAXB::parseGpsData() {

        // read buffered data into line buffer
        readLine();

        // parse GPS data based on its format
        if(strncmp(rxLineBuffer, "$GPGGA", 6) == 0) {
            DEBUG_PRINT("Call parseGPGGA()\r\n");
            parseGPGGA();
        } else if(strncmp(rxLineBuffer, "$GPRMC", 6) == 0) {
            DEBUG_PRINT("Call parseGPRMC()\r\n");
            parseGPRMC();
        }
    }

    bool GYSFDMAXB::is3dFixed() {
        return false; //TODO: ちゃんと書く
    }

    bool GYSFDMAXB::parseGPRMC() {

        /**
         * $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
         *
         * 225446       Time of fix 22:54:46 UTC
         * A            Navigation receiver warning A = OK, V = warning
         * 4916.45,N    Latitude 49 deg. 16.45 min North
         * 12311.12,W   Longitude 123 deg. 11.12 min West
         * 000.5        Speed over ground, Knots
         * 054.7        Course Made Good, True
         * 191194       Date of fix  19 November 1994
         * 020.3,E      Magnetic variation 20.3 deg East
         * A            Mode indicator N = データなし, A = Autonomous（自律方式）, D = Differential（干渉測位方式）, E = Estimated（推定）
         * *68          mandatory checksum
         */
        //TODO: 必要に応じてグローバル変数に持たせる
        double timeUtc, longitudeRaw, latitudeRaw;
        char status; // A:ok V:warning
        char NorS, EorW; // North/South, East/West
        float speed, azimuth; // 地表相対速度(knot), 真方位角(degree)
        char date[6]; // ddmmyy
        float magDegree; // 磁北と真北の間の角度の差
        char magEorW; // 磁北と真北の間の角度の差の方向
        char mode; // N:Data Invalid, A:Autonomous, D:Differential, E:Estimated
        char checkSum[2];

        char *data;
        //char *stringPointer = strdup(rxLineBuffer);
        char *stringPointer = rxLineBuffer;
        char separator[] = ",*";

        data = strsep(&stringPointer, separator);
        if(strcmp(data, "$GPRMC") != 0 ) {
            DEBUG_PRINT("[GPRMC] Invalid Data Format.\r\n");
            return false;
        }

        // timeUtc
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get timeUtc.\r\n");
            return false;
        }
        timeUtc = strtod(data,NULL);

        // Status
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get status.\r\n");
            return false;
        }
        status = data[0];

        // latitudeRaw (dddmm.mmmm)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get latitudeRaw.\r\n");
            return false;
        }
        latitudeRaw = strtod(data,NULL);

        // latitudeIndicator (N|S)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get latitude indicator.\r\n");
            return false;
        }
        NorS = data[0];

        // longitudeRaw (dddmm.mmmm)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get longitudeRaw.\r\n");
            return false;
        }
        longitudeRaw = strtod(data,NULL);

        // longitudeIndicator (E|W)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get longitude indicator.\r\n");
            return false;
        }
        EorW = data[0];

        // speed
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get speed.\r\n");
            return false;
        }
        speed = strtof(data,NULL);

        // azimuth
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get azimuth.\r\n");
            return false;
        }
        azimuth = strtof(data,NULL);

        // date (ddmmyy)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get date.\r\n");
            return false;
        }
        strcpy(date, data);

        // magDegree
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get magDegree.\r\n");
            return false;
        }
        magDegree = strtof(data,NULL);

        // magnitude indicator (E|W)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get magnitude indicator.\r\n");
            return false;
        }
        magEorW = data[0];

        // mode; // N:Data Invalid, A:Autonomous, D:Differential, E:Estimated
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get mode indicator.\r\n");
            return false;
        }
        mode = data[0];

        // checksum
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPRMC] Failed to get checksum.\r\n");
            return false;
        }
        strcpy(checkSum, data);

        /**
         * Calc current unix timestamp
         */
        char time[6];
        sprintf(time, "%06d", (int)timeUtc); // convert to hhmmss
        //DEBUG_PRINTF("timeUtc: %f, timeUtc(int):%d time: %s\r\n", timeUtc, (int)timeUtc, time);

        struct tm tm;
        memset(&tm, 0, sizeof(struct tm));
        tm.tm_year = (uint16_t) (100 + (date[4]-'0') * 10 + (date[5]-'0'));
        tm.tm_mon  = (uint8_t) ((date[2]-'0') * 10 + (date[3]-'0') - 1);
        tm.tm_mday = (uint8_t) ((date[0]-'0') * 10 + (date[1]-'0'));
        tm.tm_hour = (uint8_t) ((time[0]-'0') * 10 + (time[1]-'0'));
        tm.tm_min  = (uint8_t) ((time[2]-'0') * 10 + (time[3]-'0'));
        tm.tm_sec  = (uint8_t) ((time[4]-'0') * 10 + (time[5]-'0'));
        currentTime = mktime(&tm);

        // update time
        if(currentTime > 1577836800) { // 2020-01-01 00:00:00
            set_time(currentTime);
        }

        /**
         * Calc latitude and longitude from raw data
         */
//        /* 座標d（ddd） */
//        int latitude_dd = (int)(latitudeRaw / 100);
//        int longitude_dd = (int)(longitudeRaw / 100);
//
//        /* 座標m（mm.mmmm） */
//        double latitude_md = (latitudeRaw - latitude_dd * 100) / 60;
//        double longitude_md = (longitudeRaw - longitude_dd * 100) / 60;
//
//        /* 座標 d+m */
//        latitude = latitude_dd + latitude_md;
//        longitude = longitude_dd + longitude_md;
//        //DEBUG_PRINTF("lat:%lf lon:%lf\r\n", latitude, longitude);

        /**
         * Set mode
         * N:Data Invalid, A:Autonomous, D:Differential, E:Estimated
         */
        if(mode == 'N'
           || mode == 'A'
           || mode == 'D'
           || mode == 'E') {
            modeIndicator = mode;
        }

        return true;
    }

    bool GYSFDMAXB::parseGPGGA() {

        //TODO: 必要に応じてグローバル変数に持たせる
        double timeUtc, longitudeRaw, latitudeRaw;
        char NorS,EorW;
        uint8_t fixQuality;
        uint8_t numOfSatellitesUsed;
        double hdop; // Relative accuracy of horizontal position
        double heightAboveSeaLevel, heightOfGeoId;
        double ageOfDgps;
        int16_t stationId;
        char checkSum[2];

        char *data;
        //char *stringPointer = strdup(rxLineBuffer);
        char *stringPointer = rxLineBuffer;
        char separator[] = ",*";

        data = strsep(&stringPointer, separator);
        if(strcmp(data, "$GPGGA") != 0 ) {
            DEBUG_PRINT("[GPGGA] Invalid Data Format.\r\n");
            return false;
        }

        // timeUtc
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get timeUtc.\r\n");
            return false;
        }
        timeUtc = strtod(data,NULL);

        // latitudeRaw (dddmm.mmmm)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get latitudeRaw.\r\n");
            return false;
        }
        latitudeRaw = strtod(data,NULL);

        // latitudeIndicator (N|S)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get latitude indicator.\r\n");
            return false;
        }
        NorS = data[0];

        // longitudeRaw (dddmm.mmmm)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get longitudeRaw.\r\n");
            return false;
        }
        longitudeRaw = strtod(data,NULL);

        // longitudeIndicator (E|W)
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get longitude indicator.\r\n");
            return false;
        }
        EorW = data[0];

        // fix quality indicator
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get fixQuality.\r\n");
            return false;
        }
        fixQuality = (uint8_t) atoi(data);

        // number of satellites used
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get numOfSatellitesUsed.\r\n");
            return false;
        }
        //sscanf(data, "%d", &numOfSatellitesUsed);
        numOfSatellitesUsed = (uint8_t) atoi(data);

        // Horizontal dilution of precision, 00.0 ~ 99.9
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get hdop.\r\n");
            return false;
        }
        hdop = strtod(data,NULL);

        // Altitude - Antenna height above/below mean sea level, -9999.9 ~ 17999.9
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get altitude.\r\n");
            return false;
        }
        heightAboveSeaLevel = strtod(data, NULL);

        // discard char 'M'
        data = strsep(&stringPointer, separator);

        // Geoidal height, -999.9 ~ 9999.9
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get heightOfGeoId.\r\n");
            return false;
        }
        heightOfGeoId = strtod(data, NULL);

        // discard char 'M'
        data = strsep(&stringPointer, separator);

        // Age of DGPS data since last valid RTCM transmission in xxx format (seconds)
        // NULL when DGPS not used
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get ageOfDgps.\r\n");
            return false;
        }
        //sscanf(data, "%d", &ageOfDgps);
        ageOfDgps = (uint8_t) atoi(data);

        // Differential reference station ID, 0000 ~ 1023
        // NULL when DGPS not used
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get stationId.\r\n");
            return false;
        }
        //sscanf(data, "%d", &stationId);
        stationId = (uint16_t) atoi(data);

        // checksum
        data = strsep(&stringPointer, separator);
        if( data == NULL ){
            DEBUG_PRINT("[GPGGA] Failed to get checksum.\r\n");
            return false;
        }
        strcpy(checkSum, data);

        /**
         * Calc latitude & longitude
         */
        /* 座標d（ddd） */
        int latitude_dd = (int)(latitudeRaw / 100);
        int longitude_dd = (int)(longitudeRaw / 100);

        /* 座標m（mm.mmmm） */
        double latitude_md = (latitudeRaw - latitude_dd * 100) / 60;
        double longitude_md = (longitudeRaw - longitude_dd * 100) / 60;

        /* 座標 d+m */
        latitude = latitude_dd + latitude_md;
        longitude = longitude_dd + longitude_md;

        /**
         * Calc altitude
         */
        //altitude = heightAboveSeaLevel - heightOfGeoId;
        altitude = heightAboveSeaLevel;

        //printf("lat:%lf, lon:%lf\r\n", latitude, longitude);

        return true;
    }

    void GYSFDMAXB::interruptRx() {
        serialGps->attach(NULL, RawSerial::RxIrq); // detach the ISR to prevent recursive calls
        osSignalSet(this->threadId, 0x01);
    }

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    void GYSFDMAXB::interruptRxThread() {

        // save thread id
        this->threadId = osThreadGetId();

        for (;;) {

            // Wait for signal 0x01 from interruptRX()
            osSignalWait(0x01, osWaitForever);

            // Stop if buffer full
            while ((serialGps->readable()) && (((rxInPointer + 1) % GPS_DATA_BUFFER_SIZE) != rxOutPointer)) {

                rxBuffer[rxInPointer] = serialGps->getc();
                rxInPointer = (rxInPointer + 1) % GPS_DATA_BUFFER_SIZE;

                // line terminator detected?
                if(rxBuffer[((rxInPointer+GPS_DATA_BUFFER_SIZE-1) % GPS_DATA_BUFFER_SIZE)] == '\n'
                   && rxBuffer[((rxInPointer+GPS_DATA_BUFFER_SIZE-2) % GPS_DATA_BUFFER_SIZE)] == '\r') {
                    GYSFDMAXB::parseGpsData();
                }
            }

            // re-attach
            serialGps->attach(callback(this, &GYSFDMAXB::interruptRx), RawSerial::RxIrq);
        }
    }
#pragma clang diagnostic pop

    void GYSFDMAXB::readLine() {

        // Don't interrupt while changing global buffer variables
        //NVIC_DisableIRQ(UART1_IRQn);

        int i = 0;
        while (rxInPointer != rxOutPointer) {
            rxLineBuffer[i] = rxBuffer[rxOutPointer];
            rxOutPointer = (rxOutPointer + 1) % GPS_DATA_BUFFER_SIZE;
            //DEBUG_PRINT("rxInPointer:%d rxOutPointer:%d\r\n", rxInPointer, rxOutPointer);

            // break at end of line character
            if(i>0 && rxLineBuffer[i-1] == '\r' && rxLineBuffer[i] == '\n') {
                break;
            }
            i++;
        }

        // re-enable interrupt
        //NVIC_EnableIRQ(UART1_IRQn);
    }
}