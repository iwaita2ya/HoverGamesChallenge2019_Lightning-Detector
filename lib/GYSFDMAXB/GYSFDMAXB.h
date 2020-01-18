//#define DEBUG

#ifndef GS_GYSFDMAXB_H
#define GS_GYSFDMAXB_H

#include "mbed.h"
#include "Thread.h"

#ifdef DEBUG
extern RawSerial *serial;
#define DEBUG_PRINT(fmt) serial->printf(fmt)
#define DEBUG_PRINTF(fmt, ...) serial->printf(fmt, __VA_ARGS__)
#define DEBUG_PUTC(x) serial->putc(x);
#else
#define DEBUG_PRINT(fmt)
#define DEBUG_PRINTF(fmt, ...)
#define DEBUG_PUTC(x)
#endif

using namespace rtos;

namespace greysound {

#define GPS_DATA_BUFFER_SIZE 1024
#define GPS_LINE_BUFFER_SIZE 256

class GYSFDMAXB {
public:
    double latitude, longitude, altitude;
    char latitudeIndicator; // N:North, S:South
    char longitudeIndicator; // E:East, W:West
    time_t currentTime;
    char modeIndicator; // N:Invalid A:Autonomous D:Differential, E:Estimated

    // Constructor
    GYSFDMAXB(PinName txd, PinName rxd, PinName pps, uint32_t baud=9600);

    virtual ~GYSFDMAXB() {
        delete(serialGps);
    }

    void start(); // start gps serial connection
    void stop(); // stop gps serial connection
    void parseGpsData(); // parse gps data stored on line buffer
    bool is3dFixed();
    bool parseGPRMC();
    bool parseGPGGA();
    bool parseGPGSA(); //MEMO: not implemented yet
    bool parseGPGSV(); //MEMO: not implemented yet
    bool parseGPVTG(); //MEMO: not implemented yet

private:
    Thread *eventThread;
    osThreadId  threadId;

    RawSerial *serialGps;

    char rxBuffer[GPS_DATA_BUFFER_SIZE];
    char rxLineBuffer[GPS_LINE_BUFFER_SIZE];

    // Circular buffer pointers
    // volatile makes read-modify-write atomic
    volatile int rxInPointer;
    volatile int rxOutPointer;

    // Serial
    void interruptRx();
    void interruptRxThread();
    void readLine();
};

}

#endif //GS_GYSFDMAXB_H
