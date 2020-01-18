#ifndef _AS3935_h_
#define _AS3935_h_

#include "mbed.h"
//#include "rtos.h"
//#include "Event.h"

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

//using namespace rtos;
//using namespace events;

namespace greysound {

class AS3935 {
public:
    /**
     * @param i2c I2C class
     * @param irq IRQ pin
     */
//    AS3935(I2C &i2c, PinName irq);
    /**
     * @param sda I2C SDA pin
     * @param scl I2C SCL pin
     * @param irq IRQ pin
     */
    AS3935(PinName sda, PinName scl, PinName irq);

    /** Initialize AS3935
     */
    void init();

    /**
     * Read Energy of the Single Lightning and Distance estimation
     * @param energy (return) Energy of the Single Lightning
     * @param distance (return) Distance estimation
     */
    void read(uint8_t &eventType, int &energy, int &distance);

    /**
     * Attach a function for lightning interrupt
     * @param fptr pointer to a void function, or 0 to set as none
     */
    void attach(void (*fptr)(void)) {
        //functionPointer.attach(callback(fptr));
        cb = callback(fptr);
    }
    /**
     * Attach a member function for lightning interrupt
     * @param tptr pointer to the object to call the member function on
     * @param mptr pointer to the member function to be called
     */
    template<typename T>
    void attach(T *tptr, void (T::*mptr)(void)) {
        cb = callback(tptr, mptr);
    }

private:
//    EventQueue *queue;
//    Thread *thread;

    I2C i2c;
    InterruptIn interruptPin;
    int frequencyCounter;
    int currentState;
    int eventType;

    Callback<void(void)> cb;

    void tuneAntenna();
    void isr_freq ();
    void isr_lightning ();
};
}

#endif
