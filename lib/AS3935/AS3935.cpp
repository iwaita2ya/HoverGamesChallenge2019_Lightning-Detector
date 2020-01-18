#define DEBUG

/*
 * AMS, Franklin Lightning Sensor "AS3935" Library
 * Copyright (c) 2015 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 */
// http://ams.com/eng/Products/Lightning-Sensor/Franklin-Lightning-Sensor/AS3935

#include "AS3935.h"

namespace greysound {

#define AS3935_ADDR 0x00
#define MODE_WRITE 0x0000
#define MODE_READ  0x4000

// energy level
enum EVENT_TYPE {
    NOT_DEFINED = 0x00,
    NOISE_LEVEL_TOO_HIGH = 0x01,
    DISTURBER_DETECTED = 0x04,
    LIGHTNING_INTERRUPT = 0x08,
};

enum DEVICE_STATE {
    STATE_NORMAL = 0,
    STATE_TUNING = 1,
};

AS3935::AS3935 (PinName sda, PinName scl, PinName irq) : i2c(sda, scl), interruptPin(irq) {
    interruptPin.mode(PullUp);
    currentState = STATE_NORMAL;
    eventType = NOT_DEFINED;

//    queue = new EventQueue(16 * EVENTS_EVENT_SIZE);
//    thread = new Thread(osPriorityAboveNormal);
}

void AS3935::init () {
    char cmd[2];

    /**
     * Reg. 0x3C
     *
     * Direct Command(直接コマンド)
     * Sets all registers in default mode
     */
    cmd[0] = 0x3c;
    cmd[1] = 0x96; // PRESET_DEFAULT
    i2c.write(AS3935_ADDR, cmd, 2);

    /**
     * Reg. 0x3D
     *
     * Direct Command(直接コマンド)
     * Calibrates automatically the internal RC Oscillators
     */
    cmd[0] = 0x3d;
    cmd[1] = 0x96; // CALIB_RCO
    i2c.write(AS3935_ADDR, cmd, 2);

    /**
     * Reg. 0x00
     *
     * AFE_GB[5:1] - Analog Front-end Gain Boost (屋外／屋内ゲイン設定)
     * 雷検知に用いるの500kHZ(帯域幅33kHZ)の信号を増幅するためのパラメータ
     * データシートは下記2値を推奨
     * 0x12 - Indoor (default)
     * 0x0E - Outdoor
     */
    cmd[0] = 0x00;
    cmd[1] = (0x12<<1); // AFE_GB=12(Indoor)
    i2c.write(AS3935_ADDR, cmd, 2);

    /**
     * Reg. 0x01
     *
     * NF_LEV[6:4] - Noise Floor Level (ノイズフロア閾値)
     * この閾値を超えてノイズが発生した場合、AS3935は割り込みを発生させて
     * オペレーションが困難であることをMCUに通知する。閾値は indoor/outdoor で異なる
     * 0x00 - in:28  out:390
     * 0x01 - in:45  out:630
     * 0x02 - in:62  out:860 (default)
     * 0x03 - in:78  out:1100
     * 0x04 - in:95  out:1140
     * 0x05 - in:112 out:1570
     * 0x06 - in:130 out:1800
     * 0x07 - in:146 out:2000
     *
     * WDTH[3:0] - Watchdog threshold
     * AFE 受信時に Signal Verification に入るか否かの閾値
     * 上げすぎると反応しにくくなる
     * 0x00 - Most Sensitive
     * 0xFF - Least Sensitive
     * default: 0x01
     */
    cmd[0] = 0x01;
    cmd[1] = (0x02<<4)|(0x01<<0); // NF_LEV & WDTH
    i2c.write(AS3935_ADDR, cmd, 2);

    /**
     * Reg. 0x02
     *
     * CL_STAT[6] - Clear the statistics built up by the lightning distance estimation
     * 雷発生距離評価の統計データをクリアする
     * ※ high->low->high に変化させるとデータクリア
     * default: 0x01
     *
     * MIN_NUM_LIGH - 0x02[5:4]
     * Minimum Number of Lightning detected in the last 15 min
     * 割り込みを発生させるために必要な雷の検出数（直近15分間）
     * 0x00 - 1 (default)
     * 0x01 - 5
     * 0x10 - 9
     * 0x11 - 16
     *
     * SREJ - 0x02[3:0] - Spike Rejection (disturber rejection)
     * スパイクノイズを除去する閾値。高くすると正確性が増すが、検知率が低くなる
     * default: 0x02
     */
    cmd[0] = 0x02;
    cmd[1] = (0x01<<6)|(0x00<<4)|(0x02<<0); // CL_STAT & MIN_NUM_LIGH & SREJ
    i2c.write(AS3935_ADDR, cmd, 2);

    /**
     * Reg. 0x03
     *
     * LCO_FDIV[7:6] - Frequency division ratio for antenna tuning
     * ループアンテナは共振周波数500kHz、品質係数15になるよう設計されているが、
     * チューニングが必要な場合、共振周波数を除算する係数を設定できる。
     * 0x00 - 16 (default)
     * 0x01 - 32
     * 0x10 - 64
     * 0x11 - 128
     *
     * MASK_DIST - 0x03[5]
     * Mask the disturber interrupts INT_D
     * 外乱発生時に割り込みを発生させるか否かを設定する
     * 0x00 - Do not Mask INT_D (default)
     * 0x01 - Mask INT_D
     *
     * INT - 0x03[3:0] (READ ONLY)
     * Interrupt Management
     * イベントが発生すると、割り込みを発生させた後、このレジスタに結果を書き込む
     * 0x01 - Noise Level Too High (INT_NH)
     * 0x04 - Disturber(外乱) Detected (INT_D)
     * 0x08 - Lightning(雷) Interrupt (INT_L)
     */
    cmd[0] = 0x03;
    cmd[1] = (0x00<<6)|(0x00<<5); // LCO_FDIV=1/16 & No INT_D Mask
    i2c.write(AS3935_ADDR, cmd, 2);

    tuneAntenna();

    // Start the event queue
//    thread->start(callback(queue, &EventQueue::dispatch_forever));
//    printf("Starting in context %p\r\n", ThisThread::get_id());

    //interruptPin.fall(queue->event(callback(this, &AS3935::isr_lightning)));
    interruptPin.fall(callback(this, &AS3935::isr_lightning));
}

// Antenna Tuning (500kHz)
void AS3935::tuneAntenna() {

    /**
     * Reg. 0x08 - Antenna Tuning
     *
     * The AS3935 uses a loop antenna based on a parallel LC resonator. The antenna has to be designed to have
     * its resonance frequency at 500kHz and a quality factor of around 15. With a register setting it is possible
     * to display on the IRQ pin the resonance frequency of the antenna as a digital signal with the register
     * Reg.0x08[7] =1. The external unit can measure this frequency and tune the antenna adding or removing the
     * internal capacitors with the register REG0x08[3:0]. It is necessary to tune the antenna with an accuracy
     * of ±3.5% to optimize the performance of the signal validation and distance estimation.
     * The resonance frequency is internally divided by a factor, which is programmable with the register Reg.0x03[7:6]
     *
     * DISP_LCO[7]
     * Display LCO on IRQ pin
     * 0x01 にすると共振周波数を割り込みピンに出力する
     * default: 0x00
     *
     * DISP_SRCO[6]
     * Display SRCO (System RCO) on IRQ pin
     * デジタル回路に1.1MHZのクロックを供給する発振回路 SRCO を割り込みピンに出力する
     * default: 0x00
     *
     * DISP_TRCO[5]
     * Display TRCO (Timer RCO) on IRQ pin
     * 32.768HHZで発信する低消費発振回路 TRCO を割り込みピンに出力する
     * default: 0x00
     *
     * TUN_CAP[3:0]
     * Internal Tuning Capacitors (from 0 to 120pF in steps of 8pf)
     * default: 0x00
     *
     * アンテナのチューニング方法
     * 1) 共振周波数(LCO)を測定する
     * 2) 内部キャパシタを追加する
     * 3) 共振周波数が 500kHz になるまで 1)-2) を繰り返す
     */

    int i, n, m = 10000, r = 0;
    char cmd[2];
    Timer timer;

    /**
     * 1) Count LCO on IRQ pin
     * 共振周波数(LCO)を測定する
     */
    currentState = STATE_NORMAL;
    interruptPin.fall(callback(this, &AS3935::isr_freq));

    for (i=0; i<0x10; i++) {

        // Display LCO on IRQ pin
        cmd[0] = 0x08;
        cmd[1] = 0x80 | i;
        i2c.write(AS3935_ADDR, cmd, 2);

        wait_ms(10);

        timer.reset();
        timer.start();

        frequencyCounter = 0; // reset counter
        currentState = STATE_TUNING; // state: normal -> tuning

        while (timer.read_ms() < 100) {
            // no nothing
        }

        currentState = STATE_NORMAL; // state: tuning -> normal

        n = abs(frequencyCounter - 3125);
        if (m > n) {
            r = i;
        } else {
            break;
        }
        m = n;
    }
    interruptPin.fall(NULL);
    timer.stop();

    /**
     * 2) update internal tuning capacitor
     * 内部キャパシタを追加する
     */
    cmd[0] = 0x08;
    cmd[1] = r;
    i2c.write(AS3935_ADDR, cmd, 2);

    DEBUG_PRINTF("- TUN_CAP:%d Freq:%d\r\n", r, frequencyCounter * 16 * 10);
}

void AS3935::read (uint8_t &_eventType, int &energy, int &distance) {

    _eventType = eventType;

    /**
     * Reg. 0x04 - Energy of the Single Lightning LSBYTE
     * S_LIG_L[7:0]
     *
     * Reg. 0x05 - Energy of the Single Lightning MSBYTE
     * S_LIG_M[7:0]
     *
     * Reg. 0x06 - Energy of the Single Lightning MMSBYTE
     * S_LIG_MM[4:0]
     *
     * Reg. 0x07 - Distance estimation
     * DISTANCE[5:0]
     */

    char cmd[4];

    cmd[0] = 0x04;
    i2c.write(AS3935_ADDR, cmd, 1, true);
    i2c.read(AS3935_ADDR, cmd, 4);
    energy = ((cmd[2] & 0x1f) << 16) | (cmd[1] << 8) | cmd[0];
    distance = cmd[3] & 0x3f;
}

void AS3935::isr_freq () {
    if (currentState == STATE_TUNING) {
        frequencyCounter ++;
    }
}

void AS3935::isr_lightning () {
    char cmd[2];

    cmd[0] = 0x03;
    i2c.write(AS3935_ADDR, cmd, 1, true);
    i2c.read(AS3935_ADDR, cmd, 1);
    //DEBUG_PRINTF("IRQ: %02x\r\n", cmd[0]);

    eventType = cmd[0] & 0x0f; // Interrupts
    switch (eventType) {
        case NOISE_LEVEL_TOO_HIGH:
            cb.call(); //MEMO: Test
            break;
        case DISTURBER_DETECTED:
            cb.call(); //MEMO: Test
            break;
        case LIGHTNING_INTERRUPT:
            cb.call();
            break;
    }
}
}
