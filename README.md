# Lightning Detector

![EyeCatch](https://github.com/iwaita2ya/ImageStore/blob/master/hovergame-challenge-2019-lightning-detector.png)

This repository holds a lightning detector source code, which watches the events of lightning 
and post them to the API server.

Lightning event will be handled by [AS3935](https://ams.com/en/AS3935), Franklin Lightning Sensor developed by [ams](https://ams.com),
which can detect both cloud to ground and cloud to cloud lightning activity within a 40km range.

The microcontroller([mbed LPC1768](https://os.mbed.com/platforms/mbed-LPC1768/)) also captures GPS data to locate its current position, and calibrate RTC as well.
What you really need is providing power supply (5V or higher), LAN cable, and microSD card. 
Other preparations/calibrations are done by automatically.

This repository has been created for applying [HoverGames Challenge 2019](https://www.hackster.io/contests/hovergames#challengeNav) held by [hackster.io](https://www.hackster.io).
Because of the reason, I will keep this repository as-is after submission closed, but will make changes/update for future.

Results of lightnings are plotted on [maps](https://lightning-detector.herokuapp.com/map/lightnings).

## Things you need
This programs works on [mbed LPC1768](https://os.mbed.com/platforms/mbed-LPC1768/), and it communicates with following devices:
* [SparkFun Lightning Detector - AS3935](https://www.sparkfun.com/products/15441)
* [GPS Module - GYSFDMAXB](https://www.digikey.com/product-detail/en/taiyo-yuden/GYSFDMAXB/GYSFDMAXB-ND/10228322)
* Ethernet via RJ45 MagJack ( like [this one](https://www.sparkfun.com/products/13021) )
* MicroSD card via [MicroSD Socket](https://www.adafruit.com/product/1660)

![Schema](https://github.com/iwaita2ya/ImageStore/blob/master/hovergame-challenge-2019-lightning-detector-schema.png)
Please see [here](https://drive.google.com/open?id=1k-RoowdKZajJ39Wr6T17Nuam9WXz578P) for the complete schematic circuit diagram.

## Installation

You need to configure [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
and [PlatformIO Core](https://docs.platformio.org/en/latest/core.html) for build.

### Clone & Build
```bash
$ git clone https://github.com/iwaita2ya/HoverGamesChallenge2019_Lightning-Detector.git
$ cd ./HoverGamesChallenge2019_Lightning-Detector
$ platformio run
```

### Install
After the build completed, you'll see binary file "firmware.bin" under .pio/build/lpc1768 directory located in the project directory.

Then connect LPC1768 onto your PC and copy the binary file.

```bash
cp -p .pio/build/lpc1768/firmware.bin [/path/to/LPC1768]
```

Under Windows10 environment, you may need to install serial port driver before connect LPC1768.
Please see [here](https://os.mbed.com/handbook/Windows-serial-configuration) for more detail.

## Development Environment

This program has been developed under following environment:
* Ubuntu ([Pop!_OS](https://system76.com/pop)) 18.04 LTS x86_64
* [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* [PlatformIO Core](https://platformio.org/)
* [CLion](https://www.jetbrains.com/clion/)


## Limitations

There are some limitations in current program such as:
* You must provide microSD card onto SD-card adapter (should be treated as optional, but mandatory at this point.)
* Network connection is supported with DHCP only (could be static in future.)
