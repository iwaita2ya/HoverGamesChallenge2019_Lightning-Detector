#define DEBUG

#include <mbed.h>
#include "EthernetInterface.h"
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include "AS3935.h"
#include "GYSFDMAXB.h"

using namespace greysound;

/**
 * flags
 */
uint8_t isActive;
uint8_t isLightningDetected;
uint8_t isReadyToLogGps;

/**
 * Serial
 */
RawSerial *serial;

/**
 * SD Card
 */
SDBlockDevice *blockDevice;
FATFileSystem *fileSystem;

// Maximum number of elements in buffer
#define BUFFER_MAX_LEN 10
// device_id
#define DEVICE_ID_SIZE 32

/**
 * AS3935 (Lightning Detector)
 */
AS3935 *lightningDetector; // sda, scl, irq

/**
 * GYSFDMAXB (GPS)
 */
GYSFDMAXB *gps;

#ifdef DEBUG
#define DEBUG_PRINT(fmt) serial->printf(fmt)
#define DEBUG_PRINTF(fmt, ...) serial->printf(fmt, __VA_ARGS__)
#define DEBUG_PUTC(x) serial->putc(x);
#else
#define DEBUG_PRINT(fmt)
#define DEBUG_PRINTF(fmt, ...)
#define DEBUG_PUTC(x)
#endif

/**
 * Ethernet
 */
// Network interface
#define HOST_URL "lightning-detector.herokuapp.com"
EthernetInterface ethernetInterface;
TCPSocket socket;

/**
 * LED
 */
DigitalOut *led;

/**
 * Global Variables
 */
char deviceId[DEVICE_ID_SIZE];

// ----- FUNCTION PROTOTYPES -----
void noMemory();
//uint8_t initSdCard(SDBlockDevice *sdBlockDevice);
void getDeviceIdFromServer(char *deviceId, uint8_t size);
void getDeviceId(char *deviceId, uint8_t size);
static void logGps();
void saveLogGps(); // save onto sd-card
void postLogGps(); // send data
static void lightning();
void lightningTest();
void saveLogLightning(uint8_t type, int energy, int distance);
void postLogLightning(uint8_t type, int energy, int distance);
void sdCardTest();
void socketTest();

// ----- MAIN -----
int main() {

    // init device id
    memset(deviceId,'\0', sizeof(deviceId));

    // set active flag
    isActive = 1;

    // clear flags
    isLightningDetected = 0;
    isReadyToLogGps = 0;

    /**
     * Init Serial
     */
    serial = new RawSerial(USBTX, USBRX, 115200);
    DEBUG_PRINT("Serial is Ready for Debugging.\r\n");

    /**
     * Init SD Card
     */
    DEBUG_PRINT("Init SD Card.\r\n");
    blockDevice = new SDBlockDevice(p5, p6, p7, p8); // mosi, miso, clk, cs
    blockDevice->init();
    fileSystem = new FATFileSystem("fs");

//    /**
//     * Perform SD Card Test (Optional)
//     */
//    DEBUG_PRINT("Perform SD Card Test.\r\n");
//    sdCardTest();

    /**
     * Init Ethernet
     */
    DEBUG_PRINT("Init Ethernet (DHCP)\r\n");
//    ethernetInterface = new EthernetInterface();
//    ethernetInterface->connect();
    nsapi_error_t connect_status = ethernetInterface.connect(); //TODO: 静的IPにも対応したい
    if (connect_status != NSAPI_ERROR_OK) {
        DEBUG_PRINTF("Failed to connect to network (%d)\n", connect_status);
        return 1;
    }

    const char *ip = ethernetInterface.get_ip_address();
    DEBUG_PRINTF("IP: %s\r\n", ip ? ip : "No IP");

//    /**
//     * Perform Network Connection Test (Optional)
//     */
//    DEBUG_PRINT("Network Connection Test.\r\n");
//    socketTest();

    /**
     * Preparation: If deviceId has not received from server, get it
     */
    getDeviceId(deviceId, sizeof(deviceId));
    DEBUG_PRINTF("Device Id:%s\r\n", deviceId);

    /**
     * Init Lightning Detector
     */
    DEBUG_PRINT("Init Lightning Detector.\r\n");
    lightningDetector = new AS3935(p9, p10, p16); // sda, scl, irq
    lightningDetector->attach(&lightning);
    lightningDetector->init();

    /**
     * Perform Lightning Test (Optional)
     */
    lightningTest();

    /**
     * Init GPS
     */
    DEBUG_PRINT("Init GPS.\r\n");
    gps = new GYSFDMAXB(p13, p14, p15, 9600);
    gps->start();
    wait(3); //wait few seconds until GPS ready

    /**
     * Init LED
     */
    DEBUG_PRINT("Init LEDs.\r\n");
    led = new DigitalOut(LED1);
    led->write(0);

    /**
     * Main Loop
     */
    DEBUG_PRINT("Start Main Loop.\r\n");
    Ticker *logGpsTicker = new Ticker();
    logGpsTicker->attach(callback(&logGps), 60.0f); // call every 60 min

    while(isActive) {

        // lightning detected?
        if(isLightningDetected) {
            // read value
            uint8_t type;
            int energy, distance;
            lightningDetector->read(type, energy, distance);

            // save data
            DEBUG_PRINTF("Detected: Type=%d Energy=%d Distance=%d\r\n", type, energy, distance);
            saveLogLightning(type, energy, distance); // on sd
            postLogLightning(type, energy, distance); // server

            // clear flag
            isLightningDetected = 0;
        }

        // should log gps data?
        if(isReadyToLogGps) {
            gps->stop();
            DEBUG_PRINT("Start Logging GPS data.\r\n");
            saveLogGps(); // to sd-card
            postLogGps(); // to server
            isReadyToLogGps = 0; // clear flag
            gps->start();
        }

        led->write(!led->read()); // toggle led
        wait(1.0f); // wait
    }

    delete(gps);
    delete(lightningDetector);
    delete(led);
//    delete(socket);
//    delete(ethernetInterface);
    delete(fileSystem);
    delete(blockDevice);
    delete(serial);

    return 0;
}

/**
 * FUNCTIONS
 */

void noMemory() {
    DEBUG_PRINT("panic: can't allocate to memory!\r\n");
    exit(-1);
}

static void lightning() {
    //TODO: 短時間で複数回検知した場合に対応する
    // add ring buffer for handling multiple events occurred within a short time period
    isLightningDetected = 1;
}

void lightningTest() {
    DEBUG_PRINT("Start lightningTest()\r\n");

    uint8_t type;
    int energy, distance;
    lightningDetector->read(type, energy, distance);

    DEBUG_PRINTF("Type:%d Energy:%d Distance:%d\r\n", type, energy, distance);
}

static void logGps() {
    isReadyToLogGps = 1;
}

/**
 * GPSログデータをSDカードに保存する
 */
void saveLogGps() {

    DEBUG_PRINT("\r\nsaveLogGps() Started.\r\n");
    DEBUG_PRINTF("mode:%c time:%lld lat:%lf lon:%lf alt:%lf\r\n", gps->modeIndicator, gps->currentTime, gps->latitude, gps->longitude, gps->altitude);

    // mount sd card
    DEBUG_PRINT("Mount SD Card\r\n");
    fflush(stdout);
    int err = fileSystem->mount(blockDevice);
    if (err) {
        error("error: %s (%d)\n", strerror(-err), err);
    }

    // open file
    DEBUG_PRINT("Open Device Log File\r\n");
    fflush(stdout);
    FILE *fp = fopen("/fs/HoverGames/device.log", "a+");
    if (!fp) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }

    // write on a file
    DEBUG_PRINT("Write GPS Data onto SD-Card.\r\n");
    fflush(stdout);
    err = fprintf(fp, "%c,%lld,%lf,%lf,%lf\r\n", gps->modeIndicator, gps->currentTime, gps->latitude, gps->longitude, gps->altitude);
    if (err < 0) {
        DEBUG_PRINT("Writing on sd card failed\r\n");
        return;
    }

    // close file
    DEBUG_PRINT("Close File.\r\n");
    err = fclose(fp);
    if (err < 0) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }

    // unmount block device
    DEBUG_PRINT("Unmount Block Device.\r\n");
    fflush(stdout);
    err = fileSystem->unmount();
    if (err < 0) {
        error("error: %s (%d)\n", strerror(-err), err);
    }
}

/**
 * サーバにGPSログデータを送る
 * @param _deviceId
 * @param lat
 * @param lon
 * @param alt
 */
void postLogGps() {

    DEBUG_PRINT("\r\npostLogGps() Started.\r\n");

    nsapi_size_or_error_t result;

    // Open a socket on the network interface, and create a TCP connection to mbed.org
    DEBUG_PRINT("Open Socket.\r\n");
    result = socket.open(&ethernetInterface);
    if(result != 0) {
        error("Error! socket.open() returned %d\r\n", result);
    }
    // Connect to host
    DEBUG_PRINT("Connect to host.\r\n");
    result = socket.connect(HOST_URL, 80);
    if(result != 0) {
        error("Error! socket.connect() returned %d\r\n", result);
    }

    /**
     * Send Request
     */
    char *sendBuffer = new char[256];

    // create datetime string
    char *timeString = new char[32];
    struct tm *ptm;
    ptm = gmtime(&gps->currentTime);
    strftime(timeString, 32, "%Y-%m-%d %H:%M:%S %Z", ptm);
    //DEBUG_PRINTF("UTC: %s\r\n", timeString);

    // create request body
    char *requestBody = new char[128];
    sprintf(requestBody, "device=%s&lat=%f&lon=%f&alt=%f&loggedAt=%s", deviceId, gps->latitude, gps->longitude, gps->altitude, timeString);
    //DEBUG_PRINTF("%s\r\n", requestBody);

    // calc body length
    size_t length = strlen(requestBody);

    // create content-length data line with body length
    char *contentLength = new char[32];
    sprintf(contentLength, "Content-Length: %d\r\n", length);
    //DEBUG_PRINTF("%s", contentLength);

    // create method line data
    char *requestMethod = new char[34];
    sprintf(requestMethod, "POST /log/device/create HTTP/1.1\r\n");
    //DEBUG_PRINTF("%s", requestMethod);

    // create host line data
    char *requestHost = new char[64];
    sprintf(requestHost, "Host: %s\r\n", HOST_URL);
    //DEBUG_PRINTF("%s", requestHost);

    // create content-type line data
    char *contentType = new char[51];
    sprintf(contentType, "Content-Type: application/x-www-form-urlencoded\r\n\r\n");
    //DEBUG_PRINTF("%s", contentType);

    strcat(sendBuffer, requestMethod);
    strcat(sendBuffer, requestHost);
    strcat(sendBuffer, contentLength);
    strcat(sendBuffer, contentType);
    strcat(sendBuffer, requestBody);

    DEBUG_PRINT("Start socket.send()\r\n");
    result = socket.send(sendBuffer, strlen(sendBuffer));
    if(result < 0) {
        DEBUG_PRINTF("Error! socket.send() returned: %d\r\n", result);
    }
//    DEBUG_PRINTF("sent %d [%.*s]\n", result, strstr(sendBuffer, "\r\n") - sendBuffer, sendBuffer);
    DEBUG_PRINTF("%s\r\n", sendBuffer);

    // release memory
    delete[] contentType;
    delete[] requestHost;
    delete[] requestMethod;
    delete[] contentLength;
    delete[] requestBody;
    delete[] timeString;
    delete[] sendBuffer;

    // Receive an HTTP response and print out the response line
    char *receiveBuffer = new char[UINT8_MAX];
    uint8_t remaining = UINT8_MAX;
    uint8_t receiveCounter = 0;
    char *pointer = receiveBuffer;
    DEBUG_PRINT("\r\nDATA WAITING");
    while (remaining > 0 && (result = socket.recv(pointer, remaining)) > 0) {
        pointer += result;
        receiveCounter += result;
        remaining -= result;
        DEBUG_PRINT(".");
        wait(1);
    }
    DEBUG_PRINT("Done.\r\n\r\n");
    if (result < 0) {
        printf("Error! socket.recv() returned: %d\n", result);
    }

    // Receive a json format response
    char lineBuffer[64];
    char *startAddress;
    char *endAddress;
    int startPos = 0;
    uint8_t dataLength = 0;

    // set start value
    startAddress = receiveBuffer;
    endAddress = strstr(&receiveBuffer[startPos] , "\r\n" );

    //TODO: Add response code check
    while(endAddress != NULL) {
        memset(lineBuffer,'\0', sizeof(lineBuffer)); // init line buffer
        dataLength = endAddress - startAddress;
        strncpy(lineBuffer, &receiveBuffer[startPos], dataLength);

        DEBUG_PRINTF("%s\r\n", lineBuffer);

        startAddress = endAddress + 2;
        startPos = startAddress - receiveBuffer;
        endAddress = strstr(&receiveBuffer[startPos] , "\r\n" );
    }

    // get last line of data (=http response body data)
    memset(lineBuffer,'\0', sizeof(lineBuffer)); // init line buffer
    dataLength = &receiveBuffer[sizeof(receiveBuffer)-1] - startAddress;
    strncpy(lineBuffer, &receiveBuffer[startPos], dataLength);
    DEBUG_PRINTF("\r\n%s\r\n", lineBuffer);

    delete[] receiveBuffer;

    // Close the socket to return its memory and bring down the network interface
    DEBUG_PRINT("Close Socket.\r\n");
    socket.close();
}

void saveLogLightning(uint8_t type, int energy, int distance) {

    DEBUG_PRINT("saveLogLightning() Started.\r\n");

    // mount sd card
    DEBUG_PRINT("Mount SD Card\r\n");
    fflush(stdout);
    int err = fileSystem->mount(blockDevice);
    if (err) {
        error("error: %s (%d)\n", strerror(-err), err);
    }

    // open file
    DEBUG_PRINT("Open Lightning Log File\r\n");
    fflush(stdout);
    FILE *fp = fopen("/fs/HoverGames/lightning.log", "a+");
    if (!fp) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }

    // write on a file
    DEBUG_PRINT("Write GPS Data onto SD-Card.\r\n");
    fflush(stdout);
    err = fprintf(fp, "%s,%lld,%lf,%lf,%lf,%d,%d,%d\r\n", deviceId, gps->currentTime, gps->latitude, gps->longitude, gps->altitude, type, energy, distance);

    if (err < 0) {
        DEBUG_PRINT("Writing on sd card failed\r\n");
        return;
    }

    // close file
    DEBUG_PRINT("Close File.\r\n");
    err = fclose(fp);
    if (err < 0) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }

    // unmount block device
    DEBUG_PRINT("Unmount Block Device.\r\n");
    fflush(stdout);
    err = fileSystem->unmount();
    if (err < 0) {
        error("error: %s (%d)\n", strerror(-err), err);
    }
}

void postLogLightning(uint8_t type, int energy, int distance) {

    DEBUG_PRINT("\r\npostLogLightning() Started.\r\n");

    nsapi_size_or_error_t result;

    // Open a socket on the network interface, and create a TCP connection to mbed.org
    DEBUG_PRINT("Open Socket.\r\n");
    result = socket.open(&ethernetInterface);
    if(result != 0) {
        error("Error! socket.open() returned %d\r\n", result);
    }
    // Connect to host
    DEBUG_PRINT("Connect to host.\r\n");
    result = socket.connect(HOST_URL, 80);
    if(result != 0) {
        error("Error! socket.connect() returned %d\r\n", result);
    }

    /**
     * Send Request
     */
    char *sendBuffer = new char[256];

    // create datetime string
    char *timeString = new char[32];
    struct tm *ptm;
    ptm = gmtime(&gps->currentTime);
    strftime(timeString, 32, "%Y-%m-%d %H:%M:%S %Z", ptm);
    DEBUG_PRINTF("UTC: %s\r\n", timeString);

    // create request body
    char *requestBody = new char[128];
    sprintf(requestBody, "device=%s&lat=%f&lon=%f&alt=%f&type=%d&energy=%d&distance=%d&loggedAt=%s", deviceId, gps->latitude, gps->longitude, gps->altitude, type, energy, distance, timeString);
    DEBUG_PRINTF("%s\r\n", requestBody);

    // calc body length
    size_t length = strlen(requestBody);

    // create content-length data line with body length
    char *contentLength = new char[32];
    sprintf(contentLength, "Content-Length: %d\r\n", length);
    DEBUG_PRINTF("%s", contentLength);

    // create method line data
    char *requestMethod = new char[34];
    sprintf(requestMethod, "POST /log/device/create HTTP/1.1\r\n");
    DEBUG_PRINTF("%s", requestMethod);

    // create host line data
    char *requestHost = new char[64];
    sprintf(requestHost, "Host: %s\r\n", HOST_URL);
    DEBUG_PRINTF("%s", requestHost);

    // create content-type line data
    char *contentType = new char[51];
    sprintf(contentType, "Content-Type: application/x-www-form-urlencoded\r\n\r\n");
    DEBUG_PRINTF("%s\r\n", contentType);

    strcat(sendBuffer, requestMethod);
    strcat(sendBuffer, requestHost);
    strcat(sendBuffer, contentLength);
    strcat(sendBuffer, contentType);
    strcat(sendBuffer, requestBody);

    DEBUG_PRINT("Start socket.send()\r\n");
    result = socket.send(sendBuffer, strlen(sendBuffer));
    if(result < 0) {
        DEBUG_PRINTF("Error! socket.send() returned: %d\r\n", result);
    }
    DEBUG_PRINTF("%s", sendBuffer);

    // release memory
    delete[] contentType;
    delete[] requestHost;
    delete[] requestMethod;
    delete[] contentLength;
    delete[] requestBody;
    delete[] timeString;
    delete[] sendBuffer;

    // Close the socket to return its memory and bring down the network interface
    DEBUG_PRINT("Close Socket.\r\n");
    socket.close();
}

void sdCardTest() {
    
    // Mount the filesystem
    DEBUG_PRINT("Mounting the filesystem... ");
    fflush(stdout);
    int err = fileSystem->mount(blockDevice);
    DEBUG_PRINTF("%s\r\n", (err ? "Fail :(" : "OK"));

    // Mount failed. Reformat the device
    if (err) {
        // Reformat if we can't mount the filesystem
        // this should only happen on the first boot
        DEBUG_PRINT("No filesystem, formatting... ");
        fflush(stdout);
        err = fileSystem->reformat(blockDevice);
        DEBUG_PRINTF("%s\r\n", (err ? "Fail :(" : "OK"));

        if (err) {
            error("error: %s (%d)\n", strerror(-err), err);
        }
    }

    // Try opening a file for write
    DEBUG_PRINT("Opening \"/fs/HoverGames/fileiotest.txt\" for write... ");
    fflush(stdout);
    FILE *fp = fopen("/fs/HoverGames/fileiotest.txt", "w+"); // a+ for append
    DEBUG_PRINTF("%s\r\n", (!fp ? "Fail :(" : "OK"));

    // File open failed. Create a new file.
    if (!fp) {
        // Try to create a file
        DEBUG_PRINT("No file found, creating a new file... ");
        fflush(stdout);
        fp = fopen("/fs/HoverGames/fileiotest.txt", "w");
        DEBUG_PRINTF("%s\r\n", (!fp ? "Fail :(" : "OK"));

        // file creation failed. Nothing we can do anymore.
        if (!fp) {
            error("error: %s (%d)\n", strerror(errno), -errno);
        }
    }

    // Go through and increment the numbers
    for (int i = 0; i < 10; i++){
        printf("Writing decimal numbers to a file (%d/10)\r\n", i);
        fprintf(fp, "%d\r\n", i);
    }
    printf("Writing decimal numbers to a file (10/10) done.\r\n");

    // Close file
    DEBUG_PRINT("Closing \"/fs/HoverGames/fileiotest.txt\"... ");
    fflush(stdout);
    err = fclose(fp);
    DEBUG_PRINTF("%s\r\n", (err < 0 ? "Fail :(" : "OK"));

    if (err < 0) {
        error("File closing error: %s (%d)\n", strerror(errno), -errno);
    }

    // Display the root directory
    DEBUG_PRINT("Opening the root directory... ");
    fflush(stdout);
    DIR *d = opendir("/fs/");
    DEBUG_PRINTF("%s\r\n", (!d ? "Fail :(" : "OK"));

    if (!d) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }

    DEBUG_PRINT("root directory:\r\n");
    while (true) {
        struct dirent *e = readdir(d);
        if (!e) {
            break;
        }
        DEBUG_PRINTF("  %s\r\n", e->d_name);
    }

    DEBUG_PRINT("Closing the root directory... ");
    fflush(stdout);
    err = closedir(d);
    DEBUG_PRINTF("%s\r\n", (err < 0 ? "Fail :(" : "OK"));
    if (err < 0) {
        error("Close the root directory error: %s (%d)\n", strerror(errno), -errno);
    }

    // Display the numbers file
    DEBUG_PRINT("Opening \"/fs/HoverGames/fileiotest.txt\"... ");
    fflush(stdout);

    fp = fopen("/fs/HoverGames/fileiotest.txt", "r");
    DEBUG_PRINTF("%s\r\n", (!fp ? "Fail :(" : "OK"));

    if (!fp) {
        error("File open (READ) error: %s (%d)\n", strerror(errno), -errno);
    }

    printf("numbers:\n");
    while (!feof(fp)) {
        int c = fgetc(fp);
        DEBUG_PRINTF("%c\r", c);
    }

    DEBUG_PRINT("\rClosing \"/fs/HoverGames/fileiotest.txt\"... ");
    fflush(stdout);
    err = fclose(fp);
    DEBUG_PRINTF("%s\r\n", (err < 0 ? "Fail :(" : "OK"));

    if (err < 0) {
        error("File closing error: %s (%d)\n", strerror(errno), -errno);
    }

    // Tidy up
    DEBUG_PRINT("Unmounting... ");
    fflush(stdout);

    err = fileSystem->unmount();
    DEBUG_PRINTF("%s\r\n", (err < 0 ? "Fail :(" : "OK"));

    if (err < 0) {
        error("error: %s (%d)\n", strerror(-err), err);
    }

    printf("sdCardTest completed\r\n");
}

void socketTest() {

    // Open a socket on the network interface, and create a TCP connection to mbed.org
    socket.open(&ethernetInterface);
    socket.connect("www.arm.com", 80);

    // Send a simple http request
    char sendBuffer[] = "GET / HTTP/1.1\r\nHost: www.arm.com\r\n\r\n";
    int sendCcount = socket.send(sendBuffer, sizeof sendBuffer);
    DEBUG_PRINTF("sent %d [%.*s]\n", sendCcount, strstr(sendBuffer, "\r\n") - sendBuffer, sendBuffer);

    // Receive a simple http response and print out the response line
    char receiveBuffer[64];
    int receiveCount = socket.recv(receiveBuffer, sizeof receiveBuffer);
    DEBUG_PRINTF("recv %d [%.*s]\n", receiveCount, strstr(receiveBuffer, "\r\n") - receiveBuffer, receiveBuffer);

    // Close the socket to return its memory and bring down the network interface
    socket.close();

    // Bring down the ethernet interface
    DEBUG_PRINT("Done.\n");
}

void getDeviceId(char *_deviceId, uint8_t size) {

    // mount sd card
    DEBUG_PRINT("Mount block device.\r\n");
    fflush(stdout);
    int err = fileSystem->mount(blockDevice);
    if (err) {
        error("error: %s (%d)\n", strerror(-err), err);
    }

    // open file
    DEBUG_PRINT("Open deviceId File.\r\n");
    fflush(stdout);
    FILE *fp = fopen("/fs/HoverGames/device_id", "r+");

    // if fine does not exist, create new one.
    if (!fp) {
        // get from server
        DEBUG_PRINT("Get deviceId from Server.\r\n");
        getDeviceIdFromServer(_deviceId, size);

        if(!_deviceId) {
            error("Failed to get device id.");
        }

        // write onto file
        DEBUG_PRINT("Write deviceId onto File.\r\n");
        fflush(stdout);
        fp = fopen("/fs/HoverGames/device_id", "w+");
        if (!fp) {
            error("error: %s (%d)\n", strerror(errno), -errno);
        }
        err = fprintf(fp, "%s", _deviceId);
        if (err < 0) {
            DEBUG_PRINT("Writing on sd card failed\r\n");
        }
    }
    // file exists. Read device id
    else {
        DEBUG_PRINT("Read DeviceId from File.\r\n");
        if (!fgets(_deviceId, size, fp)) {
            error("error: %s (%d)\n", strerror(errno), -errno);
        }
        DEBUG_PRINTF("deviceId:%s\r\n", _deviceId);
    }

    // close file
    DEBUG_PRINT("Close File.\r\n");
    err = fclose(fp);
    if (err < 0) {
        error("error: %s (%d)\n", strerror(errno), -errno);
    }

    // unmount block device
    DEBUG_PRINT("Unmount block device.\r\n");
    fflush(stdout);
    err = fileSystem->unmount();
    if (err < 0) {
        error("error: %s (%d)\n", strerror(-err), err);
    }
}

void getDeviceIdFromServer(char *_deviceId, uint8_t size) {

    nsapi_size_or_error_t result;

    // Open a socket on the network interface, and create a TCP connection to mbed.org
    result = socket.open(&ethernetInterface);
    if(result != 0) {
        error("Error! socket.open() returned %d\r\n", result);
    }

    result = socket.connect("lightning-detector.herokuapp.com", 80);
    if(result != 0) {
        error("Error! socket.connect() returned %d\r\n", result);
    }

    // Send request
    char sendBuffer[] = "POST /device/create HTTP/1.1\r\nHost: lightning-detector.herokuapp.com\r\n\r\n";
    result = socket.send(sendBuffer, sizeof(sendBuffer));
    if(result < 0) {
        DEBUG_PRINTF("Error! socket.send() returned: %d\r\n", result);
    }
    DEBUG_PRINTF("sent %d [%.*s]\n", result, strstr(sendBuffer, "\r\n") - sendBuffer, sendBuffer);

    // Receive an HTTP response and print out the response line
    char receiveBuffer[UINT8_MAX];
    uint8_t remaining = UINT8_MAX;
    uint8_t receiveCounter = 0;
    char *pointer = receiveBuffer;
    DEBUG_PRINT("DATA WAITING");
    while (remaining > 0 && (result = socket.recv(pointer, remaining)) > 0) {
        pointer += result;
        receiveCounter += result;
        remaining -= result;
        DEBUG_PRINT(".");
        wait_ns(1000000);
    }
    DEBUG_PRINT("\r\n");
    if (result < 0) {
        printf("Error! socket.recv() returned: %d\n", result);
    }
    
    // Receive a json format response
    char lineBuffer[64];
    char *startAddress;
    char *endAddress;
    int startPos = 0;
    uint8_t dataLength = 0;

    // set start value
    startAddress = receiveBuffer;
    endAddress = strstr(&receiveBuffer[startPos] , "\r\n" );

    //TODO: Add response code check
    while(endAddress != NULL) {
        memset(lineBuffer,'\0', sizeof(lineBuffer)); // init line buffer
        dataLength = endAddress - startAddress;
        strncpy(lineBuffer, &receiveBuffer[startPos], dataLength);

        DEBUG_PRINTF("%s\r\n", lineBuffer);

        startAddress = endAddress + 2;
        startPos = startAddress - receiveBuffer;
        endAddress = strstr(&receiveBuffer[startPos] , "\r\n" );
    }

    // get last line of data (=http response body data)
    memset(lineBuffer,'\0', sizeof(lineBuffer)); // init line buffer
    dataLength = &receiveBuffer[sizeof(receiveBuffer)-1] - startAddress;
    strncpy(lineBuffer, &receiveBuffer[startPos], dataLength);
    DEBUG_PRINTF("%s\r\n", lineBuffer);

    // extract deviceId value from json data
    memset(_deviceId,'\0', size); // init line buffer
    sscanf(lineBuffer, "{\"hash\":\"%[^\"]\"}", _deviceId);
    DEBUG_PRINTF("deviceId: %s\r\n", _deviceId);

    // Close the socket to return its memory and bring down the network interface
    DEBUG_PRINT("Close Socket.\r\n");
    socket.close();
}