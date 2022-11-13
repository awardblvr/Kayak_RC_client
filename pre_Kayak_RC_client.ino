/*
  pre_Kayak_RC_client.ino   (WAS SimpleEspNowConnectionClient)

  Basic EspNowConnection Client implementation

  HOWTO Arduino IDE:
  - Prepare two ESP8266 or ESP32 based devices (eg. WeMos)
  - Start two separate instances of Arduino IDE and load 
    on the first one the 'SimpleEspNowConnectionServer.ino' and
    on the second one the 'SimpleEspNowConnectionClient.ino' sketch and upload
    these to the two ESP devices.
  - Start the 'Serial Monitor' in both instances and set baud rate to 115200
  - Type 'startpair' into the edit box of both 'Serial Monitors' and hit Enter key (or press 'Send' button)
  - After devices are paired, type 'textsend', 'structsend' or 'bigsend' into the edit box
    of the 'Serial Monitor' and hit Enter key (or press 'Send' button)

  - You can use multiple clients which can be connected to one server

*/
//#define ESP32

#include "SimpleEspNowConnection.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <EasyButton.h>
#include "Adafruit_LC709203F.h"
#include <Adafruit_NeoPixel.h>
#include <BMI160Gen.h>
#include <vector>
#include <forward_list>
#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg
//#define STOP_ALL_SERIAL_IO
#include "debug_serial_print.h"
#include "shared_yak_messaging.h"





typedef struct {
    uint32_t color0;
    uint32_t color1;
    uint32_t durationMillis;
    uint8_t flashes;
    uint8_t current_color;    // always 0 (color0) or 1 (color1)
    uint32_t nextChangeMillis;
} PixelFlashEvent_t;
std::forward_list<PixelFlashEvent_t> PixelFlashEvents {};
PixelFlashEvent_t pixelAction;
#define PIXEL_TRIG_NOW 1

// MY ADDRESS! ESP32-S2-TFT   7C:DF:A1:95:0C:6E
//   serverAddress = "AC0BFBDCE1F1"; // Test if you know the server
//   serverAddress = "AC0BFBDD4100"; // Initial trial w/ WeMos D1Mini Pro
//   serverAddress = "A0764E5ADF70"; // ESP32-C3-mini-1 (board is ESP32-C3-DevKitM-1)
#define SERVER_ADDRESS "A0764E5ADF70"



//  Use RTC_DATA_ATT as a prefix to any var below to save in RTC Ram
//  (not lost uin sleep modes, but resets on full power-cycle  or reboot
String inputString;
String serverAddress;
String clientAddress;  // This is me
//std::vector<String> gear{ "NEUTRAL", "FORWARD", "REVERSE" };   Replaced by GEAR_t_v
char tft_lin_buf[50];
uint16_t motorSpeedPotVal=0;
uint16_t lastMotorSpeedPotVal=1;
uint8_t motorSpeedPercent=0;
//static void task16ms();
//static void task128ms();
static uint32_t tick50=0, tick128=0, tick30000=0;
uint16_t maxPotAdcVal=5000;  // used in scaling, adjusts higher if new value is higher
uint32_t lastActionMillis=0;
int avgRightTouchVal=1, lastAvgRightTouchVal=1;
bool touchUpdateHoldoff=false;
bool LC709203FisOK = false;
uint32_t nextMotionUnblockMillis=0;
uint32_t motionDisplayNextOffMillis=0;
bool motionWasTriggered=false;
bool wifiIsUp=false;
bool yakMessageUpdated = false;
uint32_t msgIDcounter=0;
GEAR_t motorSwitch=NEUTRAL;
GEAR_t lastMotorSwitch=motorSwitch;

// uint8_t motorSwitch=0;   // Motor switch state: 0=Neutral, 1=Forward, 2=Reverse, all else invalid, (like 0)

#define DEFAULT_I2C_PORT &Wire

#define BUTTON_LEFT_PIN 6
#define BUTTON_RIGHT_PIN 5
#define BUTTON_RIGHT_TOUCH_PIN 14
#define SWITCH_FORWARD_PIN 10
#define SWITCH_REVERSE_PIN 9
#define SPEED_CONTROL_POT_PIN 8
#define ACCEL_INT_1_PIN 15

#define I2C_ADDR_ACCEL_BMI160 0x69

#define DISP_GEAR_LINE_PIXEL 42
#define DISP_GEAR_SIZE 3
#define DISP_BOTTOM_WARN_PIXEL 114
#define DISP_BOTTOM_WARN_SIZE 3
#define DISP_ADDR_LINE_PIXEL 15
#define DISP_ADDR_LINE_SIZE 2

#define DISP_DEBUG_LINE_PIXEL 68
#define DISP_DEBUG_LINE_SIZE 2
#define DISP_BATT_LINE_PIXEL 120
#define DISP_BATT_LINE_SIZE 2

#define INACTIVITY_MS_BEFORE_REDUCE_PWR 60000
#define INACTIVITY_MS_BEFORE_SLEEP 1200000
#define AFTER_MOTION_IGNORE_MS 30000
#define MOTION_DISPLAY_MS 5000

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
SimpleEspNowConnection simpleEspConnection(SimpleEspNowRole::CLIENT);
Adafruit_LC709203F lc;
Adafruit_NeoPixel pixel(1 /*NUMPIXELS*/, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
movingAvg avgSpeed(100);                  // define the moving average object
movingAvg avgRightTouch(100);              // define the moving average object

EasyButton buttonRight(BUTTON_RIGHT_PIN);
EasyButton buttonLeft(BUTTON_LEFT_PIN);
EasyButton switchForward(SWITCH_FORWARD_PIN);
EasyButton switchReverse(SWITCH_REVERSE_PIN);

// ---------------------- Yak Messaging for ESP NOW  ------------------------

yakMessage_t yakMessage = { YAK,        // msgType
                            0,          // msgID  <future> to track messages lost or ensure/check ordering
                            PING,       // action
                            NEUTRAL,    // gear
                            0           // speed (0-100)
                          };   // A default message


// -------------------

// Neopixel colors
#define NeoBlack    0x000000
#define NeoWhite    0xFFFFFF
#define NeoRed      0xFF0000
#define NeoGreen    0x00FF00
#define NeoBlue     0x0000FF
#define NeoYellow   0xFFFF00
#define NeoMagenta  0xFF00FF
#define NeoPurple   0x800080
#define NeoOrange   0xFF8C00
#define NeoLime     0x00FF00
#define NeoNavyBlue 0x000080
#define NeoGray     0x696969
#define NeoSilver   0xC0C0C0

void flash_pixel(uint32_t color0, uint8_t flashes=1, uint32_t duration=500, uint32_t color1=NeoBlack);


/* TFT Color Choices:
  ST77XX_BLACK
  ST77XX_WHITE
  ST77XX_RED
  ST77XX_GREEN
  ST77XX_BLUE
  ST77XX_CYAN
  ST77XX_MAGENTA
  ST77XX_YELLOW
  ST77XX_ORANGE

  Line Clearing:
  size 1:   i=41 for char(0xDA)
  size 2:   i=21 for char(0xDA)
  size 3:   i=41 for char(0xDA)
*/

void lineLocator(int8_t fontSize, int8_t pixelLine, int8_t chars=0)
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(fontSize);
    tft.setCursor(0 /*x*/, pixelLine);
    while (chars--) {
        tft.print(" ");
    }
    tft.print(char(0xDA)); // 0xDA is the CP437(false) square block
}


void clearTextLine(int8_t chars, bool debug=false)
{

    tft.setTextColor(ST77XX_BLACK);
    while (--chars) {
        tft.print(char(0xDA)); // 0xDA is the CP437(false) square block
    }
    if (debug) {
        tft.setTextColor(ST77XX_RED);
        tft.println(".");
    }
}


void clearSizedTextLine(int8_t fontSize, int8_t pixelLine, bool debug=false)
{
    int chars=0;

    // size 1:   i=41 for char(0xDA)
    // size 2:   i=21 for char(0xDA)
    // size 3:   i=14 for char(0xDA)
    switch (fontSize) {
        case 1:
            chars=41;
            break;
        case 2:
            chars=21;
            break;
        case 3:
            chars=14;
            break;
        default:
            return;
    }

    tft.setTextSize(fontSize);
    tft.setCursor(0 /*x*/, pixelLine);
    tft.setTextColor(ST77XX_BLACK);
    while (--chars) {
        tft.print(char(0xDA)); // 0xDA is the CP437(false) square block
    }
    if (debug) {
        tft.setTextColor(ST77XX_RED);
        tft.print(".");
    }
    tft.setCursor(0 /*x*/, pixelLine);
}


void updateGearStateDisplay(void)
{
    uint16_t color = ST77XX_ORANGE;  // orange means this is unexpected
    switch (motorSwitch) {
    case NEUTRAL:
        color = ST77XX_YELLOW;
        break;
    case FORWARD:
        color = ST77XX_GREEN;
        break;
    case REVERSE:
        color = ST77XX_RED;
        break;
    default:
        clearSizedTextLine(DISP_GEAR_SIZE, DISP_GEAR_LINE_PIXEL);
        tft.print("OH SHIT!!");
        return;
        }

    clearSizedTextLine(DISP_GEAR_SIZE, DISP_GEAR_LINE_PIXEL);
    tft.setTextColor(color);
    tft.print(GEAR_t_v[motorSwitch]);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" " + String(motorSpeedPercent));
}

void updateTouchDisplay(void)
{
    uint16_t color = ST77XX_ORANGE;
    uint32_t aRTV=0;

    clearSizedTextLine(DISP_DEBUG_LINE_SIZE, DISP_DEBUG_LINE_PIXEL);
    if (!touchUpdateHoldoff) {
        tft.setTextColor(color);
        tft.print("Touch:");
        tft.setTextColor(ST77XX_WHITE);
        aRTV = avgRightTouchVal > 1000 ? avgRightTouchVal = avgRightTouchVal/100 : avgRightTouchVal;
        tft.print(" " + String(aRTV));
        if (motionDisplayNextOffMillis != 0) {
            tft.setTextColor(ST77XX_MAGENTA);
            tft.print("  MOTION");
        }
    } else {
        debug_pln("touchUpdateHoldoff!  Skipping display update");
    }
}


void updateBatteryDisplay(void)
{
    uint16_t color = ST77XX_ORANGE;

    clearSizedTextLine(DISP_BATT_LINE_SIZE, DISP_BATT_LINE_PIXEL);
    if (LC709203FisOK) {
        tft.setTextColor(ST77XX_GREEN);
        snprintf(tft_lin_buf, sizeof(tft_lin_buf), "B v:%.2f, charge:%.0f%%", lc.cellVoltage(), lc.cellPercent() );
    } else {
        tft.setTextColor(ST77XX_RED);
        snprintf(tft_lin_buf, sizeof(tft_lin_buf), "LC709203F is NOT OK");
    }
    tft.println( tft_lin_buf );
}


// ---------------------- Button Callbacks & Handlers  ------------------------
void onLeftPressShortRelease(void)
{
    lastActionMillis = millis();

    debug_pln("Short Left Press (TFT Backlight ON)");
    digitalWrite(NEOPIXEL_POWER, 1);
    delay(20);
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(10, 0, 10));
    pixel.show();

    digitalWrite(TFT_BACKLITE, HIGH);

    updateGearStateDisplay();
    updateBatteryDisplay();

    //lineLocator(2, 13);
    //lineLocator(2, 14, 1);
    //lineLocator(2, 15, 3);
    //
    //lineLocator(3, 13, 5);
    //lineLocator(3, 14, 7);
    //lineLocator(3, 15, 9);

}


void onLeftPressLongStart(void)
{
    lastActionMillis = millis();

    debug_pln("Long Left Press (TFT Backlight + Pixel OFF)");

    touchUpdateHoldoff = true;

    digitalWrite(NEOPIXEL_POWER, 0);

    clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
    for (int i=2; i; i--){
        tft.setTextColor(ST77XX_RED);
        tft.print("BL OFF in " + String(i));
        //clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
        delay(1000);
        clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
    }

    digitalWrite(TFT_BACKLITE, LOW);

    //pixel.clear();
    //pixel.setPixelColor(0, pixel.Color(150, 0, 0));
    //pixel.show();

    //tft.fillScreen(ST77XX_BLACK);
}

void onRightPressShortRelease(void)
{
    debug_pln("Short Right Press (nothing)");

    flash_pixel(NeoPurple, 3);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=250,uint32_t color1=NeoBlack)

    touchUpdateHoldoff = false;

}


void onRightPressLongRelease(void)
{
    lastActionMillis = millis();
    pixel.clear();
    pixel.show();
    digitalWrite(NEOPIXEL_POWER, 0);

    debug_pln("Long Right Press (Possible reboot...)");

    if (buttonLeft.isPressed()) {
        debug_pln("Long Right Press w/ Left (Initiating Reboot Sequence)");
        for (int i=4; i; i--){
            clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
            tft.setTextColor(ST77XX_RED);
            tft.print("REBOOT in " + String(i));
            delay(1000);
        }
        ESP.restart();
    }
}


void onSwitchForward(void)
{
    lastActionMillis = millis();
    debug_pln("onSwitchForward!");
    motorSwitch = FORWARD;
    clearSizedTextLine(DISP_ADDR_LINE_SIZE, DISP_ADDR_LINE_PIXEL);
    updateGearStateDisplay();
}


void onSwitchForwardRelease(void)
{
    lastActionMillis = millis();
    debug_pln("onSwitchForwardRelease!");
    motorSwitch = NEUTRAL;
    updateGearStateDisplay();
}


void onSwitchReverse(void)
{
    lastActionMillis = millis();
    debug_pln("onSwitchReverse!");
    motorSwitch = REVERSE;
    clearSizedTextLine(DISP_ADDR_LINE_SIZE, DISP_ADDR_LINE_PIXEL);
    updateGearStateDisplay();
}


void onSwitchReverseRelease(void)
{
    lastActionMillis = millis();
    debug_pln("onSwitchReverseRelease!");
    motorSwitch = NEUTRAL;
    updateGearStateDisplay();
}


void bmi160_intr(void)
{
    debug_pln("BMI160 interrupt: MOTION?");
    motionWasTriggered = true;
    // unblock motion interrupts after 30 sec
    //nextMotionUnblockMillis = millis() + AFTER_MOTION_IGNORE_MS;
    //BMI160.setIntMotionEnabled(false);
}


float convertRawGyro(int gRaw) {
    // since we are using 250 degrees/seconds range
    // -250 maps to a raw value of -32768
    // +250 maps to a raw value of 32767

    float g = (gRaw * 250.0) / 32768.0;

    return g;
}


void check_IMU(void)
{
    int gxRaw, gyRaw, gzRaw;         // raw gyro values
    float gx, gy, gz;

    // read raw gyro measurements from device
    BMI160.readGyro(gxRaw, gyRaw, gzRaw);

    // convert the raw gyro data to degrees/second
    gx = convertRawGyro(gxRaw);
    gy = convertRawGyro(gyRaw);
    gz = convertRawGyro(gzRaw);

    // display tab-separated gyro x/y/z values
    debug_p("g:\t");
    debug_p(gx);
    debug_p("\t");
    debug_p(gy);
    debug_p("\t");
    debug_p(gz);
    debug_pln();

}


bool sendYakMessage()
{
    yakMessage_t sendableMessage = yakMessage;

    return (simpleEspConnection.sendMessage((uint8_t * ) & sendableMessage, sizeof(sendableMessage)));
}

void dump_pixelAction(char *prefix, PixelFlashEvent_t pe)
{
    char buf[strlen(prefix) + 140];
    uint32_t current_millis = millis();

    // "color0-0x        , color1=0x        , dur=       , flashes=  , cur_col=ox        , current_millis=        ,  nextChangeMillis=        ",
    snprintf(buf, sizeof(buf), "%s color0-0x%lX, color1=0x%lX, dur=%d, flashes=%d, cur_col=%d, current_millis=%ld,  nextChangeMillis=0x%ld",
              prefix, pe.color0, pe.color1, pe.durationMillis, pe.flashes,
              pe.current_color, current_millis, pe.nextChangeMillis);
    debug_pln(buf);
}

//void flash_pixel(uint32_t color0, uint8_t flashes, uint32_t duration=250, uint32_t color1=NeoBlack)
void flash_pixel(uint32_t color0, uint8_t flashes, uint32_t duration, uint32_t color1)
{
    /* reference only
        typedef struct {
            uint32_t color0;
            uint32_t color1;
            uint32_t durationMillis;
            uint8_t flashes;
            uint8_t current_color;    // always 0 (color0) or 1 (color1)
            uint32_t nextChangeMillis;
        } PixelFlashEvent_t;
        std::forward_list<PixelFlashEvent_t> PixelFlashEvents {};
    */

    //PixelFlashEvents.push_front({color0, color1, duration, flashes, NeoBlack, 1});
    pixelAction = {color0, color1, duration, flashes, NeoBlack, PIXEL_TRIG_NOW};
    //dump_pixelAction("@instant: ", pixelAction);
}

void OnSendError(uint8_t * ad)
{
  debug_pln("SENDING TO '" + simpleEspConnection.macToStr(ad) + "' WAS NOT POSSIBLE!");

}


void OnMessage(uint8_t * ad, const uint8_t * message, size_t len)
{
    //debug_pln("Got Something!");
    char buf[100];
    snprintf(buf, sizeof(buf), "Got Something! MESSAGE:[%d]%s from %s\n", len, (char * ) message, simpleEspConnection.macToStr(ad).c_str());
    debug_pln(String(buf));

    if ((char) message[0] == '#') { // however you distinguish....
        //struct_message myData;
        //
        //memcpy( & myData, message, len);
        //Serial.printf("Structure:\n");
        //Serial.printf("a:%s\n", myData.a);
        //Serial.printf("b:%d\n", myData.b);
        //Serial.printf("c:%f\n", myData.c);
        //Serial.printf("e:%s\n", myData.e ? "true" : "false");
    } else {
        //Serial.printf("MESSAGE:[%d]%s from %s\n", len, (char * ) message, simpleEspConnection.macToStr(ad).c_str());
    }
}


void OnNewGatewayAddress(uint8_t * ga, String ad)
{
    debug_pln("New GatewayAddress '" + ad + "'");
    serverAddress = ad;

    simpleEspConnection.setServerMac(ga);
}

void OnConnected(uint8_t *ga, String ad)
{
    debug_pln("OnConnected function called");
    // if(newTimeout != "") {
    //   simpleEspConnection.sendMessage((char *)(String("timeout:"+newTimeout).c_str()), ad );
    //   newTimeout = "";
    // }
}


// ---------------------- simpleEspConnection (comms initialization) ------------------------
void connectionStart(void)
{
    bool simpleConncectIsOK = simpleEspConnection.begin();   // true == OK, false == error

    if (! simpleConncectIsOK) {
        debug_pln("ERROR - simpleEspConnection begin FAILED");
    }
    //  simpleEspConnection.setPairingBlinkPort(2);
    serverAddress = SERVER_ADDRESS ; // Address discovered by manual pairing
    simpleEspConnection.setServerMac(serverAddress);
    simpleEspConnection.onNewGatewayAddress( & OnNewGatewayAddress);
    simpleEspConnection.onSendError( & OnSendError);
    simpleEspConnection.onMessage( & OnMessage);
    simpleEspConnection.onConnected( & OnConnected);


    wifiIsUp = true;
}

/*
 * Stuff that I can do :
 * WiFi. members:
 *     static wifi_mode_t getMode();
 *     bool setSleep(bool enabled);
 *     bool setSleep(wifi_ps_type_t sleepType);
 *     wifi_ps_type_t getSleep();
 *     bool setTxPower(wifi_power_t power);
 *     wifi_power_t getTxPower();
 *
 *      See /Users/andrewward/Library/Arduino15/packages/esp32/hardware/esp32/2.0.5/libraries/WiFi/examples/WiFiScan/WiFiScan.ino
 *
 *     Getting rssi:  (CANNOT FIND ANY MATCHES)
 *     You have to call the ESP API directly. E.g. see here for the esp8266 API: https://www.espressif.com/sites/default/files/documentation/2c-esp8266_non_os_sdk_api_reference_en.pdf
 *     Functions: wifi_station_get_ap_info and wifi_station_get_current_ap_id should work
 *
 */

void connectionSleep()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
}


void connectionWake()
{
    connectionStart();
}

/*

Wifi shutdown, restart ideas:

esp_now_deinit();
WiFi.disconnect();
WiFi.mode(WIFI_OFF);
WiFi.forceSleepBegin();


void enableWiFi(){
    adc_power_on();
    WiFi.disconnect(false);  // Reconnect the network
    WiFi.mode(WIFI_STA);    // Switch WiFi off

    Serial2.println("START WIFI");
    WiFi.begin(STA_SSID, STA_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial2.print(".");
    }

    Serial2.println("");
    Serial2.println("WiFi connected");
    Serial2.println("IP address: ");
    Serial2.println(WiFi.localIP());
}

*/







void setup() {
    Serial.begin(115200);
    debug_pln("\n");
# if !defined(STOP_ALL_SERIAL_IO)
    delay(1500);  // 400 required for ESP8266 "D1 Mini Pro"
#endif
    debug_p("\nClient Setup...Client address: ");

    // ---------------------- TFT Screen Prep ------------------------
    // turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);

    // initialize TFT
    tft.init(135, 240); // Init ST7789 240x135
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(1);

    snprintf(tft_lin_buf, sizeof(tft_lin_buf), "Built %s @ %s", __DATE__, __TIME__);
    tft.println( tft_lin_buf );

    if (!lc.begin()) {
        tft.setTextColor(ST77XX_RED);
        snprintf(tft_lin_buf, sizeof(tft_lin_buf), "NOT FINDING LC709203F! Batt unplugged??");
        tft.println( tft_lin_buf );
        debug_pln(F("Couldn't find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    } else {
        debug_pln(F("Found LC709203F"));
        debug_p("Version: 0x"); debug_pln(lc.getICversion(), HEX);

        lc.setThermistorB(3950);
        debug_p("Thermistor B = "); debug_pln(lc.getThermistorB());

        //   LC709203F_APA_500MAH = 0x10,
        //   LC709203F_APA_1000MAH = 0x19,
        // Our battery is 850 mA.. So trying 0x13  (slightly less than intuitive)
        lc.setPackSize((lc709203_adjustment_t)0x13);

        lc.setAlarmVoltage(3.8);  // Not really needed (or used), since the alarm pin out is NC

        tft.setTextColor(ST77XX_CYAN);
        snprintf(tft_lin_buf, sizeof(tft_lin_buf), "LC709203F: @Boot v:%.2f, :%.0f%%", lc.cellVoltage(), lc.cellPercent() );
        tft.println( tft_lin_buf );

        LC709203FisOK = true;
    }

    // ---------------------- BMI160 IMU (Accelerometer/Gyro) Initialization ------------------------
    debug_pln("Initializing IMU device...");
    //BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
    BMI160.begin(BMI160GenClass::I2C_MODE, I2C_ADDR_ACCEL_BMI160, ACCEL_INT_1_PIN);
    uint8_t dev_id = BMI160.getDeviceID();
    debug_p("IMU DEVICE ID: ");
    debug_pln(dev_id, HEX);

    BMI160.attachInterrupt(bmi160_intr);
    //BMI160.setIntTapEnabled(true);

     // Set the accelerometer range to 250 degrees/second
    BMI160.setGyroRange(250);

    debug_pln("IMU: setIntMotionEnabled...");
    BMI160.setIntMotionEnabled(true);

    // FullScaleGyroRange default:  3 = +/-  250 degrees/sec
    debug_p("IMU: getFullScaleGyroRange:");
    debug_p(BMI160.getFullScaleGyroRange());
    debug_p(", IntMotionEnabled:");
    debug_p(BMI160.getIntMotionEnabled());
    debug_p(", Orig MotionDetectionThreshold:");
    debug_p(BMI160.getMotionDetectionThreshold());
    debug_pln();

    BMI160.setMotionDetectionThreshold(150.0);

    debug_p("NEW MotionDetectionThreshold:");
    debug_p(BMI160.getMotionDetectionThreshold());
    // debug_p(", FullScaleAccelRange: ");
    // debug_p(BMI160.getFullScaleAccelRange());
    // debug_p(", [MotionDetectionDuration: ");
    // debug_p(BMI160.getMotionDetectionDuration());
    // debug_p("]");
    debug_pln();
    debug_pln("Initializing IMU device...done.");

    // ---------------------- simpleEspConnection (comms initialization) ------------------------
    uint32_t wiFiUpMillis=0;
    uint32_t wiFiStartMillis = millis();
    connectionStart();
    wiFiUpMillis=millis();
    debug_pln("Wifi startup time: " + String(wiFiUpMillis - wiFiStartMillis) + " ms");

    //debug_pln("I'm the client! My MAC address is " + WiFi.macAddress());
    clientAddress = WiFi.macAddress();

    // Remove ':' from address string
    String tmpClientAddr="";
    int x=0, wmaLen=clientAddress.length();
    while(x < wmaLen){
        if (clientAddress.charAt(x) == ':') {
            x += 1;
            continue;
        }
        tmpClientAddr += clientAddress.charAt(x++);
    }
    debug_pln("My MAC address is " + tmpClientAddr);

    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.println("I'm " + tmpClientAddr);
    // --- end of simpleEspConnection ----

    buttonRight.begin();
    buttonLeft.begin();
    switchForward.begin();
    switchReverse.begin();

    buttonLeft.onPressed(onLeftPressShortRelease);
    buttonLeft.onPressedFor(2000, onLeftPressLongStart);
    buttonRight.onPressed(onRightPressShortRelease);
    buttonRight.onPressedFor(3000, onRightPressLongRelease);
    switchForward.onPressedFor(200, onSwitchForward);
    switchReverse.onPressedFor(200, onSwitchReverse);

    // ---------------------- set up general IO  (Or Analog Stuff?)   ------------------------
    avgSpeed.begin();
    avgRightTouch.begin();



    updateGearStateDisplay();

    // THIS will suck power.. Find a way to turn off!
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, 1);
    pixel.begin();
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(0, 0, 150));
    pixel.show();
}

void restoreAllNormalDisplay(void)
{
    digitalWrite(TFT_BACKLITE, HIGH);
    updateGearStateDisplay();
    updateTouchDisplay();
    updateBatteryDisplay();

}

void task30s(void) {
    updateBatteryDisplay();
}

void task250ms(void)
{

}

void task128ms(void)
{
    if (yakMessageUpdated) {
        yakMessage.msgID = ++msgIDcounter;
        sendYakMessage();
        yakMessageUpdated = false;
    }

    motorSpeedPotVal = avgSpeed.getAvg();
    float diffVal =     motorSpeedPotVal-lastMotorSpeedPotVal;
    float percentChange = (diffVal/lastMotorSpeedPotVal) * 100.0;
    //debug_p("motorSpeedPotVal="+String(motorSpeedPotVal)+ "   lastMotorSpeedPotVal="+ String(lastMotorSpeedPotVal));
    //debug_p("  diffVal="+ String(diffVal) + ", change(%):" + String(percentChange));
    if ( abs(percentChange) > 2.0 ) {
        //Serial.println(" Update");
        lastActionMillis = millis();
        lastMotorSpeedPotVal = motorSpeedPotVal;
        if (motorSpeedPotVal > maxPotAdcVal ) {
            maxPotAdcVal = motorSpeedPotVal;
        }
        motorSpeedPercent = map(motorSpeedPotVal, 0, maxPotAdcVal, 0, 100);
        updateGearStateDisplay();

        yakMessage.speed = motorSpeedPercent;
        yakMessage.action = MOTOR_CONTROL;
        yakMessageUpdated = true;

    } else {
        //Serial.println("");
    }

    if (motorSwitch != lastMotorSwitch) {
        yakMessage.gear = motorSwitch;
        yakMessage.action = MOTOR_CONTROL;
        yakMessageUpdated = true;

        lastMotorSwitch = motorSwitch;
    }

    // Touch Debug/Adjust: Serial.println("Touch Raw: " + String(touchRead(BUTTON_RIGHT_TOUCH_PIN)));
    avgRightTouchVal = avgRightTouch.getAvg();
    // Touch Debug/Adjust: Serial.println("TOUCH AVG: " + String(avgRightTouchVal) );

    long percentIncr = (lastAvgRightTouchVal *  110)/100;  // integer math: val + 10%
    long percentDecr = (lastAvgRightTouchVal *  90)/100;   // integer math: val - 10%
    // Touch Debug/Adjust: Serial.println("percentIncr="+String(percentIncr)+" of "+String(lastAvgRightTouchVal));
    // Touch Debug/Adjust: Serial.println("percentDecr="+String(percentDecr)+" of "+String(lastAvgRightTouchVal));
    // Touch Debug/Adjust: Serial.println("Avg now " + String(avgRightTouchVal) );

    if ((avgRightTouchVal < percentDecr) ||
        (avgRightTouchVal > percentIncr)) {
         lastAvgRightTouchVal = avgRightTouchVal;
        updateTouchDisplay();
    }

    //check_IMU();
    if (nextMotionUnblockMillis == 1) {
        // WHAT ?? Serial.println("")

    } else if ( (nextMotionUnblockMillis != 0)  && (nextMotionUnblockMillis < millis())) {
        debug_pln("ALL DONE Blocking Motion Interrupts!");
        nextMotionUnblockMillis = 1;
        BMI160.setIntMotionEnabled(true);
    }

    // doing this here so we are not doing stuff during an interrupt
    if (motionWasTriggered) {
        debug_p("MOTION! one-shot! (Update display, Set up for dsp OFF: ");
        debug_p(MOTION_DISPLAY_MS);
        debug_p("ms  and int back en after  ");
        debug_pln(AFTER_MOTION_IGNORE_MS);
        debug_p("ms");
        motionDisplayNextOffMillis = millis() + MOTION_DISPLAY_MS;
        nextMotionUnblockMillis = millis() + AFTER_MOTION_IGNORE_MS;
        motionWasTriggered = false;
        // NOW The Action part.. DO STUFF
        restoreAllNormalDisplay();
    }

    if ((motionDisplayNextOffMillis != 0) && motionDisplayNextOffMillis < millis()) {
        debug_pln("Turn off Temporary display");
        motionDisplayNextOffMillis = 0;
        updateTouchDisplay();
    }

}

//void task50ms(void)
//{
//    /* reference only
//        typedef struct {
//            uint32_t color0;
//            uint32_t color1;
//            uint32_t durationMillis;
//            uint8_t flashes;
//            uint8_t current_color;    // always 0 (color0) or 1 (color1)
//            uint32_t nextChangeMillis;
//        } PixelFlashEvent_t;
//        std::forward_list<PixelFlashEvent_t> PixelFlashEvents {};
//
//    */
//
//    uint32_t current_millis = millis();
//
//    for (const auto &PixelFlashEvent : PixelFlashEvents) {
//        if (PixelFlashEvent.nextChangeMillis <= current_millis  ) {
//
//            if (PixelFlashEvent.current_color == 0) {
//                pixel.setPixelColor(0, PixelFlashEvent.color0);
//                pixel.show();
//                PixelFlashEvent.nextChangeMillis = millis() + (PixelFlashEvent.durationMillis / 2);
//                PixelFlashEvent.current_color += 1;
//
//            } else if (PixelFlashEvent.current_color == 1) {
//                pixel.setPixelColor(0, PixelFlashEvent.color1);
//                pixel.show();
//                PixelFlashEvent.nextChangeMillis = millis() + (PixelFlashEvent.durationMillis / 2);
//                PixelFlashEvent.current_color += 1;
//                // PixelFlashEvent.flashes = PixelFlashEvent.flashes - 1;
//
//            } else if ((PixelFlashEvent.current_color >= 1) {
//                PixelFlashEvent.flashes += 1;
//            }
//        }
//    }
//    PixelFlashEvents.remove_if([current_millis](const PixelFlashEvent& PixelFlashEvent) {
//        return (PixelFlashEvent.nextChangeMillis <=current_millis && PixelFlashEvent.current_color && PixelFlashEvent.flashes <= 0);
//    });
//}


//void task50ms(void)
//{
//    /* reference only
//        typedef struct {
//            uint32_t color0;
//            uint32_t color1;
//            uint32_t durationMillis;
//            uint8_t flashes;
//            uint8_t current_color;    // always 0 (color0) or 1 (color1)
//            uint32_t nextChangeMillis;
//        } PixelFlashEvent_t;
//        std::forward_list<PixelFlashEvent_t> PixelFlashEvents {};
//
//          PixelFlashEvents.push_front({color0, color1, duration, flashes, NeoBlack, 1});
//    */
//
//    uint32_t PixelFlashEventsSize=distance(PixelFlashEvents.begin(), PixelFlashEvents.end());
//    uint32_t current_millis = millis();
//    char buf[100];
//    snprintf(buf, sizeof(buf), "BEFORE For, PixelFlashEventsSize=%d", PixelFlashEventsSize );
//    debug_pln(String(buf));
//
//
//
//    //
//    //
//    //for (const auto &PixelFlashEvent : PixelFlashEvents) {
//    //    snprintf(buf, sizeof(buf), "Begin For: color0-0x%lX, color1=0x%lX, dur=%d, flashes=%d, cur_col=%d, current_millis=%ld,  nextChangeMillis=0x%ld",
//    //              PixelFlashEvent.color0, PixelFlashEvent.color1, PixelFlashEvent.durationMillis, PixelFlashEvent.flashes,
//    //              PixelFlashEvent.current_color, current_millis, PixelFlashEvent.nextChangeMillis);
//    //    debug_pln(String(buf));
//    //
//    //    if (PixelFlashEvent.nextChangeMillis <= current_millis  ) {
//    //
//    //        if (PixelFlashEvent.current_color == 0) {
//    //            debug_pln("In Color 0");
//    //            pixel.setPixelColor(0, PixelFlashEvent.color0);
//    //            pixel.show();
//    //            PixelFlashEvents.push_front({PixelFlashEvent.color0, PixelFlashEvent.color1, PixelFlashEvent.durationMillis,
//    //                                         PixelFlashEvent.flashes, 1, millis() + (PixelFlashEvent.durationMillis / 2)});
//    //
//    //        } else if (PixelFlashEvent.current_color == 1) {
//    //            debug_pln("In Color 1");
//    //            pixel.setPixelColor(0, PixelFlashEvent.color1);
//    //            pixel.show();
//    //            uint8_t flashCntr = PixelFlashEvent.flashes - 1;
//    //            uint8_t nextColor;
//    //            if (flashCntr == 0) {
//    //                nextColor = PixelFlashEvent.current_color + 1;
//    //            } else {
//    //                nextColor = 0;
//    //            }
//    //            PixelFlashEvents.push_front({PixelFlashEvent.color0, PixelFlashEvent.color1, PixelFlashEvent.durationMillis,
//    //                                         flashCntr, nextColor, millis() + (PixelFlashEvent.durationMillis / 2)});
//    //
//    //        } else if (PixelFlashEvent.current_color > 1) {
//    //            debug_pln("In Color >1");
//    //            //PixelFlashEvent.flashes += 1;
//    //        }
//    //    }
//    //}
//    //
//    //PixelFlashEventsSize = distance(PixelFlashEvents.begin(), PixelFlashEvents.end());
//    //snprintf(buf, sizeof(buf), "After For, PixelFlashEventsSize=%d", PixelFlashEventsSize);
//    //debug_pln(String(buf));
//    //
//    //PixelFlashEvents.remove_if([current_millis](const PixelFlashEvent& PixelFlashEvent) {
//    //    return (PixelFlashEvent.nextChangeMillis <=current_millis && PixelFlashEvent.flashes <= 0);
//    //});
//}

void task50ms(void)
{
    /* reference only
        typedef struct {
            uint32_t color0;
            uint32_t color1;
            uint32_t durationMillis;
            uint8_t flashes;
            uint8_t current_color;    // always 0 (color0) or 1 (color1)
            uint32_t nextChangeMillis;
        } PixelFlashEvent_t;

    */
    //static PixelFlashEvent_t last_pe={NeoBlack, NeoBlack, 0, 0, NeoBlack, PIXEL_TRIG_NOW};
    //static bool didAction=false;
    uint32_t current_millis = millis();

    //if (memcmp((const void *) &last_pe, (const void *) &pixelAction, sizeof(PixelFlashEvent_t)) != 0) {
    //    dump_pixelAction("task50ms new pe: ", pixelAction);
    //    last_pe = pixelAction;
    //}

    if (pixelAction.nextChangeMillis > 0 && pixelAction.nextChangeMillis <= current_millis  ) {
        if (pixelAction.current_color == 0) {
            digitalWrite(NEOPIXEL_POWER, 1);
            pixel.setPixelColor(0, pixelAction.color0);
            pixel.show();
            //debug_p("Sent color0 (");
            //debug_p(pixelAction.color0);
            //debug_pln(") to pixel");
            pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
            pixelAction.current_color = 1;

        } else if (pixelAction.current_color == 1) {
            digitalWrite(NEOPIXEL_POWER, 1);
            pixel.setPixelColor(0, pixelAction.color1);
            pixel.show();
            //debug_p("Sent color1 (");
            //debug_p(pixelAction.color1);
            //debug_pln(") to pixel");
            pixelAction.nextChangeMillis = millis() + (pixelAction.durationMillis / 2);
            pixelAction.current_color = 0 ;
            // pixelAction.flashes = pixelAction.flashes - 1;
            pixelAction.flashes -= 1;

        }
    }
    if (pixelAction.flashes == 0) {
        pixelAction.nextChangeMillis = 0;
        digitalWrite(NEOPIXEL_POWER, 0);

    }
}



void loop() {
    uint32_t sysTick = millis();

    if (motionWasTriggered) {
        BMI160.setIntMotionEnabled(false);
    }
    // ----- simpleEspConnection Handling ------
    // needed to manage the communication in the background!
    simpleEspConnection.loop();

    // ----- Button / user interaction Handling ------
    buttonRight.read();
    buttonLeft.read();
    switchForward.read();
    switchReverse.read();
    if (switchForward.wasReleased()) {
        onSwitchForwardRelease();
    } else if (switchReverse.wasReleased()) {
        onSwitchReverseRelease();
    }

    avgSpeed.reading(analogRead(SPEED_CONTROL_POT_PIN));
    avgRightTouch.reading(touchRead(BUTTON_RIGHT_TOUCH_PIN));

    if(sysTick - tick50 > 50){
      tick50 = sysTick;
      task50ms();
    }
    if(sysTick - tick128 > 128){
      tick128 = sysTick;
      task128ms();
    }

    if(sysTick - tick30000 > 30000){
      tick30000 = sysTick;
      task30s();
    }

    // ----- Serial Commands Handling  (should go away?) ------
    //while (Serial.available()) {
    //    char inChar = (char) Serial.read();
    //    if (inChar == '\n') {
    //        debug_pln(inputString);
    //
    //        if (inputString == "startpair") {
    //            simpleEspConnection.startPairing(30);
    //        } else if (inputString == "endpair") {
    //            simpleEspConnection.endPairing();
    //        } else if (inputString == "changepairingmac") {
    //            uint8_t np[] {0xCE, 0x50, 0xE3, 0x15, 0xB7, 0x33};
    //
    //            simpleEspConnection.setPairingMac(np);
    //        } else if (inputString == "textsend") {
    //            if (!simpleEspConnection.sendMessage("This comes from the Client")) {
    //                debug_pln("SENDING TO '" + serverAddress + "' WAS NOT POSSIBLE!");
    //            }
    //        } else if (inputString == "structsend") {
    //            if (!sendStructMessage()) {
    //                debug_pln("SENDING TO '" + serverAddress + "' WAS NOT POSSIBLE!");
    //            }
    //        } else if (inputString == "bigsend") {
    //            if (!sendBigMessage()) {
    //                debug_pln("SENDING TO '" + serverAddress + "' WAS NOT POSSIBLE!");
    //            }
    //        }
    //
    //        inputString = "";
    //    } else {
    //        inputString += inChar;
    //    }
    //}
}