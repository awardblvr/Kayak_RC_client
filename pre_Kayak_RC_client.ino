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
#include <EEPROM.h>
#include <SPI.h>
#include <EasyButton.h>
#include "Adafruit_LC709203F.h"
#include <Adafruit_NeoPixel.h>
#include <BMI160Gen.h>
#include <vector>
#include <forward_list>
#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg
//#define STOP_ALL_SERIAL_IO
//#include <rtc_io.h>
//#include <components/driver/include/driver/rtc_io.h>
#include <driver/rtc_io.h>
#include "debug_serial_print.h"
#include "shared_yak_messaging.h"

//#define SKIP_IMU

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
//   serverAddress = "4091512D0DA0"; // Adafruit Feather ESP32   (NOT ESP32 V2)
#define SERVER_ADDRESS "4091512D0DA0"

#define EEPROM_SIZE 1
#define EE_LAST_BOOT_BEHAVIOR_ADDR 0x0
#define EE_LAST_BOOT_BEHAVIOR_OK 0x1
#define EE_LAST_BOOT_BEHAVIOR_CRASH 0x2


/* Wifi power from
    WIFI_POWER_19_5dBm = 78,// 19.5dBm    DEFAULT
    WIFI_POWER_19dBm = 76,// 19dBm
    WIFI_POWER_18_5dBm = 74,// 18.5dBm
    WIFI_POWER_17dBm = 68,// 17dBm
    WIFI_POWER_15dBm = 60,// 15dBm
    WIFI_POWER_13dBm = 52,// 13dBm
    WIFI_POWER_11dBm = 44,// 11dBm
    WIFI_POWER_8_5dBm = 34,// 8.5dBm
    WIFI_POWER_7dBm = 28,// 7dBm
    WIFI_POWER_5dBm = 20,// 5dBm
    WIFI_POWER_2dBm = 8,// 2dBm
    WIFI_POWER_MINUS_1dBm = -4// -1dBm
*/

#define WIFI_XMIT_PWR WIFI_POWER_2dBm


//  Use RTC_DATA_ATT as a prefix to any var below to save in RTC Ram
//  (not lost uin sleep modes, but resets on full power-cycle  or reboot
String inputString;
String serverAddress;
String clientAddress;  // This is me
    String tmpClientAddr="";

//std::vector<String> gear{ "NEUTRAL", "FORWARD", "REVERSE" };   Replaced by GEAR_t_v
char tft_lin_buf[50];
uint16_t motorSpeedPotVal=0;
uint16_t lastMotorSpeedPotVal=1;
uint8_t motorSpeedPercent=0;
static uint32_t tick50=0, tick128=0, tick10000=0, tick30000=0;
uint16_t maxPotAdcVal=5000;  // used in scaling, adjusts higher if new value is higher
uint32_t lastActionMillis=0;
int avgRightTouchVal=1, lastAvgRightTouchVal=1, longTermAvgRightTouch=0;  // touchRead() returns an int
bool statusUpdateHoldoff=false;
bool batteryUpdateHoldoff=false;
bool LC709203FisOK = false;
uint32_t nextMotionUnblockMillis=0;
uint32_t motionDisplayNextOffMillis=0;
bool motionWasTriggered=false;
bool wifiIsUp=false;
bool yakMessageUpdated = false;
uint32_t msgIDcounter=0;
GEAR_t motorSwitch=NEUTRAL;
GEAR_t lastMotorSwitch=motorSwitch;
uint32_t nextWiFiOffMS=0;
bool isBeingTouched=false;
bool lastIsBeingTouched=false;
uint32_t nextActiveTouchStateCheckMillis=0;
bool trigUpdateStatusLineDisplay=false;
bool crashCheckDone=false;

#define DEFAULT_I2C_PORT &Wire

#define BUTTON_LEFT_PIN 6
#define BUTTON_LEFT_PIN 6
#define BUTTON_RIGHT_PIN 5
//#define BUTTON_RIGHT_TOUCH_PIN 14
#define BUTTON_RIGHT_TOUCH_PIN 12
#define SWITCH_FORWARD_PIN 10
#define SWITCH_REVERSE_PIN 9
#define SPEED_CONTROL_POT_PIN 8
#define ACCEL_INT_1_PIN 15
#define BOARD_LED   13

#define I2C_ADDR_ACCEL_BMI160 0x69

#define DISP_ADDR_LINE_PIXEL 21 // 15
#define DISP_ADDR_LINE_SIZE 2

#define DISP_GEAR_LINE_PIXEL 42
#define DISP_GEAR_SIZE 3

#define DISP_STATUS_LINE_PIXEL 68
#define DISP_STATUS_LINE_SIZE 2

#define DISP_TOP_STATUS_LINE_PIXEL 0
#define DISP_TOP_STATUS_LINE_SIZE 2

#define DISP_BOTTOM_WARN_PIXEL 114
#define DISP_BOTTOM_WARN_SIZE 3

#define DISP_BATT_LINE_PIXEL 120
#define DISP_BATT_LINE_SIZE 2

#define INACTIVITY_MS_BEFORE_REDUCE_PWR 60000
#define INACTIVITY_MS_BEFORE_SLEEP 1200000
#define AFTER_MOTION_IGNORE_MS 6000
#define MOTION_DISPLAY_MS 5000
#define WIFI_INACTIVITY_SLEEP 500
#define uS_TO_S_FACTOR 1000000LL  /* Conversion factor for micro seconds to seconds */

#define LC709203F_APA_500MAH = 0x10,
#define LC709203F_APA_850MAH_interpolted 0x13
#define LC709203F_APA_1000MAH = 0x19,
#define LC709203F_APA_1000MAH = 0x19,
#define LC709203F_APA_1500MAH_interpolted 0x20
#define LC709203F_APA_2000MAH = 0x2D,
#define LC709203F_APA LC709203F_APA_1500MAH_interpolted

        // Our battery is 850 mA.. So trying 0x13  (slightly less than intuitive)


uint8_t currentStatusLinePixel=DISP_STATUS_LINE_PIXEL;

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
SimpleEspNowConnection simpleEspConnection(SimpleEspNowRole::CLIENT);
Adafruit_LC709203F lc;
Adafruit_NeoPixel pixel(1 /*NUMPIXELS*/, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
movingAvg avgSpeed(100);                  // define the moving average object
movingAvg avgRightTouch(100);             // define the moving average object
movingAvg avgRightTouchLong(10);          // Long term Touch average

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
#define NeoDimGreen 0x000F00
#define NeoMinGreen 0x000100
#define NeoBlue     0x0000FF
#define NeoYellow   0xFFFF00
#define NeoMagenta  0xFF00FF
#define NeoPurple   0x800080
#define NeoDarkPurple 0x2c0222
#define NeoOrange   0xFF8C00
#define NeoLime     0x00FF00
#define NeoNavyBlue 0x000080
#define NeoGray     0x696969
#define NeoSilver   0xC0C0C0
#define NeoBrown    0x8B4513


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

  #define ST77XX_ORANGE 0xFC00
  #define ST77XX_
  #define ST77XX_
  #define ST77XX_
  #define ST77XX_
  #define ST77XX_
  #define ST77XX_
  #define ST77XX_
  #define ST77XX_DkGray 0x3186

  Details on special chars for display: https://learn.adafruit.com/adafruit-gfx-graphics-library/graphics-primitives
  Line Clearing:
  size 1:   i=41 for char(0xDA)
  size 2:   i=21 for char(0xDA)
  size 3:   i=41 for char(0xDA)
*/
#define ST77XX_DkGray  0x3186
#define ST77XX_MedGray 0x738e
#define ST77XX_DarkRed 0x78c3


void lineLocator(int8_t fontSize, int8_t pixelLine, int8_t insetChars=0)
{
    debug_pln("Locate line at size " + String(fontSize) + "pix Y:" + String(pixelLine) + " offset:" + String(insetChars));
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(fontSize);
    tft.setCursor(0 /*x*/, pixelLine);
    while (insetChars--) {
        tft.print(" ");
    }
    tft.print(char(0xDA)); // 0xDA is the CP437(false) square block
}


//void clearTextLine(int8_t chars, bool debug=false)
//{
//
//    tft.setTextColor(ST77XX_BLACK);
//    while (--chars) {
//        tft.print(char(0xDA)); // 0xDA is the CP437(false) square block
//    }
//    if (debug) {
//        tft.setTextColor(ST77XX_RED);
//        tft.println(".");
//    }
//}


void clearSizedTextLine(int8_t fontSize, int8_t pixelLine, bool plus=false, bool debug=false)
{
    int8_t chars, charsCntr;

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

    charsCntr = chars;
    tft.setTextSize(fontSize);
    if (plus) {
        debug_pln("plus");
        charsCntr -= 1;
        tft.setCursor(2 /*x*/, pixelLine);
        tft.setTextColor(ST77XX_BLACK);
        while (--charsCntr) {
            tft.print(char(0xDA)); // 0xDA is the CP437(false) square block
        }
    }
    charsCntr = chars;
    tft.setCursor(0 /*x*/, pixelLine);
    tft.setTextColor(ST77XX_BLACK);
    while (--charsCntr) {
        tft.print(char(0xDA)); // 0xDA is the CP437(false) square block
    }

    if (debug) {
        debug_pln("debug ASSERTED");

        tft.setTextColor(ST77XX_RED);
        //tft.print(".");
        tft.print(char(0xDA));
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

//void updateTouchDisplay(void)
//{
//    uint16_t color = ST77XX_ORANGE;
//    uint32_t aRTV=0;
//
//    clearSizedTextLine(DISP_STATUS_LINE_SIZE, DISP_STATUS_LINE_PIXEL);
//    if (!touchUpdateHoldoff) {
//        tft.setTextColor(color);
//        tft.print("Touch:");
//        tft.setTextColor(ST77XX_WHITE);
//        aRTV = avgRightTouchVal > 1000 ? avgRightTouchVal = avgRightTouchVal/100 : avgRightTouchVal;
//        tft.print(" " + String(aRTV));
//        if (motionDisplayNextOffMillis != 0) {
//            tft.setTextColor(ST77XX_MAGENTA);
//            tft.print("  MOTION");
//        }
//    } else {
//        debug_pln("touchUpdateHoldoff!  Skipping display update");
//    }
//}

void updateStatusLineDisplay(void)
{
    uint16_t color = ST77XX_ORANGE;
    uint32_t aRTV=0;
    char buf[20];
    char block[]={0xDA,0xDA,0xDA,0xDA,0xDA,0xDA,0x0};
    // 21 Characters  (go for 20)
    // "                    "
    //  Touch: 118  MOTION
    // Need Status: WiFi State
    //              RSSI?
    //              Motion
    //              Touch
    // "T___ MOT WiFi <rssi>"
    // "%c%3d %3s %4s "
    // printf("%-40s", "Test");   Right Justify te string
    if (!statusUpdateHoldoff) {
        clearSizedTextLine(DISP_STATUS_LINE_SIZE, currentStatusLinePixel);
        if (isBeingTouched) {
            tft.setTextColor(ST77XX_ORANGE);
        } else {
            tft.setTextColor(ST77XX_DkGray);
        }
        tft.print("T");

        aRTV = avgRightTouchVal > 1000 ? avgRightTouchVal = avgRightTouchVal/100 : avgRightTouchVal;
        //snprintf(buf, sizeof(buf), "%3.0f", (1.0 *
        snprintf(buf, sizeof(buf), "%-3s", String(aRTV));
        tft.setTextColor(ST77XX_MedGray);
        tft.print(buf);  tft.print("  ");

        if (wifiIsUp) {
            tft.setTextColor(ST77XX_GREEN);
        } else {
            tft.setTextColor(ST77XX_DarkRed);
        }
        tft.print("WiFi  ");

        if (motionDisplayNextOffMillis != 0) {
            tft.setTextColor(ST77XX_MAGENTA);
            tft.print("MOTION");
        } else {
            tft.setTextColor(ST77XX_BLACK);
            //buf={0xDA,0xDA,0xDA,0xDA,0xDA,0xDA,0x0};
            tft.print(block); // 0xDA is the CP437(false) square block
        }
    } else {
        debug_pln("statusUpdateHoldoff!  Skipping display update");
    }
}

void relocate_updateStatusLineDisplay(void)
{
    clearSizedTextLine(DISP_STATUS_LINE_SIZE, currentStatusLinePixel);
    currentStatusLinePixel = DISP_TOP_STATUS_LINE_PIXEL;
    // clearSizedTextLine(int8_t fontSize, int8_t pixelLine, bool plus=false, bool debug=false)
    clearSizedTextLine(DISP_STATUS_LINE_SIZE, currentStatusLinePixel, true);
    updateStatusLineDisplay();
}


void updateBatteryDisplay(void)
{
    uint16_t color = ST77XX_ORANGE;

    if (!batteryUpdateHoldoff) {
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
}

void updateMyIPaddressLine(void)
{
    clearSizedTextLine(DISP_ADDR_LINE_SIZE, DISP_ADDR_LINE_PIXEL);
    tft.setTextColor(ST77XX_YELLOW);
    tft.println( "I'm " + clientAddress);
}

// ---------------------- Button Callbacks & Handlers  ------------------------
void onLeftPressShortRelease(void)
{
    lastActionMillis = millis();

    debug_pln("Short Left Press (TFT Backlight ON)");
    //digitalWrite(NEOPIXEL_POWER, 1);
    //delay(20);
    //pixel.clear();
    //pixel.setPixelColor(0, pixel.Color(10, 0, 10));
    //pixel.show();

    digitalWrite(TFT_BACKLITE, HIGH);

    batteryUpdateHoldoff = false;

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


void onLeftPressLongRelease(void)
{
    lastActionMillis = millis();

    debug_pln("Long Left Press (TFT Backlight + Pixel OFF)");

    if (!buttonRight.isPressed()) {
        batteryUpdateHoldoff = true;
        clearSizedTextLine(DISP_BATT_LINE_SIZE, DISP_BATT_LINE_PIXEL);
        clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
        for (int i=2; i; i--){
            tft.setTextColor(ST77XX_RED);
            tft.print("BL OFF in " + String(i));
            //clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
            delay(1000);
            //clearSizedTextLine(DISP_BATT_LINE_SIZE, DISP_BATT_LINE_PIXEL);
            clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
        }

        digitalWrite(TFT_BACKLITE, LOW);
    }

    //batteryUpdateHoldoff = true;
    //statusUpdateHoldoff = true;


    //pixel.clear();
    //pixel.setPixelColor(0, pixel.Color(150, 0, 0));
    //pixel.show();

    //tft.fillScreen(ST77XX_BLACK);
}

void onRightPressShortRelease(void)
{
    debug_pln("Short Right Press (Locate)");

    lineLocator(DISP_ADDR_LINE_SIZE, DISP_ADDR_LINE_PIXEL, 3);

    //delay(3000);
    updateMyIPaddressLine();

    //delay(3000);
    relocate_updateStatusLineDisplay();
    //updateBatteryDisplay();
    //
    //statusUpdateHoldoff = false;
}


void onRightPressLongRelease(void)
{
    lastActionMillis = millis();
    pixel.clear();
    pixel.show();

    debug_pln("Long Right Press (reboot or WiFi OFF)");
    batteryUpdateHoldoff = true;

    if (buttonLeft.isPressed()) {
        batteryUpdateHoldoff = true;
        clearSizedTextLine(DISP_BATT_LINE_SIZE, DISP_BATT_LINE_PIXEL);
        debug_pln("Long Right Press w/ Left (Initiating Reboot Sequence)");
        for (int i=4; i; i--){
            clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
            tft.setTextColor(ST77XX_RED);
            tft.print("REBOOT in " + String(i));
            delay(1000);
        }
        ESP.restart();
    } else {
        //debug_pln("Long Right Press alone... Inducing Light Sleep");
        //flash_pixel(NeoGreen, 1, 1000, NeoNavyBlue);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=250,uint32_t color1=NeoBlack)
        //flash_pixel(NeoOrange, 1, 500);
        //flash_pixel(NeoRed, 1, 100);
        //
        //batteryUpdateHoldoff = true;
        //clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
        ////tft.setTextColor(ST77XX_RED);
        ////tft.print("WiFi ...OFF");
        ////delay(1000);
        ////nextWiFiOffMS = millis() + WIFI_INACTIVITY_SLEEP;
        //connectionLightSleep();
    }

    batteryUpdateHoldoff = false;
    updateBatteryDisplay();
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

#if !defined(SKIP_IMU)
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
#endif

void sendYakMessage()
{
    yakMessage_t sendableMessage = yakMessage;

    if (!wifiIsUp) {
        debug_pln("sendYakMessage: connectionWake");
        connectionWake();
    }
    // resp is bool: true=OK, false=FAIL
    simpleEspConnection.sendMessage((uint8_t * ) & sendableMessage, sizeof(sendableMessage));
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
    debug_pln("Send to " + simpleEspConnection.macToStr(ad) + " FAILED");
    flash_pixel(NeoRed, 1, 100);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)
}

void OnSendDone(uint8_t * ad)
{
    flash_pixel(NeoMinGreen, 1, 10);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)
}

void OnMessage(uint8_t * ad, const uint8_t * message, size_t len)
{
    //debug_pln("Got Something!");
    char buf[100];
    snprintf(buf, sizeof(buf), "Got Something! MESSAGE:[%d]%s from %s\n", len, (char * ) message, simpleEspConnection.macToStr(ad).c_str());
    debug_pln(String(buf));

    yakCommsMessage_t *yakCommsMessage = (yakCommsMessage_t *) message;

    switch (yakCommsMessage->msgType) {
    case COMMS:
        //yakCommsMessage_t *yakCommsMessage = (yakCommsMessage_t *) message;
        switch(yakCommsMessage->commState) {
            case CONNECTED:
                flash_pixel(NeoNavyBlue, 2, 200);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)
                break;

        default:
            flash_pixel(NeoLime, 1);
            snprintf(buf, sizeof(buf), "Received UNRECOGNIZED COMMS message from %s\n", simpleEspConnection.macToStr(ad).c_str());
            debug_pln(String(buf));

        }
        break;

    default:
        flash_pixel(NeoOrange, 1);
        //snprintf(buf, sizeof(buf), "Received UNKNOWN message from %s\n", (char * ) message, simpleEspConnection.macToStr(ad).c_str());
        snprintf(buf, sizeof(buf), "Received UNKNOWN message from %s\n", simpleEspConnection.macToStr(ad).c_str());
        debug_pln(String(buf));
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
    flash_pixel(NeoGray, 1, 200);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)

    // if(newTimeout != "") {
    //   simpleEspConnection.sendMessage((char *)(String("timeout:"+newTimeout).c_str()), ad );
    //   newTimeout = "";
    // }
}


// ---------------------- simpleEspConnection (comms initialization) ------------------------
void connectionStart(void)
{
    debug_pln("connectionStart() begin");

    bool simpleConncectIsOK = simpleEspConnection.begin();   // true == OK, false == error
    //int default_Wifi_power = WiFi.getTxPower();
    //debug_p("Current Default WiFi power is ");
    //debug_pln(default_Wifi_power, HEX);

    WiFi.setTxPower(WIFI_XMIT_PWR);

    if (! simpleConncectIsOK) {
        debug_pln("ERROR - simpleEspConnection begin FAILED");
        flash_pixel(NeoOrange, 1, 100);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)
    } else {
        flash_pixel(NeoDimGreen, 1, 50);  //  uint32_t color0, uint8_t flashes=1, uint32_t duration=500,uint32_t color1=NeoBlack)
    }
    //  simpleEspConnection.setPairingBlinkPort(2);
    serverAddress = SERVER_ADDRESS ; // Address discovered by manual pairing
    simpleEspConnection.setServerMac(serverAddress);
    simpleEspConnection.onNewGatewayAddress( & OnNewGatewayAddress);
    simpleEspConnection.onSendError( & OnSendError);
    simpleEspConnection.onSendDone( & OnSendDone);
    simpleEspConnection.onMessage( & OnMessage);
    simpleEspConnection.onConnected( & OnConnected);

    //wifiIsUp = true;

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
    wifiIsUp = false;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    //updateStatusLineDisplay();
    trigUpdateStatusLineDisplay = true;
}

void print_wakeup_reason()
{
    uint8_t wakeUpReasons=0;
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason & ESP_SLEEP_WAKEUP_EXT0) {
        debug_pln("Wakeup caused by external signal using RTC_IO");
        wakeUpReasons +=1;
    }
    if (wakeup_reason & ESP_SLEEP_WAKEUP_EXT1) {
        debug_pln("Wakeup caused by external signal using RTC_CNTL");
        wakeUpReasons +=1;
    }
    if (wakeup_reason & ESP_SLEEP_WAKEUP_TIMER) {
        debug_pln("Wakeup caused by timer");
        wakeUpReasons +=1;
    }
    if (wakeup_reason & ESP_SLEEP_WAKEUP_TOUCHPAD) {
        debug_pln("Wakeup caused by touchpad");
        wakeUpReasons +=1;
    }
    if (wakeup_reason & ESP_SLEEP_WAKEUP_ULP) {
        debug_pln("Wakeup caused by ULP program");
        wakeUpReasons +=1;
    }

    if (wakeUpReasons == 0) {
        debug_p("Wakeup was not caused by deep sleep:");
        debug_pln(wakeup_reason, HEX);
    }
}



void connectionLightSleep()
{
    wifiIsUp = false;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    //updateStatusLineDisplay();
    debug_pln("connectionLightSleep: WiFi now OFF");
    debug_p("connectionLightSleep: Button state of BUTTON_RIGHT_PIN is ");
    debug_pln(digitalRead(BUTTON_RIGHT_PIN));

    debug_pln("Now inducing Light Sleep in 2 sec");
    delay(2000);
    debug_p("connectionLightSleep: LAST CHECK of BUTTON_RIGHT_PIN is ");
    debug_pln(digitalRead(BUTTON_RIGHT_PIN));
    delay(100);
    BMI160.detachInterrupt();
    delay(100);


    //rtc_gpio_pullup_en((gpio_num_t) BUTTON_RIGHT_PIN);
    //gpio_wakeup_enable((gpio_num_t) BUTTON_RIGHT_PIN, GPIO_INTR_LOW_LEVEL);
    //esp_sleep_enable_ext0_wakeup((gpio_num_t) BUTTON_RIGHT_PIN, GPIO_INTR_LOW_LEVEL);
    //esp_sleep_enable_gpio_wakeup();

    uint32_t SleepTimeSeconds=5;
    esp_sleep_enable_timer_wakeup(uint64_t(SleepTimeSeconds) * uS_TO_S_FACTOR);
    delay(100);

    //gpio_pullup_en((gpio_num_t) BUTTON_RIGHT_PIN);
    // //rtc_gpio_set_direction_in_sleep((gpio_num_t) BUTTON_RIGHT_PIN, INPUT_PULLUP);
    //gpio_set_direction_in_sleep((gpio_num_t) BUTTON_RIGHT_PIN, INPUT_PULLUP);
    //esp_sleep_enable_ext0_wakeup((gpio_num_t) BUTTON_RIGHT_PIN, 0); //1 = High, 0 = Low
    // //esp_sleep_enable_timer_wakeup(1000000); //1 seconds
    //esp_sleep_enable_timer_wakeup(100000000); //100 seconds

    esp_light_sleep_start();

    delay(200);
    debug_pln("connectionLightSleep: Back from Light Sleep");
    delay(200);
    print_wakeup_reason();

    delay(200);
    // //detachInterrupt((gpio_num_t) BUTTON_RIGHT_PIN);
    rtc_gpio_deinit((gpio_num_t) BUTTON_RIGHT_PIN);

    //buttonRight.begin();
    //buttonRight.onPressed(onRightPressShortRelease);
    //buttonRight.onPressedFor(3000, onRightPressLongRelease);

    debug_pln("HERE.. Maybe put WiFi ON, connection?");

    trigUpdateStatusLineDisplay = true;
}


void connectionWake()
{
    debug_pln("connectionWake()");

    connectionStart();

    //delay(51);       // ToDo: Check if this is needed.. affects responsiveness
    wifiIsUp = true;
    trigUpdateStatusLineDisplay = true;
}


void setup() {
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, HIGH);
    delay(100);
    digitalWrite(BOARD_LED, LOW);

    EEPROM.begin(EEPROM_SIZE);

    uint8_t ee_last_boot_behavior = EEPROM.read(EE_LAST_BOOT_BEHAVIOR_ADDR);
    if (ee_last_boot_behavior == 0xFF) {   // uninitialized eeproms are -1 (0xFFFFFFFF)
        EEPROM.write(EE_LAST_BOOT_BEHAVIOR_ADDR, EE_LAST_BOOT_BEHAVIOR_CRASH);
        EEPROM.commit();
    }
    ee_last_boot_behavior = EEPROM.read(EE_LAST_BOOT_BEHAVIOR_ADDR);

    Serial.begin(115200);
    debug_pln("\n");
# if !defined(STOP_ALL_SERIAL_IO)
    delay(1500);  // 400 required for ESP8266 "D1 Mini Pro"
#endif

    if (ee_last_boot_behavior == EE_LAST_BOOT_BEHAVIOR_CRASH) {
        debug_pln("\nLast boot CRASHED.. DELAYING 20 Sec NOW to allow a better Sketch upload");
        for (int x=0; x<133; x++) {    // 30000/150
            digitalWrite(BOARD_LED, HIGH);
            delay(150);
            digitalWrite(BOARD_LED, LOW);
            delay(150);
        }
    } else {
        EEPROM.write(EE_LAST_BOOT_BEHAVIOR_ADDR, EE_LAST_BOOT_BEHAVIOR_CRASH);
        EEPROM.commit();
        debug_pln("\nSetting EE_LAST_BOOT_BEHAVIOR_CRASH as default. (updates to EE_LAST_BOOT_BEHAVIOR_OK in 30 sec)");
    }

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
        //   LC709203F_APA_850MAH_interpolted 0x13
        //   LC709203F_APA_1000MAH = 0x19,
        //   LC709203F_APA_2000MAH = 0x2D,
        // Our battery was 850 mAH, NOW 1500 mAH.. so a little interpolation
        lc.setPackSize((lc709203_adjustment_t)LC709203F_APA);
        debug_p(F("LC709203F, Using 0x"));
        debug_p(LC709203F_APA, HEX);
        debug_pln(F(" for APA (interpolated for  1500mAH battery)"));

        lc.setAlarmVoltage(3.8);  // Not really needed (or used), since the alarm pin out is NC

        tft.setTextColor(ST77XX_CYAN);
        snprintf(tft_lin_buf, sizeof(tft_lin_buf), "LC709203F: @Boot v:%.2f, :%.0f%%", lc.cellVoltage(), lc.cellPercent() );
        tft.println( tft_lin_buf );

        LC709203FisOK = true;
    }

    // ---------------------- BMI160 IMU (Accelerometer/Gyro) Initialization ------------------------
#if !defined(SKIP_IMU)
    debug_pln("Initializing IMU device...");
    //BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
    BMI160.begin(BMI160GenClass::I2C_MODE, I2C_ADDR_ACCEL_BMI160, ACCEL_INT_1_PIN);
    uint8_t dev_id = BMI160.getDeviceID();
    debug_p("IMU DEVICE ID: ");
    debug_pln(dev_id, HEX);

//   AW_DEBUG out for Sleep Debug!   BMI160.attachInterrupt(bmi160_intr);

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
#else
    debug_pln("SKIPING IMU device");
#endif

    // ---------------------- simpleEspConnection (comms initialization) ------------------------
    uint32_t wiFiUpMillis=0;
    uint32_t wiFiStartMillis = millis();
    connectionStart();
    wiFiUpMillis=millis();
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

    clientAddress = tmpClientAddr;


    debug_pln("Wifi "+String(clientAddress)+", startup time: " + String(wiFiUpMillis - wiFiStartMillis) + " ms " + String(clientAddress));

    //debug_pln("I'm the client! My MAC address is " + WiFi.macAddress());

    nextWiFiOffMS = millis() + WIFI_INACTIVITY_SLEEP;

    //// Remove ':' from address string
    //String tmpClientAddr="";
    //int x=0, wmaLen=clientAddress.length();
    //while(x < wmaLen){
    //    if (clientAddress.charAt(x) == ':') {
    //        x += 1;
    //        continue;
    //    }
    //    tmpClientAddr += clientAddress.charAt(x++);
    //}
    //debug_pln("My MAC address is " + tmpClientAddr);

    // --- end of simpleEspConnection ----

    buttonRight.begin();
    buttonLeft.begin();
    switchForward.begin();
    switchReverse.begin();

    buttonLeft.onPressed(onLeftPressShortRelease);
    buttonLeft.onPressedFor(2000, onLeftPressLongRelease);
    buttonRight.onPressed(onRightPressShortRelease);
    buttonRight.onPressedFor(3000, onRightPressLongRelease);
    switchForward.onPressedFor(200, onSwitchForward);
    switchReverse.onPressedFor(200, onSwitchReverse);

    // ---------------------- set up general IO  (Or Analog Stuff?)   ------------------------
    avgSpeed.begin();
    avgRightTouch.begin();
    avgRightTouchLong.begin();

    // THIS will suck power.. Find a way to turn off!
    pinMode(NEOPIXEL_POWER, OUTPUT);
    //digitalWrite(NEOPIXEL_POWER, 1);
    pixel.begin();
    //pixel.clear();
    //pixel.setPixelColor(0, pixel.Color(0, 0, 150));
    //pixel.show();

    updateGearStateDisplay();
    updateMyIPaddressLine();

    delay(2000);

}

void restoreAllNormalDisplay(void)
{
    debug_pln("restoreAllNormalDisplay()");
    batteryUpdateHoldoff = false;
    statusUpdateHoldoff = false;
    trigUpdateStatusLineDisplay = true;

    digitalWrite(TFT_BACKLITE, HIGH);
    updateGearStateDisplay();
    updateStatusLineDisplay();
    updateBatteryDisplay();
}


void task30s(void) {
    updateBatteryDisplay();

    uint8_t ee_last_boot_behavior = EEPROM.read(EE_LAST_BOOT_BEHAVIOR_ADDR);
    if ( ee_last_boot_behavior == EE_LAST_BOOT_BEHAVIOR_CRASH) {
        EEPROM.write(EE_LAST_BOOT_BEHAVIOR_ADDR, EE_LAST_BOOT_BEHAVIOR_OK);
        EEPROM.commit();
        debug_pln("Updated EEPROM to EE_LAST_BOOT_BEHAVIOR_OK");
    }
}


void task10s(void)
{
    int x;
    updateBatteryDisplay();

    if (!crashCheckDone) {
        crashCheckDone=true;
        uint8_t ee_last_boot_behavior = EEPROM.read(EE_LAST_BOOT_BEHAVIOR_ADDR);
        if (ee_last_boot_behavior == EE_LAST_BOOT_BEHAVIOR_CRASH) {
            EEPROM.write(EE_LAST_BOOT_BEHAVIOR_ADDR, EE_LAST_BOOT_BEHAVIOR_OK);
            EEPROM.commit();
            debug_pln("Updated EEPROM to EE_LAST_BOOT_BEHAVIOR_OK");
        }
    }

    //debug_p("task10s.. Begin: avgRightTouchLong.getCount():");
    //debug_p(avgRightTouchLong.getCount());
    //debug_p(", avgRightTouch.getCount():");
    //debug_pln(avgRightTouch.getCount());

    // Do touch averaging
    if (longTermAvgRightTouch == 0) {
        x = avgRightTouch.getAvg();
        avgRightTouchLong.reading(x);
        longTermAvgRightTouch = avgRightTouchLong.getAvg();
    //}else if (!isBeingTouched) {
    //    x = avgRightTouch.getAvg();
    //    avgRightTouchLong.reading(x);
    //    longTermAvgRightTouch = avgRightTouchLong.getAvg();
    //}
    }else {
        x = avgRightTouch.getAvg();
        avgRightTouchLong.reading(x);
        longTermAvgRightTouch = avgRightTouchLong.getAvg();
    }
    //debug_p("task10s.. END: avgRightTouchLong.getCount():");
    //debug_p(avgRightTouchLong.getCount());
    //debug_p(", avgRightTouch.getCount():");
    //debug_pln(avgRightTouch.getCount());
}


void task250ms(void)
{

}


void task128ms(void)
{
    static bool stillGatheringMsgShown=false;

    if (yakMessageUpdated) {
        yakMessage.msgID = ++msgIDcounter;
        nextWiFiOffMS = millis() + WIFI_INACTIVITY_SLEEP;
        sendYakMessage();
        yakMessageUpdated = false;
    }

    // ------------- Variable Speed (potentiometer/ADC) ---------------
    motorSpeedPotVal = avgSpeed.getAvg();
    float diffVal =     motorSpeedPotVal-lastMotorSpeedPotVal;
    float percentChange = (diffVal/lastMotorSpeedPotVal) * 100.0;
    //debug_p("motorSpeedPotVal="+String(motorSpeedPotVal)+ "   lastMotorSpeedPotVal="+ String(lastMotorSpeedPotVal));
    //debug_p("  diffVal="+ String(diffVal) + ", change(%):" + String(percentChange));
    if ( abs(percentChange) > 3.0 ) {
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
        debug_pln("Speed Control yakMessage");
        yakMessageUpdated = true;

    } else {
        //Serial.println("");
    }

    // ------------- MOTOR SWITCH  ---------------
    if (motorSwitch != lastMotorSwitch) {
        yakMessage.gear = motorSwitch;
        yakMessage.action = MOTOR_CONTROL;
        yakMessageUpdated = true;

        lastMotorSwitch = motorSwitch;
    }

    if  (trigUpdateStatusLineDisplay) {
        updateStatusLineDisplay();
        trigUpdateStatusLineDisplay = false;
    }

    // ------------- TOUCH CONTROL  (RIGHT BUTTON Body)  ---------------
    // Touch Debug/Adjust: Serial.println("Touch Raw: " + String(touchRead(BUTTON_RIGHT_TOUCH_PIN)));

    // must check that it has at least 1 value, else it will CRASH!
    int numLTsamples = avgRightTouchLong.getCount();

    //if (numLTsamples < 2) {
    //    debug_p("Touch.. numLTsamples:");
    //    debug_p(numLTsamples);
    //    debug_p(", getCount():");
    //    debug_pln(avgRightTouchLong.getCount());
    //}

    if (numLTsamples > 0) {
        avgRightTouchVal = avgRightTouch.getAvg();

        int samplesToAvg = numLTsamples < 4 ? numLTsamples: 4;
        if (longTermAvgRightTouch) {
            longTermAvgRightTouch = avgRightTouchLong.getAvg(samplesToAvg);
        }
        int avgRightTouchVal5 = avgRightTouch.getAvg(5);
        // Touch Debug/Adjust: Serial.println("TOUCH AVG: " + String(avgRightTouchVal) );

        long percentIncr = (lastAvgRightTouchVal *  110)/100;  // integer math: val + 10%
        long percentDecr = (lastAvgRightTouchVal *  90)/100;   // integer math: val - 10%
        long percentFive = (longTermAvgRightTouch *  5)/100;   // integer math: Get 5%

        int diff = abs(longTermAvgRightTouch-avgRightTouchVal5);
        //debug_p("5% decide: LT samps:");
        //debug_p(samplesToAvg);
        //debug_p(", PIN:");
        //debug_p(touchRead(BUTTON_RIGHT_TOUCH_PIN));
        //debug_p(", longTermAvgRightTouch:");
        //debug_p(longTermAvgRightTouch);
        ////debug_p(", avgRightTouchVal:");
        ////debug_p(avgRightTouchVal);
        //debug_p(", avgRightTouchVal5:");
        //debug_p(avgRightTouchVal5);
        //debug_p(", diff :");
        //debug_p(diff);
        //debug_p(", percentFive:");
        //debug_p(percentFive);

        if (abs(longTermAvgRightTouch-avgRightTouchVal5) > percentFive)  {
            //debug_p(" --->5% ");
            isBeingTouched = true;
            if (isBeingTouched != lastIsBeingTouched) {
                //debug_p(" TOUCHED");
                lastIsBeingTouched = isBeingTouched;
                trigUpdateStatusLineDisplay = true;
            } else {
                //debug_p(" still TOUCHED");
            }
        } else { // UNDER 5%
            isBeingTouched = false;
            //debug_p(" ---steady ");
            if (isBeingTouched != lastIsBeingTouched) {
                //debug_p(" UNTouch CHANGE");
                lastIsBeingTouched = isBeingTouched;
                trigUpdateStatusLineDisplay = true;
            } else {
                //debug_p(" NOT Touched ");
                isBeingTouched = false;
            }
        }
        //debug_pln("");

        if ((avgRightTouchVal < percentDecr) ||
            (avgRightTouchVal > percentIncr)) {
             lastAvgRightTouchVal = avgRightTouchVal;
            //updateStatusLineDisplay();
            trigUpdateStatusLineDisplay = true;
        }
    } else {
        //if (!stillGatheringMsgShown) {
        //    debug_p("Touch.. Still gathering log term average ");
        //    debug_pln(numLTsamples);
        //    stillGatheringMsgShown = true;
        //}
    }

    // ------------- IMU   (Accelerometer/Gyroscope)  ---------------
    if (nextMotionUnblockMillis == 1) {
        // WHAT ?? Serial.println("")

    } else if ( (nextMotionUnblockMillis != 0)  && (nextMotionUnblockMillis < millis())) {
 #if !defined(SKIP_IMU)
       debug_pln("ALL DONE Blocking Motion Interrupts!");
        nextMotionUnblockMillis = 1;
        BMI160.setIntMotionEnabled(true);
 #endif
    }

    // doing this here so we are not doing stuff during an interrupt
    if (motionWasTriggered) {
        debug_p("MOTION! one-shot! (Update display, Set up for dsp OFF: ");
        debug_p(MOTION_DISPLAY_MS);
        debug_p("ms  and int back en after  ");
        debug_p(AFTER_MOTION_IGNORE_MS);
        debug_pln("ms");

        motionDisplayNextOffMillis = millis() + MOTION_DISPLAY_MS;
        nextMotionUnblockMillis = millis() + AFTER_MOTION_IGNORE_MS;
        motionWasTriggered = false;
        // NOW The Action part.. DO STUFF
        restoreAllNormalDisplay();
    }

    if ((motionDisplayNextOffMillis != 0) && motionDisplayNextOffMillis < millis()) {
        debug_pln("Turn off Temporary 'MOTION' display");
        motionDisplayNextOffMillis = 0;
        //updateStatusLineDisplay();
        trigUpdateStatusLineDisplay = true;
    }

    // ------------- WiFi management ---------------
    if (nextWiFiOffMS > 0 && nextWiFiOffMS < millis() && wifiIsUp) {
        debug_pln("Turning OFF WiFi");
        connectionSleep();
        nextWiFiOffMS = 0;
        //updateStatusLineDisplay();
        trigUpdateStatusLineDisplay = true;
    }
}


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

    // can also pixel.setBrightness(50);  (before .show)

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

#if !defined(SKIP_IMU)
    if (motionWasTriggered) {
        BMI160.setIntMotionEnabled(false);
    }
#endif
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
    avgRightTouch.reading(touchRead(BUTTON_RIGHT_TOUCH_PIN));  // returns an INT

    if(sysTick - tick50 > 50){
      tick50 = sysTick;
      task50ms();
    }
    if(sysTick - tick128 > 128){
      tick128 = sysTick;
      task128ms();
    }

    if(sysTick - tick10000 > 10000){
      tick10000 = sysTick;
      task10s();
    }

    //if(sysTick - tick30000 > 30000){
    //  tick30000 = sysTick;
    //  task30s();
    //}

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