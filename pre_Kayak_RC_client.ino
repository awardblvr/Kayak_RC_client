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
#include <vector>
#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg


//ESP32-S2-TFT   7C:DF:A1:95:0C:6E

String inputString;
String serverAddress;
String clientAddress;  // This is me
uint8_t motorSwitch=0;   // Motor switch state: 0=Neutral, 1=Forward, 2=Reverse, all else invalid, (like 0)
std::vector<String> gear{ "NEUTRAL", "FORWARD", "REVERSE" };
char tft_lin_buf[50];
uint16_t motorSpeedPotVal=0;
uint16_t lastMotorSpeedPotVal=1;
uint8_t motorSpeedPercent=0;
//static void task16ms();
//static void task128ms();
static uint32_t tick16=0, tick128=0, tick30000=0;
uint16_t maxPotAdcVal=5000;  // used in scaling, adjusts higher if new value is higher
uint32_t lastActionMillis=0;
int avgRightTouchVal=0, lastAvgRightTouchVal=0;

#define BUTTON_LEFT_PIN 6
#define BUTTON_RIGHT_PIN 5
#define BUTTON_RIGHT_TOUCH_PIN 14
#define SWITCH_FORWARD_PIN 10
#define SWITCH_REVERSE_PIN 9
#define SPEED_CONTROL_POT_PIN 8

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

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
SimpleEspNowConnection simpleEspConnection(SimpleEspNowRole::CLIENT);
Adafruit_LC709203F lc;
Adafruit_NeoPixel pixel(1 /*NUMPIXELS*/, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
movingAvg avgSpeed(100);                  // define the moving average object
movingAvg avgRightTouch(40);              // define the moving average object

EasyButton buttonRight(BUTTON_RIGHT_PIN);
EasyButton buttonLeft(BUTTON_LEFT_PIN);
EasyButton switchForward(SWITCH_FORWARD_PIN);
EasyButton switchReverse(SWITCH_REVERSE_PIN);

int buttonRightTOUCHval = 0;

// -------------------

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

/*
  size 1:   i=41 for char(0xDA)
  size 2:   i=21 for char(0xDA)
  size 3:   i=14 for char(0xDA)
*/
void clearSizedTextLine(int8_t fontSize, int8_t pixelLine, bool debug=false)
{
    int chars=0;

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
    case 0:
        color = ST77XX_YELLOW;
        break;
    case 1:
        color = ST77XX_GREEN;
        break;
    case 2:
        color = ST77XX_RED;
        break;
    default:
        clearSizedTextLine(DISP_GEAR_SIZE, DISP_GEAR_LINE_PIXEL);
        tft.print("OH SHIT!!");
        return;
        }

    clearSizedTextLine(DISP_GEAR_SIZE, DISP_GEAR_LINE_PIXEL);
    tft.setTextColor(color);
    tft.print(gear[motorSwitch]);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" " + String(motorSpeedPercent));


}

void updateTouchDisplay(void)
{
    uint16_t color = ST77XX_ORANGE;

    clearSizedTextLine(DISP_DEBUG_LINE_SIZE, DISP_DEBUG_LINE_PIXEL);
    tft.setTextColor(color);
    tft.print("Touch:");
    tft.setTextColor(ST77XX_WHITE);
    tft.print(" " + String(avgRightTouchVal));

}

void updateBatteryDisplay(void)
{
    uint16_t color = ST77XX_ORANGE;

    clearSizedTextLine(DISP_BATT_LINE_SIZE, DISP_BATT_LINE_PIXEL);
    tft.setTextColor(ST77XX_GREEN);
    snprintf(tft_lin_buf, sizeof(tft_lin_buf), "B v:%.2f, charge:%.0f\\%", lc.cellVoltage(), lc.cellPercent() );
    tft.println( tft_lin_buf );
}

// ---------------------- Button Callbacks & Handlers  ------------------------
void onLeftPressShortRelease(void)
{
    lastActionMillis = millis();

    digitalWrite(NEOPIXEL_POWER, 1);
    Serial.println("Short Left Press");
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(150, 0, 150));
    pixel.show();

    //lineLocator(2, 13);
    //lineLocator(2, 14, 1);
    //lineLocator(2, 15, 3);
    //
    //lineLocator(3, 13, 5);
    //lineLocator(3, 14, 7);
    //lineLocator(3, 15, 9);


     //  tft.setCursor(0 /*int16_t x0*/, 0 /*int16_t y0*/);
     //  tft.setTextSize(1);
     //  clearTextLine(41);
     //
     //  tft.setTextSize(3);
     //  // tft.setTextColor(ST77XX_RED);
     //  // tft.println(".");
     //  tft.setCursor(0 /*int16_t x0*/, 25 /*int16_t y0*/);
     //  clearTextLine(14, true);
     //  tft.setTextColor(ST77XX_BLUE);
     //  tft.println("SUPER MAN!");
}

void onLeftPressLongStart(void)
{
    lastActionMillis = millis();

    Serial.println("Long Left Press");
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(150, 0, 0));
    pixel.show();


    tft.fillScreen(ST77XX_BLACK);
}

void onRightPressShortRelease(void)
{
    Serial.println("Short Right Press");
    pixel.clear();
    pixel.show();

}

void onRightPressLongRelease(void)
{
    lastActionMillis = millis();
    pixel.clear();
    pixel.show();
    digitalWrite(NEOPIXEL_POWER, 0);

    Serial.println("Long Right Press");

    if (buttonLeft.isPressed()) {
        Serial.println("Initiating Reboot Sequence");
        clearSizedTextLine(DISP_BOTTOM_WARN_SIZE, DISP_BOTTOM_WARN_PIXEL);
        tft.setTextColor(ST77XX_RED);
        tft.print("REBOOT in 4");
        delay(4000);
        ESP.restart();
    }
}

void onSwitchForward(void)
{
    lastActionMillis = millis();
    Serial.println("onSwitchForward!");
    motorSwitch = 1;
    clearSizedTextLine(DISP_ADDR_LINE_SIZE, DISP_ADDR_LINE_PIXEL);
    updateGearStateDisplay();
}

void onSwitchForwardRelease(void)
{
    lastActionMillis = millis();
    Serial.println("onSwitchForwardRelease!");
    motorSwitch = 0;
    updateGearStateDisplay();
}

void onSwitchReverse(void)
{
    lastActionMillis = millis();
    Serial.println("onSwitchReverse!");
    motorSwitch = 2;
    clearSizedTextLine(DISP_ADDR_LINE_SIZE, DISP_ADDR_LINE_PIXEL);
    updateGearStateDisplay();
}

void onSwitchReverseRelease(void)
{
    lastActionMillis = millis();
    Serial.println("onSwitchReverseRelease!");
    motorSwitch = 0;
    updateGearStateDisplay();
}



// Notes:
//    My Adafruit Feather ES:32-S2 TFT  mac address is 7C:DF:A1:95:0C:6E   This is the CLIENT (This code)
//    For Debug, the initial WeMos D1Mini Pro SERVER is AC:0B:FB:DD:41:00

typedef struct struct_message {
  char type;
  char a[32];
  int b;
  float c;
  bool e;
}
struct_message;

bool sendBigMessage() {
  char bigMessage[] = "\n\
There was once a woman who was very, very cheerful, though she had little to make her so; for she was old, and poor, and lonely. She lived in a little bit of a cottage and earned a scant living by running errands for her neighbours, getting a bite here, a sup there, as reward for her services. So she made shift to get on, and always looked as spry and cheery as if she had not a want in the world.\n\
Now one summer evening, as she was trotting, full of smiles as ever, along the high road to her hovel, what should she see but a big black pot lying in the ditch!\n\
\"Goodness me!\" she cried, \"that would be just the very thing for me if I only had something to put in it! But I haven't! Now who could have left it in the ditch?\"\n\
And she looked about her expecting the owner would not be far off; but she could see nobody.\n\
\"Maybe there is a hole in it,\" she went on, \"and that's why it has been cast away. But it would do fine to put a flower in for my window; so I'll just take it home with me.\"\n\
And with that she lifted the lid and looked inside. \"Mercy me!\" she cried, fair amazed. \"If it isn't full of gold pieces. Here's luck!\"\n\
And so it was, brimful of great gold coins. Well, at first she simply stood stock-still, wondering if she was standing on her head or her heels. Then she began saying:\n\
\"Lawks! But I do feel rich. I feel awful rich!\"\n\
";

  return (simpleEspConnection.sendMessage(bigMessage));
}

bool sendStructMessage() {
  struct_message myData;

  myData.type = '#'; // just to mark first byte. It's on you how to distinguish between struct and text message
  sprintf(myData.a, "Greetings from %s", simpleEspConnection.myAddress.c_str());
  myData.b = random(1, 20);
  myData.c = (float) random(1, 100000) / (float) 10000;
  myData.e = true;

  return (simpleEspConnection.sendMessage((uint8_t * ) & myData, sizeof(myData)));
}

void OnSendError(uint8_t * ad) {
  Serial.println("SENDING TO '" + simpleEspConnection.macToStr(ad) + "' WAS NOT POSSIBLE!");
}

void OnMessage(uint8_t * ad,
  const uint8_t * message, size_t len) {
  if ((char) message[0] == '#') // however you distinguish....
  {
    struct_message myData;

    memcpy( & myData, message, len);
    Serial.printf("Structure:\n");
    Serial.printf("a:%s\n", myData.a);
    Serial.printf("b:%d\n", myData.b);
    Serial.printf("c:%f\n", myData.c);
    Serial.printf("e:%s\n", myData.e ? "true" : "false");
  } else
    Serial.printf("MESSAGE:[%d]%s from %s\n", len, (char * ) message, simpleEspConnection.macToStr(ad).c_str());
}

void OnNewGatewayAddress(uint8_t * ga, String ad) {
  Serial.println("New GatewayAddress '" + ad + "'");
  serverAddress = ad;

  simpleEspConnection.setServerMac(ga);
}


//void toScreen(char *textBuf, textSize, textColor, int lineNum=-1 )
//{
//
//
//}

void setup() {
    Serial.begin(115200);
    Serial.println("\n");
    delay(1500);  // 400 required for ESP8266 "D1 Mini Pro"
    Serial.print("\nClient Setup...Client address: ");

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
    tft.println();

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // ---------------------- LiON Battery Monitor (LC709203F) ------------------------
    Serial.println(F("Check for Adafruit LC709203F..."));

    // For the Feather ESP32-S2, we need to enable I2C power first!
    // this section can be deleted for other boards
    // turn on the I2C power by setting pin to opposite of 'rest state'
    pinMode(PIN_I2C_POWER, INPUT);
    delay(1);
    bool polarity = digitalRead(PIN_I2C_POWER);
    pinMode(PIN_I2C_POWER, OUTPUT);
    digitalWrite(PIN_I2C_POWER, !polarity);
#else
    Serial.println(F("\nNOT SET UP - (undefined ARDUINO_ADAFRUIT_FEATHER_ESP32S2)!"));
#endif

    if (!lc.begin()) {
      Serial.println(F("Couldn't find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
      //while (1) delay(10);
    }
    Serial.println(F("Found LC709203F"));
    Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    lc.setThermistorB(3950);
    Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    lc.setPackSize(LC709203F_APA_500MAH);

    lc.setAlarmVoltage(3.8);


    // ---------------------- simpleEspConnection (comms initialization) ------------------------

    simpleEspConnection.begin();
    //  simpleEspConnection.setPairingBlinkPort(2);

    //   serverAddress = "AC0BFBDCE1F1"; // Test if you know the server
    serverAddress = "AC0BFBDD4100"; // Address discovered by manual pairing
    simpleEspConnection.setServerMac(serverAddress);
    simpleEspConnection.onNewGatewayAddress( & OnNewGatewayAddress);
    simpleEspConnection.onSendError( & OnSendError);
    simpleEspConnection.onMessage( & OnMessage);

    Serial.println("I'm the client! My MAC address is " + WiFi.macAddress());
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
    Serial.println("My Cleaned MAC address is " + tmpClientAddr);

    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.println("I'm " + tmpClientAddr);

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
    //avgRightTouch.begin();



    updateGearStateDisplay();

    // THIS will suck power.. Find a way to turn off!
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, 1);
    pixel.begin();
    pixel.clear();
    pixel.setPixelColor(0, pixel.Color(0, 0, 150));
    pixel.show();
}

void task30s(void) {
    updateBatteryDisplay();
}

void task128ms(void)
{
    static float touchPercentChange=0;
    static float lastRightTouch=0;

    motorSpeedPotVal = avgSpeed.getAvg();
    float diffVal =     motorSpeedPotVal-lastMotorSpeedPotVal;
    float percentChange = (diffVal/lastMotorSpeedPotVal) * 100.0;
    //Serial.print("motorSpeedPotVal="+String(motorSpeedPotVal)+ "   lastMotorSpeedPotVal="+ String(lastMotorSpeedPotVal));
    //Serial.print("  diffVal="+ String(diffVal) + ", change(%):" + String(percentChange));
    if ( abs(percentChange) > 2.0 ) {
        //Serial.println(" Update");
        lastActionMillis = millis();
        lastMotorSpeedPotVal = motorSpeedPotVal;
        if (motorSpeedPotVal > maxPotAdcVal ) {
            maxPotAdcVal = motorSpeedPotVal;
        }
        motorSpeedPercent = map(motorSpeedPotVal, 0, maxPotAdcVal, 0, 100);
        updateGearStateDisplay();
    } else {
        //Serial.println("");
    }


    //avgRightTouchVal = avgRightTouch.getAvg();
    //if (avgRightTouchVal != lastAvgRightTouchVal) {
    //    lastAvgRightTouchVal = avgRightTouchVal;
    //    updateTouchDisplay();
    //}

    //Serial.print("Avg=");
    //Serial.print(motorSpeedPotVal);
    //Serial.print("   ");
    //Serial.print(percentChange);
    //Serial.println("% change");
    //Serial.println("Avg="+String(motorSpeedPotVal)+"   "+String(percentChange)+"% change");
    //if ( percentChange > 20 ) {
    //    Serial.println("Refresh!");
    //    lastMotorSpeedPotVal = motorSpeedPotVal;
    //    updateGearStateDisplay();
    //}

}

void loop() {
    uint32_t sysTick = millis();

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
    //avgRightTouch.reading(touchRead(BUTTON_RIGHT_TOUCH_PIN));

    //if(sysTick - tick16 > 16){
    //  tick16 = sysTick;
    //  //task16ms();
    //}
    if(sysTick - tick128 > 128){
      tick128 = sysTick;
      task128ms();
    }

    if(sysTick - tick30000 > 30000){
      tick30000 = sysTick;
      task30s();
    }


    // ----- Serial Commands Handling  (should go away?) ------
    while (Serial.available()) {
        char inChar = (char) Serial.read();
        if (inChar == '\n') {
            Serial.println(inputString);

            if (inputString == "startpair") {
                simpleEspConnection.startPairing(30);
            } else if (inputString == "endpair") {
                simpleEspConnection.endPairing();
            } else if (inputString == "changepairingmac") {
                uint8_t np[] {0xCE, 0x50, 0xE3, 0x15, 0xB7, 0x33};

                simpleEspConnection.setPairingMac(np);
            } else if (inputString == "textsend") {
                if (!simpleEspConnection.sendMessage("This comes from the Client")) {
                    Serial.println("SENDING TO '" + serverAddress + "' WAS NOT POSSIBLE!");
                }
            } else if (inputString == "structsend") {
                if (!sendStructMessage()) {
                    Serial.println("SENDING TO '" + serverAddress + "' WAS NOT POSSIBLE!");
                }
            } else if (inputString == "bigsend") {
                if (!sendBigMessage()) {
                    Serial.println("SENDING TO '" + serverAddress + "' WAS NOT POSSIBLE!");
                }
            }

            inputString = "";
        } else {
            inputString += inChar;
        }
    }
}