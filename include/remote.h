#include <Arduino.h>
#include <UMS3.h>
#include <joystick_pinout.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_seesaw.h>

#define TFT_GREY 0x5AEB // New colour

#define SS_SWITCH_SELECT 1
#define SS_SWITCH_UP     2
#define SS_SWITCH_LEFT   3
#define SS_SWITCH_DOWN   4
#define SS_SWITCH_RIGHT  5

#define SEESAW_ADDR      0x49

UMS3 ums3;
Adafruit_seesaw ss;
esp_now_peer_info_t peerInfo;

// Example Mac Addresses
// EC:DA:3B:41:A3:FC
// F4:12:FA:40:9A:A4
// 70:04:1D:AD:D5:08

uint8_t broadcastAddress[] = robotMac;
TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

struct RemoteData {
    double rightX;
    double rightY;
    double leftX;
    double leftY;
    bool swch1;
    bool swch2;
    bool rotaryUp;
    bool rotaryDown;
    bool rotaryLeft;
    bool rotaryRight;
    bool rotarySelect;
    int rotaryEncoder;
  };
RemoteData data;

// ControllerMessage structs (matching robot receiver format)
struct JoystickReading {
    float x;
    float y;
};

struct DPadReading {
    bool up;
    bool down;
    bool left;
    bool right;
    bool select;
    int32_t encoderPosition;
};

struct TouchReading {
    int16_t x;
    int16_t y;
    int16_t pressure;
};

struct ControllerMessage {
    unsigned long millis;
    JoystickReading joystick1;
    JoystickReading joystick2;
    DPadReading dPad;
    bool buttonL;
    bool buttonR;
    TouchReading touchPoint;
};
ControllerMessage controllerMessage;

// Battery Variables
float batteryVoltage, batteryPercent;
float batteryAlpha = 0.00001;

// Joystick variables
double leftX, leftY, rightX, rightY;
double joystickAlpha = 0.01;

int32_t rotaryStart;

void initPeripherals() {
    // init joysticks
    pinMode(JSX1, INPUT);
    pinMode(JSX2, INPUT);
    pinMode(JSY1, INPUT);
    pinMode(JSY2, INPUT);

    // init switches
    pinMode(SWCH1, INPUT);
    pinMode(SWCH2, INPUT);

    // init haptic motors
    pinMode(BZZR1, OUTPUT);
    pinMode(BZZR2, OUTPUT);
}

void initRotary() {
    // init rotary encoder
    if (! ss.begin(SEESAW_ADDR)) {
        Serial.println("Couldn't find seesaw on default address");
        delay(100);
    } else {
    Serial.println("seesaw started");
    uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
    if (version  != 5740){
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while(1) delay(100);
    }
    Serial.println("Found Product 5740");

    ss.pinMode(SS_SWITCH_UP, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_DOWN, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_LEFT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_RIGHT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_SELECT, INPUT_PULLUP);

    Serial.println("Turning on interrupts");
    ss.enableEncoderInterrupt();
    ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH_UP, 1);

    // get starting position
    data.rotaryEncoder = 0;
    rotaryStart = ss.getEncoderPosition();
    data.rotaryUp = data.rotaryDown = false;
    data.rotaryRight = data.rotaryLeft = false;
    data.rightX = data.rightY = 0.0;
    data.leftX = data.leftY = 0.0;
    data.swch1 = data.swch2 = false;
    }
}

void readBattery() {
    // Alpha filter for the voltage 
    batteryVoltage = (batteryAlpha*ums3.getBatteryVoltage()) + ((1-batteryAlpha)*batteryVoltage);
    // Percentage mapping
    batteryPercent = (batteryVoltage - 3.0)*100.0;
}

// Joystick center offsets (calibrated)
const double rightXOffset = -0.01;
const double rightYOffset =  0.00;
const double leftXOffset  =  0.17;
const double leftYOffset  = -0.00;
const double joystickDeadZone = 0.03;

double applyOffsetAndDeadZone(double raw, double offset, double deadZone) {
    double centered = raw - offset;
    // Rescale each side so full deflection reaches ±1.0
    if (centered > 0)
        centered /= (1.0 - offset);
    else
        centered /= (1.0 + offset);
    // Dead zone with rescale to preserve full -1 to 1 range
    if (fabs(centered) < deadZone) return 0.0;
    double sign = (centered > 0) ? 1.0 : -1.0;
    return sign * (fabs(centered) - deadZone) / (1.0 - deadZone);
}

void readJoysticks() {
    data.rightX = min(1.0, max(-1.0, applyOffsetAndDeadZone(analogRead(JSX1)/2040.0 - 1.0, rightXOffset, joystickDeadZone)));
    data.rightY = min(1.0, max(-1.0, applyOffsetAndDeadZone(analogRead(JSY1)/2040.0 - 1.0, rightYOffset, joystickDeadZone)));
    data.leftX  = min(1.0, max(-1.0, applyOffsetAndDeadZone(analogRead(JSX2)/2040.0 - 1.0, leftXOffset,  joystickDeadZone)));
    data.leftY  = min(1.0, max(-1.0, applyOffsetAndDeadZone(analogRead(JSY2)/2040.0 - 1.0, leftYOffset,  joystickDeadZone)));
}

void readSwitches() {
    if (analogRead(SWCH1) == 0.0) {
        data.swch1 = true;
    } else { data.swch1 = false; }

    if (analogRead(SWCH2) == 0.0) {
        data.swch2 = true;
    } else { data.swch2 = false; }
}

void readRotary() {
    if (! ss.digitalRead(SS_SWITCH_UP)) {data.rotaryUp = true;}
    else {data.rotaryUp = false;}
    if (! ss.digitalRead(SS_SWITCH_DOWN)) {data.rotaryDown = true;}
    else {data.rotaryDown = false;}
    if (! ss.digitalRead(SS_SWITCH_RIGHT)) {data.rotaryRight = true;}
    else {data.rotaryRight = false;}
    if (! ss.digitalRead(SS_SWITCH_LEFT)) {data.rotaryLeft = true;}
    else {data.rotaryLeft = false;}
    if (! ss.digitalRead(SS_SWITCH_SELECT)) {data.rotarySelect = true;}
    else {data.rotarySelect = false;}
    data.rotaryEncoder = ss.getEncoderPosition() - rotaryStart;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

    bool success = status == ESP_NOW_SEND_SUCCESS ;
    //if (success) {
    //  Serial.printf("Sent x:%.1f y:%.1f\n", data.rightX, data.rightY);
    //} else {
    //  Serial.println("Failed");
    //}
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&data, incomingData, sizeof(data));
    //freshData = true;
    Serial.print("Data: ");
    Serial.println(data.rightX);
}

void initSender() {
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    } 
    
    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    }

void initReceiver() {
    WiFi.mode(WIFI_STA);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void sendData() {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
    //if (result == ESP_OK) {
    //    Serial.println("Sent with success");
    //} else {
    //    Serial.println("Error sending the data");
    //}
}

void sendControllerData() {
    controllerMessage.millis = millis();
    controllerMessage.joystick1 = {(float)data.rightX, (float)data.rightY};
    controllerMessage.joystick2 = {(float)data.leftX, (float)data.leftY};
    controllerMessage.dPad = {data.rotaryUp, data.rotaryDown, data.rotaryLeft, data.rotaryRight, data.rotarySelect, data.rotaryEncoder};
    controllerMessage.buttonL = data.swch1;
    controllerMessage.buttonR = data.swch2;
    controllerMessage.touchPoint = {0, 0, 0};
    esp_now_send(broadcastAddress, (uint8_t *) &controllerMessage, sizeof(controllerMessage));
}

void initScreen() {
    delay(1000);
    Serial.println("Starting");
    tft.init();
    Serial.println("Initialized");
    tft.setRotation(1);
    delay(100);
    tft.fillScreen(TFT_WHITE);
}

void printScreen() {
    // Top dashboard
    tft.setCursor(0, 0, 2);
    tft.setTextColor(TFT_WHITE,TFT_BLACK);  tft.setTextSize(3);
    tft.println("  Remote Control Data   ");
    tft.println();
    
    // Joysticks
    tft.setTextColor(TFT_BLACK,TFT_WHITE); tft.setTextFont(1);
    tft.printf(" Right: X:%.1f, Y:%.1f ", data.rightX, data.rightY);
    tft.println();

    tft.setTextColor(TFT_BLACK, TFT_WHITE); tft.setTextFont(1);
    tft.printf(" Left: X:%.1f, Y:%.1f ", data.leftX, data.leftY);
    tft.println(); tft.println();
    
    // Trigger Switches
    if (data.swch1) {tft.setTextColor(TFT_BLACK,TFT_GREEN); tft.print("On ");}
    else {tft.setTextColor(TFT_BLACK,TFT_RED); tft.print("Off");}
    tft.setTextColor(TFT_WHITE); tft.print(" ");
    if (data.swch2) {tft.setTextColor(TFT_BLACK,TFT_GREEN);tft.print("On ");}
    else {tft.setTextColor(TFT_BLACK,TFT_RED);tft.print("Off");}
    tft.println(); tft.println();

    // Rotary Switches
    tft.setTextColor(TFT_WHITE);tft.print("\t\t");
    if (data.rotaryUp) {tft.setTextColor(TFT_BLACK,TFT_GREEN);}
    else {tft.setTextColor(TFT_BLACK,TFT_RED);}
    tft.print(" UP "); tft.println();
    if (data.rotaryLeft) {tft.setTextColor(TFT_BLACK,TFT_GREEN);}
    else {tft.setTextColor(TFT_BLACK,TFT_RED);}
    tft.print("LEFT"); tft.setTextColor(TFT_WHITE);tft.print("\t");
    if (data.rotaryRight) {tft.setTextColor(TFT_BLACK,TFT_GREEN);}
    else {tft.setTextColor(TFT_BLACK,TFT_RED);}
    tft.print("Right"); 

    tft.setTextColor(TFT_WHITE); tft.print("\t\t"); 
    tft.setTextColor(TFT_BLACK, TFT_WHITE); tft.print("Rotary: ");
    if (data.rotaryEncoder >= 0) {tft.print(" ");}
    tft.print(int(data.rotaryEncoder % 80));tft.print(" ");

    tft.println();tft.setTextColor(TFT_WHITE); tft.print("\t\t");
    if (data.rotaryDown) {tft.setTextColor(TFT_BLACK,TFT_GREEN);}
    else {tft.setTextColor(TFT_BLACK,TFT_RED);}
    tft.print("Down");

    tft.setTextColor(TFT_WHITE); tft.print("\t\t");
    if (data.rotarySelect) {tft.setTextColor(TFT_BLACK,TFT_GREEN);}
    else {tft.setTextColor(TFT_BLACK,TFT_RED);}
    tft.print("Select");

    delay(20);
}

void printData() {
    Serial.print("Joysticks:");
    Serial.printf("Right X: %.2f, Y: %.2f", data.rightX, data.rightY);
    Serial.printf("Left X: %.2f, Y: %.2f", data.leftX, data.leftY);
  
    Serial.print("Switches: L: ");
    Serial.print(data.swch1);
    Serial.print(" R: ");
    Serial.println(data.swch2);
    delay(100);
}

void printRotary() {
    Serial.print("Rotary: U: ");
    Serial.print(data.rotaryUp);
    Serial.print(" D: ");
    Serial.print(data.rotaryDown);
    Serial.print(" L: ");
    Serial.print(data.rotaryRight);
    Serial.print(" R: ");
    Serial.print(data.rotaryLeft);
    Serial.print(" S: ");
    Serial.print(data.rotarySelect);
    Serial.print(" Enc: ");
    Serial.println(data.rotaryEncoder);

    delay(100);
  }
