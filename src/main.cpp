// code by James Mu and Nick Trigger

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define ECG_AMP_PIN A0
#define ECG_COMP_PIN A1
#define SAMPLING_RATE 200  // Hz
#define COMP_THRESHOLD 4.0 // V

// init globals
int ECG_amp;
int ECG_comp;
int samplingRate;
int samplingInterval;
int LRI;
float true_ECG_amp;
float true_ECG_comp;

bool processFlag = false;

unsigned long previousMillis = 0; // to store the last time the sampling was done
unsigned long currentMillis;

float RR_interval; // reported rr from func (via comp values)
float prevCompVoltage;
unsigned long prevEdgeTime = 0; // time of last rising edge

// funcs
int findRR(void);

// init state machine
enum
{
    INIT,
    ACQUIRING,
    PROCESSING,
    PACING,
    ERROR
};

int currentState = INIT;

void setup()
{
    pinMode(ECG_AMP_PIN, INPUT);
    pinMode(ECG_COMP_PIN, INPUT);

    Serial.begin(115200);
    Serial.println("Hello World");
}

void loop()
{
    currentMillis = millis();

    // check if arduino is functioning normally
    // run state machine
    switch (currentState)
    {
    case INIT:
        samplingInterval = 1000 / SAMPLING_RATE; // in ms
        LRI = 1000;                              // in ms CHANGE THIS TO ACTUAL VALUE LATER
        currentState = ACQUIRING;
        break;
    case ACQUIRING:
        if (currentMillis - previousMillis >= samplingInterval)
        {
            previousMillis = currentMillis;
            // acquire analog data
            ECG_amp = analogRead(ECG_AMP_PIN);
            ECG_comp = analogRead(ECG_COMP_PIN);
            currentState = PROCESSING;
            processFlag = true;
        }
        break;
    case PROCESSING:
        if (processFlag == true) {
            // convert analog data to true voltage
            true_ECG_amp = map(ECG_amp, 0, 1023, 0, 5000) / 1000.0;
            true_ECG_comp = map(ECG_comp, 0, 1023, 0, 5000) / 1000.0;
            Serial.print(">ECG_amp:"); // formatted for Teleplot
            Serial.println(true_ECG_amp);
            Serial.print(">ECG_comp:");
            Serial.println(true_ECG_comp);

            RR_interval = findRR();

            processFlag = false;
            currentState = ACQUIRING;
        } else {
            processFlag = false;
            currentState = ACQUIRING;
        }
        break;
    case PACING:
        // shock em
        break;
    case ERROR:
        Serial.println("error state");
        break;
    default:
        currentState = ERROR;
        Serial.println("default state");
        break;
    }
}

int findRR()
{
    if ((true_ECG_comp >= COMP_THRESHOLD) && (prevCompVoltage < COMP_THRESHOLD) && (currentMillis - prevEdgeTime > 10))
        {
        int rr = (currentMillis - prevEdgeTime);
        prevEdgeTime = currentMillis;
        // Serial.print("true_ECG_comp: ");
        // Serial.print(true_ECG_comp);
        // Serial.print(", prevCompVoltage: ");
        // Serial.print(prevCompVoltage);
        // Serial.print(", prevEdgeTime: ");
        // Serial.print(prevEdgeTime);
        // Serial.print(", currentMillis: ");
        // Serial.print(currentMillis);
        Serial.print(", RR: ");
        Serial.println(rr);
        Serial.print(">Detetcted R wave:");
        Serial.println(1);
        }
    prevCompVoltage = true_ECG_comp;
}