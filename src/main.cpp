// code by James Mu and Nick Trigger

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// init variables
int ECG_amp;
int ECG_comp;
int samplingRate;
int samplingInterval;
int LRI;
unsigned long previousMillis = 0; // to store the last time the sampling was done

// init state machine
enum { INIT, ACQUIRING, PROCESSING, PACING, ERROR };
int currentState = INIT;

void setup(){
    Serial.begin(9600);
}

void loop(){
    unsigned long currentMillis = millis();

    // check if arduino is functioning normally

    // run state machine
    switch (currentState){
        case INIT:
            // apply user inputted values
            samplingRate = 200; // in Hz
            samplingInterval = 1000 / samplingRate; // in ms
            LRI = 1000; // in ms CHANGE THIS TO ACTUAL VALUE LATER
            currentState = ACQUIRING;
            break;
        case ACQUIRING:
            if (currentMillis - previousMillis >= samplingInterval) {
                previousMillis = currentMillis;
                // acquire analog data
                ECG_amp = analogRead(A0);
                ECG_comp = analogRead(A1);
                currentState = PROCESSING;
            }
            break;
        case PROCESSING:
            // convert analog data to true voltage
            float true_ECG_amp = map(ECG_amp, 0, 1023, 0, 5000);
            float true_ECG_comp = map(ECG_comp, 0, 1023, 0, 5000);
            Serial.println(true_ECG_amp / 1000.0);
            Serial.println(true_ECG_comp / 1000.0);

            // get R-R interval lol

            // compare R-R interval with LRI and send to next state

            currentState = ACQUIRING;
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