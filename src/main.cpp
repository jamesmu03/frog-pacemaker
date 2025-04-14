// Code developed by Nick Trigger and James Mu

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pin and hardware definitions
#define ECG_AMP_PIN A0
#define ECG_COMP_PIN A1
#define SAMPLING_RATE 200
#define COMP_THRESHOLD 4.0
#define PACING_PIN 2
#define CHRONAXIE 1.7895
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1

// Variables
int ECG_amp;
int ECG_comp;
int samplingRate;
int samplingInterval;
int LRI;
int LRI_BPM;

unsigned long lastPaceTime = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis;
unsigned long prevEdgeTime = 0;

float true_ECG_amp;
float true_ECG_comp;
float HR;
float RR_interval;
float prevCompVoltage;

bool processFlag = false;

// OLED display
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// State machine
enum State {
    INIT,
    ACQUIRING,
    PROCESSING,
    PACING,
    ERROR
};

int currentState = INIT;

void setup() {
    pinMode(ECG_AMP_PIN, INPUT);
    pinMode(ECG_COMP_PIN, INPUT);
    pinMode(PACING_PIN, OUTPUT);
    digitalWrite(PACING_PIN, LOW);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for (;;); // Halt if OLED initialization fails
    }

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Measuring...");
    display.display();
    delay(2000);
    display.clearDisplay();

    Serial.begin(115200);
}

void loop() {
    currentMillis = millis();

    switch (currentState) {
        case INIT:
            samplingInterval = 1000 / SAMPLING_RATE;
            LRI_BPM = 120;
            LRI = 60000 / LRI_BPM;
            currentState = ACQUIRING;
            Serial.println("Initialized");
            delay(500);
            display.clearDisplay();
            break;

        case ACQUIRING:
            if (currentMillis - previousMillis >= samplingInterval) {
                previousMillis = currentMillis;
                ECG_amp = analogRead(ECG_AMP_PIN);
                ECG_comp = analogRead(ECG_COMP_PIN);
                currentState = PROCESSING;
            }
            break;

        case PROCESSING:
            true_ECG_amp = map(ECG_amp, 0, 1023, 0, 5000) / 1000.0;
            true_ECG_comp = map(ECG_comp, 0, 1023, 0, 5000) / 1000.0;

            Serial.print(">ECG_amp: ");
            Serial.println(true_ECG_amp);
            Serial.print(">ECG_comp: ");
            Serial.println(true_ECG_comp);

            // RR interval detection logic
            if ((true_ECG_comp >= COMP_THRESHOLD) && (prevCompVoltage < COMP_THRESHOLD) && (currentMillis - prevEdgeTime > 10)) {
                RR_interval = currentMillis - prevEdgeTime;
                lastPaceTime = currentMillis;
                prevEdgeTime = currentMillis;

                // Instant HR calculation
                HR = 60000.0 / RR_interval;

                Serial.print("RR: ");
                Serial.println(RR_interval);
                Serial.print(">Detected R wave: ");
                Serial.println(1);
                Serial.print("Instant HR: ");
                Serial.println(HR);

                display.clearDisplay();
                display.setCursor(0, 0);
                display.println("Beat Detected!");
                display.print("HR = ");
                display.print(HR);
                display.println(" BPM");
                display.print("R-R = ");
                display.println(RR_interval);
                display.display();
            } else {
                Serial.print(">Detected R wave: ");
                Serial.println(0);
            }

            prevCompVoltage = true_ECG_comp;

            if (currentMillis - prevEdgeTime > LRI) {
                currentState = PACING;
            } else {
                currentState = ACQUIRING;
            }

            Serial.print(">Pace: ");
            Serial.println(0);
            break;

        case PACING:
            if (currentMillis - lastPaceTime >= LRI) {
                Serial.print(">Pace: ");
                Serial.println(1);
                lastPaceTime = currentMillis;

                digitalWrite(PACING_PIN, HIGH);
                delay(CHRONAXIE);
                digitalWrite(PACING_PIN, LOW);
            }
            currentState = ACQUIRING;
            break;

        case ERROR:
            Serial.println("Error state");
            break;

        default:
            currentState = ERROR;
            break;
    }
}