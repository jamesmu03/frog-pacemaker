// Code developed by Nick Trigger and James Mu

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <deque>
#include <algorithm>
#include <cmath>
#include <vector> // Add this include for std::vector

// Pin and hardware definitions
#define ECG_AMP_PIN 35
#define ECG_COMP_PIN 4
#define SAMPLING_RATE 200
#define COMP_THRESHOLD 4.0
#define PACING_PIN 2
#define CHRONAXIE 1.7895
#define DAC_PIN 25
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1

// Function prototypes
void updateOLED(float HR, float RR_interval);
void sendToTeleplot(float ECG_amp, float ECG_comp, float HR, float RR_interval, int pace);
void updateVComp();
void handleMissedBeat();
void processPeak(float peakAmplitude);

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
unsigned long pacingDisplayTime = 0;

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

// Simplified variables for peak amplitudes
float peakAmplitudes[10] = {0}; // Fixed-size array for the last 10 peaks
int peakIndex = 0;             // Index to track the current position in the array
int peakCount = 0;             // Number of valid peaks stored in the array

float vComp = COMP_THRESHOLD;    // Initial Vcomp value

void setup() {
    Serial.begin(115200);
    pinMode(ECG_AMP_PIN, INPUT);
    pinMode(ECG_COMP_PIN, INPUT);
    pinMode(PACING_PIN, OUTPUT);
    digitalWrite(PACING_PIN, LOW);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Error: OLED initialization failed. Halting execution.");
        while (true);
    }

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Measuring...");
    display.display();
    delay(2000);
    display.clearDisplay();
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

                // Process the detected peak
                processPeak(true_ECG_amp);

                updateOLED(HR, RR_interval);
            } else if (currentMillis - prevEdgeTime > LRI) {
                // Handle missed beat
                handleMissedBeat();
            }

            prevCompVoltage = true_ECG_comp;

            if (currentMillis - prevEdgeTime > LRI) {
                currentState = PACING;
            } else {
                currentState = ACQUIRING;
            }

            sendToTeleplot(true_ECG_amp, true_ECG_comp, HR, RR_interval, 0);
            break;

        case PACING:
            if (currentMillis - lastPaceTime >= LRI) {
                Serial.print(">Pace: ");
                Serial.println(1);
                lastPaceTime = currentMillis;

                digitalWrite(PACING_PIN, HIGH);
                delay(CHRONAXIE * 2); // Pacing pulse duration
                digitalWrite(PACING_PIN, LOW);

                // Record the time to keep "Pacing..." on the screen
                pacingDisplayTime = currentMillis;
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

// Function to update the OLED display
void updateOLED(float HR, float RR_interval) {
    display.clearDisplay();

    display.setTextSize(1);

    // LRI BPM
    display.setCursor(0, 0);
    display.print("LRI BPM: ");
    display.println(LRI_BPM);

    // Beat detection and data
    display.setCursor(0, 20);
    display.print("HR: ");
    display.print(HR);
    display.println(" BPM");

    display.setCursor(0, 30);
    display.print("R-R: ");
    display.print(RR_interval);
    display.println(" ms");

    display.setCursor(0, 40);
    // Keep "Pacing..." on the screen for a longer duration (e.g., 1 second)
    if (digitalRead(PACING_PIN) == HIGH || (currentMillis - pacingDisplayTime < 300)) {
        display.println("Pacing...");
    }

    display.setCursor(0, 50);
    display.print("Comparator: ");
    display.print(vComp, 2);
    display.println(" V");

    // Finalize display
    display.display();
}

// Function to send data to Teleplot
void sendToTeleplot(float ECG_amp, float ECG_comp, float HR, float RR_interval, int pace) {
    Serial.print(">ECG_amp:");
    Serial.print(ECG_amp);
    Serial.print(">ECG_comp:");
    Serial.print(ECG_comp);
    Serial.print(">HR:");
    Serial.print(HR);
    Serial.print(">RR_interval:");
    Serial.print(RR_interval);
    Serial.print(">Pace:");
    Serial.println(pace);
}

void updateVComp() {
    if (peakCount >= 10) {
        // Sort the array to find the 5th percentile
        float sortedPeaks[10];
        memcpy(sortedPeaks, peakAmplitudes, sizeof(peakAmplitudes));
        for (int i = 0; i < 10 - 1; i++) {
            for (int j = i + 1; j < 10; j++) {
                if (sortedPeaks[i] > sortedPeaks[j]) {
                    float temp = sortedPeaks[i];
                    sortedPeaks[i] = sortedPeaks[j];
                    sortedPeaks[j] = temp;
                }
            }
        }

        // Calculate the 5th percentile index
        int index = (int)(0.05 * peakCount);
        vComp = sortedPeaks[index];
    }

    // Clamp vComp to the valid range
    if (vComp < 0.0f) {
        vComp = 0.0f;
    } else if (vComp > 3.3f) {
        vComp = 3.3f;
    }

    // Debug print
    Serial.print("vComp (clamped): ");
    Serial.println(vComp);

    // Write the new Vcomp to the DAC pin
    int dacValue = map(vComp * 1000, 0, 3300, 0, 255);
    dacWrite(DAC_PIN, dacValue);

    // Debug print
    Serial.print("DAC Value: ");
    Serial.println(dacValue);
}

void handleMissedBeat() {
    vComp = std::max(0.0f, vComp - 0.1f); // Ensure vComp doesn't go below 0
    vComp = std::min(vComp, 3.3f);        // Clamp to 3.3V

    int dacValue = map(vComp * 1000, 0, 3300, 0, 255);
    dacWrite(DAC_PIN, dacValue);

    // Debug print
    Serial.print("Missed Beat - vComp: ");
    Serial.println(vComp);
    Serial.print("DAC Value: ");
    Serial.println(dacValue);
}

void processPeak(float peakAmplitude) {
    // Add the new peak to the array in a circular manner
    peakAmplitudes[peakIndex] = peakAmplitude;
    peakIndex = (peakIndex + 1) % 10; // Wrap around the index
    if (peakCount < 10) {
        peakCount++; // Increment the count until the array is full
    }

    // Debug print
    Serial.print("New Peak Amplitude: ");
    Serial.println(peakAmplitude);

    updateVComp();
}