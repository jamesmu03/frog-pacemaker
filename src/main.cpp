// code by James Mu and Nick Trigger

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define ECG_AMP_PIN A0 // Analog pin for ECG amplitude signal
#define ECG_COMP_PIN A1 // Analog pin for ECG comparison signal
#define SAMPLING_RATE 200 // Sampling rate in Hz
#define COMP_THRESHOLD 4.0 // Voltage threshold for detecting R-wave
#define PACING_PIN 2 // Digital output pin for pacing signal
#define CHRONAXIE 1.7895 // Chronaxie duration in ms (example value, replace with actual value)

// Global variables
int ECG_amp; // Variable to store ECG amplitude reading
int ECG_comp; // Variable to store ECG comparison reading
int samplingRate; // Sampling rate
int samplingInterval; // Sampling interval in ms
int LRI; // Lower rate interval in ms
int LRI_BPM; // Lower rate interval in BPM
float true_ECG_amp; // True ECG amplitude in volts
float true_ECG_comp; // True ECG comparison voltage in volts
float HR; // Heart rate

bool processFlag = false; // Flag to indicate if processing is needed

unsigned long previousMillis = 0; // Variable to store the last time sampling was done
unsigned long currentMillis; // Variable to store the current time

float RR_interval; // Variable to store the RR interval
float prevCompVoltage; // Variable to store the previous comparison voltage
unsigned long prevEdgeTime = 0; // Variable to store the time of the last rising edge

// Function prototypes
int findRR(void);
void findInstantHR(int);
void pace(void);

// State machine states
enum
{
    INIT, // Initialization state
    ACQUIRING, // Acquiring data state
    PROCESSING, // Processing data state
    PACING, // Pacing state
    ERROR // Error state
};

int currentState = INIT; // Variable to store the current state

void setup()
{
    // Initialize pin modes
    pinMode(ECG_AMP_PIN, INPUT);
    pinMode(ECG_COMP_PIN, INPUT);
    pinMode(PACING_PIN, OUTPUT);
    digitalWrite(PACING_PIN, LOW); // Set pacing pin to LOW initially

    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("Hello World");
}

void loop()
{
    currentMillis = millis(); // Get the current time

    // Run state machine
    switch (currentState)
    {
    case INIT:
        // Initialization state
        samplingInterval = 1000 / SAMPLING_RATE; // Calculate sampling interval in ms
        LRI_BPM = 60; // Set lower rate interval in BPM
        LRI = 60000 / LRI_BPM; // Convert BPM to ms
        currentState = ACQUIRING; // Transition to ACQUIRING state
        break;
    case ACQUIRING:
        // Acquiring data state
        if (currentMillis - previousMillis >= samplingInterval)
        {
            previousMillis = currentMillis; // Update previousMillis
            // Acquire analog data
            ECG_amp = analogRead(ECG_AMP_PIN);
            ECG_comp = analogRead(ECG_COMP_PIN);
            currentState = PROCESSING; // Transition to PROCESSING state
            processFlag = true; // Set processFlag to true
        }
        break;
    case PROCESSING:
        // Processing data state
        if (processFlag == true) {
            // Convert analog data to true voltage
            true_ECG_amp = map(ECG_amp, 0, 1023, 0, 5000) / 1000.0;
            true_ECG_comp = map(ECG_comp, 0, 1023, 0, 5000) / 1000.0;
            Serial.print(">ECG_amp:"); // Print ECG amplitude for Teleplot
            Serial.println(true_ECG_amp);
            Serial.print(">ECG_comp:"); // Print ECG comparison voltage for Teleplot
            Serial.println(true_ECG_comp);

            RR_interval = findRR(); // Find RR interval

            if (RR_interval > LRI) {
                currentState = PACING; // Transition to PACING state if RR interval exceeds LRI
            } else {
                processFlag = false; // Reset processFlag
                currentState = ACQUIRING; // Transition to ACQUIRING state
            }
        } else {
            processFlag = false; // Reset processFlag
            currentState = ACQUIRING; // Transition to ACQUIRING state
        }
        break;
    case PACING:
        // Pacing state
        pace(); // Call pace function
        currentState = ACQUIRING; // Transition to ACQUIRING state
        break;
    case ERROR:
        // Error state
        Serial.println("error state"); // Print error message
        break;
    default:
        // Default state
        currentState = ERROR; // Transition to ERROR state
        Serial.println("default state"); // Print default state message
        break;
    }
}

int findRR()
{
    // Function to find RR interval
    if ((true_ECG_comp >= COMP_THRESHOLD) && (prevCompVoltage < COMP_THRESHOLD) && (currentMillis - prevEdgeTime > 10))
    {
        int rr = (currentMillis - prevEdgeTime); // Calculate RR interval
        prevEdgeTime = currentMillis; // Update prevEdgeTime
        Serial.print(", RR: "); // Print RR interval
        Serial.println(rr);
        Serial.print(">Detected R wave:"); // Print R wave detection message
        Serial.println(1);
        
        findInstantHR(rr); // Find instant heart rate
        Serial.print(" Instant HR:"); // Print instant heart rate
        Serial.println(HR);
        return rr; // Return RR interval
    }
    prevCompVoltage = true_ECG_comp; // Update prevCompVoltage
    Serial.print(">Detected R wave:"); // Print R wave detection message
    Serial.println(0);
    return 0; // Return 0 if no R wave detected
}

void findInstantHR(int rr) {
    // Function to find instant heart rate
    HR = 60000.0 / rr; // Calculate heart rate from RR interval
}

void pace() {
    // Function to simulate pacing
    Serial.println("Pacing..."); // Print pacing message
    digitalWrite(PACING_PIN, HIGH); // Set pacing pin HIGH
    delay(CHRONAXIE * 2); // Delay for twice the chronaxie duration
    digitalWrite(PACING_PIN, LOW); // Set pacing pin LOW
}