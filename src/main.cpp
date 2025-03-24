// code by James Mu and Nick Trigger

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definitions
#define ECG_AMP_PIN A0 // Amplified ECG Signal Input
#define ECG_COMP_PIN A1 // ECG Comparator Signal Input
#define SAMPLING_RATE 200 // Hz
#define COMP_THRESHOLD 4.0 // Voltage threshold for detecting R-wave
#define PACING_PIN 2 // Pacing output pin
#define CHRONAXIE 1.7895 // Chronaxie duration in ms
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1

// Global variables
int ECG_amp; // 0-1023 (Raw Variable)
int ECG_comp; // 0-1023
int samplingRate; 
int samplingInterval; 
int LRI; // Lower rate interval in ms
int LRI_BPM; // Lower rate interval in BPM
float true_ECG_amp; // Calculated ECG amplitude in volts
float true_ECG_comp; // Calculated ECG comparison voltage in volts
float HR;

bool processFlag = false; // Used to only select rising edge once

unsigned long previousMillis = 0; //Last time that data was acquired (Governed by sampling rate)
unsigned long currentMillis; 

float RR_interval; // Current MEasured RR interval
float prevCompVoltage; // To determine if it is a rising edge
unsigned long prevEdgeTime = 0; // Variable to store the time of the last rising edge (For R-R calcualtions)

// Function declarations
int findRR(void);
void findInstantHR(int);
void pace(void);

//Init Oled Display
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// State machine
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
    // Initialize pin modes
    pinMode(ECG_AMP_PIN, INPUT);
    pinMode(ECG_COMP_PIN, INPUT);
    pinMode(PACING_PIN, OUTPUT);
    digitalWrite(PACING_PIN, LOW); 

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    
    display.setTextSize(1);
    display.display();
    delay(1000);
    display.clearDisplay();

    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Measuring...");
    display.display();
    delay(2000);
    display.clearDisplay();
    display.setCursor(0, 0);

    Serial.begin(115200);
}

void loop()
{
    currentMillis = millis(); // Get the current time

    // Run state machine
    switch (currentState)
    {
    case INIT:
        samplingInterval = 1000 / SAMPLING_RATE; // Calculate sampling interval in ms
        LRI_BPM = 60; // Set lower rate interval in BPM
        LRI = 60000 / LRI_BPM; // LRI in ms
        currentState = ACQUIRING;
        Serial.println("Initialized");
        delay(500);
        display.clearDisplay();
        break;
    case ACQUIRING:
        // Acquiring data state
        if (currentMillis - previousMillis >= samplingInterval)
        {
            previousMillis = currentMillis;
            ECG_amp = analogRead(ECG_AMP_PIN);
            ECG_comp = analogRead(ECG_COMP_PIN);
            currentState = PROCESSING; 
            processFlag = true; 
        }
        break;
    case PROCESSING:
        if (processFlag == true) {
            // Convert analog data to true voltage
            true_ECG_amp = map(ECG_amp, 0, 1023, 0, 5000) / 1000.0;
            true_ECG_comp = map(ECG_comp, 0, 1023, 0, 5000) / 1000.0;
            Serial.print(">ECG_amp:"); // Formatted For Plotting in teleplot
            Serial.println(true_ECG_amp);
            Serial.print(">ECG_comp:"); // ""
            Serial.println(true_ECG_comp);

            RR_interval = findRR(); // Find RR interval

            if (RR_interval > LRI) {
                currentState = PACING; // Transition to PACING state if RR interval exceeds LRI
            } else {
                processFlag = false; 
                currentState = ACQUIRING;
            }
        } else {
            processFlag = false; 
            currentState = ACQUIRING; 
        }
        Serial.print(">Pace:");
        Serial.println(0);
        break;
    case PACING:
        pace(); // Send pacing signal
        currentState = ACQUIRING; 
        break;
    case ERROR:
        Serial.println("error state"); 
        break;
    default:
        // Default state -> Only reached if an invalid currentState happens, should not happen -> Error
        currentState = ERROR; // Transition to ERROR state
        break;
    }
}

int findRR()
{
    // Function to find RR interval
    if ((true_ECG_comp >= COMP_THRESHOLD) && (prevCompVoltage < COMP_THRESHOLD) && (currentMillis - prevEdgeTime > 10))
    { // Check for rising edge w/ debouce bc fuzzy near top of signal
        int rr = (currentMillis - prevEdgeTime); 
        prevEdgeTime = currentMillis; // Update prevEdgeTime
        Serial.print("RR: "); // Print RR interval
        Serial.println(rr);
        Serial.print(">Detected R wave:"); // plot
        Serial.println(1);
        
        findInstantHR(rr); // Find instant heart rate
        Serial.print("Instant HR:"); 
        Serial.println(HR);
        

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Beat Detected!");
        display.print("HR = ");
        display.print(HR);
        display.println(" BPM");
        display.print("R-R = ");
        display.println(rr);
        display.display();

        return rr; 
    }
    prevCompVoltage = true_ECG_comp; //Excludes falling edge
    Serial.print(">Detected R wave:"); 
    Serial.println(0);
    return 0; 
}

void findInstantHR(int rr) {
    HR = 60000.0 / rr; // Calculate heart rate from RR interval (ms -> bpm)
}

void pace() {
    Serial.println("Pacing...");
    Serial.print(">Pace:");  //plot
    Serial.println(1);
    digitalWrite(PACING_PIN, HIGH); // Set pacing pin HIGH
    delay(CHRONAXIE * 2); // Delay for twice the chronaxie duration
    digitalWrite(PACING_PIN, LOW); // Set pacing pin LOW
}