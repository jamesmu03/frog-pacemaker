// Code developed by Nick Trigger and James Mu

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <numeric>
#include "realtime/Detectors.h"

// Pin and hardware definitions
#define ECG_AMP_PIN 35
#define ECG_COMP_PIN 4
#define SAMPLING_RATE 200
#define COMP_THRESHOLD 2.5
#define PACING_PIN 2
#define CHRONAXIE 1.7895
#define DAC_PIN 25
#define PEAK_QUEUE_SIZE 10
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1

// Function prototypes
void paceMakerTask(void *pvParameters);
void compVoltageTask(void *pvParameters);
void updateOLED(float HR, float RR_interval, Adafruit_SSD1306 &display);

// Threading
TaskHandle_t pacemakerTaskHandle;
TaskHandle_t compVoltageTaskHandle;

// Globals
SemaphoreHandle_t ecgAmpMutex;
SemaphoreHandle_t compVoltageSemaphore;   // <-- NEW: Binary semaphore for synchronization
volatile int ECG_amp;
float true_ECG_amp;
float compVoltage;
int numOfSaved = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(PACING_PIN, OUTPUT);
    pinMode(DAC_PIN, OUTPUT);
    pinMode(ECG_AMP_PIN, INPUT_PULLDOWN);
    pinMode(ECG_COMP_PIN, INPUT_PULLDOWN);

    xTaskCreatePinnedToCore(paceMakerTask, "PaceMakerTask", 5000, NULL, 1, &pacemakerTaskHandle, 0);
    xTaskCreatePinnedToCore(compVoltageTask, "CompVoltageTask", 20000, NULL, 1, &compVoltageTaskHandle, 1);

    ecgAmpMutex = xSemaphoreCreateMutex();
    compVoltageSemaphore = xSemaphoreCreateBinary(); // <-- NEW: Create the binary semaphore
}

void loop()
{
    // Empty (Check Tasks)
}

// Function to update the OLED display
void updateOLED(float HR, float RR_interval, Adafruit_SSD1306 &display)
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print(HR, 2);
    display.println(" BPM");

    display.setTextSize(1);
    display.print("RR: ");
    display.print(RR_interval, 0);
    display.println(" ms");

    display.print("VComp: ");
    display.print(compVoltage, 2);
    display.println(" V");

    display.print("Num of saved: ");
    display.print(numOfSaved, 0);

    display.display();
}

void paceMakerTask(void *pvParameters)
{
    // State machine
    enum State
    {
        INIT,
        ACQUIRING,
        PROCESSING,
        PACING,
        ERROR
    };

    // Initialize the OLED display
    Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();

    State currentState = INIT;
    volatile int ECG_comp;
    int samplingRate;
    int samplingInterval;
    int LRI;
    int LRI_BPM;
    unsigned long lastPaceTime = 0;
    unsigned long previousMillis = 0;
    unsigned long currentMillis;
    unsigned long prevEdgeTime = 0;
    unsigned long pacingDisplayTime = 0;
    float true_ECG_comp;
    float HR = 0;
    float RR_interval = 0;
    float prevCompVoltage = 0;

    while (true)
    {
        currentMillis = millis();

        switch (currentState)
        {
        case INIT:
            samplingInterval = 1000 / SAMPLING_RATE;
            LRI_BPM = 60;
            LRI = 60000 / LRI_BPM;
            currentState = ACQUIRING;
            Serial.println("Initialized");
            delay(500);
            display.clearDisplay();
            break;

        case ACQUIRING:
            if (currentMillis - previousMillis >= samplingInterval)
            {
                previousMillis = currentMillis;
                if (xSemaphoreTake(ecgAmpMutex, portMAX_DELAY))
                {
                    ECG_amp = analogRead(ECG_AMP_PIN);
                    xSemaphoreGive(ecgAmpMutex);
                }
                else
                {
                    Serial.println("Failed to take mutex in paceMakerTask");
                }
                
                ECG_comp = analogRead(ECG_COMP_PIN);
                xSemaphoreGive(compVoltageSemaphore); // Notify compVoltageTask

                updateOLED(HR, RR_interval, display);

                currentState = PROCESSING;
            }
            break;

        case PROCESSING:
            true_ECG_amp = map(ECG_amp, 0, 4096, 0, 3300) / 1000.0;
            true_ECG_comp = map(ECG_comp, 0, 4096, 0, 3300) / 1000.0;

            // RR interval detection logic
            if ((true_ECG_comp >= COMP_THRESHOLD) && (prevCompVoltage < COMP_THRESHOLD) && (currentMillis - prevEdgeTime > 10))
            {
                RR_interval = currentMillis - prevEdgeTime;
                lastPaceTime = currentMillis;
                prevEdgeTime = currentMillis;

                HR = 60000.0 / RR_interval;

            }

            prevCompVoltage = true_ECG_comp;

            if (currentMillis - prevEdgeTime > LRI)
            {
                currentState = PACING;
            }
            else
            {
                currentState = ACQUIRING;
            }
            break;

        case PACING:
            if (currentMillis - lastPaceTime >= LRI)
            {
                lastPaceTime = currentMillis;

                digitalWrite(PACING_PIN, HIGH);
                delay(CHRONAXIE * 2); // Pacing pulse duration
                digitalWrite(PACING_PIN, LOW);

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
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void compVoltageTask(void *pvParameters)
{
    float currentECG_amp = 0.0;
    DetectorPanTompkins detector(SAMPLING_RATE);

    // Fixed-size circular buffer for last 10 peaks
    float last10Peaks[PEAK_QUEUE_SIZE] = {0};
    int peakIndex = 0;
    int peakCount = 0;
    float avgRPeak = 0.0;
    float stdev = 0.0;
    compVoltage = -1.0;
    static int sampleCounter = 0;
    volatile int dacValue = 0;
    // Create a circular buffer to store the last 100 values of currentECG_amp
    static float ecg_amp_buffer[100] = {0};
    static int bufferIndex = 0;

    while (true)
    {
        // --- NEW: Wait for semaphore from paceMakerTask ---
        if (xSemaphoreTake(compVoltageSemaphore, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(ecgAmpMutex, portMAX_DELAY))
            {
                currentECG_amp = ECG_amp;
                xSemaphoreGive(ecgAmpMutex);
            }
            else
            {
                Serial.println("Failed to take mutex in compVoltageTask");
            }

            sampleCounter++;
            if (sampleCounter >= 4000) {
                detector.reset();
                sampleCounter = 0;
            }

            // Add the currentECG_amp to the buffer
            ecg_amp_buffer[bufferIndex] = currentECG_amp;
            bufferIndex = (bufferIndex + 1) % 100;

            int qrs = detector.processSample(currentECG_amp);
            Serial.print(">ECG: ");
            Serial.println(currentECG_amp / 4096, 2);

            if (qrs != -1)
            {
                // Find the maximum value in the buffer
                float maxECG_amp = ecg_amp_buffer[0];
                for (int i = 1; i < 100; i++)
                {
                    if (ecg_amp_buffer[i] > maxECG_amp)
                    {
                        maxECG_amp = ecg_amp_buffer[i];
                    }
                }

                // Store the maximum value in the circular buffer for last10Peaks
                Serial.print(">Peak: ");
                Serial.println(1);
                last10Peaks[peakIndex] = maxECG_amp;
                peakIndex = (peakIndex + 1) % PEAK_QUEUE_SIZE;
                if (peakCount < PEAK_QUEUE_SIZE)
                    peakCount++;
            }
            else
            {
                Serial.print(">Peak: ");
                Serial.println(0);
            }
            
            numOfSaved = peakCount;

            if (peakCount >= PEAK_QUEUE_SIZE)
            {
                // Calculate avg and stdev of last10peaks'
                avgRPeak = std::accumulate(last10Peaks, last10Peaks + PEAK_QUEUE_SIZE, 0.0) / PEAK_QUEUE_SIZE;

                float sumSquaredDiffs = 0.0;
                for (int i = 0; i < PEAK_QUEUE_SIZE; i++)
                {
                    float diff = last10Peaks[i] - avgRPeak;
                    sumSquaredDiffs += diff * diff;
                }
                stdev = sqrt(sumSquaredDiffs / PEAK_QUEUE_SIZE);

                // Calculate compVoltage
                compVoltage = avgRPeak - (stdev * 0.7);
                compVoltage = map(compVoltage, 0, 4096, 0, 3300) / 1000.0; // Convert to volts
                avgRPeak = map(avgRPeak, 0, 4096, 0, 3300) / 1000.0; // Convert to volts
                stdev = map(stdev, 0, 4096, 0, 3300) / 1000.0; // Convert to volts

                dacValue = (u_int8_t)(compVoltage/3.3 * 255.0); // Scale to DAC range

                Serial.print("Avg: ");
                Serial.print(avgRPeak, 2);
                Serial.print(" Stdev: ");
                Serial.print(stdev, 2);
                Serial.print(" dacValue: ");
                Serial.println(dacValue);
                dacWrite(DAC_PIN, dacValue);
            }
        }
    }
}
