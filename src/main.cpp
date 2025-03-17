#include <Arduino.h>

// Definitions

// Globals

// Declare Funcs
int myFunction(int, int);

// State Machine Definitions

enum states
{
    IDLE,
    STATE2,
};

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);

  Serial.begin(9600);
  Serial.print("INF: Device initialized. Time: ");
  Serial.println(millis());
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}