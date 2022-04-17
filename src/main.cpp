#include <Arduino.h>
#include <board.h>

#define DSerial SerialUSB
#define ATSerial Serial1

void setup() {
    DSerial .begin(9600);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}

void loop() {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    delay(500);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    DSerial .println("Blinky");
    delay(500);
}