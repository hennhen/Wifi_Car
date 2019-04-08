/*
   Sketch for Arduino Nano on the Wifi Car

   - Writes distance in byte to Serial2 on Mega through D0 and D1 UART port
   - Times out for distance greater than 100 cm and sends 0
   - When the nearest wall is 4m away, the sensor gives out random readings of 5, so I did some software fixing

   Author: Henry Wu
*/
#include <TimedAction.h>

byte outgoingPacket[3];
/*  OUTGOING DATA
    [0]: Lead byte of value 1
    [1]: Distance from ultrasonic sensor
    [2]: How many pulses are collected from the encoder
*/

/* ULTRASONIC SENSOR VARIABLES */
#define TRIG_PIN 4
#define ECHO_PIN 3

byte* distancePtr = &outgoingPacket[1];         // Pointer to the address of the second index of the outgoingPacket
unsigned int distance;
unsigned long duration;
unsigned int lastDist1;                         // To fix issue with long range readings giving random number
unsigned int lastDist2;

/* ENCODER VAIRABLES */
#define ENCODER_PIN 2
#define RPM_SAMPLE_DURATION 100                 // How long to collect pulse counts for (ms)

byte* pulseCountPtr = &outgoingPacket[2];       // Pointer pointing to the third index of outgoingpacket
unsigned short pulseCount;
unsigned long timeA;                            // First point in time to calculate RPM

/* FUNCTIONS */
void getDistance();
void getSpeed();
void sendData();
void pulseIncrement();

TimedAction sendToMega = TimedAction(50, sendData);   // Send data to Mega every 50 ms

void setup() {
  pinMode(TRIG_PIN, OUTPUT);                    // Sets the TRIG_PIN as an Output
  pinMode(ECHO_PIN, INPUT);                     // Sets the ECHO_PIN as an Input

  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), pulseIncrement, CHANGE);
  pulseCount = 0;
  timeA = millis();   // Get current point in time

  outgoingPacket[0] = 1;

  Serial.begin(9600);
}


void loop() {
  getDistance();
  getSpeed();
}

void sendData() {                                     // Send data to the Mega through UART ports
  //Serial.write(outgoingPacket, 3);
}

void getDistance() {                                  // Get distance from ultrasonic sensor
  // Clears the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Sets the TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG_PIN, LOW);

  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH, 6600);            //6600 mircoseconds = ~ 100 cm

  // Calculating the distance
  distance = duration * 0.034 / 2;

  // Make sure that this reading isn't random
  if (distance == 5 && lastDist1 == 0 && lastDist2 == 0) distance = 0;

  Serial.print("Distance: ");
  Serial.println(distance);

  // Stores distance into the outgoingPacket array
  *distancePtr = (byte) distance;
  
  lastDist2 = lastDist1;
  lastDist1 = distance;
}

void getSpeed() {                                        // Get rpmPtr and speed from encoder
  if (millis() >= timeA + RPM_SAMPLE_DURATION) {         // If the end of pulse collection interval is reached

    // Calculate
    Serial.print("Pulses counted: ");
    Serial.println(pulseCount);
    Serial.print("Speed in m/s: ");
    Serial.println(((float)pulseCount / (float)RPM_SAMPLE_DURATION) * 14.81481);  // Print out RPM

    // Reset counting values
    timeA = millis();
    pulseCount = 0;
  }
}

void pulseIncrement() {                                  // Interrupt routine; increases pulse count by 1
  pulseCount++;
} 

