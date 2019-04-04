/*
   Sketch for Arduino Nano on the Wifi Car

   - Writes distance in byte to Serial2 on Mega through D0 and D1 UART port
   - Times out for distance greater than 100 cm and sends 0
   - When the nearest wall is 4m away, the sensor gives out random readings of 5, so I did some software fixing

   Author: Henry Wu
*/

#define trigPin 4
#define echoPin 3

unsigned int lastDist1;   // To fix issue with long range readings giving random number
unsigned int lastDist2;

unsigned long duration;
unsigned int distance;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 6600);    //6600 mircoseconds = ~ 100 cm

  // Calculating the distance
  distance = duration * 0.034 / 2;

  // Make sure that this reading isn't random
  if (distance == 5 && lastDist1 == 0 && lastDist2 == 0) distance = 0;

  Serial.write((byte)distance);

  lastDist2 = lastDist1;
  lastDist1 = distance;

  delay(50);
}
