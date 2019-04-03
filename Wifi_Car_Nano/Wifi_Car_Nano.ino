/*
   Senses if ultrasonic sensor is too close, count 10 times, then send HIGH to interrupt the UNO through pin 8

   - Henry Wu
*/

#define SIGNAL_PIN 8
#define TRIG_PIN 5
#define ECHO_PIN 4
#define THRESHOLD 30  //Threshold in cm to determine too close

//#define DEBUG   // Uncomment this line to print debug message
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#endif

int distance;
long duration;
uint8_t count;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  count = 0;
  pinMode(SIGNAL_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  count = (distance < THRESHOLD && distance != 0) ? count + 1 : 0;

  if (count == 10) {              // Send a pulse to interrupt when the distance is too short for 10 readings in a row
    digitalWrite(SIGNAL_PIN, HIGH);
    delay(5);
    digitalWrite(SIGNAL_PIN, LOW);
    count = 0;
    DEBUG_PRINT("Too close");
  }
  DEBUG_PRINT(distance);
}
