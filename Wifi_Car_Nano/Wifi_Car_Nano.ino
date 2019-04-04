
const int trigPin = 4;
const int echoPin = 3;

unsigned long duration;
unsigned int tempDistance;
unsigned int totDistance;
byte distance;
short count;
const short cap = 10;

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
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance

  tempDistance = duration * 0.034 / 2;
  Serial.print(tempDistance);
  Serial.write((byte)tempDistance);
  delay(50);
}
