/*
   Sketch for Arduino Mega 2560 on the Wifi RC Car

   - Receives signal wirelessly from a computer through UDP packets
   - WiFly RN-XV module connected to Serial1
   - Sends sensor data to the computer for processing
   - Sensor data is collected from an Arduino Nano, and then sent to Serial2 on the Mega
   
   Created by Henry Wu
   April 4, 2019
*/

#include <WiFlyHQ.h>
#include <Servo.h>
#include "Motor.h"
#include "Car.h"

#define DEBUG   // Comment out this line to ignore all Serial prints in compilation
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
# define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

/* Wifi Variables */
WiFly wifly;

const char mySSID[] = "Wendell";
const char myPassword[] = "12345678";

const char ServerIP[] = "192.168.43.135";
const uint16_t Port = 54613;

byte buf[80];
byte in[3];   // Incoming packet (Direction, Speed, Servo Angle)
byte out[3];  // Outgoing packet (Lead byte, Distance, Pulses per Interval)

/* Car Variables */
Motor motor(5, 4, 3);   // slp, pwm, dir
Servo servo;            // Attached to pin 9 in Car constructor
Car car(motor, servo);

void wiflyInit();
void terminal();

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  car.init();
  wiflyInit();
  out[0] = 126;   // Set leading byte to 126, computer will listen for this to find the start of a packet
}

void loop() {
 
  if (Serial2.available()) {          // If sensor data from Nano is available
    while(Serial2.read() != 126);     // Wait til lead byte is received and 2 bytes to come
    while(Serial2.available() < 2);
    
    out[1] = Serial2.read();
    out[2] = Serial2.read();
    
    DEBUG_PRINTLN();
    DEBUG_PRINTLN(out[0]);
    DEBUG_PRINTLN(out[1]);
    DEBUG_PRINTLN(out[2]);
    
    wifly.write(out, 3);
  }

  if (Serial1.read() == 1) {   // Leading byte is received
    while (Serial1.available() < 3);   // Wait for 3 bytes to come in
    
    in[0] = Serial1.read();         // Direction (2, forward; 3, backward)
    in[1] = Serial1.read();         // PWM
    in[2] = Serial1.read();         // Angle

    DEBUG_PRINTLN("\nBytes= ");
    DEBUG_PRINTLN(in[0]);
    DEBUG_PRINTLN(in[1]);
    DEBUG_PRINTLN(in[2]);

    // Set Direction and PWM
    if (in[0] == 2) {   // Forward, drive with positive speed
      car.drive(in[1]);
    } else if (in[0] == 3) {    // Backward
      car.drive(-in[1]);
    }

    // Set Servo Angle
    if (in[2] > 128) {    // Turn left
      car.turn(in[2] - 256);
    } else {
      car.turn(in[2]);    // Turn right
    }
    
    DEBUG_PRINTLN("PWM & Servo: ");
    DEBUG_PRINTLN(in[1]);
    DEBUG_PRINTLN(in[2]);
  }
}

void wiflyInit() {
  DEBUG_PRINTLN("Starting");
  if (!wifly.begin(&Serial1, &Serial)) {
    DEBUG_PRINTLN("Failed to start wifly");
    terminal();
  }

  if (wifly.getFlushTimeout() != 10) {
    DEBUG_PRINTLN("Restoring flush timeout to 10msecs");
    wifly.setFlushTimeout(10);
    wifly.save();
    wifly.reboot();
  }

  /* Join wifi network if not already associated */
  if (!wifly.isAssociated()) {
    /* Setup the WiFly to connect to a wifi network */
    DEBUG_PRINTLN("Joining network");
    wifly.setSSID(mySSID);
    wifly.setPassphrase(myPassword);
    wifly.enableDHCP();

    if (wifly.join(mySSID)) {
      DEBUG_PRINTLN("Joined wifi network");
    } else {
      DEBUG_PRINTLN("Failed to join wifi network");
      terminal();
    }
  } else {
    DEBUG_PRINTLN("Already joined network");
  }

  /* Setup for UDP packets, sent automatically */
  wifly.setIpProtocol(WIFLY_PROTOCOL_UDP);

  DEBUG_PRINT("IP: ");
  DEBUG_PRINTLN(wifly.getIP(buf, sizeof(buf)));

  wifly.setHost(ServerIP, Port);  // Send UDP packet to this server and port

  DEBUG_PRINTLN("WiFly ready");
}

void terminal() {
  DEBUG_PRINTLN("Terminal ready");
  while (1) {
    if (wifly.available() > 0) {
      Serial.write(wifly.read());
    }

    if (Serial.available()) {
      wifly.write(Serial.read());
    }
  }
}
