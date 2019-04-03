/*
 * Controll an RC Car from a computer via UDP connection
 * 
 * Author: Henry Wu
 * Date: April 3, 2019
 */


#include "Motor.h"
#include "Car.h"
#include <Servo.h>
#include <WiFlyHQ.h>

#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Wifi Stuff
WiFly wifly;

const char mySSID[] = "Wendell";
const char myPassword[] = "12345678";

const char ServerIP[] = "192.168.43.135";
const uint16_t Port = 5009;

byte buf[80];
byte test;
byte in[3];
byte out[2];

int8_t servoAngle;
int8_t pwm;

// Car Stuff
Motor motor(5, 4, 3);
Servo servo;
Car car(motor, servo);

void wiflyInit();
void testConnection();
void terminal();

void setup() {

  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  car.init();
  wiflyInit();
  DEBUG_PRINTLN("STARTING!");

  out[0] = 1;
}

void loop() {
 
  if (Serial2.available()){   // If sensor data from Nano is available
    out[1] = Serial2.read();
    wifly.write(out, 2);
  }
  
  if (Serial1.read() == 1) {   // Leading byte is received
    while (Serial1.available() < 3);   // Wait for 3 bytes to come in (Dir, PWM and Servo)
    in[0] = Serial1.read();         // Direction (2, forward; 3, backward)
    in[1] = Serial1.read();         // PWM
    in[2] = Serial1.read();         // Angle

    DEBUG_PRINTLN("\n3 bytes: ");
    DEBUG_PRINTLN(in[0]);
    DEBUG_PRINTLN(in[1]);
    DEBUG_PRINTLN(in[2]);

    // Set Direction
    if (in[0] == 2){
      car.drive(in[1]);   // Drive forward with PWM
    } else if (in[0] == 3){
      car.drive(-in[1]);
    }

    // Calc Servo angle
    if (in[2] > 128) {
      servoAngle = in[2] - 256;
    } else {
      servoAngle = in[2];
    }
    DEBUG_PRINTLN("PWM + Servo: ");
    DEBUG_PRINTLN(pwm);
    DEBUG_PRINTLN(servoAngle);
    
    car.turn(servoAngle);
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
