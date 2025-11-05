#include <SBUS.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

// SBUS setup
#define SBUS_RX_PIN 18
#define SBUS_TX_PIN 17

HardwareSerial sbus_serial(1);
SBUS r88(sbus_serial);
uint16_t channels[16];
bool failSafe;
bool lostFrame;

// Motor pins
#define MOTOR1_PIN 16
#define MOTOR2_PIN 15

Servo motor1;
Servo motor2;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // SBUS init
  r88.begin(SBUS_RX_PIN, SBUS_TX_PIN, true, 100000);

  // Attach ESCs (1ms–2ms range)s
  motor1.attach(MOTOR1_PIN, 1000, 2000);
  motor2.attach(MOTOR2_PIN, 1000, 2000);

  Serial.println("Setup complete");
}

void loop() {
  if (r88.read(&channels[0], &failSafe, &lostFrame)) {
    int ch2Raw = channels[1];  // CH2 RIGHT STICK FORWARD/BACK
    int ch4Raw = channels[3];  // CH4 LEFT STICK LEFT/RIGHT

    int leftPulse = map(ch4Raw, 172, 1811, 1000, 2000);
    int rightPulse = map(ch2Raw, 172, 1811, 1000, 2000);

    // Send pulses to ESCs
    motor1.writeMicroseconds(leftPulse);
    motor2.writeMicroseconds(rightPulse);

    Serial.printf("CH2:%d CH4:%d Left:%d Right:%d\n", ch2Raw, ch4Raw, leftPulse, rightPulse);
  }
}
