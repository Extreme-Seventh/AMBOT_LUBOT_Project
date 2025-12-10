// EYES
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define i2c_Address 0x3c

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <FluxGarage_RoboEyes.h>
RoboEyes<Adafruit_SSD1306> roboEyes(display); 

// END EYES 


#include <SBUS.h>
#include <HardwareSerial.h>

// SBUS setup
#define SBUS_RX_PIN 18 // White Signal Wire
#define SBUS_TX_PIN 17

HardwareSerial sbus_serial(1);
SBUS r88(sbus_serial);
uint16_t channels[16];
bool failSafe;
bool lostFrame;

// Motor pins
#include <ESP32Servo.h>

#define MOTOR1_PIN 15
#define MOTOR2_PIN 16

#define HEADTILT_PIN 48
#define HEADPAN_PIN 47

Servo motor1;
Servo motor2;

Servo headTilt;
Servo headPan;

unsigned long prevInputTime =  millis();

const long idleInterval = 30000;
bool stickInput = false;
bool idleMode = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // SBUS init
  r88.begin(SBUS_RX_PIN, SBUS_TX_PIN, true, 100000);

  // Attach ESCs (1ms–2ms range)s
  motor1.attach(MOTOR1_PIN, 1000, 2000);
  motor2.attach(MOTOR2_PIN, 1000, 2000);

  headTilt.setPeriodHertz(50);
  headTilt.attach(HEADTILT_PIN, 1000, 2000);
  headPan.setPeriodHertz(50);
  headPan.attach(HEADPAN_PIN, 1000, 2000);

  // Startup robo eyes
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100); // screen-width, screen-height, max framerate
  roboEyes.setAutoblinker(ON, 3, 2); 
  
  roboEyes.setCuriosity(ON);
  headPan.write(90);
  headTilt.write(90);
  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();

  if (r88.read(&channels[0], &failSafe, &lostFrame)) {
    int ch1Raw = channels[0];  // CH1 RIGHT STICK LEFT/RIGHT
    int ch2Raw = channels[1];  // CH2 RIGHT STICK FORWARD/BACK
    int ch3Raw = channels[2];  // CH3 Throttle LEFT STICK UP/DOWN
    int ch4Raw = channels[3];  // CH4 LEFT STICK LEFT/RIGHT
    // int ch5Raw = channels[4];  // CH5 SW A ON/OFF SW
    // int ch6Raw = channels[5]; // CH6 SW B 3-Way SW L = 172 M = 992 H = 1809
    // int ch7Raw = channels[6]; // CH7 SW C 3-Way SW L = 172 M = 992 H = 1809
    // int ch8Raw = channels[7]; // CH8 SW D ON/OFF SW
    // int ch9Raw = channels[8]; // CH9 SW E Momentary SW
    // int ch10Raw = channels[9]; // CH10 Potentiometer
    // int ch11Raw = channels[10]; // CH11
    // int ch12Raw = channels[11]; // CH12
    // int ch13Raw = channels[12]; // CH13
    // int ch14Raw = channels[13]; // CH14
    // int ch15Raw = channels[14]; // CH15
    // int ch16Raw = channels[15]; // CH16

    int moveFB = ch2Raw;
    int moveLR = ch1Raw;
    int headPan_in = ch4Raw;
    int headTilt_in = ch3Raw;
    

    int leftPulse = map(moveLR, 172, 1809, 1000, 2000);
    int rightPulse = map(moveFB, 172, 1809, 1000, 2000);
    int tiltPulse = map(headTilt_in, 172, 1809, 180, 0);
    int panPulse = map(headPan_in, 172, 1809, 180, 0);

    // Send pulses to ESCs
    motor1.writeMicroseconds(leftPulse);
    motor2.writeMicroseconds(rightPulse);
  
    headTilt.write(tiltPulse);
    headPan.write(panPulse);


    if (moveLR >= 1100) {      
      roboEyes.setPosition(W);            
    } else if (moveLR <= 858) {      
      roboEyes.setPosition(E);      
    } else {
      if (!idleMode) {
        roboEyes.setPosition(DEFAULT);            
      }
    }

    if (moveFB >= 1500) {      
      roboEyes.setMood(ANGRY);      
    } else {
      roboEyes.setMood(DEFAULT);      
    }

    if ((moveFB >= 1000 || moveFB <= 990) || (moveLR >= 1000 || moveLR <= 975)) {
      stickInput = true;
      idleMode = false;      
      prevInputTime = millis();      
      roboEyes.setIdleMode(OFF, 2, 2);  
    } else {            
      stickInput = false;      
    }

    if (!stickInput && (currentMillis - prevInputTime >= idleInterval) ) {            
      idleMode = true;         
      roboEyes.setIdleMode(ON, 2, 2);
    }   

    Serial.printf("CH2:%d CH4:%d Left:%d Right:%d Input: %s IdleMode: %s\n  ", 
                ch2Raw, ch4Raw, leftPulse, rightPulse, stickInput ? "True" : "False",
                idleMode ? "True" : "False");  // Debugging Purposes
  }
  roboEyes.update();
}
