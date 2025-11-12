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
#define SBUS_RX_PIN 18
#define SBUS_TX_PIN 17

HardwareSerial sbus_serial(1);
SBUS r88(sbus_serial);
uint16_t channels[16];
bool failSafe;
bool lostFrame;

// Motor pins
#include <ESP32Servo.h>

#define MOTOR1_PIN 16
#define MOTOR2_PIN 15

Servo motor1;
Servo motor2;

unsigned long prevInputTime =  millis();

const long idleInterval = 10000;
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

  // Startup robo eyes
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100); // screen-width, screen-height, max framerate
  roboEyes.setAutoblinker(ON, 3, 2); 
  
  roboEyes.setCuriosity(ON);
  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();

  if (r88.read(&channels[0], &failSafe, &lostFrame)) {
    int ch2Raw = channels[1];  // CH2 RIGHT STICK FORWARD/BACK
    int ch4Raw = channels[3];  // CH4 LEFT STICK LEFT/RIGHT

    int leftPulse = map(ch4Raw, 172, 1809, 1000, 2000);
    int rightPulse = map(ch2Raw, 172, 1809, 1000, 2000);

    // Send pulses to ESCs
    motor1.writeMicroseconds(leftPulse);
    motor2.writeMicroseconds(rightPulse);

    if (ch4Raw >= 1100) {      
      roboEyes.setPosition(W);            
    } else if (ch4Raw <= 858) {      
      roboEyes.setPosition(E);      
    } else {
      if (!idleMode) {
        roboEyes.setPosition(DEFAULT);            
      }
    }

    if (ch2Raw >= 1500) {      
      roboEyes.setMood(ANGRY);      
    } else {
      roboEyes.setMood(DEFAULT);      
    }

    if ((ch2Raw >= 1000 || ch2Raw <= 990) || (ch4Raw >= 1000 || ch4Raw <= 975)) {
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
