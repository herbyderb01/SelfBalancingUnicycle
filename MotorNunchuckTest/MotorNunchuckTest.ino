// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <Wire.h>
#include <AFMotor.h>
#include <ArduinoNunchuk.h>

#define BAUDRATE 19200
#define DEADBUFF 15

AF_DCMotor motor(1);

ArduinoNunchuk nunchuk = ArduinoNunchuk();


void setup() {
  Serial.begin(BAUDRATE);           // set up Serial library
  nunchuk.init();

  // turn on motor
  motor.setSpeed(200);
  motor.run(RELEASE);
  motor.setSpeed(0);
}

void loop() {
  nunchuk.update();

  // if()

//   uint8_t i;
  

  if(nunchuk.analogY > 126+DEADBUFF){ //joystick up
    motor.run(FORWARD);
    int speed = (nunchuk.analogY - 126-DEADBUFF) * 2.7;
    Serial.println(speed);
    // Serial.print(' ');
    // Serial.print(nunchuk.analogY);
    // Serial.println("");
    motor.setSpeed(speed);
  }
  else if(nunchuk.analogY < 126-DEADBUFF){ //joystick down
    motor.run(BACKWARD);
    int speed = (126-DEADBUFF - nunchuk.analogY) * 2.7;
    Serial.println(speed);
    motor.setSpeed(speed);
  }
  else {
    motor.setSpeed(0);
    motor.run(RELEASE);
    Serial.println("");
  }  

  // motor.run(RELEASE);
}
