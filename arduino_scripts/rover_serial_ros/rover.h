#import <Arduino.h>

class Motor {
  bool dir;
  byte vel;
  byte vel_pin;
  byte dir_pin;
  bool invert=false;
public:
  void pins(byte v, byte d) {
    vel_pin = v;
    dir_pin = d;
    pinMode(vel_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
  }
  void cmd(float cmd_vel) {
    if (cmd_vel > 1) cmd_vel = 1;
    if (cmd_vel > 0) dir = HIGH;
    else {dir = LOW; cmd_vel*=-1;}
    if (cmd_vel < -1) cmd_vel = 1;
    vel = cmd_vel*255;
    if (invert) dir=!dir;
    digitalWrite(dir_pin, dir);
    analogWrite(vel_pin, vel);
  }
  void reverse() {
    invert=!invert;
  }
};

class Rover {
  float right_vel;
  float left_vel;
  Motor left_motor;
  Motor right_motor;
public:
  Rover(byte lv, byte ld, byte rv, byte rd){
    left_motor.pins(lv, ld);
    right_motor.pins(rv, rd);
    right_motor.reverse();
  }
  void send_cmd(float vel, float turn){
    right_vel = vel+turn;
    left_vel = vel - turn;
    left_motor.cmd(left_vel);
    right_motor.cmd(right_vel);
  }
};

