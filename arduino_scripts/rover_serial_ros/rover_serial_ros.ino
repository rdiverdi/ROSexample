/* rover communication code
serial communication receives JSON messages and controls rover
motors based on input
Also sends sensor data over serial in the JSON format
*/

//#include <ArduinoJson.h> //existing library (install form library manager)
//#include "JSON.h" //my wrapper for ArduinoJson which gives you auto-updating variables
#include "rover.h" //my library for controlling the rover

#include <mod_ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

std_msgs::Int16MultiArray ir;
ros::Publisher ir_sensors("ir_sensors", &ir);

float vel;
float turn;
int flash_rate=50;
bool got_msg = false;

void cmd_vel_cb( const geometry_msgs::Twist& cmd_vel ) {
  vel = cmd_vel.linear.x;
  turn = cmd_vel.angular.z;
  got_msg = true;
}
ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", &cmd_vel_cb );

void blink_cb(const std_msgs::Int16& blinkRate ){
  flash_rate = blinkRate.data;
}
ros::Subscriber<std_msgs::Int16> flash("blink_rate", &blink_cb);


bool light=false; // led on/off
int count = flash_rate; // blink time (in loop cycles)

const int kill_pin = 2;
bool kill = LOW;
const int led_1 = 12;

const int IR_1_pin = A1;
int IR_1;

Rover rover(5, 7, 6, 8); // initialize rover with motor pins

void setup() {
  // runs once at the beginning of the script
  //Serial.begin(115200); // start serial communication at a baud rate of 9600
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  //nh.advertise(ir_sensors);
  nh.subscribe(cmd);
  nh.subscribe(flash);
  ir.data = (int16_t*) malloc(2);
  ir.data_length = 1;
  pinMode(led_1, OUTPUT); // setup pin 12 as an output for the led
  pinMode(IR_1_pin, INPUT);
  pinMode(kill_pin, INPUT_PULLUP);
}

void loop() {
  // runs continuously
  kill = digitalRead(kill_pin);
  
  if (count > flash_rate){ //if count is high enough, toggle the led
    count = 0; //reset count
    light = !light; // toggle led variable
    digitalWrite(led_1, light); // write to led
  }

  if (kill){
    rover.send_cmd(0, 0);
  }

  if (got_msg && !kill){
    got_msg = false;
    rover.send_cmd(vel, turn); // command motors
  }

  IR_1 = analogRead(IR_1_pin);
  ir.data[0] = IR_1;
  ir_sensors.publish( &ir );
  nh.spinOnce();
  delay(5); // delay for stability
  count++; // increment count
}


