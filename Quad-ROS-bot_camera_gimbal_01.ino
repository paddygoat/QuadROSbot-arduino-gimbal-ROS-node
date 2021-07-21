#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  200 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  1400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;



//Set up the ros node and publisher
std_msgs::Float32 throttle_msg;
std_msgs::Float32 ArduinoGPSx_msg;
std_msgs::Float32 ArduinoGPSy_msg;
std_msgs::Float32 act_steer_ang_msg;
std_msgs::Float32 throttControl_msg;
std_msgs::Float32 steerControl_msg;
std_msgs::Float32 camera_gimbal_pan_msg;
std_msgs::Float32 camera_gimbal_tilt_msg;

// ros::Publisher pub_throttle("throttle", &throttle_msg);
// ros::Publisher pub_ArduinoGPSx("ArduinoGPSx", &ArduinoGPSx_msg);
// ros::Publisher pub_ArduinoGPSy("ArduinoGPSy", &ArduinoGPSy_msg);
// ros::Publisher pub_act_steer_ang("act_steer_ang", &act_steer_ang_msg);
ros::NodeHandle nh;

// Testing:
// roscore
// rostopic echo temperature

const int ledPin1 =  13;
int ledState = LOW;
unsigned long prevMillis_01 = 0;
unsigned long prevMillis_02 = 0;
unsigned long prevMillis_03 = 0;
const long interval_01 = 50;
const long interval_02 = 250;
const long interval_03 = 1000;

int ledState_02 = LOW;
int ledState_03 = LOW;

int pan = 0;
int tilt = 0;

/////////////////////////////////////////////////////////////////////////////////////////////
// Subscriber call back setups:

void messageCb1( const std_msgs::Float32 camera_gimbal_pan_msg )
{
  pan = camera_gimbal_pan_msg.data;
}
ros::Subscriber<std_msgs::Float32> sub1("camera_gimbal_pan_msg", &messageCb1 );

void messageCb2( const std_msgs::Float32 camera_gimbal_tilt_msg )
{
  tilt = (int)camera_gimbal_tilt_msg.data;
}
ros::Subscriber<std_msgs::Float32> sub2("camera_gimbal_tilt_msg", &messageCb2 );
////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  nh.initNode();

//////////////////////////////////////////////
  nh.subscribe(sub1);
  nh.subscribe(sub2);
/////////////////////////////////////////////

  // Serial.begin(115200);
  pinMode(ledPin1, OUTPUT);

  digitalWrite(ledPin1, HIGH);  // LED 13
  delay(1000);
  digitalWrite(ledPin1, LOW);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  // Serial.println("Hello!");
}

void loop() 
{
  // 2300 to 250 gives 180 degrees. Tilt horizontal = 1350.

  pwm.writeMicroseconds(0, pan);
  pwm.writeMicroseconds(1, tilt);

  blinkLED_13();  // Blink LED 13 at 2Hz.

  delay(1);
  nh.spinOnce();
}

void blinkLED_13() 
{
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis_03 >= interval_03) 
  {
    prevMillis_03 = currentMillis;
    if (ledState_03 == LOW) 
    {
      // Serial.print("pan reading:  ");Serial.println(pan); 
      // pan = 1750;
      // tilt = 1750;
      ledState_03 = HIGH;
    } else {
      // pan = 1350;
      // tilt = 1350;
      ledState_03 = LOW;
    }
    digitalWrite(ledPin1, ledState_03);
  }
}
