/*
Microcontroller Pinout:
pin 10 -> bin1
pin 11 -> bin2

pin6 -> ain1
pin9 -> ain2

pin 3 -> left encoder wire B
pin 13 -> left encoder wire A


pin 12 -> right encoder wire A
pin 2 -> right encoder wire B

*/

#include <PID_v1.h>
#include <ros.h>

#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_callback);

nav_msgs::Odometry odom_msg;

ros::Publisher pub_odom("odom", &odom_msg);

const int encoder0pinA = 23;
const int encoder0pinB = 24;
const int encoder1pinA = 25;
const int encoder1pinB = 26;

const int E_left =5; //The enabling of L298PDC motor driver board connection to the digital interface port 5
const int M_left =4; //The enabling of L298PDC motor driver board connection to the digital interface port 4

byte encoder0PinALast;
byte encoder1PinALast;

double duration, abs_duration;//the number of the pulses
boolean Direction;//the rotation direction
boolean result;

double val_output;//Power supplied to the motor PWM value.
double Setpoint;
double Kp = 0.6, Ki = 5, Kd = 0;
PID pidController(&abs_duration, &val_output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
    nh.initNode();
    nh.subscriber(sub)

    Serial.begin(9600);//Initialize the serial port

    pinMode(M_left, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
    pinMode(E_left, OUTPUT);

    Setpoint = 80;  //Set the output value of the PID

    pidController.SetMode(AUTOMATIC);//PID is set to automatic mode
    pidController.SetSampleTime(100);//Set PID sampling frequency is 100ms

    EncoderInit();//Initialize the module
}

void loop()
{
    advance();//Motor Forward
    abs_duration=abs(duration);
    result=pidController.Compute();//PID conversion is complete and returns 1
    if(result)
    {
        duration = 0; //Count clear, wait for the next count
    }


}

void cmd_vel_callback(const geometry_msgs::Twist& toggle_msg){
}

void EncoderInit()
{
    Direction = true;//default -> Forward
    pinMode(encoder0pinB,INPUT);
    attachInterrupt(0, wheelSpeed, CHANGE);
}

void wheelSpeed()
{
    int Lstate = digitalRead(encoder0pinA);
    if((encoder0PinALast == LOW) && Lstate==HIGH)
    {
        int val = digitalRead(encoder0pinB);
        if(val == LOW && Direction)
        {
            Direction = false; //Reverse
        }
        else if(val == HIGH && !Direction)
        {
            Direction = true;  //Forward
        }
    }
    encoder0PinALast = Lstate;

    if(!Direction)  duration++;
    else  duration--;
}

void advance()//Motor Forward
{
    digitalWrite(M_left,LOW);
    analogWrite(E_left,val_output);
}

void back()//Motor reverse
{
    digitalWrite(M_left,HIGH);
    analogWrite(E_left,val_output);
}

void Stop()//Motor stops
{
    digitalWrite(E_left, LOW);
}
