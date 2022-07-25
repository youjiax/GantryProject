#include <util/delay.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <math.h>

#define EncoderPinA 2   // Encoder Pin A pin 2 and pin 3 are inturrpt pins
#define EncoderPinB 3   // Encoder Pin B

//Arduino PWM Speed Control： 
int E1 = 5; 
int M1 = 4; 
int E2 = 6; 
int M2 = 7; 

int desiredLocation = 50;
long encoderCount = 0;

void readEncoder();
void stop();

void setup() 
{ 
    pinMode(M1, OUTPUT); 
    pinMode(M2, OUTPUT); 

    Serial.begin(9600);
    pinMode(EncoderPinA, INPUT); //initialize Encoder Pins
    pinMode(EncoderPinB, INPUT);  
    digitalWrite(EncoderPinA, LOW); //initialize Pin States
    digitalWrite(EncoderPinB, LOW);
    attachInterrupt(digitalPinToInterrupt(EncoderPinA), readEncoder, RISING); //attach interrupt to PIN 2 
} 
void loop() 
{ 
    digitalWrite(M1, LOW); //LOW clockwise, HIGH anticlockwise
    analogWrite(E1, 75);   //PWM Speed Control 255 is high 0 is low

    delay(1000);
    stopMotor(E1);
    delay(1000);

    digitalWrite(M1, HIGH); //LOW clockwise, HIGH anticlockwise
    analogWrite(E1, 75);   //PWM Speed Control 255 is high 0 is low
    delay(1000);
    stopMotor(E1);
    delay(1000);
}

void stopMotor(int EncoderX) {
  analogWrite(EncoderX, 0);
}

void distanceMove(float target, float kp, float ki, float kp_straight){

}

void readEncoder() //this function is triggered by the encoder CHANGE, and increments the encoder counte
{ 
  if(digitalRead(EncoderPinB)) //PIN A HIGH PIN B HIGH is REVERSE
  {
    encoderCount++; //may need to redefine positive and negative directions
  }
  else
  {
    encoderCount--;
  }
  Serial.println(encoderCount);  
}
