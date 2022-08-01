#include <util/delay.h>

#define ENCAR 2 //YELLOW
#define ENCBR 8 //WHITE
#define ENCAL 3
#define ENCBL 9
#define M1P 5 //Arduino PWM Speed control
#define M1D 4
#define M2P 6
#define M2D 7

int pos1 = 0;
//int pos2 = 0;

unsigned long currentTime;
unsigned long prevTime;

//Controller gains, Dubls for extra precision
double Kp = 1.5;
double Ki = 0.01;
double Kd = 0.01;

void setup() {

    Serial.begin(9600);

    pinMode(M1D, OUTPUT); //CHECK IF M1P M2P need to be outputs
    pinMode(M2D, OUTPUT); 

    pinMode(ENCAR, INPUT); //Motor Encoder Initiliasation
    pinMode(ENCBR, INPUT);
    pinMode(ENCAL, INPUT);
    pinMode(ENCBL, INPUT);

    pinMode(leftLim, INPUT); //Limit Switch Initialisation
    pinMode(rightLim, INPUT);
    pinMode(botLim, INPUT);
    pinMode(topLim, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCAR), readEncoder1, RISING); //INTERRUPT TRIGGERED ON RISING EDGE FOR ENCA, RUN READ ENCODER FUNC 
    attachInterrupt(digitalPinToInterrupt(ENCAL), readEncoder2, RISING); //INTERRUPT TRIGGERED ON RISING EDGE FOR ENCA, RUN READ ENCODER FUNC
    attachInterrupt(digitalPinToInterrupt(leftLim), leftLimit, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightLim), rightLimit, CHANGE);
    attachInterrupt(digitalPinToInterrupt(botLim), botLimit, CHANGE);
    attachInterrupt(digitalPinToInterrupt(topLim), topLimit, CHANGE);
}

void leftLimit(){ 
  
}

void rightLimit(){
  
}

void botLimit(){
  
}

void topLimit(){
  
}

void readEncoder1(){ //Anticlockwise incr pos
  digitalRead(ENCBR) ? pos1++ : pos1--; //if ENCB1 is high, increase pos, else decrease
} 

void readEncoder2(){ //Anticlockwise incr pos
  digitalRead(ENCBL) ? pos2++ : pos2--;
}

void moveDegrees(int degrees, int motorDirec, int motorPower){ //positive degrees = anticlockise
  double counts = degrees*5.7333; //weird fracion is 2064/360 -- 86/15 CAUSES OVERFLOW ISSUE
  //int absPos = abs(pos%2064); //need absolute
  int error = counts - pos;
  int power = error*Kp;

  if(power > 255){
    power = 255;
  }
  
  if(error >= 0){ //rotate acw
    digitalWrite(motorDirec, HIGH); //LOW clockwise, HIGH anticlockwise
    analogWrite(motorPower, power);   //PWM Speed Control 255 is high 0 is low 
  }

  if(error < 0){//rotate cw
    digitalWrite(motorDirec, LOW); //LOW clockwise, HIGH anticlockwise
    analogWrite(motorPower, power);   //PWM Speed Control 255 is high 0 is low 
    
  }//NEED TO CONVERT TO WHILE LOOPS ----------------------------------------------------------------
}


void loop() {
 
  Serial.println(pos);

    moveDegrees(180, M1D, M1P); //Bottom right
    //moveDegrees(-360, M1D, M1P); //TOP LEFT
    //moveDegrees(360, M2D, M2P);// TOP RIGHT
   // moveDegrees(-180, M2D, M2P); //BOTTOM LEFT
    

    /*
    delay(1);
    moveDegrees(720, M1D, M1P);
    moveDegrees(-720, M1D, M1P);
    delay(1);
    moveDegrees(-720, M1D, M1P);
    moveDegrees(-720, M1D, M1P);
    delay(1);
    moveDegrees(-720, M1D, M1P);
    moveDegrees(720, M1D, M1P);
    delay(1);
    */ 
}
