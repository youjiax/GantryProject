#include <util/delay.h>


#define ENCA 2 //YELLOW 20
#define ENCB 8 //WHITE 21
#define ENCA2 3
#define ENCB2 9



int pos = 0;
int pos2 = 0;


unsigned long currentTime;
unsigned long prevTime;



//Arduino PWM Speed Control： 
int M1p = 5; //Motor 1 = Right motor
int M1d = 4; 
int M2p = 6; 
int M2d = 7; 



//Controller gains, Dubls for exta precision
double Kp = 1.5;
double Ki = 0.01;
double Kd = 0.01;

void setup() {
    pinMode(M1d, OUTPUT); 
    pinMode(M2d, OUTPUT); 


//NOT SURE IF NEED
   // pinMode(M1p, OUTPUT); 
   // pinMode(M2p, OUTPUT); 

  
    Serial.begin(9600);

    
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING); //INTERRUPT TRIGGERED ON RISING EDGE FOR ENCA, RUN READ ENCODER FUNC 



    pinMode(ENCA2, INPUT);
    pinMode(ENCB2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING); //INTERRUPT TRIGGERED ON RISING EDGE FOR ENCA, RUN READ ENCODER FUNC 
 
}



void readEncoder(){ //Anticlockwise incr pos
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }else{
    pos--;
  }
}


void readEncoder2(){ //Anticlockwise incr pos
  int b2 = digitalRead(ENCB2);
  if(b2>0){
    pos2++;
  }else{
    pos2--;
  }
}









//positive degrees = anticlockise
void moveDegrees(int degrees, int motorDirec, int motorPower){
  //Serial.print(degrees);
  //Serial.println();
  double counts = degrees*5.7333; //weird fracion is 2064/360 -- 86/15 CAUSES OVERFLOW ISSUE
  //int absPos = abs(pos%2064); //need absolute
 // Serial.print(counts);

int error = 0;


 if(motorDirec == M1d){
  error = counts - pos;
 }

//use position 2 for error if motor 2
  if(motorDirec == M2d){
  error = counts - pos2;
 }
  

  int power = error*Kp;

  if(power > 255){
    power = 255;
  }
  
  if(error <= 0){ //rotate acw
    digitalWrite(motorDirec, HIGH); //LOW clockwise, HIGH anticlockwise
    analogWrite(motorPower, power);   //PWM Speed Control 255 is high 0 is low 
    
  }

  if(error > 0){//rotate cw
    digitalWrite(motorDirec, LOW); //LOW clockwise, HIGH anticlockwise
    analogWrite(motorPower, power);   //PWM Speed Control 255 is high 0 is low 
    
  }//NEED TO CONVERT TO WHILE LOOPS ----------------------------------------------------------------
}


void loop() {
 
  Serial.println(pos);

    moveDegrees(720, M1d, M1p); //TOP LEFT ON GANTRY
    //moveDegrees(-720, M1d, M1p); //BOTTOM RIGHT ON GANTRY
    //moveDegrees(720, M2d, M2p);//BOTTOM LEFT ON GANTRY
    moveDegrees(-720, M2d, M2p); //TOP RIGHT ON GANTRY
    
}
