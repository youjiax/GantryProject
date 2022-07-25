#define ENCA 2 //YELLOW
#define ENCB 3 //WHITE

int pos = 0;

//Arduino PWM Speed Control： 
int M1p = 5; 
int M1d = 4; 
int M2p = 6; 
int M2d = 7; 

void setup() {
    pinMode(M1d, OUTPUT); 
    pinMode(M2d, OUTPUT); 

  
    Serial.begin(9600);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING); //INTERRUPT TRIGGERED ON RISING EDGE FOR ENCA, RUN READ ENCODER FUNC 


 
}


void readEncoder(){ //Anticlockwise incr pos
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }else{
    pos--;
  }
}


//positive degrees = anticlockise
void moveDegrees(int degrees){
  int counts = degrees*86/15; //weird fracion is 2064/360
  Serial.print(counts);
  //int absPos = abs(pos%2064); //need absolute

  int error = pos - counts;

  int Kp = 1;
  
  if(error <= 0){ //rotate acw
    digitalWrite(M1d, HIGH); //LOW clockwise, HIGH anticlockwise
    analogWrite(M1p, error*Kp);   //PWM Speed Control 255 is high 0 is low 
    
  }

  if(error > 0){//rotate cw
    digitalWrite(M1d, LOW); //LOW clockwise, HIGH anticlockwise
    analogWrite(M1p, error*Kp);   //PWM Speed Control 255 is high 0 is low 
    
  }
}


void loop() {
  
  //Serial.print(counts);
  Serial.println();
    moveDegrees(720);


}
