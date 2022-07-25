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


void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }else{
    pos--;
  }
}


void loop() {
  
  Serial.print(pos);
  Serial.println();

    digitalWrite(M1d, HIGH); //LOW clockwise, HIGH anticlockwise
    analogWrite(M1p, 175);   //PWM Speed Control 255 is high 0 is low
}
