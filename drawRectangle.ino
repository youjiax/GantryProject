#include <util/delay.h>
#include <math.h>

#define ENCAR 2  //YELLOW
#define ENCBR 8  //WHITE
#define ENCAL 3
#define ENCBL 9

#define rightMP 5  //Arduino PWM Speed control
#define rightMD 4
#define leftMP 6
#define leftMD 7

#define leftLim 18
#define rightLim 19
#define botLim 20
#define topLim 21

#define motorDirection(errorValue) ((errorValue) > 0 ? (1) : (0))  //motor clockwise if error is larger than 0, else anticlockwise

int radius = 7.0;  //6.8

int rightPos = 0, leftPos = 0, hitLeft = 0, hitRight = 0, hitBottom = 0, hitTop = 0;

unsigned long currentTime;
unsigned long prevTime;

//Controller gains
double Kp = 30;

void setup() {

  Serial.begin(9600);

  pinMode(rightMD, OUTPUT);  //CHECK IF M1P M2P need to be outputs
  pinMode(leftMD, OUTPUT);

  pinMode(ENCAR, INPUT);  //Motor Encoder Initiliasation
  pinMode(ENCBR, INPUT);
  pinMode(ENCAL, INPUT);
  pinMode(ENCBL, INPUT);

  pinMode(leftLim, INPUT);  //Limit Switch Initialisation
  pinMode(rightLim, INPUT);
  pinMode(botLim, INPUT);
  pinMode(topLim, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCAR), readRightEncoder, RISING);  //INTERRUPT TRIGGERED ON RISING EDGE FOR ENCA, RUN READ ENCODER FUNC
  attachInterrupt(digitalPinToInterrupt(ENCAL), readLeftEncoder, RISING);   //INTERRUPT TRIGGERED ON RISING EDGE FOR ENCA, RUN READ ENCODER FUNC

  attachInterrupt(digitalPinToInterrupt(leftLim), leftLimit, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightLim), rightLimit, CHANGE);
  attachInterrupt(digitalPinToInterrupt(botLim), botLimit, CHANGE);
  attachInterrupt(digitalPinToInterrupt(topLim), topLimit, CHANGE);
}

void leftLimit() {
  digitalRead(leftLim) ? hitLeft = 1 : hitLeft = 0;
}

void rightLimit() {
  digitalRead(rightLim) ? hitRight = 1 : hitRight = 0;
}

void botLimit() {
  digitalRead(botLim) ? hitBottom = 1 : hitBottom = 0;
}

void topLimit() {
  digitalRead(topLim) ? hitTop = 1 : hitTop = 0;
}

void readRightEncoder() {                        //Anticlockwise incr pos VERIFY PLEASE
  digitalRead(ENCBR) ? rightPos++ : rightPos--;  //if ENCB1 is high, increase pos, else decrease
}

void readLeftEncoder() {                          //Anticlockwise incr pos
  digitalRead(ENCBL) ? leftPos++ : leftPos--;
}

double getX() {
  double distRM = -rightPos * radius / 328.4958;  //degrees clockwise, formula assumes anticlockwise
  double distLM = -leftPos * radius / 328.4958;   //328.4958 counts per rad

  double xPos = (distLM + distRM) / 2;

  return xPos;
}

double getY() {
  double distRM = -rightPos * radius / 328.4958;  //degrees clockwise, formula assumes anticlockwise
  double distLM = -leftPos * radius / 328.4958;

  double yPos = (distLM - distRM) / 2;

  return yPos;
}

void callibrationMode(){
  while(!hitLeft){
      digitalWrite(rightMD, 1);  
      analogWrite(rightMP, 120);                   
      digitalWrite(leftMD, 1);   
      analogWrite(leftMP, 120);
  }
  while(hitLeft){
      digitalWrite(rightMD, 0);  
      analogWrite(rightMP, 120);                   
      digitalWrite(leftMD, 0);   
      analogWrite(leftMP, 120);
  }
  while(!hitBottom){
      digitalWrite(rightMD, 0);  
      analogWrite(rightMP, 120);                   
      digitalWrite(leftMD, 1);   
      analogWrite(leftMP, 120);
  }
  while(hitBottom){
      digitalWrite(rightMD, 1);  
      analogWrite(rightMP, 120);                   
      digitalWrite(leftMD, 0);   
      analogWrite(leftMP, 120);
  }
  analogWrite(rightMP, 0);  
  analogWrite(leftMP, 0);

  rightPos = 0;
  leftPos = 0;
}

void drawRectangle(double Length, double Width) {  //length in mm breaks down rectangle in to 4 coordinates
  drawLine(0, Width);
  drawLine(Length, 0);
  drawLine(0, -Width);
  drawLine(-Length, 0);
}

void drawLine(double xPos, double yPos) {  //Inputs are in direction of an XY coordinate plane

  double xError = 0, yError = 0;
  int xPower = 0, yPower = 0;
  rightPos = 0;
  leftPos = 0;

  do {
    xError = xPos - getX();
    yError = yPos - getY();
    //Serial.println(rightPos);
    //Serial.println(leftPos);

    if (!hitLeft && !hitRight && !hitBottom && !hitTop) {

      if (abs(xError)>1) {  //both clockwise/anticlockwise for right/left movement

        xPower = abs(xError) * Kp;
        xPower > 255 ? xPower = 255 : xPower = xPower;  //saturate power to 255 limit

        digitalWrite(rightMD, motorDirection(xError));  //Right motor turns clockwise if error is positive
        analogWrite(rightMP, xPower);                   
        digitalWrite(leftMD, motorDirection(xError));   //Left motor turns clockwise if error is positive
        analogWrite(leftMP, xPower);
      }

      if (abs(yError)>1) { //Right is anticlockwise and left is clockwise for top movement

        yPower = abs(yError) * Kp;
        yPower > 255 ? yPower = 255 : yPower = yPower;    //saturate power to 255 limit

        digitalWrite(rightMD, motorDirection(-yError));   //Right motor turns anticlockwise if error is positive
        analogWrite(rightMP, yPower);
        digitalWrite(leftMD, motorDirection(yError));     //Left motor turns clockwise if error is positive
        analogWrite(leftMP, yPower);
      }

    } else {
      analogWrite(rightMP, 0);
      analogWrite(leftMP, 0);
    }
    Serial.println(yError); 
    Serial.println(xError);
  } while ((abs(yError)>1) || (abs(xError)>1));
}

void loop() {
  //callibrationMode();
  drawLine(-50,0);
  drawLine(0,-50);
  drawRectangle(100, 50);
}
