#include <Encoder.h>

#define RADIUS 25.0 // wheel radius in mm
#define LENGTH 175.0 // wheel base length in mm
#define TICKS_PER_REV 240

// TIME INTERVALS
#define SEND_INTERVAL 100 // milliseconds

#define trigPin 13 // sonar
#define echoPin 12 // sonar
  
Encoder rightMtrEnc(2, 8);
Encoder leftMtrEnc(3, 9);

// Number of left and right tick counts on the encoder.
// volatile unsigned int leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;


//DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);

double wheelcur = 157.0; // 157 mm
double ppr = 240.0*4;

//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control

///For previous Romeo, please use these pins.
//int E1 = 6;     //M1 Speed Control
//int E2 = 9;     //M2 Speed Control
//int M1 = 7;    //M1 Direction Control
//int M2 = 8;    //M1 Direction Control


void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);
}
void advance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void back_off (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void setup(void)
{
  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);
  Serial.begin(19200);      //Set Baud Rate
  Serial.println("Run keyboard control");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

long leftcurpos  = 0; // in encoder pulses.
long rightcurpos  = 0; // in encoder pulses.
float angle = 0;
double realposX = 0;
double realposY = 0;

int state = 0;

void loop(void)
{
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  stop();
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1; // sonar

  if (distance >= 200 || distance <= 0){
    // Serial.println("Out of range");
  } else if (distance > 10){
  //   Serial.print("go forward ");
  //   Serial.print(distance);
  //   Serial.println("mm");
} else {
   // Serial.print(distance);
   // Serial.println(" cm");
  }

  double deltaleft = 0;
  long newPos = -leftMtrEnc.read();
  if (newPos != leftcurpos) {
    deltaleft = ((newPos - leftcurpos)/(4.0 *TICKS_PER_REV)*RADIUS*2.0*PI);
    leftcurpos = newPos;
   // leftTicks+=abs(deltaleft);
  }
  float deltaright = 0;
  newPos = rightMtrEnc.read();
  if (newPos != rightcurpos) {
    deltaright = ((newPos - rightcurpos)/(4.0 *TICKS_PER_REV)*RADIUS*2.0*PI);
    rightcurpos = newPos;
   // rightTicks+=abs(deltaright);
  }
  double dcenter = (deltaleft+deltaright)/2;
  angle += (deltaright-deltaleft)/LENGTH;
  while (angle < 0) {
    angle += 2 * PI;
  }
  while (angle > 2 * PI) {
    angle -= 2 * PI;
  }
  realposX += dcenter*cos(angle);
  realposY += dcenter*sin(angle);
  Serial.print("X = (");
  Serial.print(realposX);
  Serial.print("mm) Y = (");
  Serial.print(realposY);
  Serial.print("mm) Angle (");
  Serial.print(angle*180/PI);
  Serial.println(")");
  if (state == 0) {
 
    if (angle > .03 && angle <= PI){
      Serial.println("right");
       turn_L(60,120);
       delay(10);
    } else if (angle > PI && angle <= (2*PI) - .03) {
      // turn left
      Serial.println("left");
      turn_R(120,60);
      delay(10);
    } else {
      Serial.println("straight");
      delay(100);
    }
    
  } else if (state == 1) {
  } else if (state == 2) {
  } else if (state == 3) {
  } else if (state == 4) {
  } else if (state == 5) {
  } else if (state == 6) {
  }
/*
 Serial.print("left (");
 Serial.print(deltaright);
 Serial.print("mm) right(");
 Serial.print(rightcurpos/(4.0* TICKS_PER_REV))*RADIUS*2*PI;
 Serial.println("mm)");

 */
  /* if (millis() - prevSendTime > SEND_INTERVAL) {
    deadReckoner.computePosition();
    // Cartesian coordinate of latest location estimate.
    // Length unit correspond to the one specified under MEASUREMENTS.
    double x = deadReckoner.getX();
    double y = deadReckoner.getY();

    // Left and right angular velocities.
    double wl = deadReckoner.getWl();
    double wr = deadReckoner.getWr();

    // getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
    // This angle is set initially at zero before the robot starts moving.
    double theta = deadReckoner.getTheta();

    // Total distance robot has troubled.
    
    double distance = sqrt(x * x + y * y);
    Serial.print("x: "); Serial.print(x);
    Serial.print("\ty: "); Serial.print(y);
    Serial.print("\twl: "); Serial.print(wl);
    Serial.print("\twr: "); Serial.print(wr);
    Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
    Serial.print("\tdist: "); Serial.println(distance);
    
  prevSendTime = millis();
  }
  */
/*
  Serial.print("x(");
  Serial.print(realposX);
  Serial.print("mm) u(");
  Serial.print(realposY);
  Serial.println("mm)");
  */
  /*
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        advance (255,255);   //move forward in max speed
        break;
      case 's'://Move Backward
        back_off (255,255);   //move back in max speed
        break;
      case 'a'://Turn Left
        turn_L (100,100);
        break;
      case 'd'://Turn Right
        turn_R (100,100);
        break;
      case 'z':
        Serial.println("Hello");
        break;
      case 'x':
        stop();
        break;
      }
    }
    else stop();
  }
  */
}
