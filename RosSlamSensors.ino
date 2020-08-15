/*
  Sensor+ test for ros slam build
*/
#include <Encoder.h>

#define RADIUS 25.0 // wheel radius in mm
#define LENGTH 175.0 // wheel base length in mm
#define TICKS_PER_REV 240
  
Encoder rightMtrEnc(2, 11);
Encoder leftMtrEnc(3, 12);

int leftbumper = 9;
int rightbumper = 8;
int leftprox = A2;
int rightprox = A3;
const int trigPin = A0; 
const int echoPin = A1;


long leftcurpos  = 0; // encoder pulses.
long rightcurpos  = 0; // encoder pulses.
float angle = 0; // angle fron start in encoder pulses
double realposX = 0; // calculated pos from encoders
double realposY = 0; // calculated pos from encoders

float duration, distance;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(leftbumper, INPUT_PULLUP);
  pinMode(rightbumper, INPUT_PULLUP);
   pinMode(trigPin, OUTPUT); 
 pinMode(echoPin, INPUT); 
 // RomeoV1.3 HBridge driver
  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);
}

void loop() {
  // read the digital input pins:
  int leftbumper1 = digitalRead(leftbumper);
  int rightbumper1 = digitalRead(rightbumper);
  int leftprox1 = digitalRead(leftprox);
  int rightprox1 = digitalRead(rightprox);
  // print out the state of the button:
  if (leftbumper1 == HIGH) {
  Serial.print("leftbumper ");
  }
    if (rightbumper1 == HIGH) {
  Serial.print("rightbumper ");
  }
    if (leftprox1 == LOW) {
  Serial.print("leftprox ");
  }
    if (rightprox1 == LOW) {
  Serial.print("rightprox ");
  }
 
  // Read the sonar
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

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

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  if (distance < 2000 && distance != 0) {
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(29);
  }else {
     Serial.println();
  }
}
