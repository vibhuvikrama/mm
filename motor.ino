//mm

#include<NewPing.h>
#include<FastLED.h>	///

#define num_leds_l 6
#define data_pin /// *****************
CRGB leds[num_leds]; 

///define digital pins **********************
//#define tpl  
//#define epl  
//#define tpf  
//#define epf  
//#define tpr  
//#define epr  
//#define max_dis 
int dir;	///

#define stop 0
#define forward 1
#define backward 2
#define left 3
#define right 4

float P=0.65;
float I=0.45;
float D=0.55;
float oldErrP;
float totalErr;
int offset=5;	///

/// ********************************
//int wall_threshold=;
//int front_threshold=;

///
boolean frontwall;
boolean leftwall;
boolean rightwall;
boolean first_turn;
boolean rightWallFollow;
boolean leftWallFollow;

//int lmf=;	int lmb=;
//int rmf=;	int rmb=;
//int enA=;	int enB=;

//int baseSpeed=;	///************************************

///
int RMS;
int LMS;

NewPing sonarLeft(tpl, epl, max_dis);
NewPing sonarRight(tpr, epr, max_dis);
NewPing sonarFront(tpf, epf, max_dis);

unsigned int pingSpeed=30; ///
unsigned long pingTimer;   ///

float oldLeftSensor, oldRightSensor, oldFrontSensor, leftSensor, rightSensor, frontSensor, lSensor, rSensor, fSensor;	///

void setup() 
{
  Serial.begin(115200); 
  ///
  
  FastLED.addLeds<WS2812B, data_pin, RGB>(leds, num_leds);
  
  first_turn=false;
  rightWallFollow=false;
  leftWallFollow=false;
}

void loop() 
{
  ReadSensors();
  walls();
  if ( first_turn == false ) 
  {
    pid_start();
  }
  else if (leftWallFollow == true ) 
  {
    PID(true);
  }
  else if(rightWallFollow==true) 
  {
    PID(false) ;
  }
  if (leftwall == true && rightwall == false && frontwall == true ) {
    PID(false) ;
    if ( first_turn == false ) 
	{
	  first_turn = true ;
      rightWallFollow = true ;
    }
  }
   if (leftwall == false && rightwall == true && frontwall == true ) {
    PID(true) ;
    if ( first_turn == false ) 
	{
    first_turn = true ;
    leftWallFollow = true ;
    digitalWrite(LED , HIGH);
    }
  }
   if ( leftSensor == 0 || leftSensor > 100 && rightSensor == 0 || rightSensor > 100 && frontSensor == 0 || frontSensor > 100 ) 
   {
    setDirection(stop);
   }
  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");
  Serial.print("error=");
  Serial.println(totalError);
}

void setDirection(int dir) 
{
  FastLED.clear();
  if(dir==forward) 
  {
  	for(int i=0;i<=5;i++)
    {
    	leds[i]=CRGB(GREEN);
	}
	FastLED.show();
    
	digitalWrite(lmb, LOW);   
    digitalWrite(lmf, HIGH);
    digitalWrite(rmb, LOW);
    digitalWrite(rmf, HIGH);
    ///FastLED.h
    }
  else if(dir==left)
  {
   	FastLED.clear();
  	for(int i=0;i<3;i++)
    {
    	leds[i]=CRGB(yellow);
    	delay(50);
	}
	FastLED.show();
    
	digitalWrite(lmb, HIGH);
    digitalWrite(lmf, LOW );
    digitalWrite(rmb, LOW );
    digitalWrite(rmf, HIGH);
  }
  else if(dir==right) 
  {
   	FastLED.clear();
  	for(int i=3;i<6;i++)
    {
    	leds[i]=CRGB(yellow);
    	delay(50);
	}
	FastLED.show();
	
    digitalWrite(lmb, LOW); 
    digitalWrite(lmf, HIGH);
    digitalWrite(rmb, HIGH);
    digitalWrite(rmf, LOW);
  }
  else if (dir==stop) 
  {
   	FastLED.clear();
  	for(int i=0;i<6;i++)
    {
    	leds[i]=CRGB(red);
    	delay(50);
	}
	FastLED.show();
	
    digitalWrite(lmb, LOW); 
    digitalWrite(lmf, LOW);
    digitalWrite(rmf, LOW );
    digitalWrite(rmb, LOW);
  }
  else if ( dir == backward )
   ///leds
   {
    digitalWrite(lmb, HIGH );
    digitalWrite(lmf, LOW );
    digitalWrite(rmb, HIGH );  // Right wheel forward
    digitalWrite(rmf, LOW );
  }
}

void ReadSensors() 
{
  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();
  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;
  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;
}

void pid_start() 
{
  float errorP = leftSensor - rightSensor ;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * errorI + errorP ;
  totalError = P * errorP + D * errorD + I * errorI ;
  oldErrorP = errorP ;
  RMS = baseSpeed + totalError ;
  LMS = baseSpeed - totalError;
  if (RMS < 0) 
  {
    RMS = map(RMS , 0 , -255, 0, 255);
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
    setDirection(RIGHT);
  }
  else if (LMS < 0) 
  {
    LMS = map(LMS , 0 , -255, 0, 255);
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
    setDirection(LEFT);
  }
  else 
  {
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
    setDirection(FORWARD);
  }
}
void PID( boolean left ) 
{
  if (left == true ) 
  {
    float errorP = leftSensor - rightSensor - offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;
    totalError = P * errorP + D * errorD + I * errorI ;
    oldErrorP = errorP ;
    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;
    if (RMS < 0) 
	{
      RMS = map(RMS , 0 , -255, 0, 255);
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(RIGHT);
    }
    else if (LMS < 0) 
	{
      LMS = map(LMS , 0 , -255, 0, 255);
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(LEFT);
    }
    else 
	{
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(FORWARD);
    }
  }
  else 
  {
    float errorP = leftSensor - rightSensor + offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;
    totalError = P * errorP + D * errorD + I * errorI ;
    oldErrorP = errorP ;
    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;
    if (RMS < 0) 
	{
      RMS = map(RMS , 0 , -255, 0, 255);
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(RIGHT);
    }
    else if (LMS < 0) 
	{
      LMS = map(LMS , 0 , -255, 0, 255);
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(LEFT);
    }
    else 
	{
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(FORWARD);
    }
  }
}

void walls() 
{
  if(leftSensor<wall_threshold) 
    leftwall=true ;
  else
    leftwall=false;
  if(rightSensor<wall_threshold) 
    rightwall = true ;
  else 
    rightwall = false ;
  if (frontSensor<front_threshold)
    frontwall=true ;
  else 
    frontwall=false;
}

void turnright() 
{
  LMS=baseSpeed;
  RMS=LMS*rightSensor/(rightSensor+11);
}
void turnleft() 
{
  RMS=baseSpeed;
  LMS=RMS*leftSensor/(leftSensor+11);	///
}
