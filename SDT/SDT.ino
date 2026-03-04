#include  <avr/interrupt.h>
#include  <avr/io.h>
#include  <stdio.h>
#include  <math.h>
#include  <TimerOne.h>

#define pi  3.141592654

#define AI_SPAN           1023
#define JOY_STICK_SPAN    200
#define JOY_STICK_EGL     100

#define DT_TRACK_WIDTH    100
#define DT_WHEEL_BASE     100

#define JOYSTICK_MOVE_IN    A0
#define JOYSTICK_STRAFE_IN  A1
#define JOYSTICK_ROTATE_IN  A2

bool TimePulse;

struct joystick{
  int     yAXISraw;             // analog input from joystick 1 for forward and reverse
  int     xAXISraw;             // analog input from joystick 1 for left and right
  float   yAXISreq;
  float   xAXISreq;
};

struct  swerveModule {
  int             sunENCODERwindows;
  unsigned  int   sunPULSEcount;
  unsigned  long  sunPULSEtime;
  unsigned  long  sunPULSElast;
  unsigned  long  sunRPS;
  int             sunMOTORspeedREQ;
  float           sunMOTORspeedOUT;
  unsigned  int   sunMOTORpinOUTPUT;
  int             ringENCODERwindows;
  unsigned  int   ringPULSEcount;
  unsigned  long  ringPULSEtime;
  unsigned  long  ringPULSElast;
  unsigned  long  ringRPS;
  int             ringMOTORspeedREQ;
  float           ringMOTORspeedOUT;
  unsigned  int   ringMOTORpinOUTPUT;

};

struct            joystick      joy1;
struct            joystick      joy2;
volatile  struct  swerveModule  swMOD1;
volatile  struct  swerveModule  swMOD2;

bool  firstPASS;

const byte  mod1sunTACHpin = 2;
const byte  mod1ringTACHpin = 3;
const byte  mod2sunTACHpin = 18;
const byte  mod2ringTACHpin = 19;

int speedREQsun = 512;
int speedREQring = 512;
int speedINC = 0;

void  read_joysticks() {
  
  // read and condition information from each of the control joysticks

  bool  j1_FwdRvs;
  bool  j1_LeftRight;
  bool  j2_Rotate_CW;
  bool  j2_ROtate_CCW;
  char  disp[200];

  joy1.yAXISraw = analogRead(JOYSTICK_MOVE_IN);       // raw analog input for Forward/Reverse
  joy1.xAXISraw = analogRead(JOYSTICK_STRAFE_IN);   // raw analog input for STRAFE

  joy1.yAXISreq = (float(joy1.yAXISraw)/AI_SPAN) * JOY_STICK_SPAN - JOY_STICK_EGL;
  joy1.xAXISreq = (float(joy1.xAXISraw)/AI_SPAN) * JOY_STICK_SPAN - JOY_STICK_EGL;
}

void  calc_module_ctrl(float FWD, float STR, float RCW, float GYRO) {

  // calculate the speed and angle required for each swerve module

  float L;  // the vehicles wheelbase
  float W;  // the vehicles traackwidth;
  float R;  // ratio of wheelbase to trackwidth

  float ws1,ws2,ws3,ws4;
  float wa1,wa2,wa3,wa4;
  float max_ws;

  float xSTR,xFWD;

  float A,B,C,D;  // intermedary variable to calculate speed and direction

  char  disp[200];

  xFWD = FWD * cos(GYRO) + STR * sin(GYRO);
  xSTR = -1 * FWD * sin(GYRO) + STR * cos(GYRO);

  W = DT_TRACK_WIDTH;
  L = DT_WHEEL_BASE;
  R = sqrt((W*W)+(L*L));

  A = STR - RCW * (L/R);
  B = STR + RCW * (L/R);
  C = FWD - RCW * (W/R);
  D = FWD + RCW * (W/R);

  ws1 = sqrt(B*B + C*C);
  ws2 = sqrt(B*B + D*D);
  ws3 = sqrt(A*A + D*D);
  ws4 = sqrt(A*A + C*C);

  wa1 = atan2(B,C) * 180.0/pi -45;
  wa2 = atan2(B,D) * 180.0/pi -45;
  wa3 = atan2(A,D) * 180.0/pi -45;
  wa4 = atan2(A,C) * 180.0/pi -45;

  sprintf(disp,"FWD:%3d STR:%3d Wheel1(%3d,%3d) Wheel2(%3d,%3d) Wheel3(%3d,%3d) Wheel4(%3d,%3d)",int(FWD),int(STR),int(ws1),int(wa1),int(ws2),int(wa2),int(ws3),int(wa3),int(ws4),int(wa4));
  Serial.println(disp);

  max_ws = ws1;
  if (ws2 > max_ws) max_ws = ws2;
  if (ws3 > max_ws) max_ws = ws3;
  if (ws4 > max_ws) max_ws - ws4;

  if (max_ws > 1){
    ws1 /= max_ws;
    ws2 /= max_ws;
    ws3 /= max_ws;
    ws4 /= max_ws;
  }
}

// This is the interrupt routine executed when a TACH pulse is detected for any of the attached motors

void mod1sunTACHpulse(){
  swMOD1.sunPULSEcount++;  
}
void mod1ringTACHpulse(){
  swMOD1.ringPULSEcount++;  
}
void mod2sunTACHpulse(){
  swMOD2.sunPULSEcount++;  
}
void mod2ringTACHpulse(){
  swMOD2.ringPULSEcount++;  
}

void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  TimePulse = 1;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

int myEraser = 7; // used to reset the timer control bits (0, 1, and 2)
int myPrescaler = 4; // used to select prescaler 4 for a PWM frequency of 120 hz

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(53,INPUT);
  pinMode(30,OUTPUT);
  pinMode(32,OUTPUT);
  pinMode(34,OUTPUT);

  swMOD1.sunMOTORpinOUTPUT = 2;
  swMOD1.ringMOTORpinOUTPUT = 3;
  swMOD2.sunMOTORpinOUTPUT = 5;
  swMOD2.ringMOTORpinOUTPUT = 6;
  
  pinMode (swMOD1.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD1.ringMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.ringMOTORpinOUTPUT, OUTPUT);

  Timer1.initialize(1000000); // set timer for 0.10sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer

  pinMode(mod1sunTACHpin,INPUT_PULLUP);
  pinMode(mod1ringTACHpin,INPUT_PULLUP);
  pinMode(mod2sunTACHpin,INPUT_PULLUP);
  pinMode(mod2ringTACHpin,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(mod1sunTACHpin),mod1sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(mod1ringTACHpin),mod1ringTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(mod2sunTACHpin),mod2sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(mod2ringTACHpin),mod2ringTACHpulse,RISING);

  firstPASS = 1;

  // initialize all tachometer pulse counters

  swMOD1.sunPULSEcount = 0;
  swMOD1.ringPULSEcount = 0;
  swMOD2.sunPULSEcount = 0;
  swMOD2.ringPULSEcount = 0;

  TimePulse = 0;

  // Change timer 1 prescalar which controls the PWN for pins 11 and 12 to be 120hz from default of 490 hz

    TCCR1B &= 0b11111000;
    TCCR1B |= 0b00000011;
    
  // Change timer 2 prescalar which controls the PWN for pins 9 and 10 to be 120hz from default of 490 hz
    TCCR2B &= 0b11111000;
    TCCR2B |= 0b00000011;


Serial.print(TCCR1B);
Serial.print(",");
Serial.print(TCCR2B);
Serial.print(",");
Serial.print(TCCR3B);
Serial.print(",");
Serial.println(TCCR4B);
}



void loop() {
  // put your main code here, to run repeatedly:

  //xVAL = analogRead(A1);


  speedREQsun += (analogRead(A0) - speedREQsun)*.2;
  speedREQring += (analogRead(A1)- speedREQring)*.2;

  swMOD1.sunMOTORspeedREQ = speedREQsun / 1023.0 * 180 + 65;
  swMOD1.ringMOTORspeedREQ = speedREQring / 1023.0 * 180 + 65;
  swMOD2.sunMOTORspeedREQ = speedREQsun / 1023.0 * 180 + 65;
  swMOD2.ringMOTORspeedREQ = speedREQring / 1023.0 * 180 + 65;

  swMOD1.sunMOTORspeedOUT = speedREQsun / 1023.0 * 200.0 - 100.0;
  swMOD1.ringMOTORspeedOUT = speedREQring / 1023.0 * 200.0 - 100.0;
  swMOD2.sunMOTORspeedOUT = speedREQsun / 1023.0 * 200.0 - 100.0;
  swMOD2.ringMOTORspeedOUT = speedREQring / 1023.0 * 200.0 - 100.0;

  analogWrite(swMOD1.sunMOTORpinOUTPUT,swMOD1.sunMOTORspeedREQ);
  analogWrite(swMOD1.ringMOTORpinOUTPUT,swMOD1.ringMOTORspeedREQ);
  analogWrite(swMOD2.sunMOTORpinOUTPUT,swMOD2.sunMOTORspeedREQ);
  analogWrite(swMOD2.ringMOTORpinOUTPUT,swMOD2.ringMOTORspeedREQ);

  if (TimePulse == 1){
    
    swMOD1.sunRPS = swMOD1.sunPULSEcount / 3;
    swMOD1.ringRPS = swMOD1.ringPULSEcount / 3;
    swMOD2.sunRPS = swMOD2.sunPULSEcount / 3;
    swMOD2.ringRPS = swMOD2.ringPULSEcount / 3;

    Serial.print(0);
    Serial.print(",");
    /*
    Serial.print(",module 1 [ SUN, (");
    Serial.print(swMOD1.sunRPS);
    Serial.print(") ");
    Serial.print(swMOD1.sunMOTORspeedREQ);
    Serial.print(" RING, (");
    Serial.print(swMOD1.ringRPS);
    Serial.print(") ");
    Serial.print(swMOD1.ringMOTORspeedREQ);
    Serial.print("] module 2 [ SUN, (");
    Serial.print(swMOD2.sunRPS);
    Serial.print(") ");
    Serial.print(swMOD2.sunMOTORspeedREQ);
    Serial.print(" RING, (");
    Serial.print(swMOD2.ringRPS);
    Serial.print(") ");
    Serial.print(swMOD2.ringMOTORspeedREQ);
    */

  Serial.print(swMOD2.sunPULSEcount);
  Serial.print(",");
  Serial.print(swMOD2.ringPULSEcount);

    Serial.print(",");
    Serial.println(256);
  
    //swMOD1.sunPULSEcount = 0;
    //swMOD1.ringPULSEcount = 0;
    //swMOD2.sunPULSEcount = 0;
    //swMOD2.ringPULSEcount = 0;
    TimePulse = 0;
    
    speedINC++;
  }

  if (speedINC > 30){
    // speedREQ += 50;
    speedINC = 0;
    // if (speedREQ >= 1000) speedREQ = 512;
  }

}

