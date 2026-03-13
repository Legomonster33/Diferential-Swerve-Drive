#include  <avr/interrupt.h>
#include  <avr/io.h>
#include  <stdio.h>
#include  <math.h>
#include  <TimerOne.h>
#include  <TimerThree.h>
#include  <TimerFour.h>
#include  <PID_v1.h>
#include  <LiquidCrystal.h>

LiquidCrystal lcd(7,8,9,10,11,12);

#define AIRAWMIN  0
#define AIRAWMAX  1023
#define PWM_MIN   128
#define PWM_MAX   252
#define PWM_MAN   188

#define TIME_PERIOD 1000000

struct  swerveModule {
  double          sunPV;
  double          sunCV;
  double          sunSP;
  double          sunP;
  double          sunI;
  double          sunD;
  int             sunTACHpinINPUT;
  int             sunPULSEcount;
  int             sunMOTORspeedREQ;
  int             sunMOTORspeedOUT;
  unsigned  int   sunMOTORpinOUTPUT;
  float           sunMOTORspeedRPS;
  double          ringPV;
  double          ringCV;
  double          ringSP;
  double          ringP;
  double          ringI;
  double          ringD;
  int             ringTACHpinINPUT;
  int             ringPULSEcount;
  int             ringMOTORspeedREQ;
  int             ringMOTORspeedOUT;
  unsigned  int   ringMOTORpinOUTPUT;
  float           ringMOTORspeedRPS;
};

volatile  struct  swerveModule  swMOD1;
volatile  struct  swerveModule  swMOD2;

PID mod1SUN_PID(&swMOD1.sunPV, &swMOD1.sunCV, &swMOD1.sunSP, 2.0, .2, 0, DIRECT);
PID mod1RING_PID(&swMOD1.ringPV, &swMOD1.ringCV, &swMOD1.ringSP, swMOD1.ringP, swMOD1.ringI, swMOD1.ringD, DIRECT);

int xVAL;
int yVAL;
int TimePulse;

char  buffer[40];

// This interrupt is triggered once per second
// and causes the current tach pulse counts to
// be converted into a RPM value
void timerONEisr()
{
  Timer1.detachInterrupt();  //stop the timer
  TimePulse = 1;
  Timer1.attachInterrupt( timerONEisr );  //enable the timer
}

// Interrupt routine to count Mod1 Sun Motor Pulses
void mod1sunTACHpulse(){
  swMOD1.sunPULSEcount++;  
}
// Interrupt routine to count Mod1 Ring Motor Pulses
void mod1ringTACHpulse(){
  swMOD1.ringPULSEcount++;  
}
// Interrupt routine to count Mod2 Sun Motor Pulses
void mod2sunTACHpulse(){
  swMOD2.sunPULSEcount++;  
}
// Interrupt routine to count Mod2 Ring Motor Pulses
void mod2ringTACHpulse(){
  swMOD2.ringPULSEcount++;  
}



void setup() {
  //
  // put your setup code here, to run once:
  //

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.

  // Initialize PID and place into AUTOMATIC (on) mode
  // myPID.SetMode(AUTOMATIC);

  // set up the timer interrupt that triggers every 1 second (1000000 ms)
  Timer1.initialize(TIME_PERIOD); // set timer for 1sec
  Timer1.attachInterrupt( timerONEisr ); // enable the timer

  // Set the PWM pin assignments for sun and ring motors for each module
  swMOD1.sunMOTORpinOUTPUT = 5;
  swMOD1.ringMOTORpinOUTPUT = 6;
  swMOD2.sunMOTORpinOUTPUT = 44;
  swMOD2.ringMOTORpinOUTPUT = 45;

  // configure the PWM pins to be output for each spark motor controller
  pinMode (swMOD1.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD1.ringMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.ringMOTORpinOUTPUT, OUTPUT);

  // Set the pin assignments for the sun and ring motor TACH inputs
  swMOD1.sunTACHpinINPUT = 2;
  swMOD1.ringTACHpinINPUT = 3;
  swMOD2.sunTACHpinINPUT = 18;
  swMOD2.ringTACHpinINPUT = 19;

  // configure the TACH input pins to have a pull up resistor
  pinMode(swMOD1.sunTACHpinINPUT,INPUT_PULLUP);
  pinMode(swMOD1.ringTACHpinINPUT,INPUT_PULLUP);
  pinMode(swMOD2.sunTACHpinINPUT,INPUT_PULLUP);
  pinMode(swMOD2.ringTACHpinINPUT,INPUT_PULLUP);

  // assign interrupt routines to each of the TACH inputs
  attachInterrupt(digitalPinToInterrupt(swMOD1.sunTACHpinINPUT),mod1sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(swMOD1.ringTACHpinINPUT),mod1ringTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(swMOD2.sunTACHpinINPUT),mod2sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(swMOD2.ringTACHpinINPUT),mod2ringTACHpulse,RISING);
 
  // Begin a serial communication session with the programming environment
  // for trouble shooting purposes
  Serial.begin(9600);

  swMOD1.sunPULSEcount = 0;
  swMOD1.ringPULSEcount = 0;
  swMOD2.sunPULSEcount = 0;
  swMOD2.ringPULSEcount = 0;

  mod1SUN_PID.SetMode(AUTOMATIC);
  mod1RING_PID.SetMode(AUTOMATIC);

  mod1SUN_PID.SetOutputLimits(PWM_MIN, PWM_MAX);

}

void loop() {


  // put your main code here, to run repeatedly:

  xVAL = (analogRead(A0) * .1) + (xVAL * .9);
  
  Serial.println(xVAL);

  sprintf(buffer,"%4d",xVAL);
  lcd.setCursor(0,1);
  lcd.print(buffer);

  swMOD1.sunSP = (double)((xVAL / 1023.0) *  200.0 ) - 100.0;
  swMOD1.ringSP = (double)((xVAL / 1023.0) * 200.0) - 100.0;

  swMOD1.sunPV = swMOD1.sunPULSEcount / 2;
  swMOD1.ringPV = swMOD1.ringPULSEcount / 2;
  
  mod1SUN_PID.Compute();
  mod1RING_PID.Compute();
  
  // Update the motor speed request from the
  // output of the speed PID controller
  swMOD1.sunMOTORspeedREQ = swMOD1.sunSP;
  swMOD1.ringMOTORspeedREQ = 0;
  swMOD2.sunMOTORspeedREQ = 0;
  swMOD2.ringMOTORspeedREQ = 0;




  //sprintf(buffer,"S:%3d C:%3d P:%3d",(int)swMOD1.sunSP, (int)swMOD1.sunCV, (int)swMOD1.sunPV);
  //lcd.setCursor(0,1);
  //lcd.print(buffer);


  // Calculate the requested speed output
  // This is based on the output from the
  // PID controller which spans from
  // -100 (full reverse) to 100 (full forward)
  
  swMOD1.sunMOTORspeedOUT = map(swMOD1.sunMOTORspeedREQ,-100,100,PWM_MIN,PWM_MAX);
  swMOD1.ringMOTORspeedOUT = map(swMOD1.ringMOTORspeedREQ,-100,100,PWM_MIN,PWM_MAX);
  swMOD2.sunMOTORspeedOUT = map(swMOD2.sunMOTORspeedREQ,-100,100,PWM_MIN,PWM_MAX);
  swMOD2.ringMOTORspeedOUT = map(swMOD2.ringMOTORspeedREQ,-100,100,PWM_MIN,PWM_MAX);

  if (TimePulse){
    
    sprintf(buffer," %3d %3d %3d %3d",swMOD1.sunPULSEcount/2,swMOD1.ringPULSEcount/2,swMOD2.sunPULSEcount/2,swMOD2.ringPULSEcount/2);
    lcd.setCursor(0,0);
    lcd.print(buffer);

    swMOD1.sunPULSEcount = 0;
    swMOD1.ringPULSEcount = 0;
    swMOD2.sunPULSEcount = 0;
    swMOD2.ringPULSEcount = 0;
    

    // clear the time pulse trigger flag    
    TimePulse = 0;

  };

  // update the PWM outputs for each of the modules motors
  analogWrite(swMOD1.sunMOTORpinOUTPUT,swMOD1.sunMOTORspeedOUT);
  analogWrite(swMOD1.ringMOTORpinOUTPUT,swMOD1.ringMOTORspeedOUT);
  analogWrite(swMOD2.sunMOTORpinOUTPUT,swMOD2.sunMOTORspeedOUT);
  analogWrite(swMOD2.ringMOTORpinOUTPUT,swMOD2.ringMOTORspeedOUT);

}
