
#include  <avr/interrupt.h>
#include  <avr/io.h>
#include  <stdio.h>
#include  <math.h>
#include  <PID_v1.h>

const int LED_pin = 13; 
volatile byte count;
byte reload = 0x9C; 

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
char  sBuf[80];


// Interrupt routine to count Mod1 Sun Motor Pulses
void mod1sunTACHpulse(){
  swMOD1.sunPULSEcount++;  
}
// Interrupt routine to count Mod1 Ring Motor Pulses
void mod1ringTACHpulse(){
  swMOD1.ringPULSEcount++;  
}

ISR(TIMER2_COMPA_vect)
{
count++;
OCR2A = reload;
}

void setup()
{
Serial.begin(9600);
pinMode(LED_pin, OUTPUT);
digitalWrite(LED_pin, LOW);
cli();
TCCR0B = 0; 
OCR2A = reload; // When Timer2 Counter reaches value 'reload (156)' the interrupt will be triggered
TCCR2A = 1<<WGM21;


TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);
TIMSK2 = (1<<OCIE2A);
sei();
Serial.print("OCR2A: "); 
Serial.println(OCR2A, HEX);
Serial.print("TCCR2A: "); 
Serial.println(TCCR2A, HEX);
Serial.print("TCCR2B: "); 
Serial.println(TCCR2B, HEX);
Serial.print("TIMSK2: "); 
Serial.println(TIMSK2, HEX);
Serial.println("TIMER2 Setup Finished.");

  // Set the PWM pin assignments for sun and ring motors for each module
  swMOD1.sunMOTORpinOUTPUT = 9;
  swMOD1.ringMOTORpinOUTPUT = 10;

  // configure the PWM pins to be output for each spark motor controller
  pinMode (swMOD1.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD1.ringMOTORpinOUTPUT, OUTPUT);

  // Set the pin assignments for the sun and ring motor TACH inputs
  swMOD1.sunTACHpinINPUT = 2;
  swMOD1.ringTACHpinINPUT = 3;
  
  // configure the TACH input pins to have a pull up resistor
  pinMode(swMOD1.sunTACHpinINPUT,INPUT_PULLUP);
  pinMode(swMOD1.ringTACHpinINPUT,INPUT_PULLUP);

  swMOD1.sunPULSEcount = 0;
  swMOD1.ringPULSEcount = 0;

  mod1SUN_PID.SetMode(AUTOMATIC);
  mod1RING_PID.SetMode(AUTOMATIC);

  mod1SUN_PID.SetOutputLimits(PWM_MIN, PWM_MAX);

// assign interrupt routines to each of the TACH inputs
  attachInterrupt(digitalPinToInterrupt(swMOD1.sunTACHpinINPUT),mod1sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(swMOD1.ringTACHpinINPUT),mod1ringTACHpulse,RISING);

}
void loop()
{

  float xVAL;
  unsigned long mS;

  xVAL = analogRead(A0);
  swMOD1.sunSP = (double)((xVAL / 1023.0) *  200.0 ) - 100.0;
  swMOD1.ringSP = (double)((xVAL / 1023.0) * 200.0) - 100.0;

  // Update the motor speed request from the
  // output of the speed PID controller
  
  //  swMOD1.sunMOTORspeedREQ = swMOD1.sunSP;
  //  swMOD1.ringMOTORspeedREQ = swMOD1.ringSP;
  swMOD1.sunMOTORspeedREQ = 0;
  swMOD1.ringMOTORspeedREQ = 0;

  swMOD1.sunMOTORspeedOUT = map(swMOD1.sunMOTORspeedREQ,-100,100,PWM_MIN,PWM_MAX);
  swMOD1.ringMOTORspeedOUT = map(swMOD1.ringMOTORspeedREQ,-100,100,PWM_MIN,PWM_MAX);  

  if (count >= 100) {

    Serial.print(xVAL);
    Serial.print(" ");
    sprintf(sBuf,"Sun SpeedOut %3d RPS %3d, Ring SpeedOut %3d RPS %3d",swMOD1.sunMOTORspeedOUT,swMOD1.sunPULSEcount/2,swMOD1.ringMOTORspeedOUT,swMOD1.ringPULSEcount/2);
    Serial.println(sBuf);

    swMOD1.sunPULSEcount = 0;
    swMOD1.ringPULSEcount = 0;

    flash();
    count = 0;
  }

  // update the PWM outputs for each of the modules motors
  analogWrite(swMOD1.sunMOTORpinOUTPUT,swMOD1.sunMOTORspeedOUT);
  analogWrite(swMOD1.ringMOTORpinOUTPUT,swMOD1.ringMOTORspeedOUT);

}

void flash()
{
static boolean output = HIGH;
digitalWrite(LED_pin, output);
output = !output;
}