#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


float AvgMicrosPerPulse[12],
MicrosPerRevolution[12],
MotorOutPut[12],
PotVal[12],
Rpm[12],
Rpm1[12],
Rpm2[12],
TachoDeltaTimePrevious[12],
MicrosPerPulse[12],
TachoTimer[12],
TachoValue[12],
OldTachoValue[12],
TachoDeltaTime[12],
ControlValue[12],
ProcessValue[12],
SetPoint[12],
P[12],
I[12],
D[12],
MaxSummativeError[12],
Error[12],
LastError[12],
ErrorSummative[12],
PreviousTime[12],
DeltaTime[12],
MaxControl[12],
Proportional,
Integral,
Derivative;

void PID(int Idx){
  if(MotorOutPut[Idx]<160){ProcessValue[Idx]=ProcessValue[Idx]*(-1.0);}
 Error[Idx] = SetPoint[Idx] - ProcessValue[Idx];
  Proportional = P[Idx]*Error[Idx];

  ErrorSummative[Idx] = ErrorSummative[Idx] + Error[Idx];
  if(ErrorSummative[Idx]>MaxSummativeError[Idx]){ErrorSummative[Idx]=MaxSummativeError[Idx];}
  if(ErrorSummative[Idx]<MaxSummativeError[Idx]*-1.0){ErrorSummative[Idx]=MaxSummativeError[Idx]*-1.0;}

  DeltaTime[Idx] = micros() - PreviousTime[Idx];

  Integral = DeltaTime[Idx]*I[Idx]*ErrorSummative[Idx];

  Derivative = ((D[Idx]/DeltaTime[Idx])/(Error[Idx]-LastError[Idx]));

  ControlValue[Idx] = 1.0*(Proportional + Integral + Derivative);
  if(ControlValue[Idx]>MaxControl[Idx]){ControlValue[Idx]=MaxControl[Idx];}
  if(ControlValue[Idx]<-1.0*MaxControl[Idx]){ControlValue[Idx]=-1.0*MaxControl[Idx];}
}


#define PulsesPerRevolution 1.0
#define MicrosecondsPerMinute 60000000.0



void ReadRpm(int Idx){
  
  analogWrite (9,245);
  
  TachoValue[Idx] = digitalRead(Idx+30);
  //Serial.print(TachoValue[Idx]);
  
  if(TachoValue[Idx] != OldTachoValue[Idx] && TachoValue[Idx]==HIGH){

    TachoDeltaTime[Idx] = micros()-TachoTimer[Idx]; 
    
    if (TachoDeltaTime[Idx] < 10) {
      lcd.setCursor(0,0);
      lcd.print("    ");
      lcd.setCursor(4,0);
      lcd.print(TachoDeltaTime[Idx]);
    }
    if (TachoDeltaTime[Idx] < 100 && TachoDeltaTime[Idx] >= 10) {
      lcd.setCursor(0,0);
      lcd.print("   ");
      lcd.setCursor(3,0);
      lcd.print(TachoDeltaTime[Idx]);
    }
    if (TachoDeltaTime[Idx] < 1000 && TachoDeltaTime[Idx] >= 100) {
      lcd.setCursor(0,0);
      lcd.print("  ");
      lcd.setCursor(2,0);
      lcd.print(TachoDeltaTime[Idx]);
    }
    if (TachoDeltaTime[Idx] < 10000 && TachoDeltaTime[Idx] >= 1000) {
      lcd.setCursor(0,0);
      lcd.print(" ");
      lcd.setCursor(1,0);
      lcd.print(TachoDeltaTime[Idx]);
    }
    if (TachoDeltaTime[Idx] >= 10000) {
      lcd.setCursor(0,0);
      lcd.print(TachoDeltaTime[Idx]);
    }

    lcd.println(TachoDeltaTime[Idx]);

    AvgMicrosPerPulse[Idx] = TachoDeltaTime[Idx]*0.5+TachoDeltaTimePrevious[Idx]*0.5;
    //micros per pulse
    //Serial.println(AvgMicrosPerPulse[Idx]);

    MicrosPerRevolution[Idx] = AvgMicrosPerPulse[Idx]*PulsesPerRevolution;
    //micros per rotation

    Rpm1[Idx] = MicrosecondsPerMinute/MicrosPerRevolution[Idx];
    // divide how many microseconds per minute by how many microseconds per revo to get rotations per minute

    Rpm[Idx] = Rpm1[Idx]*0.5+Rpm2[Idx]*0.5;
    //average last and current rpm
    
    Rpm2[Idx] = Rpm1[Idx] ;
        
    
    TachoDeltaTimePrevious[Idx] = TachoDeltaTime[Idx];
   
    TachoTimer[Idx] = micros();

    //lcd.clear();
    //lcd.print(Rpm[Idx]);
  }
  OldTachoValue[Idx] = TachoValue[Idx];
}

void ReadPot(int Pin,int Idx){
  PotVal[Idx] = PotVal[Idx]*0.99+0.01*((((analogRead(Pin)/512.0)-1.0)));
}

void MotorOutPutCalc(int Idx){
  MotorOutPut[Idx] = MotorOutPut[Idx] + MotorOutPut[Idx]*ControlValue[Idx];
  if(MotorOutPut[Idx]>250){MotorOutPut[Idx]=250.0;}
  if(MotorOutPut[Idx]<70){MotorOutPut[Idx]=70.0;} 
}

void PotRead(int PinNum,int Idx){
  PotVal[Idx] = PotVal[Idx]*0.999+0.001*(((analogRead(PinNum)-512.0)/512.0));
}

void PrintAll(int Idx){

  Serial.print("SetPoint_: ");
  Serial.print(SetPoint[Idx]);
  Serial.print("\t");
  
  Serial.print("ProcessValue_: ");
  Serial.print(ProcessValue[Idx]);
  Serial.print("\t");

  Serial.print("ControlValue_: ");
  Serial.print(ControlValue[Idx]);
  Serial.print("\t");

  Serial.print("MotorOutPut_: ");
  Serial.print(MotorOutPut[Idx]);
  Serial.print("\t");

  Serial.print("Error_: ");
  Serial.print(Error[Idx]);
  Serial.print("\t");

  Serial.print("Rpm_: ");
  Serial.print(Rpm[Idx]);
  Serial.print("\t");

  Serial.print("TargetRpm_: ");
  Serial.print(SetPoint[Idx]*5000.0);
  Serial.print("\t");  

  Serial.println();
}
// do it
void PrintToLCD(int Idx){
  lcd.clear();
  lcd.print(Rpm[Idx]);
}

void setup() {

  lcd.begin(16, 2);
  //lcd.print("hello, world!");
  
  // put your setup code here, to run once:
  //Serial.begin(500000);
  MotorOutPut[0]=160;
  P[0]=0.005;
  I[0]=0.0;
  D[0]=0.0;
  MaxSummativeError[0] = 1.0;
  MaxControl[0]=1.0;
  
}
int LastMillis;
void loop() {

  ReadRpm(0);
/*  
  ReadPot(A0,0);
  MotorOutPutCalc(0);
  PotRead(A0,0);
  analogWrite(9, MotorOutPut[0]);
*/
  //analogWrite(2, 200);
/*
  ProcessValue[0] = Rpm[0]/6000.0;
  SetPoint[0]=PotVal[0];
  PID(0);
*/
  //PrintAll(0);
/*
  if(millis()-LastMillis>2000){

  /*lcd.clear();
  lcd.print(SetPoint[0]);
  lcd.setCursor(0, 1);
  lcd.print(ProcessValue[0]);
  lcd.setCursor(7, 0);
  lcd.print(100*ControlValue[0]);
  lcd.setCursor(7, 1);
  lcd.print(Rpm[0]);
  LastMillis = millis();
  }
*/
}