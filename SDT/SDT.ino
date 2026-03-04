
#include  <stdio.h>
#include  <math.h>

#define pi  3.141592654

#define DT_TRACK_WIDTH    100
#define DT_WHEEL_BASE     100


#define MOD1_SUN_MOTOR_OUT  5
#define MOD1_RING_MOTOR_OUT 6
#define MOD2_SUN_MOTOR_OUT  7
#define MOD2_RING_MOTOR_OUT 8

#define JOYSTICK_MOVE_IN    A0
#define JOYSTICK_STRAFE_IN  A1
#define JOYSTICK_ROTATE_IN  A2

struct joystick{
  float yAXISraw;             // analog input from joystick 1 for forward and reverse
  float xAXISraw;             // analog input from joystick 1 for left and right
  float frREQ;
  float lrREQ;
};

struct  swerveModule {
  float pwSUN;            // Pulse Width from Rotary Encoder for SUN motor
  float rpmSUN;           // RPM for SUN motor;
  float pwRING;           // Pulse Width from Rotaru Encoder for Ring motor
  float rpmRING;          // RPM for RING motor;
  float speedREQ;         // forward speed request for module
  float directionREQ;     // steering angle request for module
};

struct joystick       joy1;
struct joystick       joy2;
struct  swerveModule  swMOD1;
struct  swerveModule  swMOD2;


void  read_joysticks() {
  
  // read and condition information from each of the control joysticks

  bool  j1_FwdRvs;
  bool  j1_LeftRight;
  bool  j2_Rotate_CW;
  bool  j2_ROtate_CCW;
  char  disp[200];

  joy1.yAXISraw = analogRead(JOYSTICK_MOVE_IN);       // raw analog input for Forward/Reverse
  joy1.xAXISraw = analogRead(JOYSTICK_STRAFE_IN);   // raw analog input for STRAFE

  joy1.frREQ = (joy1.yAXISraw/1023)*200;
  joy1.lrREQ = (joy1.xAXISraw/1023)*200;
  // wtf  
  sprintf(disp,"%f %f", joy1.yAXISraw, joy1.xAXISraw);
  Serial.println(disp);

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

  wa1 = atan2(B,C) * 180/pi;
  wa2 = atan2(B,D) * 180/pi;
  wa3 = atan2(A,D) * 180/pi;
  wa4 = atan2(A,C) * 180/pi;

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

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  read_joysticks();
  calc_module_ctrl(joy1.frREQ, joy1.lrREQ, 0, 0);
  delay(200);

}
