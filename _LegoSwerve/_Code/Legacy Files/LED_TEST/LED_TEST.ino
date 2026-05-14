#include "esp_sntp.h"
#include "time.h"
#include "driver/ledc.h"
#include "esp32-hal-timer.h"

#define TRUE                1
#define FALSE               0

#define LED_HB_PIN          23
#define LED_GREEN_PIN       16
#define LED_RED_PIN         17
#define LED_BLUE_PIN        4

#define Umotor_FB_PIN       34
#define Umotor_SP_PIN       32
#define Lmotor_FB_PIN       35
#define Lmotor_SP_PIN       33

#define ESC_CHANNEL LEDC_CHANNEL_0


int   LED_SELECTOR = 1;
float UmotorSPEED = 0.0;
float LmotorSPEED = 0.0;


const int PWM_CHANNEL_0 = 0;
const int PWM_CHANNEL_1 = 1;
const int PWM_FREQ = 60;
const int PWM_RESOLUTION = 12;

// THe max duty cycle value base on PWM resolution
const int MAX_DUTY_CYCLE = (int)(pow(2,PWM_RESOLUTION) - 1);

float PWM_dutyCycle = 0.0;    // 0 - 100%
float PWM_dutyCycleDir = 1.0; // +1.0 or -1.0


const int DELAY_MS = 4;

volatile bool TRIGGER = false;  // State of the LED
hw_timer_t *timer = NULL;  // declaration of timer

int HEARTBEAT = 0;
int DIRECTION_LED = 1;

int LED15_STATUS = 0;
int LED14_STATUS = 0;
int ESC_COMMAND = 0;
int HALL21_STATUS = 0;

int i = 0;
volatile bool flag = false;

int full_forward = 10500;
int full_backward = 2300;
int nuetral = (full_forward+full_backward)/2;


// function call by the timer interruption
void IRAM_ATTR onTimer() { TRIGGER = TRUE; }

void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);

  // SET UP THE 4 STATUS LEDs AS OUTPUTS
  pinMode(LED_HB_PIN,OUTPUT);      // OVERALL HEALTH OF CONTROLLER

//  pinMode(LED_GREEN_PIN, OUTPUT);
//  pinMode(LED_RED_PIN, OUTPUT);
//  pinMode(LED_BLUE_PIN, OUTPUT);


  pinMode(Umotor_FB_PIN,INPUT_PULLUP);  // SPEED FEEDBACK FOR UPPER MOTOR
  pinMode(Lmotor_FB_PIN,INPUT_PULLUP);  // SPEED FEEDBACK FOR LOWER MOTOR

/*
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_16_BIT,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = 50,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);
  // 2. Configure the Channel
  ledc_channel_config_t ledc_channel = {
    .gpio_num       = MOTOR1_SPEED_REQ_PIN,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = ESC_CHANNEL,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 128,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel);
*/

//  Attach the GPIO to the Channel
  ledcAttachChannel (LED_RED_PIN,  PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_0);
  ledcAttachChannel (LED_GREEN_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_1);

// CREATE A TIMER INTERRUPT AS A TIME BASE FOR SPEED FEEDBACK USING PULSES FROM HALL SENSOR

  // Timer initialisation at a frequency of 1 MHz (1 µs per tick)
  timer = timerBegin(1000000);   
  if (timer == NULL) { Serial.println("Error with the start of the timer"); while (1); }
  // Attaches the interrupt function to the timer
  timerAttachInterrupt(timer, &onTimer);
  // Configure an alarm to trigger the interrupt every 1000 ms (1000000 µs)
  timerAlarm(timer, 10000, TRUE, 0);  // 1000000 µs = 1000ms = 1s
  // Start of the timer
  timerStart(timer);
}

void loop() {

  // TRIGGER IS SET BY THE TIMER INTERRUPT (100ms tick)
  if (TRIGGER) {
    TRIGGER = FALSE;

    digitalWrite (LED_HB_PIN,HEARTBEAT);
    HEARTBEAT = 1 - HEARTBEAT;

    PWM_dutyCycle = PWM_dutyCycle + PWM_dutyCycleDir;
    if ((PWM_dutyCycle > 99) or (PWM_dutyCycle < 1)) {
      PWM_dutyCycleDir = PWM_dutyCycleDir * -1;
    }

  Serial.print(MAX_DUTY_CYCLE);
  Serial.print(" - ");
  Serial.println(PWM_dutyCycle);

  ledcWriteChannel(PWM_CHANNEL_0, (PWM_dutyCycle/100*MAX_DUTY_CYCLE));
  ledcWriteChannel(PWM_CHANNEL_1, ((PWM_dutyCycle)/100*MAX_DUTY_CYCLE));

  }

}
