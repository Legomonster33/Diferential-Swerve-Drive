#include "esp_sntp.h"
#include "time.h"
#include "driver/ledc.h"
#include "esp32-hal-timer.h"

#define TRUE                1
#define FALSE               0

#define LED_HB_PIN          23
#define LED_RED_PIN         4
#define LED_BLUE_PIN        16
#define LED_GREEN_PIN       17

#define Umotor_FB_PIN       27
#define Umotor_SP_PIN       32
#define Lmotor_FB_PIN       28
#define Lmotor_SP_PIN       33

#define ESC_CHANNEL LEDC_CHANNEL_0

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
  pinMode(LED_RED_PIN,OUTPUT);            // EXECUTING A LEFT TURN
  pinMode(LED_BLUE_PIN,OUTPUT);           // NO TURNS (MOTOR1 SPEED = MOTOR2 SPEED)
  pinMode(LED_GREEN_PIN,OUTPUT);          // EXECUTING A RIGHT TURN

  pinMode(Umotor_PIN,INPUT_PULLUP);  // SPEED FEEDBACK FOR UPPER MOTOR
  pinMode(Lmotor_PIN,INPUT_PULLUP);  // SPEED FEEDBACK FOR LOWER MOTOR

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


// CREATE A TIMER INTERRUPT AS A TIME BASE FOR SPEED FEEDBACK USING PULSES FROM HALL SENSOR

  // Timer initialisation at a frequency of 1 MHz (1 µs per tick)
  timer = timerBegin(1000000);   
  if (timer == NULL) { Serial.println("Error with the start of the timer"); while (1); }
  // Attaches the interrupt function to the timer
  timerAttachInterrupt(timer, &onTimer);
  // Configure an alarm to trigger the interrupt every 1000 ms (1000000 µs)
  timerAlarm(timer, 100000, true, 0);  // 1000000 µs = 1000ms = 1s
  // Start of the timer
  timerStart(timer);

}

void loop() {

  // TRIGGER IS SET BY THE TIMER INTERRUPT
  if (TRIGGER) {
    TRIGGER = FALSE;

    Serial.println("Boom");
    
    digitalWrite (LED_HEARTBEAT_PIN,HEARTBEAT);
    HEARTBEAT = 1 - HEARTBEAT;

  }

/*

  ESC_COMMAND = nuetral;

  ledc_set_duty(
    LEDC_LOW_SPEED_MODE,
    LEDC_CHANNEL_0,
    ESC_COMMAND
    );

  ledc_update_duty(
    LEDC_LOW_SPEED_MODE,
    LEDC_CHANNEL_0
    );

*/
/*
  Serial.println(" nuetral, starting 15s timer, press button now");
*/
/*

  delay(15000);

  Serial.println("starting sweep");

  ESC_COMMAND = full_backward;

  ledc_set_duty(
    LEDC_LOW_SPEED_MODE,
    LEDC_CHANNEL_0,
    ESC_COMMAND
    );

  ledc_update_duty(
    LEDC_LOW_SPEED_MODE,
    LEDC_CHANNEL_0
    );

  delay(2000);

*/
/*

  for (int i = full_backward; i <=full_forward; i+=10){
    //delay(50);
    ESC_COMMAND = i;

    ledc_set_duty(
      LEDC_LOW_SPEED_MODE,
      LEDC_CHANNEL_0,
      ESC_COMMAND
      );

    ledc_update_duty(
      LEDC_LOW_SPEED_MODE,
      LEDC_CHANNEL_0
      );

    Serial.println(i);

  }

*/
/*

  ESC_COMMAND = full_forward;

  ledc_set_duty(
    LEDC_LOW_SPEED_MODE,
    LEDC_CHANNEL_0,
    ESC_COMMAND
    );

  ledc_update_duty(
    LEDC_LOW_SPEED_MODE,
    LEDC_CHANNEL_0
    );

  delay(2000);
  Serial.println("sweep complete,returning to neutral");
*/


}
