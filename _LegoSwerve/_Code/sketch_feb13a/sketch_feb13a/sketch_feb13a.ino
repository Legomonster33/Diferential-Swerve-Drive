#include "esp32-hal-timer.h"

volatile bool flag = false;  // State of the LED

hw_timer_t *timer = NULL;  // declaration of timer
int c=0; 

// function call by the timer interruption
void IRAM_ATTR onTimer() {
    flag = true;  // Inverser l'état de la LED
  
}

void setup() {
    Serial.begin(9600);
    Serial.println("Start...");

    // Timer initialisation at a frequency of 1 MHz (1 µs per tick)
    timer = timerBegin(1000000);   

    if (timer == NULL) {
        Serial.println("Error with the start of the timer");
        while (1);
    }

    // Attaches the interrupt function to the timer
    timerAttachInterrupt(timer, &onTimer);

    // Configure an alarm to trigger the interrupt every 100 ms (100000 µs)
    timerAlarm(timer, 100000, true, 0);  // 1000000 µs = 100ms = 0.1s

  
    // Start of the timer
    timerStart(timer);
}

void loop() {
  if (flag){
    c+=1;
    Serial.println(c);
    flag=false;
  }

  if (timer == NULL) {
    Serial.println("Erreur : timerBegin() a échoué !");
  }

}

