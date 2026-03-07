#include "map_speed_to_pulsewidth.h"

#include "esp_log.h"

static const char *TAG = "map_speed_to_pulsewidth:";

uint32_t map_speed_to_pulsewidth(int speed)
{
    uint32_t pulsewidth = CENTER_PULSE;
    
    if (speed == 0) {
        pulsewidth = CENTER_PULSE;
    } 
    
    if (speed < MIN_SPEED || speed > MAX_SPEED) {
        ESP_LOGW(TAG, "Speed should be between %d and %d", MIN_SPEED, MAX_SPEED);
        pulsewidth = CENTER_PULSE;
    }

    if (speed < 0) {
        pulsewidth = (DEADBAND_LOWER+(speed*(DEADBAND_LOWER-MIN_PULSEWIDTH))/-MIN_SPEED);
    } 

    if (speed > 0) {
        pulsewidth = (DEADBAND_UPPER+(speed*(MAX_PULSEWIDTH-DEADBAND_UPPER))/MAX_SPEED);
    }
    
    return pulsewidth;
}