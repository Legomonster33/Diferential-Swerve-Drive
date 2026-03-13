#include "map_speed_to_pulsewidth.h"

#include "esp_log.h"

static const char *TAG = "map_speed_to_pulsewidth:";

uint32_t map_speed_to_pulsewidth(float speed)
{
    uint32_t pulsewidth = CENTER_PULSE;
    
    if (speed == 0) {
        pulsewidth = CENTER_PULSE;
    } 
    
    if (speed < MIN_SPEED*1.5 || speed > MAX_SPEED*1.5) {
        ESP_LOGW(TAG, "Speed should be between %d and %d", MIN_SPEED*1.5, MAX_SPEED*1.5);
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