#include "map_target_rpm_to_speed.h"
#include "map_speed_to_pulsewidth.h"

#include "esp_log.h"

float map_target_rpm_to_speed(float target_rpm, int max_rpm, int min_rpm){
    return (target_rpm >= 0.0f) ? (target_rpm / max_rpm) * MAX_SPEED : (target_rpm / min_rpm) * MIN_SPEED;
}