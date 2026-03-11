#include "map_target_rpm_to_speed.h"
#include "map_speed_to_pulsewidth.h"

#include "esp_log.h"

float map_target_rpm_to_speed(float target_rpm){
    return (target_rpm >= 0.0f) ? (target_rpm / MAX_RPM) * MAX_SPEED : (target_rpm / MIN_RPM) * MIN_SPEED;
}