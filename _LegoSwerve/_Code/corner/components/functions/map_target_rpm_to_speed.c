#include "map_target_rpm_to_speed.h"
#include "map_speed_to_pulsewidth.h"

#include "esp_log.h"

#include <stdint.h>


float map_target_rpm_to_speed(float target_rpm, int32_t rpm_speed_lookuptable[LUT_SIZE])
{
    // clamp input
    if (target_rpm <= RPM_MIN)
        return (float)rpm_speed_lookuptable[0];

    if (target_rpm >= RPM_MAX)
        return (float)rpm_speed_lookuptable[LUT_SIZE - 1];

    // normalize RPM to index space (0 → 511)
    float norm = (target_rpm - RPM_MIN) * (LUT_SIZE - 1) / (RPM_MAX - RPM_MIN);

    int index = (int)norm;
    float t = norm - index;

    // safety (edge case)
    if (index >= LUT_SIZE - 1)
        return (float)rpm_speed_lookuptable[LUT_SIZE - 1];

    int32_t v0 = rpm_speed_lookuptable[index];
    int32_t v1 = rpm_speed_lookuptable[index + 1];

    // linear interpolation
    return (float)v0 + t * (float)(v1 - v0);
}