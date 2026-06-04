#ifndef HOME_SENSOR_H
#define HOME_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool home1_active;
    bool home2_active;
    uint8_t active_mask;
} HomeSensorState;

void HomeSensor_GetState(HomeSensorState *out);
bool HomeSensor_AllActive(void);

#endif
