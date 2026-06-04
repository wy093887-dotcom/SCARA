#include "home_sensor.h"

/* 限位/回零输入读取：PB0/PB1 默认上拉、低电平触发。 */

#include "app_config.h"
#include "board_pins.h"

static bool read_active(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);
#if APP_HOME_SWITCH_ACTIVE_LEVEL == 0u
    return state == GPIO_PIN_RESET;
#else
    return state == GPIO_PIN_SET;
#endif
}

void HomeSensor_GetState(HomeSensorState *out)
{
    if (out == 0) {
        return;
    }

    out->home1_active = read_active(BOARD_HOME1_PORT, BOARD_HOME1_PIN);
    out->home2_active = read_active(BOARD_HOME2_PORT, BOARD_HOME2_PIN);
    out->active_mask = 0u;
    if (out->home1_active) {
        out->active_mask |= 0x01u;
    }
    if (out->home2_active) {
        out->active_mask |= 0x02u;
    }
}

bool HomeSensor_AllActive(void)
{
    HomeSensorState state;
    HomeSensor_GetState(&state);
    return state.home1_active && state.home2_active;
}
