#ifndef HOME_CONTROLLER_H
#define HOME_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HOME_CTRL_IDLE = 0,
    HOME_CTRL_AXIS1_SEARCH,
    HOME_CTRL_AXIS1_RETURN,
    HOME_CTRL_AXIS2_SEARCH,
    HOME_CTRL_AXIS2_RETURN,
    HOME_CTRL_DONE,
    HOME_CTRL_ERROR
} HomeControllerState;

void HomeController_Init(void);
bool HomeController_Start(bool simulated);
void HomeController_Stop(void);
void HomeController_ClearError(void);
void HomeController_Loop(void);
void HomeController_Tick1kHz(void);
HomeControllerState HomeController_GetState(void);
uint8_t HomeController_Error(void);
const char *HomeController_StateName(HomeControllerState state);

#endif
