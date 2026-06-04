#include "home_controller.h"

/* 回零状态机：保留给后续接限位开关使用，课程通信测试可暂时不依赖它。 */

#include "app_config.h"
#include "home_sensor.h"
#include "stepper_driver.h"

typedef enum {
    HOME_ERR_NONE = 0,
    HOME_ERR_BUSY = 1,
    HOME_ERR_TIMEOUT = 2,
    HOME_ERR_MOVE = 3
} HomeError;

static HomeControllerState s_state;
static uint32_t s_elapsed_ms;
static uint16_t s_debounce_ms;
static uint8_t s_error;
static uint8_t s_backoff_started;

static int32_t axis_search_pps(uint32_t axis)
{
    int32_t dir = axis == 0u ? APP_HOME_AXIS1_DIR : APP_HOME_AXIS2_DIR;
    return dir >= 0 ? APP_HOME_SEARCH_PPS : -APP_HOME_SEARCH_PPS;
}

static int64_t axis_backoff_delta(uint32_t axis)
{
    int32_t dir = axis == 0u ? APP_HOME_AXIS1_DIR : APP_HOME_AXIS2_DIR;
    return dir >= 0 ? -(int64_t)APP_HOME_BACKOFF_PULSE : (int64_t)APP_HOME_BACKOFF_PULSE;
}

static void enter_state(HomeControllerState state)
{
    s_state = state;
    s_elapsed_ms = 0;
    s_debounce_ms = 0;
    s_backoff_started = 0;
}

static bool sensor_active(uint32_t axis)
{
    HomeSensorState state;
    HomeSensor_GetState(&state);
    return axis == 0u ? state.home1_active : state.home2_active;
}

static bool debounce_axis(uint32_t axis)
{
    if (sensor_active(axis)) {
        if (s_debounce_ms >= APP_HOME_DEBOUNCE_MS) {
            return true;
        }
    } else {
        s_debounce_ms = 0;
    }
    return false;
}

static bool start_backoff(uint32_t axis)
{
    StepperState stepper;
    Stepper_GetStateSnapshot(&stepper);
    int64_t p1 = stepper.axis[0].position_pulse;
    int64_t p2 = stepper.axis[1].position_pulse;
    if (axis == 0u) {
        p1 += axis_backoff_delta(0);
    } else {
        p2 += axis_backoff_delta(1);
    }
    return Stepper_MoveAbs(p1, p2, APP_HOME_SEARCH_PPS, APP_HOME_SEARCH_PPS);
}

void HomeController_Init(void)
{
    s_state = HOME_CTRL_IDLE;
    s_elapsed_ms = 0;
    s_debounce_ms = 0;
    s_error = HOME_ERR_NONE;
    s_backoff_started = 0;
}

bool HomeController_Start(void)
{
    if (s_state == HOME_CTRL_DONE || s_state == HOME_CTRL_ERROR) {
        enter_state(HOME_CTRL_IDLE);
        s_error = HOME_ERR_NONE;
    }

    if (Stepper_IsBusy() || s_state == HOME_CTRL_AXIS1_SEARCH || s_state == HOME_CTRL_AXIS1_BACKOFF ||
        s_state == HOME_CTRL_AXIS2_SEARCH || s_state == HOME_CTRL_AXIS2_BACKOFF ||
        s_state == HOME_CTRL_SET_ZERO) {
        s_error = HOME_ERR_BUSY;
        return false;
    }
    Stepper_ClearError();
    Stepper_EnableAll(true);
    Stepper_SetPps(STEPPER_AXIS_1, axis_search_pps(0));
    s_error = HOME_ERR_NONE;
    enter_state(HOME_CTRL_AXIS1_SEARCH);
    return true;
}

void HomeController_Stop(void)
{
    Stepper_StopAll();
    if (s_state != HOME_CTRL_IDLE && s_state != HOME_CTRL_DONE) {
        enter_state(HOME_CTRL_ERROR);
        s_error = HOME_ERR_MOVE;
    }
}

void HomeController_ClearError(void)
{
    if (s_state == HOME_CTRL_ERROR || s_state == HOME_CTRL_DONE) {
        enter_state(HOME_CTRL_IDLE);
    }
    s_error = HOME_ERR_NONE;
}

void HomeController_Loop(void)
{
    if (s_state == HOME_CTRL_IDLE || s_state == HOME_CTRL_DONE || s_state == HOME_CTRL_ERROR) {
        return;
    }

    if (s_elapsed_ms > APP_HOME_TIMEOUT_MS) {
        Stepper_StopAll();
        enter_state(HOME_CTRL_ERROR);
        s_error = HOME_ERR_TIMEOUT;
        return;
    }

    if (s_state == HOME_CTRL_AXIS1_SEARCH) {
        if (debounce_axis(0)) {
            Stepper_Stop(STEPPER_AXIS_1);
            enter_state(HOME_CTRL_AXIS1_BACKOFF);
        }
    } else if (s_state == HOME_CTRL_AXIS1_BACKOFF) {
        if (!s_backoff_started && Stepper_CanAcceptMove()) {
            if (!start_backoff(0)) {
                enter_state(HOME_CTRL_ERROR);
                s_error = HOME_ERR_MOVE;
                return;
            }
            s_backoff_started = 1;
        } else if (s_backoff_started && Stepper_CanAcceptMove()) {
            Stepper_SetPps(STEPPER_AXIS_2, axis_search_pps(1));
            enter_state(HOME_CTRL_AXIS2_SEARCH);
        }
    } else if (s_state == HOME_CTRL_AXIS2_SEARCH) {
        if (debounce_axis(1)) {
            Stepper_Stop(STEPPER_AXIS_2);
            enter_state(HOME_CTRL_AXIS2_BACKOFF);
        }
    } else if (s_state == HOME_CTRL_AXIS2_BACKOFF) {
        if (!s_backoff_started && Stepper_CanAcceptMove()) {
            if (!start_backoff(1)) {
                enter_state(HOME_CTRL_ERROR);
                s_error = HOME_ERR_MOVE;
                return;
            }
            s_backoff_started = 1;
        } else if (s_backoff_started && Stepper_CanAcceptMove()) {
            enter_state(HOME_CTRL_SET_ZERO);
        }
    } else if (s_state == HOME_CTRL_SET_ZERO) {
        Stepper_Zero();
        enter_state(HOME_CTRL_DONE);
    }
}

void HomeController_Tick1kHz(void)
{
    if (s_state != HOME_CTRL_IDLE && s_state != HOME_CTRL_DONE && s_state != HOME_CTRL_ERROR) {
        s_elapsed_ms++;
        if (s_debounce_ms < 0xFFFFu) {
            s_debounce_ms++;
        }
    }
}

HomeControllerState HomeController_GetState(void)
{
    return s_state;
}

uint8_t HomeController_Error(void)
{
    return s_error;
}

const char *HomeController_StateName(HomeControllerState state)
{
    switch (state) {
    case HOME_CTRL_IDLE: return "Idle";
    case HOME_CTRL_AXIS1_SEARCH: return "Axis1Search";
    case HOME_CTRL_AXIS1_BACKOFF: return "Axis1Backoff";
    case HOME_CTRL_AXIS2_SEARCH: return "Axis2Search";
    case HOME_CTRL_AXIS2_BACKOFF: return "Axis2Backoff";
    case HOME_CTRL_SET_ZERO: return "SetZero";
    case HOME_CTRL_DONE: return "Done";
    case HOME_CTRL_ERROR: return "Error";
    default: return "Unknown";
    }
}
