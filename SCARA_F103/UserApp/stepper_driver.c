#include "stepper_driver.h"

/* 步进底层：将规划好的目标脉冲和 pps 转成 PWM 输出，并维护软件脉冲计数。 */

#include "app_config.h"
#include "app_params.h"
#include "board_pins.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
    TIM_HandleTypeDef *tim;
    uint32_t channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *ena_port;
    uint16_t ena_pin;
    int32_t pulse_accum;
    int32_t applied_pps;
} StepperHw;

static StepperState s_state;
static StepperHw s_hw[2] = {
    {BOARD_M1_TIM, BOARD_M1_TIM_CHANNEL, BOARD_M1_DIR_PORT, BOARD_M1_DIR_PIN, BOARD_M1_ENA_PORT, BOARD_M1_ENA_PIN, 0, 0},
    {BOARD_M2_TIM, BOARD_M2_TIM_CHANNEL, BOARD_M2_DIR_PORT, BOARD_M2_DIR_PIN, BOARD_M2_ENA_PORT, BOARD_M2_ENA_PIN, 0, 0},
};

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int64_t i64_abs(int64_t value)
{
    return value < 0 ? -value : value;
}

static bool axis_valid(StepperAxis axis)
{
    return axis == STEPPER_AXIS_1 || axis == STEPPER_AXIS_2;
}

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if (primask == 0u) {
        __enable_irq();
    }
}

static void pwm_apply(uint32_t index, int32_t pps)
{
    /* pps 低于最小有效值或轴未使能时关闭 PWM，避免空闲状态继续输出脉冲。 */
    TIM_HandleTypeDef *tim = s_hw[index].tim;
    uint32_t channel = s_hw[index].channel;
    int32_t abs_pps = abs(pps);

    if (abs_pps < APP_MIN_EFFECTIVE_PPS || !s_state.axis[index].enabled) {
        if (s_hw[index].applied_pps != 0) {
            __HAL_TIM_SET_COMPARE(tim, channel, 0);
            s_hw[index].applied_pps = 0;
        }
        s_state.axis[index].running = false;
        return;
    }

    abs_pps = clamp_i32(abs_pps, APP_MIN_EFFECTIVE_PPS, s_state.axis[index].max_pps);
    pps = pps < 0 ? -abs_pps : abs_pps;
    if (s_hw[index].applied_pps == pps) {
        s_state.axis[index].running = true;
        return;
    }
    uint32_t arr = (APP_STEPPER_TIMER_HZ / (uint32_t)abs_pps);
    if (arr == 0u) {
        arr = 1u;
    }
    arr -= 1u;

    __HAL_TIM_SET_AUTORELOAD(tim, arr);
    __HAL_TIM_SET_COMPARE(tim, channel, (arr + 1u) / 2u);
    __HAL_TIM_SET_COUNTER(tim, 0u);
    s_hw[index].applied_pps = pps;
    s_state.axis[index].running = true;
}

static void set_dir_index(uint32_t index, int8_t dir)
{
    if (dir == 0) {
        return;
    }
    s_state.axis[index].dir = dir > 0 ? 1 : -1;
    HAL_GPIO_WritePin(s_hw[index].dir_port, s_hw[index].dir_pin,
                      s_state.axis[index].dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void axis_set_target_pps(uint32_t index, int32_t pps)
{
    StepperAxisState *axis = &s_state.axis[index];

    if (pps == 0) {
        axis->target_pps = 0;
        return;
    }

    pps = clamp_i32(pps, -axis->max_pps, axis->max_pps);
    set_dir_index(index, pps > 0 ? 1 : -1);
    axis->target_pps = pps;
}

static void axis_stop_now(uint32_t index)
{
    StepperAxisState *axis = &s_state.axis[index];
    axis->current_pps = 0;
    axis->target_pps = 0;
    axis->exit_pps = 0;
    axis->remaining_pulse = 0;
    axis->mode = STEPPER_MODE_IDLE;
    pwm_apply(index, 0);
}

void Stepper_Init(void)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        s_state.axis[i].enabled = false;
        s_state.axis[i].running = false;
        s_state.axis[i].dir = 1;
        s_state.axis[i].mode = STEPPER_MODE_IDLE;
        s_state.axis[i].current_pps = 0;
        s_state.axis[i].target_pps = 0;
        s_state.axis[i].exit_pps = 0;
        s_state.axis[i].accel_pps_s = APP_ACCEL_DEFAULT;
        s_state.axis[i].max_pps = APP_MAX_PPS_DEFAULT;
        s_state.axis[i].position_pulse = 0;
        s_state.axis[i].target_position_pulse = 0;
        s_state.axis[i].remaining_pulse = 0;
        s_state.axis[i].error = 0;
        s_hw[i].pulse_accum = 0;
        s_hw[i].applied_pps = 0;
        HAL_TIM_PWM_Start(s_hw[i].tim, s_hw[i].channel);
        pwm_apply(i, 0);
        HAL_GPIO_WritePin(s_hw[i].ena_port, s_hw[i].ena_pin, GPIO_PIN_SET);
    }
}

void Stepper_Enable(StepperAxis axis, bool enable)
{
    if (!axis_valid(axis)) {
        return;
    }

    uint32_t i = (uint32_t)axis;
    s_state.axis[i].enabled = enable;
    HAL_GPIO_WritePin(s_hw[i].ena_port, s_hw[i].ena_pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (!enable) {
        axis_stop_now(i);
    }
}

void Stepper_EnableAll(bool enable)
{
    Stepper_Enable(STEPPER_AXIS_1, enable);
    Stepper_Enable(STEPPER_AXIS_2, enable);
}

void Stepper_SetDir(StepperAxis axis, int8_t dir)
{
    if (axis_valid(axis)) {
        set_dir_index((uint32_t)axis, dir);
    }
}

void Stepper_SetPps(StepperAxis axis, int32_t pps)
{
    if (!axis_valid(axis)) {
        return;
    }
    uint32_t i = (uint32_t)axis;
    s_state.axis[i].mode = pps == 0 ? STEPPER_MODE_STOPPING : STEPPER_MODE_SPEED;
    axis_set_target_pps(i, pps);
}

void Stepper_SetAccel(StepperAxis axis, int32_t accel_pps_s)
{
    if (!axis_valid(axis)) {
        return;
    }
    s_state.axis[(uint32_t)axis].accel_pps_s = clamp_i32(accel_pps_s, 1, APP_ACCEL_MAX);
}

static bool target_in_range(int64_t target)
{
#if APP_HOST_OWNS_LIMIT_CHECKS
    (void)target;
    return true;
#else
    const AppParams *p = AppParams_Get();
    return target >= p->motor_min_pos && target <= p->motor_max_pos;
#endif
}

static bool move_prepare(uint32_t index, int64_t current_position, int64_t target, int32_t vmax, int32_t exit_pps)
{
    /* 准备单轴移动段：目标位置、剩余脉冲、方向、目标速度和出口速度。 */
    StepperAxisState *axis = &s_state.axis[index];
    int64_t delta = target - current_position;

    if (!target_in_range(target)) {
        axis->error |= STEPPER_ERR_SOFT_LIMIT;
        return false;
    }

    axis->target_position_pulse = target;
    axis->remaining_pulse = i64_abs(delta);
    if (axis->remaining_pulse == 0 || vmax == 0) {
        axis->target_pps = 0;
        axis->exit_pps = 0;
        axis->mode = STEPPER_MODE_IDLE;
        pwm_apply(index, 0);
        axis->target_position_pulse = target;
        return true;
    }

    int8_t dir = delta > 0 ? 1 : -1;
    vmax = abs(vmax);
    vmax = clamp_i32(vmax, APP_MIN_EFFECTIVE_PPS, axis->max_pps);
    set_dir_index(index, dir);
    axis->mode = STEPPER_MODE_MOVE;
    axis->target_pps = dir > 0 ? vmax : -vmax;
    exit_pps = abs(exit_pps);
    if (exit_pps > vmax) {
        exit_pps = vmax;
    }
    axis->exit_pps = dir > 0 ? exit_pps : -exit_pps;
    return true;
}

bool Stepper_MoveRel(int64_t delta1, int64_t delta2, int32_t v1, int32_t v2)
{
    StepperState snapshot;
    Stepper_GetStateSnapshot(&snapshot);
    return Stepper_MoveAbs(snapshot.axis[0].position_pulse + delta1,
                           snapshot.axis[1].position_pulse + delta2,
                           v1, v2);
}

bool Stepper_MoveAbs(int64_t pos1, int64_t pos2, int32_t v1, int32_t v2)
{
    return Stepper_MoveAbsBlend(pos1, pos2, v1, v2, 0, 0);
}

bool Stepper_MoveAbsBlend(int64_t pos1, int64_t pos2, int32_t v1, int32_t v2, int32_t exit1, int32_t exit2)
{
    StepperState snapshot;
    Stepper_GetStateSnapshot(&snapshot);
    int64_t cur1 = snapshot.axis[0].position_pulse;
    int64_t cur2 = snapshot.axis[1].position_pulse;
    int64_t d1 = i64_abs(pos1 - cur1);
    int64_t d2 = i64_abs(pos2 - cur2);
    int32_t av1 = abs(v1);
    int32_t av2 = abs(v2);

    if (snapshot.axis[0].mode == STEPPER_MODE_ESTOP || snapshot.axis[1].mode == STEPPER_MODE_ESTOP) {
        s_state.axis[0].error |= STEPPER_ERR_ESTOP;
        s_state.axis[1].error |= STEPPER_ERR_ESTOP;
        return false;
    }

    if (!target_in_range(pos1) || !target_in_range(pos2)) {
        if (!target_in_range(pos1)) {
            s_state.axis[0].error |= STEPPER_ERR_SOFT_LIMIT;
        }
        if (!target_in_range(pos2)) {
            s_state.axis[1].error |= STEPPER_ERR_SOFT_LIMIT;
        }
        return false;
    }

    if ((d1 > 0 && !snapshot.axis[0].enabled) || (d2 > 0 && !snapshot.axis[1].enabled)) {
        if (d1 > 0 && !snapshot.axis[0].enabled) {
            s_state.axis[0].error |= STEPPER_ERR_DISABLED;
        }
        if (d2 > 0 && !snapshot.axis[1].enabled) {
            s_state.axis[1].error |= STEPPER_ERR_DISABLED;
        }
        return false;
    }

    if (d1 > 0 && av1 == 0) {
        av1 = APP_MAX_PPS_DEFAULT;
    }
    if (d2 > 0 && av2 == 0) {
        av2 = APP_MAX_PPS_DEFAULT;
    }

    if (d1 > 0 && d2 > 0 && av1 > 0 && av2 > 0) {
        uint64_t t1_us = ((uint64_t)d1 * 1000000ull) / (uint32_t)av1;
        uint64_t t2_us = ((uint64_t)d2 * 1000000ull) / (uint32_t)av2;
        uint64_t t_us = t1_us > t2_us ? t1_us : t2_us;
        if (t_us > 0) {
            av1 = (int32_t)(((uint64_t)d1 * 1000000ull) / t_us);
            av2 = (int32_t)(((uint64_t)d2 * 1000000ull) / t_us);
            if (av1 > 0 && av1 < APP_MIN_EFFECTIVE_PPS) {
                av1 = APP_MIN_EFFECTIVE_PPS;
            }
            if (av2 > 0 && av2 < APP_MIN_EFFECTIVE_PPS) {
                av2 = APP_MIN_EFFECTIVE_PPS;
            }
        }
    }

    uint32_t primask = irq_save();
    bool ok1 = move_prepare(0, cur1, pos1, av1, exit1);
    bool ok2 = move_prepare(1, cur2, pos2, av2, exit2);
    irq_restore(primask);
    return ok1 && ok2;
}

void Stepper_Stop(StepperAxis axis)
{
    if (!axis_valid(axis)) {
        return;
    }
    StepperAxisState *s = &s_state.axis[(uint32_t)axis];
    if (s->mode != STEPPER_MODE_ESTOP) {
        s->target_pps = 0;
        s->mode = STEPPER_MODE_STOPPING;
    }
}

void Stepper_StopAll(void)
{
    Stepper_Stop(STEPPER_AXIS_1);
    Stepper_Stop(STEPPER_AXIS_2);
}

void Stepper_EStopAll(void)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        axis_stop_now(i);
        s_state.axis[i].mode = STEPPER_MODE_ESTOP;
    }
}

void Stepper_ClearError(void)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        if (s_state.axis[i].mode == STEPPER_MODE_ESTOP) {
            s_state.axis[i].mode = STEPPER_MODE_IDLE;
        }
        s_state.axis[i].error = 0;
    }
}

void Stepper_SetErrorAll(uint32_t error_bits)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        s_state.axis[i].error |= error_bits;
    }
}

void Stepper_Zero(void)
{
    uint32_t primask = irq_save();
    for (uint32_t i = 0; i < 2u; ++i) {
        s_state.axis[i].position_pulse = 0;
        s_state.axis[i].target_position_pulse = 0;
        s_state.axis[i].remaining_pulse = 0;
        s_hw[i].pulse_accum = 0;
    }
    irq_restore(primask);
}

static void axis_tick(uint32_t index)
{
    /* 1 kHz 轻量 tick：只做速度斜坡、PWM 更新和软件计脉冲，不解析串口。 */
    StepperAxisState *axis = &s_state.axis[index];
    int32_t accel_step = axis->accel_pps_s / (int32_t)APP_CONTROL_HZ;

    if (!axis->enabled) {
        axis_stop_now(index);
        s_hw[index].pulse_accum = 0;
        return;
    }

    if (accel_step < 1) {
        accel_step = 1;
    }

    if (axis->mode == STEPPER_MODE_IDLE) {
        axis->target_pps = 0;
        if (axis->current_pps > 0) {
            axis->current_pps -= accel_step;
            if (axis->current_pps < 0) {
                axis->current_pps = 0;
            }
        } else if (axis->current_pps < 0) {
            axis->current_pps += accel_step;
            if (axis->current_pps > 0) {
                axis->current_pps = 0;
            }
        }
        pwm_apply(index, 0);
        return;
    }

    if (axis->mode == STEPPER_MODE_MOVE &&
        axis->remaining_pulse > 0 &&
        axis->current_pps == 0 &&
        axis->target_pps == 0) {
        int8_t dir = axis->target_position_pulse >= axis->position_pulse ? 1 : -1;
        axis->target_pps = dir > 0 ? APP_MIN_EFFECTIVE_PPS : -APP_MIN_EFFECTIVE_PPS;
    }

    if (axis->mode == STEPPER_MODE_MOVE && axis->remaining_pulse > 0) {
        int32_t abs_cur = abs(axis->current_pps);
        int32_t abs_exit = abs(axis->exit_pps);
        int64_t stop_dist = 0;
        if (abs_cur > abs_exit) {
            stop_dist = (((int64_t)abs_cur * (int64_t)abs_cur) -
                         ((int64_t)abs_exit * (int64_t)abs_exit)) / (2 * axis->accel_pps_s);
        }
        if (abs_cur > APP_MIN_EFFECTIVE_PPS && axis->remaining_pulse <= stop_dist + 1) {
            axis->target_pps = axis->exit_pps;
        }
    }

    if (axis->current_pps < axis->target_pps) {
        axis->current_pps += accel_step;
        if (axis->current_pps > axis->target_pps) {
            axis->current_pps = axis->target_pps;
        }
    } else if (axis->current_pps > axis->target_pps) {
        axis->current_pps -= accel_step;
        if (axis->current_pps < axis->target_pps) {
            axis->current_pps = axis->target_pps;
        }
    }

    pwm_apply(index, axis->current_pps);

    int32_t abs_pps = abs(axis->current_pps);
    s_hw[index].pulse_accum += abs_pps;
    while (s_hw[index].pulse_accum >= (int32_t)APP_CONTROL_HZ) {
        s_hw[index].pulse_accum -= (int32_t)APP_CONTROL_HZ;
        int8_t step_dir = axis->current_pps >= 0 ? 1 : -1;
        axis->position_pulse += step_dir;
        if (axis->mode == STEPPER_MODE_MOVE && axis->remaining_pulse > 0) {
            axis->remaining_pulse--;
            if (axis->remaining_pulse == 0) {
                axis->position_pulse = axis->target_position_pulse;
                axis->mode = STEPPER_MODE_IDLE;
                axis->target_pps = 0;
                axis->exit_pps = 0;
                pwm_apply(index, 0);
                break;
            }
        }
    }

    if (axis->mode == STEPPER_MODE_STOPPING && axis->current_pps == 0) {
        axis->mode = STEPPER_MODE_IDLE;
    }
}

void Stepper_Tick1kHz(void)
{
    s_state.tick_ms++;
    axis_tick(0);
    axis_tick(1);
}

bool Stepper_IsBusy(void)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        if (s_state.axis[i].mode == STEPPER_MODE_SPEED ||
            s_state.axis[i].mode == STEPPER_MODE_MOVE ||
            s_state.axis[i].mode == STEPPER_MODE_STOPPING ||
            s_state.axis[i].running) {
            return true;
        }
    }
    return false;
}

bool Stepper_CanAcceptMove(void)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        if (s_state.axis[i].mode == STEPPER_MODE_SPEED ||
            s_state.axis[i].mode == STEPPER_MODE_MOVE ||
            s_state.axis[i].mode == STEPPER_MODE_STOPPING ||
            s_state.axis[i].mode == STEPPER_MODE_ESTOP) {
            return false;
        }
    }
    return true;
}

bool Stepper_TargetsAllowed(int64_t pos1, int64_t pos2)
{
    return target_in_range(pos1) && target_in_range(pos2);
}

const StepperState *Stepper_GetState(void)
{
    return &s_state;
}

void Stepper_GetStateSnapshot(StepperState *out)
{
    if (out == NULL) {
        return;
    }

    uint32_t primask = irq_save();
    memcpy(out, &s_state, sizeof(*out));
    irq_restore(primask);
}

const char *Stepper_ModeName(StepperMode mode)
{
    switch (mode) {
    case STEPPER_MODE_IDLE: return "IDLE";
    case STEPPER_MODE_SPEED: return "SPEED";
    case STEPPER_MODE_MOVE: return "MOVE";
    case STEPPER_MODE_STOPPING: return "STOPPING";
    case STEPPER_MODE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
    }
}
