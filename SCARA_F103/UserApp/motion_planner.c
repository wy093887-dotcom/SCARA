#include "motion_planner.h"

/* 运动规划薄封装：课程版不做复杂轨迹规划，只把上位机规划结果交给步进底层执行。 */

#include "stepper_driver.h"

void MotionPlanner_SetAccel(int32_t a1, int32_t a2)
{
    Stepper_SetAccel(STEPPER_AXIS_1, a1);
    Stepper_SetAccel(STEPPER_AXIS_2, a2);
}

void MotionPlanner_Speed(int32_t pps1, int32_t pps2)
{
    Stepper_SetPps(STEPPER_AXIS_1, pps1);
    Stepper_SetPps(STEPPER_AXIS_2, pps2);
}

bool MotionPlanner_MoveRel(int64_t dp1, int64_t dp2, int32_t v1, int32_t v2)
{
    return Stepper_MoveRel(dp1, dp2, v1, v2);
}

bool MotionPlanner_MoveAbs(int64_t p1, int64_t p2, int32_t v1, int32_t v2)
{
    return Stepper_MoveAbs(p1, p2, v1, v2);
}

bool MotionPlanner_MoveAbsBlend(int64_t p1, int64_t p2, int32_t v1, int32_t v2, int32_t exit1, int32_t exit2)
{
    return Stepper_MoveAbsBlend(p1, p2, v1, v2, exit1, exit2);
}

void MotionPlanner_Stop(void)
{
    Stepper_StopAll();
}
