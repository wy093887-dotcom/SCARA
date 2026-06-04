#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

void MotionPlanner_SetAccel(int32_t a1, int32_t a2);
void MotionPlanner_Speed(int32_t pps1, int32_t pps2);
bool MotionPlanner_MoveRel(int64_t dp1, int64_t dp2, int32_t v1, int32_t v2);
bool MotionPlanner_MoveAbs(int64_t p1, int64_t p2, int32_t v1, int32_t v2);
bool MotionPlanner_MoveAbsBlend(int64_t p1, int64_t p2, int32_t v1, int32_t v2, int32_t exit1, int32_t exit2);
void MotionPlanner_Stop(void);

#endif
