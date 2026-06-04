#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Central firmware tuning parameters.
 * Adjust mechanism, motion, homing, serial, and flash defaults here.
 */

#include <stdint.h>

/* Firmware identity shown by VERSION. */
#define APP_FW_NAME "SCARA_F103"
#define APP_FW_VERSION "0.24.1"

/* Current controller drives two stepper axes. */
#define APP_AXIS_COUNT 2u

/* Base timer clock for PWM pulse generation. */
#define APP_STEPPER_TIMER_HZ 1000000u
/* Main control loop frequency used by the software stepper model. */
#define APP_CONTROL_HZ 1000u

/* Lowest useful pulse rate. Lower than this is treated as stopped. */
#define APP_MIN_EFFECTIVE_PPS 16
/* Default per-axis speed limit used by normal motion planning. */
#define APP_MAX_PPS_DEFAULT 10000
/* Absolute hard ceiling for per-axis pulse speed. */
#define APP_MAX_PPS_HARD 50000
/* Default acceleration in pulses/s^2 for both axes. */
#define APP_ACCEL_DEFAULT 3000
/* Maximum allowed acceleration in pulses/s^2. */
#define APP_ACCEL_MAX 50000

/* Software pulse position range. Only used when MCU-side limits are enabled. */
#define APP_MOTOR_MIN_POS (-2000000000L)
#define APP_MOTOR_MAX_POS (2000000000L)
/* 1: host PC owns workspace/path limit checks.
 * 0: MCU also enforces motor pulse range / joint range checks.
 */
#define APP_HOST_OWNS_LIMIT_CHECKS 1u

/* Motor and transmission parameters used by pulse-angle conversion. */
/* Pulses per motor revolution after driver microstep configuration. */
#define APP_PULSES_PER_REV_M1 1600L
#define APP_PULSES_PER_REV_M2 1600L
/* Gear reduction ratio from motor shaft to active arm. */
#define APP_REDUCER_RATIO_M1 1L
#define APP_REDUCER_RATIO_M2 1L
/* Positive/negative sign for each motor direction. Use 1 or -1. */
#define APP_MOTOR1_DIR_SIGN 1L
#define APP_MOTOR2_DIR_SIGN 1L
/* Joint zero offsets in mrad after homing / mechanical calibration. */
#define APP_MOTOR1_ZERO_MRAD 2251L
#define APP_MOTOR2_ZERO_MRAD 890L

/* SCARA geometry in micrometers.
 * Must match the upper computer kinematic model.
 */
/* Distance between the two base motor axes. */
#define APP_SCARA_BASE_UM 150000L
/* Left and right active arm lengths. */
#define APP_SCARA_ACTIVE1_UM 160000L
#define APP_SCARA_ACTIVE2_UM 160000L
/* Left and right passive arm lengths. */
#define APP_SCARA_PASSIVE1_UM 200000L
#define APP_SCARA_PASSIVE2_UM 200000L
/* Joint angle soft ranges in milliradians. */
#define APP_SCARA_THETA1_MIN_MRAD (-3142L)
#define APP_SCARA_THETA1_MAX_MRAD 3142L
#define APP_SCARA_THETA2_MIN_MRAD 0L
#define APP_SCARA_THETA2_MAX_MRAD 6283L
/* Elbow branch selection for inverse kinematics. Usually keep paired with UI. */
#define APP_SCARA_IK_LEFT_ELBOW_SIGN 1
#define APP_SCARA_IK_RIGHT_ELBOW_SIGN (-1)

/* Legacy stream/status pacing period. */
#define APP_STREAM_PERIOD_MS 100u
/* Communication watchdog default.
 * 0 disables watchdog on boot; can be enabled later by command.
 */
#define APP_COMM_WATCHDOG_DEFAULT_MS 0u

/* Serial buffers and protocol sizing. */
/* Raw UART RX DMA ring buffer size in bytes. */
#define APP_SERIAL_RX_DMA_SIZE 256u
/* Max accepted single ASCII command line length. */
#define APP_SERIAL_LINE_SIZE 96u
/* TX staging buffer size for one formatted response. */
#define APP_SERIAL_TX_SIZE 280u
/* Number of parsed RX lines that can wait in queue. */
#define APP_SERIAL_LINE_QUEUE_DEPTH 16u
/* Number of pending TX messages that can wait in queue. */
#define APP_SERIAL_TX_QUEUE_DEPTH 8u
/* UART baudrate. Must match the upper computer. */
#define APP_SERIAL_BAUDRATE 115200u

/* Homing switch and homing process parameters. */
/* 0 means switch active-low, 1 means active-high. */
#define APP_HOME_SWITCH_ACTIVE_LEVEL 0u
/* Search direction for each homing axis. Use 1 or -1. */
#define APP_HOME_AXIS1_DIR (-1)
#define APP_HOME_AXIS2_DIR (-1)
/* Homing search speed in pulses per second. */
#define APP_HOME_SEARCH_PPS 300
/* Pull-off distance after switch trigger, in pulses. */
#define APP_HOME_BACKOFF_PULSE 200L
/* Debounce time that the switch must stay active. */
#define APP_HOME_DEBOUNCE_MS 20u
/* Maximum allowed homing duration before timeout. */
#define APP_HOME_TIMEOUT_MS 30000u

/* G-code stream planner parameters. */
/* Number of queued motion blocks kept by MCU. */
#define APP_GCODE_PLANNER_BLOCKS 32u
/* Wait time for the first block so a second block can arrive for blending. */
#define APP_GCODE_BLEND_START_DELAY_MS 20u
/* Automatic status push period while streaming. */
#define APP_GCODE_STATUS_PERIOD_MS 200u
/* Default feed rate for G1 if host does not send F, in mm/min. */
#define APP_GCODE_DEFAULT_FEED_MM_MIN 300L
/* G0 rapid speed, expressed in pulses per second. */
#define APP_GCODE_RAPID_PPS APP_MAX_PPS_DEFAULT
/* Legacy linear segmentation length in micrometers. */
#define APP_GCODE_LINEAR_SEGMENT_UM 2000L

/* MOVL segmentation length in micrometers. */
#define APP_MOVL_SEGMENT_UM 2000L

/* Flash parameter page definition. */
/* Last flash page reserved for persisted AppParams. */
#define APP_PARAM_FLASH_ADDR 0x0800F800u
/* Flash page size for STM32F103C8/CB family. */
#define APP_PARAM_FLASH_PAGE_SIZE 1024u
/* Magic tag used to validate saved parameter blocks. */
#define APP_PARAM_FLASH_MAGIC 0x53434152u
/* Increment when default parameter layout/meaning changes. */
#define APP_PARAM_FLASH_VERSION 4u

#endif
