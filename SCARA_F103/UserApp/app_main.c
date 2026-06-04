#include "app_main.h"

/* 用户主循环入口：CubeMX 初始化后进入这里，周期性处理串口、协议、运动和回零状态机。 */

#include "app_config.h"
#include "board_pins.h"
#include "app_params.h"
#include "gcode_stream.h"
#include "home_controller.h"
#include "protocol.h"
#include "scara_kinematics.h"
#include "serial_dma.h"
#include "stepper_driver.h"

void App_Init(void)
{
    AppParams_Init();
    Stepper_Init();
    ScaraKinematics_Init();
    HomeController_Init();
    GcodeStream_Init();
    Protocol_Init();
    SerialDma_Init();
    HAL_TIM_Base_Start_IT(BOARD_TICK_TIM);
    SerialDma_Send("BOOT " APP_FW_NAME " " APP_FW_VERSION " READY\r\n");
}

void App_Loop(void)
{
    char line[APP_SERIAL_LINE_SIZE];

    /* 主循环只做非中断重活：串口收包、协议解析、G-code 入队、回零流程推进。 */
    SerialDma_Poll();
    while (SerialDma_ReadLine(line, sizeof(line))) {
        Protocol_ProcessLine(line);
    }
    HomeController_Loop();
    GcodeStream_Loop();
    Protocol_Loop();
}

void App_Tick1kHz(void)
{
    Stepper_Tick1kHz();
    HomeController_Tick1kHz();
    GcodeStream_Tick1kHz();
    Protocol_Tick1kHz();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == BOARD_TICK_TIM) {
        App_Tick1kHz();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == BOARD_UART) {
        SerialDma_TxCpltCallback();
    }
}
