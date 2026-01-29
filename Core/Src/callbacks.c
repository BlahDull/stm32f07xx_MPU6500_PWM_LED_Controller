/*
 * callbacks.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Blah
 */

#include "main.h"
#include "cmsis_os.h"
#include "mpu6500.h"

extern uint32_t g_counter;
extern volatile uint32_t g_counter_2_state;
extern volatile uint32_t g_counter_3_state;
extern I2C_HandleTypeDef hi2c1;
extern osThreadId_t Sensor_ReadHandle;

#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	g_counter = 0;
	g_counter_2_state = 8000;
	g_counter_3_state = 4000;
}
#endif

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		g_counter_2_state = 0;
	} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		g_counter_3_state = 0;
	}
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim) {

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
	if (GPIO_PIN == MPU6500_INT_Pin) {
		MPU6500_Interrupt_Handle(&hi2c1);
	}
}

void MPU6500_RawReady_Callback() {
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if (Sensor_ReadHandle) {
		xTaskNotifyFromISR(Sensor_ReadHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
