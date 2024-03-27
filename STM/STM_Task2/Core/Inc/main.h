/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//////////////// DISTANCE TRAVELLED CALCULATION ////////////////
#define PI 3.141592654
#define WHEEL_CIR 21.65
#define PULSE_PER_REV 330

//////////////// MOTORS  ////////////////
#define DIR_FORWARD 1
#define DIR_BACKWARD 0

#define SPEED_MODE_TURN 0
#define SPEED_MODE_SLOW 1
#define SPEED_MODE_FAST 2

#define SMOTOR_L_MAX 83
#define SMOTOR_R_MAX 240
#define SMOTOR_CENTER 142 //142
#define SMOTOR_TURN_TIME 250

#define DIST_M 1.150067316
#define DIST_C 0.945311399 //0.945311399 //was 0.945311399

#define __SET_MOTOR_DIR(DIR) ({ \
	HAL_GPIO_WritePin(GPIOE, LMotor_ACW_CIN1_Pin, ((DIR) ? GPIO_PIN_RESET : GPIO_PIN_SET)); \
	HAL_GPIO_WritePin(GPIOC, LMotor_CW_CIN2_Pin, ((DIR) ? GPIO_PIN_SET: GPIO_PIN_RESET)); \
	HAL_GPIO_WritePin(GPIOB, RMotor_ACW_DIN2_Pin, ((DIR) ? GPIO_PIN_RESET: GPIO_PIN_SET)); \
	HAL_GPIO_WritePin(GPIOB, RMotor_CW_DIN1_Pin, ((DIR) ? GPIO_PIN_SET: GPIO_PIN_RESET)); \
})

#define __RESET_SMOTOR_ANGLE(_TIMER) ({ \
	(_TIMER)->Instance->CCR4 = SMOTOR_CENTER; \
	HAL_Delay(SMOTOR_TURN_TIME); \
})

#define __RESET_SMOTOR_ANGLE_FAST(_TIMER) ({ \
	(_TIMER)->Instance->CCR4 = SMOTOR_CENTER; \
	HAL_Delay(200); \
})

#define __SET_SMOTOR_ANGLE_MAX(_TIMER, _DIR) ({ \
	if (_DIR) (_TIMER)->Instance->CCR4 = SMOTOR_R_MAX; \
	else (_TIMER)->Instance->CCR4 = SMOTOR_L_MAX; \
	HAL_Delay(SMOTOR_TURN_TIME); \
})

#define __SET_SMOTOR_ANGLE(_TIMER, VAL) ({ \
	(_TIMER)->Instance->CCR4 = ((VAL) > SMOTOR_R_MAX) ? SMOTOR_R_MAX : ((VAL) < SMOTOR_L_MAX ? SMOTOR_L_MAX : (VAL));\
	HAL_Delay(SMOTOR_TURN_TIME); \
})

#define __SET_MOTOR_PWM(_TIMER, PWM_L, PWM_R)({ \
	(_TIMER)->Instance->CCR3 = PWM_L; \
	(_TIMER)->Instance->CCR4 = PWM_R; \
})
//////////////// ENCODER  ////////////////
#define __GET_TARGETTICK(dist, targetTick) ({ \
	targetTick = (((dist) * 0.1212885154) / WHEEL_CIR * 13120) - 10; \
})

#define __GET_DISTANCE(targetTick, targetDist) ({ \
		targetDist = (WHEEL_CIR) * (targetTick + 10) / ((0.1212885154) * 13120); \
})

//	targetTick = (((dist) * DIST_M - DIST_C) / WHEEL_CIR * 1320) - 10; \

#define __SET_ENCODER_LAST_TICK(_TIMER, LAST_TICK) ({ \
	LAST_TICK = __HAL_TIM_GET_COUNTER(_TIMER); \
})

#define __SET_ENCODER_LAST_TICK_L_R(_TIMER_L, LAST_TICK_L, _TIMER_R, LAST_TICK_R) ({ \
	__SET_ENCODER_LAST_TICKS(_TIMER_L, LAST_TICK_L); \
	__SET_ENCODER_LAST_TICKS(_TIMER_R, LAST_TICK_R); \
})

#define __GET_ENCODER_TICK_DELTA(_TIMER, LAST_TICK, _DIST) ({ \
	uint32_t CUR_TICK = __HAL_TIM_GET_COUNTER(_TIMER); \
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(_TIMER)) { \
		_DIST = (CUR_TICK <= LAST_TICK) ? LAST_TICK - CUR_TICK : (65535 - CUR_TICK) + LAST_TICK; \
	} else { \
		_DIST = (CUR_TICK >= LAST_TICK) ? CUR_TICK - LAST_TICK : (65535 - LAST_TICK) + CUR_TICK; \
	} \
	LAST_TICK = CUR_TICK; \
})

#define __GET_ENCODER_DELTA(count1, count2,DELTA, _TIMER) ({ \
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(_TIMER)) { \
        if (count2 <= count1) { \
        	DELTA = count1 - count2; \
        } else { \
        	DELTA = (65535 - count2) + count1; \
        } \
    } else { \
        if (count2 >= count1) { \
        	DELTA = count2 - count1; \
        } else { \
        	DELTA = (65535 - count1) + count2; \
        } \
    } \
})

#define __ADC_Read_Dist(_ADC, dataPoint, IR_data_raw_acc, obsDist, obsTick) ({ \
	HAL_ADC_Start(_ADC); \
	HAL_ADC_PollForConversion(_ADC,20); \
	IR_data_raw_acc += HAL_ADC_GetValue(_ADC); \
	dataPoint = (dataPoint + 1) % IR_SAMPLE; \
	if (dataPoint == IR_SAMPLE - 1) { \
		obsDist = IR_CONST_A / (IR_data_raw_acc / dataPoint - IR_CONST_B); \
		obsTick = IR_data_raw_acc / dataPoint; \
		IR_data_raw_acc = 0; \
	} \
})

//////////////// PID CONTROLLER  ////////////////
#define SPEED_MODE_TURN 0
#define SPEED_MODE_1 1
#define SPEED_MODE_2 2

#define INIT_LMOTOR_PWM_TURN 1200
#define INIT_RMOTOR_PWM_TURN 1200
#define PWM_TURN_RANGE 600

#define INIT_LMOTOR_PWM_SLOW 2300
#define INIT_RMOTOR_PWM_SLOW 2300
#define PWM_SLOW_RANGE 700

#define INIT_LMOTOR_PWM_FAST 3000
#define INIT_RMOTOR_PWM_FAST 3000
#define PWM_FAST_RANGE 700

#define __RESET_PID(pid) ({ \
	pid.error = 0; \
	pid.errorSum = 0; \
})

#define __PID_TURN(pid, error, correction, dir, LMotor_PWM, RMotor_PWM) ({ \
	correction = (pid).Kp * error + (pid).Ki * (pid).eSum + (pid).Kd * ((pid).e - error); \
	(pid).e = error; \
	(pid).eSum += error; \
	correction = correction > PWM_TURN_RANGE ? PWM_TURN_RANGE : (correction < -PWM_TURN_RANGE ? -PWM_TURN_RANGE : correction); \
	LMotor_PWM = INIT_LMOTOR_PWM_TURN + correction*dir; \
	RMotor_PWM = INIT_RMOTOR_PWM_TURN - correction*dir; \
})

#define __PID_SLOW(pid, error, correction, dir, LMotor_PWM, RMotor_PWM) ({ \
	correction = (pid).Kp * error + (pid).Ki * (pid).eSum + (pid).Kd * ((pid).e - error);\
	(pid).e = error; \
	(pid).eSum += error; \
	correction = correction > PWM_SLOW_RANGE ? PWM_SLOW_RANGE : (correction < -PWM_SLOW_RANGE ? -PWM_SLOW_RANGE : correction); \
	LMotor_PWM = INIT_LMOTOR_PWM_SLOW + correction*dir; \
	RMotor_PWM = INIT_RMOTOR_PWM_SLOW - correction*dir; \
})

#define __PID_FAST(pid, error, correction, dir, LMotor_PWM, RMotor_PWM) ({ \
	correction = (pid).Kp * error + (pid).Ki * (pid).eSum + (pid).Kd * ((pid).e - error);\
	(pid).e = error; \
	(pid).eSum += error; \
	correction = correction > PWM_FAST_RANGE ? PWM_FAST_RANGE : (correction < -PWM_FAST_RANGE ? -PWM_FAST_RANGE : correction); \
	LMotor_PWM = INIT_LMOTOR_PWM_FAST + correction*dir; \
	RMotor_PWM = INIT_RMOTOR_PWM_FAST - correction*dir; \
})

//////////////// GYROSCOPE  ////////////////
#define __READ_GYRO_Z(i2c, gZData, gyroZ) ({ \
		HAL_I2C_Mem_Read(i2c,ICM20948__I2C_SLAVE_ADDRESS_1 << 1, ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, gZData, 2, 0xFFFF); \
		gyroZ = gZData[0] << 8 | gZData[1]; \
	})

//////////////// INFRARED SENSOR ////////////////
#define IR_CONST_A 25644.81557
#define IR_CONST_B 260.4233354
#define IR_SAMPLE 100


#define __READ_DIST_IR(_ADC, dataPoint, IR_data_raw_acc, obsDist, obsTick) ({ \
	HAL_ADC_Start(_ADC); \
	HAL_ADC_PollForConversion(_ADC,20); \
	IR_data_raw_acc += HAL_ADC_GetValue(_ADC); \
	dataPoint = (dataPoint + 1) % IR_SAMPLE; \
	if (dataPoint == IR_SAMPLE - 1) { \
		obsDist = IR_CONST_A / (IR_data_raw_acc / dataPoint - IR_CONST_B); \
		obsTick = IR_data_raw_acc / dataPoint; \
		IR_data_raw_acc = 0; \
	} \
})


//////////////// TASK MANAGER  ////////////////
#define __TASK_END(_MTimer, prevTask, curTask) ({ \
	__SET_MOTOR_PWM(_MTimer, 0, 0); \
	prevTask = curTask; \
	curTask = TASK_NONE; \
})

#define __ACK_TASK_DONE(_UART, msg) ({ \
	snprintf((char *)msg, sizeof(msg) - 1, "done!"); \
	HAL_UART_Transmit(_UART, (uint8_t *) "ACK|\r\n", 6, 0xFFFF); \
})

//////////////// COMMAND PROCESSING  ////////////////
#define __SET_CMD_CONFIG(cfg, _MTIMER, _STIMER, targetAngle) ({ \
	__SET_SMOTOR_ANGLE(_STIMER, (cfg).servoTurnVal); \
	targetAngle = (cfg).targetAngle; \
	__SET_MOTOR_DIR((cfg).direction); \
	__SET_MOTOR_PWM(_MTIMER, (cfg).leftDuty, (cfg).rightDuty); \
})

#define __SET_CMD_CONFIG_NOPWM(cfg, _STIMER, targetAngle) ({ \
	__SET_SERVO_TURN(_STIMER, (cfg).servoTurnVal); \
	targetAngle = (cfg).targetAngle; \
	__SET_MOTOR_DIR((cfg).direction); \
}) \

#define __ADD_CMD(_CQ, TASK_INDEX, TASK_VAL) ({ \
	_CQ.buffer[_CQ.head].index = TASK_INDEX; \
	_CQ.buffer[_CQ.head].val = TASK_VAL; \
	_CQ.head = (_CQ.head + 1) % _CQ.size; \
})

#define __CLEAR_CURCMD(cmd) ({ \
	cmd.index = 100; \
	cmd.val = 0; \
})

#define __PEND_CURCMD(cmd) ({ \
	cmd.index = 99; \
})

#define __READ_CMD(_CQ, CMD, msg) ({ \
	CMD = _CQ.buffer[_CQ.tail]; \
	_CQ.tail = (_CQ.tail + 1) % _CQ.size; \
	snprintf((char *)msg, sizeof(msg) - 1, "doing"); \
})

#define __IS_FULL(_CQ) (_CQ.head + 1) % _CQ.size == _CQ.tail

#define __IS_EMPTY(_CQ) (_CQ.head == _CQ.tail)




/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define LMotor_CW_AIN2_Pin GPIO_PIN_2
#define LMotor_CW_AIN2_GPIO_Port GPIOA
#define LMotor_ACW_AIN1_Pin GPIO_PIN_3
#define LMotor_ACW_AIN1_GPIO_Port GPIOA
#define RMotor_CW_BIN1_Pin GPIO_PIN_4
#define RMotor_CW_BIN1_GPIO_Port GPIOA
#define RMotor_ACW_BIN2_Pin GPIO_PIN_5
#define RMotor_ACW_BIN2_GPIO_Port GPIOA
#define LMotor_CW_CIN2_Pin GPIO_PIN_5
#define LMotor_CW_CIN2_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define LMotor_ACW_CIN1_Pin GPIO_PIN_12
#define LMotor_ACW_CIN1_GPIO_Port GPIOE
#define SMotor_Pin GPIO_PIN_14
#define SMotor_GPIO_Port GPIOE
#define RMotor_CW_DIN1_Pin GPIO_PIN_11
#define RMotor_CW_DIN1_GPIO_Port GPIOB
#define RMotor_ACW_DIN2_Pin GPIO_PIN_15
#define RMotor_ACW_DIN2_GPIO_Port GPIOB
#define USER_BTN_Pin GPIO_PIN_8
#define USER_BTN_GPIO_Port GPIOD
#define USER_BTN_EXTI_IRQn EXTI9_5_IRQn
#define ULTRA_TRIG_Pin GPIO_PIN_14
#define ULTRA_TRIG_GPIO_Port GPIOD
#define ULTRA_ECHO_Pin GPIO_PIN_15
#define ULTRA_ECHO_GPIO_Port GPIOD
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define PWMC_Pin GPIO_PIN_8
#define PWMC_GPIO_Port GPIOC
#define PWMD_Pin GPIO_PIN_9
#define PWMD_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
