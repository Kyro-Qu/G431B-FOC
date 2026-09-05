/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/*
 * Latched reset/CPU exception snapshot for Keil Watch.  exception == 0 means
 * no Cortex exception has been captured since the latest reset.
 */
typedef struct
{
  volatile uint32_t reset_flags;
  volatile uint32_t exception;
  volatile uint32_t cfsr;
  volatile uint32_t hfsr;
  volatile uint32_t dfsr;
  volatile uint32_t afsr;
  volatile uint32_t mmfar;
  volatile uint32_t bfar;
  volatile uint32_t icsr;
  volatile uint32_t rcc_csr;
  volatile uint32_t tim1_bdtr;
  volatile uint32_t tim1_ccer;
  volatile uint32_t tim1_cnt;
  volatile uint32_t tim1_cr1;
  /* Snapshot copied from TAMP backup registers after the latest reset. */
  volatile uint32_t backup_valid;
  volatile uint32_t previous_checkpoint;
  volatile uint32_t previous_sequence;
  volatile uint32_t previous_tick;
  volatile uint32_t previous_tim1_bdtr;
  volatile uint32_t previous_tim1_ccer;
  volatile uint32_t previous_tim1_ccr1;
  volatile uint32_t previous_tim1_ccr2;
  volatile uint32_t previous_tim1_ccr3;
  volatile uint32_t previous_tim1_cnt;
  volatile uint32_t previous_tim1_cr1;
  volatile uint32_t previous_boot_reset_flags;
  volatile uint32_t previous_boot_count;
  volatile uint32_t previous_event;
  volatile uint32_t previous_pwm_stage;
  volatile uint32_t previous_calib_state;
  volatile uint32_t previous_foc_state;
  /* One-shot archive frozen by the first reset after power output is armed. */
  volatile uint32_t power_reset_archive_valid;
  volatile uint32_t power_reset_checkpoint;
  volatile uint32_t power_reset_flags;
  volatile uint32_t power_reset_event;
  volatile uint32_t power_reset_pwm_stage;
  volatile uint32_t power_reset_calib_state;
  volatile uint32_t power_reset_foc_state;
} system_fault_diag_t;

/* Values stored in previous_event. The value describes the last completed
 * software checkpoint before a reset or loss of the debug connection. */
typedef enum
{
  SYSTEM_CHECKPOINT_NONE = 0,
  SYSTEM_CHECKPOINT_BOOT = 1,
  SYSTEM_CHECKPOINT_TIMER_READY = 2,
  SYSTEM_CHECKPOINT_PWM_DISABLE = 3,
  SYSTEM_CHECKPOINT_BOOTSTRAP_ON = 4,
  SYSTEM_CHECKPOINT_NORMAL_PWM_ON = 5,
  SYSTEM_CHECKPOINT_CALIB_START = 10,
  SYSTEM_CHECKPOINT_CALIB_NEUTRAL = 11,
  SYSTEM_CHECKPOINT_CALIB_ALIGN = 12,
  SYSTEM_CHECKPOINT_CALIB_CLEAR = 13,
  SYSTEM_CHECKPOINT_CALIB_SEARCH = 14,
  SYSTEM_CHECKPOINT_CALIB_DONE = 15,
  SYSTEM_CHECKPOINT_CALIB_FAIL = 16,
  SYSTEM_CHECKPOINT_RUN_STOP = 17
} system_checkpoint_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim4;
extern volatile system_fault_diag_t g_system_fault_diag;
void system_fault_diag_boot_init(void);
void system_power_checkpoint(uint32_t event,
                             uint32_t pwm_stage,
                             uint32_t calib_state,
                             uint32_t foc_state);
void system_fault_capture(uint32_t exception);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CURRENT_U_IN_Pin GPIO_PIN_1
#define CURRENT_U_IN_GPIO_Port GPIOA
#define OPAMP1_OUT_ADC1_IN3_Pin GPIO_PIN_2
#define OPAMP1_OUT_ADC1_IN3_GPIO_Port GPIOA
#define OPAMP1_GAIN_BIAS_Pin GPIO_PIN_3
#define OPAMP1_GAIN_BIAS_GPIO_Port GPIOA
#define OPAMP2_GAIN_BIAS_Pin GPIO_PIN_5
#define OPAMP2_GAIN_BIAS_GPIO_Port GPIOA
#define OPAMP2_OUT_ADC2_IN3_Pin GPIO_PIN_6
#define OPAMP2_OUT_ADC2_IN3_GPIO_Port GPIOA
#define CURRENT_V_IN_Pin GPIO_PIN_7
#define CURRENT_V_IN_GPIO_Port GPIOA
#define CURRENT_W_IN_Pin GPIO_PIN_0
#define CURRENT_W_IN_GPIO_Port GPIOB
#define OPAMP3_OUT_ADC1_IN12_Pin GPIO_PIN_1
#define OPAMP3_OUT_ADC1_IN12_GPIO_Port GPIOB
#define OPAMP3_GAIN_BIAS_Pin GPIO_PIN_2
#define OPAMP3_GAIN_BIAS_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_10
#define KEY_GPIO_Port GPIOC
#define KEY_EXTI_IRQn EXTI15_10_IRQn
#define CAN_SHD_Pin GPIO_PIN_11
#define CAN_SHD_GPIO_Port GPIOC
#define ABZ_A_Pin GPIO_PIN_6
#define ABZ_A_GPIO_Port GPIOB
#define ABZ_B_Pin GPIO_PIN_7
#define ABZ_B_GPIO_Port GPIOB
#define ABZ_Z_Pin GPIO_PIN_8
#define ABZ_Z_GPIO_Port GPIOB
#define ABZ_Z_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define VBUS_ADC_Pin GPIO_PIN_0
#define VBUS_ADC_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
