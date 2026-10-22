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
#include "stm32g0xx_hal.h"
#include "motorcontrol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t volatile u8g_MAIN_flg;
extern uint8_t u8g_SPD_set;                            /*現在の目標回転数 (rps)*/
extern uint8_t u8g_SPD_cmd;                            /*通信指令回転数（rps）*/
extern uint8_t u8g_SPD_limit;
extern uint8_t u8g_ERR_remain;
extern uint8_t u8g_RUN_state;
extern uint8_t u8g_MOTOR_ctrlflg;
extern uint16_t u16g_MOTOR_errflg;
extern uint8_t  u8g_pwr_rcv_flg;
extern uint8_t  ucTRNS_sfrq_dec;
extern uint8_t u8g_TRNSset_spd_flg;		/*メインからの回転数指令セット確認フラグ*//*マイコン起動時からの通信異常カウントのために必要*/

extern void vTRNS_Outputi();
extern void Drive_Clear_WDT();
extern void vMAIN_swreset();
extern void vTRNS_Cnttrns();
extern void vMAIN_Msec10();
extern void vMAIN_type_uart();
extern void main_loop();
extern void  vMAIN_Msec10();
extern void main_loop();
extern void SubCtrl_ModeSet();
extern void Prtct_ParamSet();
extern void vMAIN_MainInttm51();
extern void vMAIN_MainPros();

extern void v2LV_Ctrl2lv();
extern void vSPD_Setrpm();

extern uint8_t Motor_Err_Chk();

void	vMAIN_GPIO_Reinit(void);				/* 再設定処理呼び出し */
void	vMAIN_DMA_Reinit(void);					/* 再設定処理呼び出し */
void	vMAIN_ADC1_Reinit(void);				/* 再設定処理呼び出し */
void	vMAIN_TIM1_Reinit(void);				/* 再設定処理呼び出し */
void	vMAIN_TIM2_Reinit(void);				/* 再設定処理呼び出し */
void	vMAIN_TIM16_Reinit(void);				/* 再設定処理呼び出し */
void	vMAIN_USART1_UART_Reinit(void);			/* 再設定処理呼び出し */
void	vMAIN_NVIC_Reinit(void);				/* 再設定処理呼び出し */
	
void ResetWatchDogTimer(void);			/* WDT再設定処理呼び出し */

extern void COM_main_process(void);
extern void MX_ADC1_Init(void); /*brake.cで呼び出しのため*/
extern void MX_DMA_Init(void); /*brake.cで呼び出しのため*/
extern void MX_TIM1_Init(void); /*brake.cで呼び出しのため*/
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

 /* MOTOR CTRL MODE */
typedef enum MtrCtrlMd{
	MOTOR_CTRL_OFFLINE,  /*0x00*/
	MOTOR_CTRL_FCT,      /*0x01*/
	MOTOR_CTRL_ONLINE    /*0x02*/
} eMtrCtrlMd;

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOB
#define unused_Pin GPIO_PIN_14
#define unused_GPIO_Port GPIOC
#define unusedC15_Pin GPIO_PIN_15
#define unusedC15_GPIO_Port GPIOC
#define unusedA0_Pin GPIO_PIN_0
#define unusedA0_GPIO_Port GPIOA
#define INV_CURR_Pin GPIO_PIN_1
#define INV_CURR_GPIO_Port GPIOA
#define POS_U_Pin GPIO_PIN_2
#define POS_U_GPIO_Port GPIOA
#define POS_V_Pin GPIO_PIN_3
#define POS_V_GPIO_Port GPIOA
#define POS_W_Pin GPIO_PIN_4
#define POS_W_GPIO_Port GPIOA
#define FREQ_PULSE_OUT_Pin GPIO_PIN_5
#define FREQ_PULSE_OUT_GPIO_Port GPIOA
#define DRV_FAULT_Pin GPIO_PIN_6
#define DRV_FAULT_GPIO_Port GPIOA
#define PWM_U_LOW_Pin GPIO_PIN_7
#define PWM_U_LOW_GPIO_Port GPIOA
#define PWM_V_LOW_Pin GPIO_PIN_0
#define PWM_V_LOW_GPIO_Port GPIOB
#define PWM_W_LOW_Pin GPIO_PIN_1
#define PWM_W_LOW_GPIO_Port GPIOB
#define VBUS_Pin GPIO_PIN_2
#define VBUS_GPIO_Port GPIOB
#define PWM_U_HIGH_Pin GPIO_PIN_8
#define PWM_U_HIGH_GPIO_Port GPIOA
#define PWM_V_HIGH_Pin GPIO_PIN_9
#define PWM_V_HIGH_GPIO_Port GPIOA
#define HEAT1_Pin GPIO_PIN_6
#define HEAT1_GPIO_Port GPIOC
#define PWM_W_HIGH_Pin GPIO_PIN_10
#define PWM_W_HIGH_GPIO_Port GPIOA
#define DRV_RELAY_Pin GPIO_PIN_11
#define DRV_RELAY_GPIO_Port GPIOA
#define HEAT2_Pin GPIO_PIN_12
#define HEAT2_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWDCLK_Pin GPIO_PIN_14
#define SWDCLK_GPIO_Port GPIOA
#define unusedA15_Pin GPIO_PIN_15
#define unusedA15_GPIO_Port GPIOA
#define unusedB3_Pin GPIO_PIN_3
#define unusedB3_GPIO_Port GPIOB
#define unusedB4_Pin GPIO_PIN_4
#define unusedB4_GPIO_Port GPIOB
#define unusedB5_Pin GPIO_PIN_5
#define unusedB5_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
#define unusedB8_Pin GPIO_PIN_8
#define unusedB8_GPIO_Port GPIOB

#define MAINAPP_UART_WAIT_100MS_COUNT_MAX   (6000U)
#define MAINAPP_TRNS_READY_MASK             (0x10U)

/* USER CODE BEGIN Private defines */
#define SYSTEM_CLOCK_CONF_RDY_TIMEOUT_MS (100000U)

#define STACK_ADDRESS (0x20001A40)	/* Stack monitor address (STACK_REGION base 0x20001A00 + 0x40) */
#define RAM_data_END_ADDRESS (0x200004E8)	/* End of .data / start of ZI(.bss) from latest .map */
#define RAM_ACCESS_END_ADDRESS  (0x20001800)	/* End address for RAM clear loop (RW_IRAM1 end) */
#define RAM_ACCESS_END_ADDRESS1 (0x20001800)	/* Sentinel area word 1 */
#define RAM_ACCESS_END_ADDRESS2 (0x20001804)	/* Sentinel area word 2 */
#define RAM_ACCESS_END_ADDRESS3 (0x20001808)	/* Sentinel area word 3 */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
