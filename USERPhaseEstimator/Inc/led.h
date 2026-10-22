/******************************************************************
<File Name> ：led.h
<Function> 	：主控通信头文件
<Program> 	：
<Author> 	：董明鉴
<Date> 		：2024/10/8
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LED_STATE_OFF               0
#define LED_STATE_ON                1
#define LED_STATE_ERR               2

#define LED_ERR_NONE                0
#define LED_ERR_HWCU                1
#define LED_ERR_SWCU                2
#define LED_ERR_OV                  3
#define LED_ERR_UV                  4
#define LED_ERR_LOCK                5
#define LED_ERR_OP                  6
#define LED_ERR_LP                  7
#define LED_ERR_OFFSET              8
#define LED_ERR_WIREPROTECT         9
/* Private macro -------------------------------------------------------------*/

/* External variables ---------------------------------------------------------*/
extern uint8_t u8g_LED_state;
extern uint8_t u8g_LED_err; 
/* External function prototypes -----------------------------------------------*/
extern void LED_Refresh(void);
extern void SET_LED_DISP(uint8_t u8_disp);





