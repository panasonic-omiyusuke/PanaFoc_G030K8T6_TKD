/******************************************************************
<File Name> ：led.c
<Function> 	：LED控制相关函数
<Program> 	：
<Author> 	：董明鉴
<Date> 		：2024/10/8
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "led.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LED_BLINK_DUTY                  50                  /*闪烁duty 500ms*/
#define LED_BLINK_CYCLE                 100                 /*闪烁周期 1s*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t u8g_LED_state;
uint8_t u8g_LED_err; 

static uint8_t u8s_LED_Holdcnt;
static uint8_t u8s_LED_Blinkcnt;
static uint8_t u8s_LED_Errstep;
/* Private function prototypes -----------------------------------------------*/
static void LED_ErrDisp(void);
/* External variables --------------------------------------------------------*/

/******************************************************************
<Title> 		：LED_Refresh(void)
<Function> 		：LED刷新处理
<Description> 	：LED状态刷新，端口电平设定
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/8
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
void LED_Refresh(void)
{
    switch (u8g_LED_state)
    {
    case LED_STATE_OFF:
        u8s_LED_Errstep = 0;
        u8s_LED_Holdcnt = 0;
        u8s_LED_Blinkcnt = 0;
        HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
        break;
    case LED_STATE_ON:
        u8s_LED_Errstep = 0;
        u8s_LED_Holdcnt = 0;
        u8s_LED_Blinkcnt = 0;
        HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
        break;
    case LED_STATE_ERR:
        LED_ErrDisp();
        break;
    default:
        break;
    }
}

/******************************************************************
<Title> 		：LED_ErrDisp(void)
<Function> 		：LED异常显示
<Description> 	：根据异常类型，设置LED显示方式
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/8
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void LED_ErrDisp(void)
{
    uint8_t u8l_blinknum;
    switch (u8s_LED_Errstep)
    {
    case 0:
    /*-----闪烁阶段------------------------*/                                                                 
        if(u8s_LED_Holdcnt < LED_BLINK_DUTY)                                /*前500ms*/
        {
            HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);          /*灭*/
        }
        else                                                                /*后500ms*/
        {
            HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);        /*亮*/
        }
        u8l_blinknum = u8g_LED_err;
        break;
    case 1:
    /*-----熄灭阶段------------------------*/                                                                 
        HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);              /*灭2s*/
        u8l_blinknum = 2;
        break;
    default:
        break;
    }
    
    u8s_LED_Holdcnt++;

    if(u8s_LED_Holdcnt > LED_BLINK_CYCLE)                                   /*1s周期到*/
    {
        u8s_LED_Holdcnt = 0;
        u8s_LED_Blinkcnt++;
        if(u8s_LED_Blinkcnt >= u8l_blinknum)                                /*闪烁周期数到达*/
        {
            u8s_LED_Blinkcnt = 0;
            u8s_LED_Errstep = (u8s_LED_Errstep + 1) % 2;                            /*step切换*/
        }
    }
    

}

/******************************************************************
<Title> 		：SET_LED_DISP(uint8_t u8_disp)
<Function> 		：设置LED显示类型
<Description> 	：根据异常类型，设置LED显示方式
<Input> 		：u8_disp:LED显示类型 OFF/ON/ERR
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/16
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
void SET_LED_DISP(uint8_t u8_disp)
{
    u8g_LED_state = u8_disp;
}

