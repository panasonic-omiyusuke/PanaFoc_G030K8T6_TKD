/* Includes ------------------------------------------------------------------*/
#include "motor_process.h"
#include "motor_error.h"
#include "led.h"
#include "main.h"
#include "usart_control.h"
#include "Sys_type.h"
#include "InvCommonFunc.h"
#include "Inv_fct.h"
#include "Main_Pros.h"
#include "Timea.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t u8g_FCT_HTNumber;                /*FCTヒータ番号*/
ST_SW sts_SW_fct;                        /*FCTスイッチ状態*/
ST_SW sts_SW_rxfct;                      /*FCT受信スイッチ状態*/    


/******************************************************************
<Title>          : HEAT_Ctrl_FCT()
<Function>       : FCTヒータ制御
<Description>    : FCTヒータ出力を制御する。
<Input>          : なし
<Return>         : なし
<Author>         : Panasonic
<Date>           : 2026/1/22
<Update>         :
<Copyright>      : Panasonic Co. , Ltd.
******************************************************************/
void HEAT_Ctrl_FCT(void){
  /*
  switch (u8g_FCT_HTNumber)
  {
  case HEAT_A:
    FCT_HEAT_OutPut(M1_HEAT_FCDEF_Port,M1_HEAT_FCDEF_Pin,GPIO_PIN_SET);         
    break;
  case HEAT_B:
    FCT_HEAT_OutPut(M1_HEAT_SDCASE_Port,M1_HEAT_SDCASE_Pin,GPIO_PIN_RESET);
    break;
  case HEAT_C:
    FCT_HEAT_OutPut(M1_HEAT_PCDF_Port,M1_HEAT_PCDF_Pin,GPIO_PIN_RESET);
    break;
  case HEAT_D:
    FCT_HEAT_OutPut(M1_HEAT_PCFLIP_Port,M1_HEAT_PCFLIP_Pin,GPIO_PIN_RESET);
    break;
  case HEAT_E:
    FCT_HEAT_OutPut(M1_HEAT_SC_Port,M1_HEAT_SC_Pin,GPIO_PIN_RESET);
    break;
  case HEAT_F:
    FCT_HEAT_OutPut(M1_HEAT_SDPC_Port,M1_HEAT_SDPC_Pin,GPIO_PIN_RESET);
    break;
  case HEAT_G:
    FCT_HEAT_OutPut(M1_HEAT_FCPIP_Port,M1_HEAT_FCPIP_Pin,GPIO_PIN_RESET);
    break;
  case HEAT_Version:
    FCT_HEAT_VersionOutPut(CODE_CTRL2);
    break;
  default:
    HEAT_Ctrl_OFF();
    break;
  }
  */
}

/******************************************************************
<Title>          : UART_Ctrl_FCT()
<Function>       : FCT通信用GPIO制御
<Description>    : PA2/PA3の通信用GPIO制御を行う。
<Input>          : なし
<Return>         : なし
<Author>         : Panasonic
<Date>           : 2026/1/22
<Update>         :
<Copyright>      : Panasonic Co. , Ltd.
******************************************************************/
void UART_Ctrl_FCT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_PinState wk;
  static uint8_t  u8_fcttxrx_flag = 0;
  if(!u8_fcttxrx_flag)
  {
    __disable_irq();                                                                  /*割り込み禁止*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /*PA2を汎用IO出力に設定*/
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*PA3を汎用IO入力に設定*/
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    __enable_irq();                                                                   /*割り込み許可*/
		
  /*PA2をHigh出力*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,GPIO_PIN_SET);
  u8_fcttxrx_flag = 1;
  }
  /*受信状態監視*/
  wk = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
  if(wk != sts_SW_rxfct.io_old)
  {
    sts_SW_rxfct.u8_cnt = 0;
    sts_SW_rxfct.io_old = wk;
  }
  else
  {
    if(sts_SW_rxfct.u8_cnt>=100)
    {
      if((sts_SW_rxfct.io_fix == GPIO_PIN_SET)&&(wk == GPIO_PIN_RESET))
      {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
      }
      sts_SW_rxfct.io_fix = wk;
    }
    else
    {
      sts_SW_rxfct.u8_cnt++;
    }
  }
}

/******************************************************************
<Title>          : MOTOR_Ctrl_FCT()
<Function>       : FCT運転制御
<Description>    : FCT運転（27rps）の開始・停止を制御する。
<Input>          : なし
<Return>         : なし
<Author>         : Panasonic
<Date>           : 2024/10/14
<Update>         :
<Copyright>      : Panasonic Co. , Ltd.
******************************************************************/
void MOTOR_Ctrl_FCT(void){
	static uint8_t u8_fct_wait = 0;
  ucMINtm_H50ct = 0;            /*タイマカウンタクリア*/

  /*---------エラー出力------------------*/
  if(u8g_RUN_state == STATE_ERRSTOP)
  {
  HAL_GPIO_WritePin(FCT_OUT_GPIO_Port,FCT_OUT_Pin,GPIO_PIN_SET);  /*PB7をHigh出力*/
  }
  else
  {
  /*---------速度指令処理------------------*/
  if((u8g_SPD_set == 0)&&(u8_fct_wait != 1))                   /*停止中のみFCT運転を開始*/
    {
  u8g_SPD_set = SPD_START;                                  /*回転数27rps指令*/
      SET_MOTOR_CMD(MOTOR_CMD_START);
      SET_LED_DISP(LED_STATE_ON);
			/*FCT_Volt_pos_fdbk_Init();*/
    }
  }
  /*------タイムアウト時にFCT終了------------*/
  if(u16s_fct_cnt >= 5000)
  {
    u16s_fct_cnt = 0;
		u8_fct_wait = 0;
    u8g_MOTOR_ctrlflg &= ~MOTOR_CTRL_FCT;                             /*FCTモード解除*/
		HAL_GPIO_WritePin(FCT_OUT_GPIO_Port,FCT_OUT_Pin,GPIO_PIN_RESET);  /*PB7をLow出力*/
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);               /*PB2をLow出力*/
  HEAT_Ctrl_OFF();                                                  /*ヒータ出力停止*/
		
      __disable_irq();                                                   /*割り込み禁止*/
  /*HAL_UART_MspInit(&huart2);*/                                        /*通信用GPIO再初期化*/
  /*MX_USART2_UART_Init();*/                                            /*UART再初期化*/  
      __enable_irq();                                                    /*割り込み許可*/
  }
  /*------FCT運転中（27rps）保持-------*/
  else if(u16s_fct_cnt >= 1300)         
  {
    u8g_SPD_set = 0;
    SET_MOTOR_CMD(MOTOR_CMD_STOP);
    SET_LED_DISP(LED_STATE_OFF);
    u16s_fct_cnt++;
  u8_fct_wait = 1;                                                  /*FCT待機状態*/
		/*Normal_Volt_pos_fdbk_Init();*/
  }
  else
  {
    u16s_fct_cnt++;
  }
}

/******************************************************************
<Title>          : HEAT_Ctrl_OFF()
<Function>       : ヒータ出力停止
<Description>    : ヒータ関連GPIOを停止状態にする。
<Input>          : なし
<Return>         : なし
<Author>         : Panasonic
<Date>           : 2026/1/22
<Update>         :
<Copyright>      : Panasonic Co. , Ltd.
******************************************************************/
void HEAT_Ctrl_OFF(void){ 
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15|GPIO_PIN_12|GPIO_PIN_11,GPIO_PIN_SET);
}


/******************************************************************
<Title>         : MainLoop_PulseOutput()
<Function>      : デバッグ用パルス出力（2ms周期呼び出し）
<Description>   : 呼び出しのたびにトグルし、4ms周期パルスを出力する。
******************************************************************/
void MainLoop_PulseOutput(void){
  /* パルス反転（2msごとに切り替え） */
    ucg_INT_PULSE ^= 0x01;

    if (vMAIN_IsOptionByteCheck() == 1){
        HAL_GPIO_WritePin(FREQ_PULSE_OUT_GPIO_Port,FREQ_PULSE_OUT_Pin,ucg_INT_PULSE);
    }else{
        HAL_GPIO_WritePin(FREQ_PULSE_OUT_GPIO_Port,FREQ_PULSE_OUT_Pin,GPIO_PIN_RESET);
    }
}
