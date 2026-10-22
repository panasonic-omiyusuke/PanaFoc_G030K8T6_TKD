#include "motor_process.h"
#include "motor_error.h"
#include "led.h"
#include "main.h"
#include "usart_control.h"
#include "Sys_type.h"
#include "InvCommonFunc.h"
#include "Trans_Inv.h"
#include "RefSpeed.h"
#include "Inv_fct.h"
#include "brake.h"
#include "Hogo.h"
#include "mc_config.h"
#include "Timea.h"
#include "lowspeed_ctrl.h"

uint8_t u8s_MOTOR_cmd;                   /*马达状态指令*/
uint8_t debug_u8s_MOTOR_cmd;                   /*马达状态指令*/
uint16_t u16s_fct_cnt;                   /*fct计数*/

uint8_t u8g_SPD_set;                            /*当前目标转速(rps)*/
uint8_t u8g_SPD_cmd;                            /*最后一回通信指令回转数（rps）*/
uint8_t u8g_SPD_limit;
uint8_t u8g_RUN_state;                          /*马达运行状态：正常、正常(转速未达到)、异常*/
uint8_t u8g_ERR_remain;                         /*异常持续标志*/

uint8_t u8g_MOTOR_ctrlflg;                      /*马达控制模式标志*/

uint16_t u16g_MOTOR_errflg;                     /*马达异常标志*/

uint8_t volatile u8g_MAIN_flg;

/*过功率保护恢复标志位*/
uint8_t  u8g_pwr_rcv_flg = FALSE;

/* usart_control.h より*/
uint8_t u8g_FCflg;                               /*工程检查模式标志*/
uint8_t u8g_FCcmd;                               /*工程检查模式指令*/
uint8_t u8g_OLPflg;                              /*OLP保护模式标志*/
uint8_t u8g_KYUYUflg = 0;

/*******************************************
		プロトタイプ宣言
*******************************************/
void vHOG_Wirehg(void);						/*ｺﾝﾌﾟ巻線温度保護 *//* @304swDOD[24/09/02] */
uint8_t MOTOR_Is2LvSpdSetAllowed(uint8_t u8spd); /* 2レベル速度設定許可判定 */

/******************************************************************
<Title> 		    ：Ram_Init(void)
<Function> 		  ：变量初始化
<Description> 	：所有全局变量，静态变量初始化
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/22
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void Ram_Init(void)
{
  uc10MSt_intct = 0;/*メインループ通信確定フラグ*/

  u8g_MAIN_flg = TRUE;
  u8g_SPD_set = SPD_ZERO;
  u8g_SPD_cmd = SPD_ZERO;
  u8g_RUN_state = STATE_NORMAL;
  u8g_MOTOR_ctrlflg = MOTOR_CTRL_OFFLINE;
  u16g_MOTOR_errflg = ERR_NONE;
  u8s_MOTOR_cmd = MOTOR_CMD_IDLE;
  u16s_fct_cnt = 0;
  ucMINtm_H50ct = 0;            /*断続運転タイマカウンタをクリア(初期化)*/
  sts_SW_fct.io_old = GPIO_PIN_SET;
  sts_SW_fct.io_fix = GPIO_PIN_SET;
  sts_SW_fct.u8_cnt = 0;

  sts_SW_rxfct.io_old = GPIO_PIN_SET;
  sts_SW_rxfct.io_fix = GPIO_PIN_SET;
  sts_SW_rxfct.u8_cnt = 0;
  
  u8g_LED_state = LED_STATE_OFF;
  u8g_LED_err = LED_ERR_NONE;

  u8g_ERR_remain = FALSE;

  u8g_OLPflg = FALSE;
  u8g_FCflg = FALSE;
  u8g_FCcmd = FALSE;
  u8g_SPD_limit = SPD_LIMIT_STATE_OFF;
  /*COM_Init();*/

  /*2Lvリレー*/
  pam.ctrl_mode = kStopOff;

  /* 推定処理初回呼び出しフラグを初期化 */
  FirstcallAtEst_flg = 1; 
  
  /*ブレーキ制御用のAD変換しているときに"1"*/
  ADC_BRAKE_FLG = 0;
  ucTRNS_stopctrl = FREE_RUN_POS;
  u8g_brake_state = BRAKE_BANNED;
}

/******************************************************************
 * <Title> 		    ：ToggleAccelationFlag(void)
 * <Function> 		：加速フラグ切替
 * <Description> 	：実回転数と内部の指令回転数が、2rps以上のときに、デバッグポートをhighにする。
 * <Input> 		    ：
 * <Return> 		：
 * <Author> 		：Sakamoto
 * <Date> 			：2026/9/8
 * <Update> 		：
 * <Copyright> 	    ：Panasonic Co. , Ltd.
 ******************************************************************/
void ToggleAccelerationFlag(void)
{
  uint16_t u16l_SpdSet10x; /* 指令回転数の10倍値 */
  uint16_t u16l_AbsSpeedDiff; /* 実回転数と指令回転数の差の絶対値 */

  u16l_SpdSet10x = (uint16_t)u8g_SPD_set * 10U;

  if(u16l_SpdSet10x >= (uint16_t)hCurMecRps)
  {
    u16l_AbsSpeedDiff = u16l_SpdSet10x - (uint16_t)hCurMecRps;
  }
  else
  {
    u16l_AbsSpeedDiff = (uint16_t)hCurMecRps - u16l_SpdSet10x;
  }

  if(u16l_AbsSpeedDiff >= ACCELEATION_FLG_THRESHOLD) /* 実回転数と内部の指令回転数が、2rps以上の差がある場合空きポートをHigh */
  {
    HAL_GPIO_WritePin(unusedB8_GPIO_Port, unusedB8_Pin, GPIO_PIN_SET);
  }
else                                                 /* 実回転数と内部の指令回転数が、2rps未満の差がある場合空きポートをLow */
  {
    HAL_GPIO_WritePin(unusedB8_GPIO_Port, unusedB8_Pin, GPIO_PIN_RESET);
  }
}

/******************************************************************
<Title> 		    ：SpdLimitOLP(void)
<Function> 		    ：OLP速度制限判定
<Description> 	    ：OLP速度制限判定処理
        ：状態遷移仕様
  ：  [STATE: SPD_LIMIT_STATE_ACTIVE(制限中)]
        ：    - 復帰カウンタ(u8l_RCV_cnt)を0にクリア
        ：    - u8l_SPD_cmd > SPD_OLP_LMT の間、制限カウンタ(u8l_LIMIT_cnt)がSPD_OLP_STEP_COUNT到達毎に1減速
        ：    - u8l_SPD_cmd <= SPD_OLP_LMT で SPD_OLP_LMT に張り付き
        ：
  ：  [STATE: SPD_LIMIT_STATE_RECOVERY(復帰中)]
        ：    - 制限カウンタ(u8l_LIMIT_cnt)を0にクリア
        ：    - u8l_SPD_cmd < u8_spd の間、復帰カウンタ(u8l_RCV_cnt)がSPD_OLP_STEP_COUNT到達毎に1加速
        ：    - u8l_SPD_cmd >= u8_spd で復帰完了
  ：      (u8g_SPD_limit=SPD_LIMIT_STATE_OFF, u8g_pwr_rcv_flg=FALSE, 制限/復帰カウンタをクリア)
        ：
        ：  [STATE: その他(通常)]
        ：    - u8l_SPD_cmd = u8_spd で内部値を同期
        ：
        ：時間換算
        ：  1ステップ時間 = ループ周期 × SPD_OLP_STEP_COUNT
        ：  例) mainloop 2ms周期かつSPD_OLP_STEP_COUNT=250の場合、0.5s/step
<Input> 		    ：通信で取得した指令回転数(DEC値)
<Return> 		    ：制限後の回転数(DEC値)
<Author> 		    ：Sakamoto
<Date> 			    ：2026/08/19
<Update> 		    ：
<Copyright> 	    ：Panasonic Co. , Ltd.
******************************************************************/
uint8_t SpdLimitOLP(uint8_t u8_spd){
  static uint8_t u8l_SPD_cmd = SPD_ZERO;
  static uint8_t u8l_LIMIT_cnt = 0;
  static uint8_t u8l_RCV_cnt = 0;
  if(SPD_LIMIT_STATE_ACTIVE == u8g_SPD_limit)
  {
    u8l_RCV_cnt = 0;
    if(u8l_SPD_cmd > SPD_OLP_LMT)
    {
        u8l_LIMIT_cnt++;
        if(u8l_LIMIT_cnt >= SPD_OLP_STEP_COUNT)
        {
            u8l_LIMIT_cnt = 0;
            u8l_SPD_cmd--;
        }
    }
    else
    {
        u8l_SPD_cmd = SPD_OLP_LMT;
    }
  }
  else if(SPD_LIMIT_STATE_RECOVERY == u8g_SPD_limit)
  {
    u8l_LIMIT_cnt = 0;
    if(u8l_SPD_cmd < u8_spd)
    {
        u8l_RCV_cnt++;
        if(u8l_RCV_cnt >= SPD_OLP_STEP_COUNT)
        {
            u8l_RCV_cnt = 0;
            u8l_SPD_cmd++;
        }
    }
    else
    { 
        u8g_SPD_limit = SPD_LIMIT_STATE_OFF;
        u8g_pwr_rcv_flg = FALSE;
        u8l_LIMIT_cnt = 0;
        u8l_RCV_cnt = 0;
    }
  }
  else if(SPD_LIMIT_STATE_BALANCE == u8g_SPD_limit)
  {
    u8l_LIMIT_cnt = 0;
    u8l_RCV_cnt = 0;
    if(u8l_SPD_cmd == u8_spd)                         /*恢复至指令转速，退出制限*/
    {
      u8g_SPD_limit = FALSE;
      u8g_pwr_rcv_flg = FALSE;
      u8l_LIMIT_cnt = 0;
      u8l_RCV_cnt = 0;
    }
    return u8l_SPD_cmd;                               /*转速钳位在当前值*/
  }
  else
  {
      u8l_SPD_cmd = u8_spd;
  }

  return u8l_SPD_cmd;
}

/******************************************************************
<Title> 		    ：FCT_chk(void)
<Function> 		  ：FCT模式检查
<Description> 	：检测FCT端口电平，判定是否进入FCT模式
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/9/30
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void FCT_chk(void)
{
  GPIO_PinState wk;
  static uint16_t u16l_fct_chkcnt = 0x00;
  u16l_fct_chkcnt++;
  if(u16l_fct_chkcnt < 1000)                                  /*FCT检查时间10s 10ms*1000*/
  {
    /*FCT 进入检查*/
    if(u8g_MOTOR_ctrlflg & MOTOR_CTRL_FCT)                    /*已进入FCT，不再检查*/
    {
      /*nothing*/
    }
    else
    {
      wk = HAL_GPIO_ReadPin(FCT_IN_GPIO_Port,FCT_IN_Pin);     /*读取端口电平*/
      if(wk != sts_SW_fct.io_old)                             /*前回电平比较*/
      {
        sts_SW_fct.u8_cnt = 0;
        sts_SW_fct.io_old = wk;
      }
      else
      {
        if(sts_SW_fct.u8_cnt >= 100)                          /*连续1s端口电平一致*/
        {
          if((sts_SW_fct.io_fix == GPIO_PIN_SET) && (wk == GPIO_PIN_RESET))       /*H→L*/
          {
            u8g_MOTOR_ctrlflg |= MOTOR_CTRL_FCT;              /*进入FCT*/
						HAL_GPIO_WritePin(FCT_OUT_GPIO_Port,FCT_OUT_Pin,GPIO_PIN_RESET);
          }
          sts_SW_fct.io_fix = wk;                             /*端口确认值更新*/
        }
        else
        {
          sts_SW_fct.u8_cnt++;
        }
      }
    }
  }
  else if(u16l_fct_chkcnt >=6000){
      u16l_fct_chkcnt = 6000;
      HAL_GPIO_WritePin(FCT_OUT_GPIO_Port,FCT_OUT_Pin,GPIO_PIN_RESET);
  }
  else{
    
  }
  
}

/******************************************************************
<Title> 		    ：MOTOR_main_process(void)
<Function> 		  ：马达控制主处理
<Description> 	：10ms周期执行，判断马达异常，判定运转模式，转速指令给定
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/14
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void MOTOR_main_process(void)
{
  /*-------------马达异常检查-------------*/
  SET_MotorErr_ucHOG_movF();
  if(TRUE == Motor_Err_Chk())
  {
    u8g_RUN_state = STATE_ERRSTOP;      /*马达运转状态：异常停机*/
    u8g_SPD_set = 0;                    /*马达转速设定0*/
    SET_MOTOR_CMD(MOTOR_CMD_STOP);      /*停机指令*/
    SET_LED_DISP(LED_STATE_ERR);        /*LED异常显示*/
  }
  else
  {
    u8g_RUN_state = STATE_NORMAL;       /*马达运转状态：正常*/
    u16g_MOTOR_errflg = ERR_NONE;   /*清除异常标志*/
  }
  /*現在の回転数取得*/
  hCurMecRps  = CalculateAndGetRpsFOC();/*現在の回転数(小数点第一位)*/
  hRampMecRps = MCI_GetMecSpeedRef_F(&Mci[M1]) / 60; /*MCI_GetMecSpeedRef_F()はrpm*1000で返すため、60で割ってrpsに変換*/
  if(((MC_GetSTMStateMotor1() == RUN) || (MC_GetSTMStateMotor1() == SWITCH_OVER))
    && (RUC_IsAlignStageNow(&RevUpControlM1) == 0U))
  {
    ToggleAccelerationFlag();
  }
  else
  {
    HAL_GPIO_WritePin(unusedB8_GPIO_Port, unusedB8_Pin, GPIO_PIN_RESET);
  }
  
  CheckDline();
  /*-------------马达运转模式检查---------*/
  MOTOR_CtrlChk();
  /*-------------马达控制指令判定---------*/
  MOTOR_CmdChk();
  /*-------------LED刷新-----------------------*/
  LED_Refresh();

}

/******************************************************************
<Title> 		    ：MOTOR_CtrlChk(void)
<Function> 		  ：马达运转模式检查
<Description> 	：检查马达异常，判定运转模式
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/15
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void MOTOR_CtrlChk(void)
{
    PumpCountStopIfNeeded(); /* 20201021 竹岡 つながりの強さからvSPD_Setrpmから移動 */

  /*-----------运转模式判定----------------*/
    if(u8g_MOTOR_ctrlflg & MOTOR_CTRL_FCT)
    {
      HEAT_Ctrl_FCT();          /*FCT控制加热器*/
      UART_Ctrl_FCT();          /*FCT控制串口*/
      MOTOR_Ctrl_FCT();         /*FCT控制马达*/
    }
    else if(u8g_MOTOR_ctrlflg & MOTOR_CTRL_ONLINE)
    {
      MOTOR_Ctrl_Online();  /*通信正常控制*/
    }
    else
    {
      MOTOR_Ctrl_Offline(); /*通信异常/默认控制*/
    }
}

/******************************************************************
<Title> 		    ：MOTOR_CmdChk(void)
<Function> 		  ：马达控制指令检查
<Description> 	：检查控制指令，给出驱动指令
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/15
<Update> 		    ：2026/6/9 START中はSPDCHGさせないように変更 sakamoto
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void MOTOR_CmdChk(void)
{
  float f_start_spd;
  uint16_t u16_start_tm;

  static uint16_t u16l_mix_cnt = 0x00;      /*给油时间计数*/
  switch (u8s_MOTOR_cmd)
  {
  /*----------------停机------------------*/
  case MOTOR_CMD_STOP:
    /*ブレーキ中ならSTOP処理をスキップ*/
    if (((u8g_brake_state == BRAKE_READY) && (!IsBreakControlEnabled(ucTRNS_stopctrl))) || (u8g_RUN_state == STATE_ERRSTOP)) { /*ブレーキ制御なしの通常停止や保護停止*/
        /*nothing*/
        u8g_brake_state = BRAKE_FINISHED; /*通常停止の場合は、そのままブレーキ制御停止完了に遷移*/
    }
    else if (u8g_brake_state == BRAKE_FINISHED) { /*ブレーキ制御が無効もしくはブレーキ制御が完了しているとき*/
        u8g_brake_state = BRAKE_BANNED;
    }
    else if (u8g_brake_state == BRAKE_BANNED) {
        MC_StopMotor1();
        MC_AcknowledgeFaultMotor1();
        SET_MOTOR_CMD(MOTOR_CMD_IDLE);
        ePMD_drvmode = D_stop;
        u8g_brake_state = BRAKE_BANNED;
    }else
    {
      /*nothing*/
    }
    break;
  /*----------------启动+给油--------------*/
  case MOTOR_CMD_START:
    /*启动指令*/
    MC_AcknowledgeFaultMotor1();
    /*HAL_GPIO_WritePin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin, GPIO_PIN_SET);*/    /*驱动IC使能*/
    if(MC_GetSTMStateMotor1() == IDLE)
    {
        if (pam.ctrl_mode == kForcedOn){
            f_start_spd = u8g_SPD_set * SPD_FORMAT;                 /*启动、给油回转数*/
            u16_start_tm = f_start_spd / SPD_ACC_START * 1000;    /*启动时间*/
            MC_ProgramSpeedRampMotor1_F(f_start_spd,u16_start_tm);
            MC_StartMotor1();
            u8g_brake_state = BRAKE_READY;
            u8g_KYUYUflg = 0; /*給油回転数フラグは給油回転スタートから1にする*/
        }
    }
    /*给油指令*/
    else if(MC_GetSTMStateMotor1() == RUN)                  /*启动完成，进入给油过程*/
    {
        if(u16l_mix_cnt >= 1500)                              /*给油过程15s后进入运行状态*/
        {
            u16l_mix_cnt = 0;
            SET_MOTOR_CMD(MOTOR_CMD_SPDCHG);                    /*进入变速阶段，调整至目标转速*/
            u8g_KYUYUflg = 0;
        }
        else
        {
            u8g_KYUYUflg = 1; /* 給油回転数中は、u8g_KYUYUflg == 1;*/
            u16l_mix_cnt++;
        }
    }
    else
    {
        /*nothing*/
    }
    u8g_RUN_state = STATE_UNREACH;
    break;
  /*----------------变速------------------*/
  case MOTOR_CMD_SPDCHG:
    if (MOTOR_Is2LvSpdSetAllowed(u8g_SPD_cmd) == TRUE){
        ePMD_drvmode = D_running;
        if(MOTOR_SpeedSet(u8g_SPD_set) == TRUE)                 /*速度切换完成*/
        {
            SET_MOTOR_CMD(MOTOR_CMD_IDLE);
        }
        else
        {
            u8g_RUN_state = STATE_UNREACH;
        }
        break;
    }
    break;
  /*----------------空闲------------------*/
    default:/*MOTOR_CMD_IDLE*/
    break;
  }
}

/******************************************************************
<Title> 		    ：MOTOR_Ctrl_Online()
<Function> 		  ：在线控制模式
<Description> 	：通信正常时按通信转速指令进行控制
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/14
<Update> 		    ：2026/6/9 START中はSPDCHGさせないように変更 sakamoto
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void MOTOR_Ctrl_Online(void)
{
  if(u8g_RUN_state != STATE_ERRSTOP)
  {
    /*---------ON line--------------*/
    if(u8g_SPD_set == SPD_ZERO){                    /*停机中*/
      if(u8g_SPD_cmd != SPD_ZERO){
        u8g_SPD_set = SPD_START; /*强制运转模式下，启动转速为默认转速*/
        SET_MOTOR_CMD(MOTOR_CMD_START);     /*启动控制*/
        SET_LED_DISP(LED_STATE_ON);
      }
      else
      {
        SET_LED_DISP(LED_STATE_OFF);
      }
    }
    else                                    /*运转中*/
    {
      /*--速度指令更新--*/
      if(u8g_SPD_cmd != u8g_SPD_set)
      {   
        if(u8g_SPD_cmd == SPD_ZERO)                /*指令为0*/
        {
          u8g_SPD_set = SPD_ZERO;
          SET_MOTOR_CMD(MOTOR_CMD_STOP);    /*停机控制*/
          SET_LED_DISP(LED_STATE_OFF);
        }
        else
        {
          /* START中はSPDCHGさせない */
          if (u8s_MOTOR_cmd != MOTOR_CMD_START)
          {
            u8g_SPD_set = u8g_SPD_cmd;
            SET_MOTOR_CMD(MOTOR_CMD_SPDCHG);
          }
            SET_LED_DISP(LED_STATE_ON);
        }
      }
    }
  }
  
  ucMINtm_H50ct = 0;            /*断続運転タイマカウンタをクリア*/
}

/******************************************************************
<Title> 		    ：MOTOR_Ctrl_Offline()
<Function> 		  ：离线控制模式
<Description> 	：上电默认控制模式，通信异常时控制模式，60min周期，
                ：默认转速运行
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/14
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void MOTOR_Ctrl_Offline(void)
{
  /*---------转速指令设定--------------------*/
  if(u8g_RUN_state != STATE_ERRSTOP)
  {
    if(ucMINtm_H50ct < MTR_CTRL_OFFLINE_STOP_MINUTE)              /*0~18min stop*/
    {
      if(u8g_SPD_set != SPD_ZERO)
      {
        u8g_SPD_set = SPD_ZERO;
        SET_MOTOR_CMD(MOTOR_CMD_STOP);
      }
      SET_LED_DISP(LED_STATE_OFF);
    }
    else if(ucMINtm_H50ct < MTR_CTRL_OFFLINE_TOTAL_MINUTE)         /*18~60min run*/
    {
      if(u8g_SPD_set == SPD_ZERO)
      {
        u8g_SPD_set = SPD_START; /*强制运转模式下，启动转速为默认转速*/
        SET_MOTOR_CMD(MOTOR_CMD_START);
        SET_LED_DISP(LED_STATE_ON);
      }
      else if(u8g_SPD_set != u8g_SPD_cmd)
      {
        /* START中はSPDCHGさせない */
        if (u8s_MOTOR_cmd != MOTOR_CMD_START)
        {
          u8g_SPD_set = u8g_SPD_cmd;
          SET_MOTOR_CMD(MOTOR_CMD_SPDCHG);
        }
        SET_LED_DISP(LED_STATE_ON);
      }
      else
      {
        /*nothing*/
      }
    }
    else                                          /*60min后固定转速42rps*/
    {
      ucMINtm_H50ct = MTR_CTRL_OFFLINE_TOTAL_MINUTE;  /*固定转速持续时间达到60min*//*リセットでマイコン再起動するために断続運転周期時間で固定*/
      /*ucMINtm_H50ct = 0; */ /*タイマカウントをクリアして"0~18min stop"状態に遷移させる*//*リセットでマイコン再起動しない場合の処理のためコメントアウト*/
    }
  }
}

/******************************************************************
<Title> 		    ：MOTOR_SpeedSet(uint8_t u8_spd)
<Function> 		  ：压缩机转速给定
<Description> 	：转速单位转换，转速切换时间计算，转速切换指令给定
<Input> 		    ：u8_spd:指令转速，单位rps
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/9/30
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
uint8_t MOTOR_SpeedSet(uint8_t u8_spd)
{
  float f_spd_tar;            /*目标速度*/
  uint16_t u16l_time;         /*切换时间*/
  uint8_t retval = FALSE;

  f_spd_tar = u8_spd * SPD_FORMAT;                    /*转速单位转换 rps → rpm*/
  
  if(MC_HasRampCompletedMotor1())                     /*前回速度切换过程已完成*/
  {
    u16l_time = MOTOR_DuraCalc(&f_spd_tar);             /*计算速度切换时间*/ /*加速度*/
    if(u16l_time != 0)
    {
        MC_ProgramSpeedRampMotor1_F(f_spd_tar,u16l_time);/*u8g_SPD_setと加速度でRAMP*/
    }
    else
    {
      retval = TRUE;                                  /*切换完成*/
    }
  }
  return retval;
}

/******************************************************************
<Title> 		    ：MOTOR_DuraCalc(float* f_tar)
<Function> 		  ：转速切换时间计算
<Description> 	：转速单位转换，转速切换时间计算，转速切换指令给定
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/9/30
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
uint16_t MOTOR_DuraCalc(float* f_tar)
{
  float f_spd_now,f_acc;
  uint16_t ret;
  f_acc = SPD_ACC_NOR;                                /*加速度默认正常值*/
  f_spd_now = MC_GetLastRampFinalSpeedM1_F();         /*前回目标速度值rpm*/

/*--------加速-------------------------------*/
  if((*f_tar) > f_spd_now)                         /*目标速度大于当前速度*/        
  {
    if((*f_tar) > (float)SPD_BAN_MAX_F)               /*目标速度在禁止带以上*/
    {
      if(f_spd_now < (float)SPD_BAN_MIN_F)            /*当前速度在禁止带以下*/
      {
        (*f_tar) = SPD_BAN_MIN_F;                     /*下一过程速度设置为禁止带下限*/
        f_acc = SPD_ACC_NOR;                          /*加速度为正常值*/
      }
      else if(f_spd_now < (float)SPD_BAN_MAX_F)       /*当前速度在禁止带中*/
      {
        (*f_tar) = SPD_BAN_MAX_F;                     /*下一过程速度设为禁止带上限*/
        f_acc = SPD_ACC_BAN;                          /*加速度为禁止带加速度*/
      }
      else
      {
        /* nothing to do */
      }
    }
    else if((*f_tar) > (float)SPD_BAN_MIN_F)          /*目标速度在禁止带中*/
    {
      return FALSE;                                   /*异常值，不进行速度切换*/
    }
    else
    {
      /*nothing to do */
    }
    ret = (uint16_t)((((*f_tar) - f_spd_now) / f_acc) * 1000);/*戻り値:ms単位, 通常の加速度:1rpm/s前提*/
  }
  /*--------减速-------------------------------*/
  else if((*f_tar) < f_spd_now)                    /*目标速度小于当前速度*/
  {
    if((*f_tar) < (float)SPD_BAN_MIN_F)               /*目标速度在禁止带以下*/
    {
      if(f_spd_now > (float)SPD_BAN_MAX_F)              /*当前速度在禁止带以上*/
      {
        (*f_tar) = SPD_BAN_MAX_F;                       /*下一过程速度设为禁止带上限*/
        f_acc = SPD_ACC_NOR;                            /*加速度为正常值*/
      }
      else if(f_spd_now > (float)SPD_BAN_MIN_F)         /*当前速度在禁止带中*/
      {
        (*f_tar) = SPD_BAN_MIN_F;                       /*下一过程速度设为禁止带下限值*/
        f_acc = SPD_ACC_BAN;                            /*加速度为禁止带加速度*/
      }
      else
      {
        /*nothing to do */
      }
    }
    else if((*f_tar) < (float)SPD_BAN_MAX_F)          /*目标速度在禁止带中*/
    {
      return FALSE;                                     /*异常值，不进行速度切换*/
    }
    else
    {
      /*nothing to do */
    }
    ret = (uint16_t)(((f_spd_now - (*f_tar)) / f_acc) * 1000);
  }
  /*--------已到达目标转速----------------------*/
  else
  {
    ret = 0;
  }
  return ret;
}

/******************************************************************
<Title> 		    ：SET_MOTOR_CMD(uint8_t u8cmd)
<Function> 		  ：马达控制指令设置
<Description> 	：设置马达控制指令
<Input> 		    ：u8cmd:要执行的指令
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/12
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
/* static void SET_MOTOR_CMD(uint8_t u8cmd) */
void SET_MOTOR_CMD(uint8_t u8cmd)
{
  u8s_MOTOR_cmd = u8cmd;
}

/******************************************************************
<Title> 		    ：TIME_Clr(ST_TIME* st_time)
<Function> 		  ：时间清除
<Description> 	：清除时间计数值
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/14
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void TIME_Clr(ST_TIME* st_time)
{
  st_time->u16_cnt_ms = 0x0000;
  st_time->u8_cnt_s = 0x00;
  st_time->u8_cnt_min = 0x00;
}

/******************************************************************
<Title> 		    ：OUTPUT_Refresh(void)
<Function> 		  ：输出刷新
<Description> 	：端口输出刷新
<Input> 		    ：
<Return> 		    ：
<Author> 		    ：董明鉴
<Date> 			    ：2024/10/16
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
void OUTPUT_Refresh(void)
{
  /*------------未使用端口刷新------------------*/
  /*Pin7、Pin22、Pin23、Pin26*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);
  /*Pin17、Pin27、Pin28、Pin29、Pin32*/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8, GPIO_PIN_RESET);
  /*Pin2、Pin3、Pin20*/
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6, GPIO_PIN_RESET);
}

/******************************************************************
<Title> 		    ：Is2LvSpdSetAllowed(uint8_t bSpdCmd)
<Function> 		  ：2レベル速度設定許可判定
<Description> 	：2レベル速度設定が許可されているかを判定する
<Input> 		    ：spd_cmd: 設定する速度コマンド
<Return> 		    ：TRUE: 許可, FALSE: 不許可
<Author> 		    ：
<Date> 			    ：2026/06/16
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
uint8_t MOTOR_Is2LvSpdSetAllowed(uint8_t u8SpdCmd)
{
  return TRUE;

  if (ucTYPE_2level == 0) {
    return TRUE;  /* 2LVなし仕様は常に許可 */
  }

  if (u8SpdCmd < D_MINHZRYON) {
    return TRUE;
  }

  if (u8SpdCmd >= D_MINHZRYON) {
    return (pam.duty_mode == kBoosted) ? TRUE : FALSE;
  }

  return TRUE;
}

/******************************************************************
<Title> 		    ：MOTOR_SanitizeSpeedCommand
<Function> 		  ：通信から取得した回転数を受け付け、コンプレッサ規定の回転数域に制限した回転数を返す
<Description> 	：
<Input> 		    ：spd_cmd: 設定する速度コマンド
<Return> 		    ：設定後の速度コマンド
<Author> 		    ：Sakamoto
<Date> 			    ：2026/06/16
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
uint8_t MOTOR_SanitizeSpeedCommand(uint8_t dec){
  uint8_t SPD_cmd;
  if(dec > SPD_MAX){
      SPD_cmd = SPD_MAX;
  }else if(dec == SPD_ZERO){
    SPD_cmd = SPD_ZERO;
  }else if(dec < SPD_MIN){
      SPD_cmd = SPD_MIN;
  }else if((dec < SPD_BAN_MIN) || (dec > SPD_BAN_MAX)){
      SPD_cmd = dec;
  }else{
      SPD_cmd = SPD_DEFAULT;
  }
  return SPD_cmd;

}
/******************************************************************
<Title> 		    ：MOTOR_SelectSpeedCommand
<Function> 		  ：オンライン/オフラインに応じた回転数を設定
<Description> 	：オンライン/オフライン情報、メインマイコンから取得した回転数設定有無の状態から、状況に応じた回転数を設定する。
<Input> 		    ：u8g_MOTOR_ctrlflg,u8g_TRNSset_spd_flg,ucTRNS_sfrq_dec,u8l_speed_cmd_offline
<Return> 		    ：状況に応じた回転数
<Author> 		    ：Takida
<Date> 			    ：2026/09/03
<Update> 		    ：
<Copyright> 	  ：Panasonic Co. , Ltd.
******************************************************************/
uint8_t MOTOR_SelectSpeedCommand(uint8_t ctrlflg, uint8_t TRNSset_spd_flg, uint8_t TRNS_sfrq_dec, uint8_t speed_cmd_offline){
  uint8_t SPD_cmd;
  if((ctrlflg == MOTOR_CTRL_ONLINE)    /* オンラインなら*/
   && (TRNSset_spd_flg != 0)){              /* 一度でも回転数をセットしているなら */
    SPD_cmd = TRNS_sfrq_dec;            /* オンライン時は受信した回転数を使用 */
  }else{                                        /* オフラインなら*/
#ifndef DEBUG /*DEBUG*/
  	SPD_cmd = speed_cmd_offline;      /* オフライン時は断続運転中のコンプON回転数を使用 */
#endif /*DEBUG*/
  }
  return SPD_cmd;
}
