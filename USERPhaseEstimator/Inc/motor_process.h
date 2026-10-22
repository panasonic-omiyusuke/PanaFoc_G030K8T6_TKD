#ifndef MOTOR_PROCESS_H
#define MOTOR_PROCESS_H

#include "stdint.h"
#include "stm32g0xx_hal.h"
#include "Sys_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN Private defines */
#define FCT_OUT_Pin GPIO_PIN_6
#define FCT_OUT_GPIO_Port GPIOB
#define FCT_IN_Pin GPIO_PIN_7
#define FCT_IN_GPIO_Port GPIOB

/*马达控制指令*/
#define MOTOR_CMD_IDLE    0x00            /*空闲*/
#define MOTOR_CMD_STOP    0x01            /*停机*/
#define MOTOR_CMD_START   0x02            /*启动*/
#define MOTOR_CMD_SPDCHG  0x03            /*变速*/


#define SPD_FORMAT        60              /*转速单位转换系数rps→rpm*/
#define SPD_ACC_NOR       100             /*通常速度带加速度 100RPM/s（1.67rps/s）*/
#define SPD_ACC_BAN       720             /*禁止周波带加速度 720RPM/s（12rps/s）*/
#define SPD_ACC_START     2400            /*启动加速度  2400RPM/s （40rps/s）*/


/* MORTOR SPD PARMETER */
#define SPD_ZERO            0               /*停机回转数 0rps*/
#define SPD_BAN_MIN         43              /*禁止回转数下限 43rps*/
#define SPD_BAN_MAX         48              /*禁止回转数上限 48rps*/
#define SPD_START           27              /*启动/给油回转数 27rps*/
#define SPD_DEFAULT         42              /*离线(默认运行速度) 42rps*/
#define SPD_OLP_LMT         37              /*OLP制限回转数 37rps (2200rpm)*/
#define SPD_OLP_STEP_COUNT  250              /*OLP制限/復帰時の1ステップ待ち回数 (mainloop内2ms周期:2ms×250=0.5s、10ms基準では10ms×50=0.5s相当)*/

#define SPD_MIN             17               /*最小回转数 17rps*/
#define SPD_MAX             80              /*最大回转数 80rps*/

#define SPD_BAN_MIN_F     (SPD_BAN_MIN * SPD_FORMAT)            /*禁止周波带下限2580RPM(43rps)*/
#define SPD_BAN_MAX_F     (SPD_BAN_MAX * SPD_FORMAT)            /*禁止周波带上限2880RPM(48rps)*/

#define SECONDS_FOR_MINUTE 60
#define MTR_CTRL_OFFLINE_STOP_MINUTE   18 /*18min STOP*/
#define MTR_CTRL_OFFLINE_TOTAL_MINUTE  60 /*60min-18min = 42minRUN*/

#define ACCELEATION_FLG_THRESHOLD    20U

extern uint8_t volatile u8g_MAIN_flg;
extern uint8_t ucTRNS_sfrq_b;                            /*当前目标转速(rps)*/
extern uint8_t u8g_SPD_set;
extern uint8_t u8g_SPD_cmd;
extern uint8_t u8g_SPD_limit;
extern uint8_t u8g_ERR_remain;
extern uint8_t u8g_RUN_state;
extern uint8_t u8g_MOTOR_ctrlflg;
extern uint16_t u16g_MOTOR_errflg;
extern struct sPamControl pam;
extern	UI08 	uc10MSt_intct;				/*main確定(10ms毎)ｶｳﾝﾀ */

typedef struct{
  uint16_t u16_cnt_ms;
  uint8_t u8_cnt_s;
  uint8_t u8_cnt_min;
}ST_TIME;

typedef struct{
  uint8_t u8_cnt;
  GPIO_PinState io_old;
  GPIO_PinState io_fix;
}ST_SW;

extern void Ram_Init(void);
extern void TIME_Clr(ST_TIME* st_time);
extern void MOTOR_main_process(void);
extern void OUTPUT_Refresh(void);

/* static void MOTOR_CmdChk(void); */
extern void MOTOR_CmdChk(void);
extern void MOTOR_CtrlChk(void);

extern void MOTOR_Ctrl_FCT(void);
extern void MOTOR_Ctrl_Online(void);
extern void MOTOR_Ctrl_Offline(void);

/* static void SET_MOTOR_CMD(uint8_t u8cmd); */
extern void SET_MOTOR_CMD(uint8_t u8cmd);

extern void HEAT_Ctrl_FCT(void);
extern void UART_Ctrl_FCT(void);
extern void FCT_chk(void);

extern void vPMD_ForceStopAndInit(void);
extern void vPMD_SoftStopDuringFreeRun(void);

extern uint8_t SpdLimitOLP(uint8_t u8_spd);
uint8_t MOTOR_SpeedSet(uint8_t u8_spd);
uint16_t MOTOR_DuraCalc(float* f_tar);

extern void MOTOR_SetSpeedCommand(uint8_t dec);
extern uint8_t MOTOR_SanitizeSpeedCommand(uint8_t dec);
extern uint8_t MOTOR_SelectSpeedCommand(uint8_t ctrlflg, uint8_t TRNSset_spd_flg, uint8_t TRNS_sfrq_dec, uint8_t speed_cmd_offline);
extern uint16_t u16s_fct_cnt;                   /*fct计数*/

extern	UI08 	uc100MS_kdwaitct;			/*ﾘﾚｰON起動待機ﾀｲﾏ(10,1s) */
extern	UI08	uc100MS_pumpct;				/*給油改善制御期間規定用ｶｳﾝﾀ(150,15s) */
#ifdef __cplusplus
}
#endif

#endif
