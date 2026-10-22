#ifndef USER_MOTOR_ERROR_H
#define USER_MOTOR_ERROR_H

/******************************************************************
<File Name> ：motor_error.h
<Function> 	：马达异常相关定义
<Program> 	：
<Author> 	：董明鉴
<Date> 		：2024/10/10
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define ERR_NUM                         8                           /*异常个数*/

#define F_ERR_NOERR                     (0x0000)                    /*无异常*/
#define F_ERR_IPMERR                    (0x0001)                    /*IPM异常（硬件过流）*/
#define F_ERR_OVERVOLT                  (0x0002)                    /*过电压*/
#define F_ERR_UNDERVOLT                 (0x0004)                    /*低电压*/
#define F_ERR_OUTCTRL                   (0x0008)                    /*脱调*/
#define F_ERR_OVERCUR                   (0x0100)                    /*软件过流*/
#define F_ERR_PHASELOSS                 (0x0200)                    /*缺相*/
#define F_ERR_OVERPWR                   (0x0400)                    /*过负荷*/
#define F_ERR_OFFSET                    (0x0800)                    /*零点漂移*/               /* コンプレッサ巻線保護 */

/*Err_Jdg_UnderVolt start*/
#define UNDERVOLT_THRESHOLD_2LVON          162                        /*2lvリレーオン时の不足電圧保護閾値V*/ /*Low Voltage Protection Threshold when 2lv relay is on*/
#define UNDERVOLT_UNLOCK_THRESHOLD_2LVON   180                        /*2lvリレーオン时の不足電圧保護解除閾値V*/ /*Low Voltage Protection Unlocked Threshold when 2lv relay is on*/

#define UNDERVOLT_THRESHOLD_2LVOFF          90                        /*2lvリレーオフ时の不足電圧保護閾値V*/ /*Low Voltage Protection Threshold when 2lv relay is off*/
#define UNDERVOLT_UNLOCK_THRESHOLD_2LVOFF   100                        /*2lvリレーオフ时の不足電圧保護解除閾値V*/ /*Low Voltage Protection Unlocked Threshold when 2lv relay is off*/
/*Err_Jdg_UnderVolt end*/

/*Err_Jdg_OverVolt start*/
#define OVERVOLT_THRESHOLD_2LVON            420                     /*2lvリレーオン时の电压上限V*/ /*Over Voltage Protection Threshold when 2lv relay is on*/
#define OVERVOLT_UNLOCK_THRESHOLD_2LVON     400                     /*2lvリレーオン时の电压上限解除V*/ /*Over Voltage Protection Unlocked Threshold when 2lv relay is on*/
#define OVERVOLT_2LVOFF_APPLY_DELAY_MS      15000                     /*kDropped移行後のOV閾値遅延時間ms*/ /*Over Voltage Protection Threshold Delay when 2lv relay is off and kDropped*/

#define OVERVOLT_THRESHOLD_2LVOFF           200                     /*2lvリレーオフ时の电压上限V*/ /*Over Voltage Protection Threshold when 2lv relay is off*/
#define OVERVOLT_UNLOCK_THRESHOLD_2LVOFF    180                     /*2lvリレーオフ时の电压上限解除V*/ /*Over Voltage Protection Unlocked Threshold when 2lv relay is off*/
/*Err_Jdg_OverVolt end*/

/*Err_Jdg_OverPwr*/
#define POWER_SHUT                      250                         /*停机功率阈值250W*/
#define POWER_SHUT_NOOLP                250                         /*停机功率阈值250W*/
#define POWER_LIMIT                     220                         /*制限功率阈值280W*/ /*制限閾値>停止閾値により、制限処理を実質無効化26.7.1*/
#define POWER_RCV                       200                         /*制限恢复阈值200W*/
#define POWER_JDG_CNT                   5                           /*过功率判定/恢复/平衡用 连续计数阈值5ms*/
#define MOTOR_PWR_AVG_MIN_F             (0.0f)                      /*平均功率クランプ下限[W] float*/
#define MOTOR_PWR_AVG_MAX_F             (65535.0f)                  /*平均功率クランプ上限[W] float*/
#define MOTOR_PWR_AVG_MIN_U16           (0u)                        /*平均功率クランプ下限[W] uint16*/
#define MOTOR_PWR_AVG_MAX_U16           (65535u)                    /*平均功率クランプ上限[W] uint16*/

#define OFFSET_MIN                      (28319)                     /*零点偏置最小值 1.426V*/
#define OFFSET_MAX                      (38328)                     /*零点偏置最大值 1.93V*/

#define LOSS_CNT_START                  1000                        /*缺相检测 启动时检测时间 1s /1ms*/
#define LOSS_CNT_RUN                    9                           /*缺相检测 运转时检测时间 9ms /1ms*/


#define CURR_MAX                        (float)2.9                  /*过流检测阈值2.9A*/
#define CURR_OVERCNT                    30                          /*过电流检测时间30ms*/

#define OVERVOLT_NUM                    5                           /*过电压判定回数 5ms/1ms*/
#define UNDERVOLT_NUM                   5
#define OVERCUR_NUM                     30                          /*过电流判定回数 30ms/1ms*/

#define OFFSET_NUM                      10                          /*零点漂移判定回数 10ms/1ms*/
#define OLP_None 1
#ifdef OLP_None
#define OLPNUM_FC_SHUT                  15000                        /*工程检查模式过负荷停机判定回数 15s/1ms*/
#else
#define OLPNUM_FC_SHUT                  3000                        /*OLP保护器下，停机检测3s*/
#endif
#define OLPNUM_NOR_SHUT                 300000                       /*正常模式过负荷判定回数 300s/1ms*/

#define LOCK_CNT_START                  150                         /*启动阶段脱调保护时间150ms*/
#define LOCK_CNT_RUN                    30                          /*运转阶段脱调保护时间30ms*/
#define LOCK_BEMF_MAX                   (int32_t)(0.6 * 256)        /*脱调保护Ed/Eq最大值 0.6*/
#define LOCK_SPDUNIT_MIN                (15 * SPEED_UNIT)           /*脱调保护速度最小值15rps #SPEED_UNIT格式*/                 

#define ERR_REMAIN_CNT                  5                           /*异常持续时间5min*/

#define SPD_LIMIT_STATE_OFF             0U
#define SPD_LIMIT_STATE_ACTIVE          1U
#define SPD_LIMIT_STATE_RECOVERY        2U
#define SPD_LIMIT_STATE_BALANCE         3U

/* additional motor fault code */
#define  MC_SW_OVERCURR  ((uint16_t)0x1000) /**< @brief Error: Software over current. */
#define  MC_PHASE_LOSS   ((uint16_t)0x2000) /**< @brief Error: Phase loss. */
#define  MC_OFS_ERROR    ((uint16_t)0x4000) /**< @brief Error: Offset calculate error. */
#define  MC_OVER_PWR     ((uint16_t)0x8000) /**< @brief Error: Software over power. */

#define NUM_HOGO5MINSTP_RST    (3)   /* リセット発生させる保護停止5min待ち回数 */
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint16_t* u16_errflg;
    void (*errjdg[ERR_NUM])(uint16_t* errflg);
}MOTOR_ERR_JDG;

/* Private macro -------------------------------------------------------------*/

/* External variables ---------------------------------------------------------*/
extern MOTOR_ERR_JDG st_motor_err;
extern uint16_t u16g_MOTOR_errflg;                     /*马达异常标志*/
extern volatile uint16_t u16g_MOTOR_pwr_avg;           /*马达平均功率[W]（通信参照用）*/
extern uint8_t  u8g_pwr_rcv_flg;
/* External function prototypes -----------------------------------------------*/
extern uint8_t Motor_Err_Chk(void);
extern void Motor_Err_Jdg(void);
extern void SET_MotorErr_ucHOG_movF(void);
extern uint8_t IsResetByHogo5minStop(void); 

#endif /* USER_MOTOR_ERROR_H */
