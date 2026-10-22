#ifndef LOWSPEED_CTRL_H
#define LOWSPEED_CTRL_H
/**************************************************************************
<File Name>		:lowspeed_ctrl.h
<Function>		:低速制御中処理
<Program>		:
<Author>		:omi
<Date>			:2026.7.17
<Update>		:Ver1.00
<Copyright>		:Panasonic Co. , Ltd.
**************************************************************************/
/********************************************
		インクルードファイル
 ********************************************/
#include "mc_type.h"
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"

/*******************************************
		マクロ定義
 *******************************************/
#define ELEC_ANGLE_CYCLE_U16   (65536)      /* u16で表現した電気角1周（360度）相当の値 */
#define ELEC_ANGLE_HALF_U16   (32768)       /* u16で表現した180度相当の値 */
#define SECTOR_NUM    (6)                   /* 電気角を分割するセクタ数 */
#define SECTOR_WIDTH_U16 (ELEC_ANGLE_CYCLE_U16 / SECTOR_NUM)  /* u16で表現した1セクタ幅相当の値 */
#define SECTOR_MAX       (SECTOR_NUM - 1)   /* セクタ番号の最大値 */
#define MEC_PERIOD_SECTOR_NUM (18)          /* 機械角1周期分のセクタ数 */
#define CURRENT_LOOP_HZ  (ISR_FREQUENCY_HZ)	/* キャリア周期 */
#define SPEED_UNERI_THRESHOLD	(10)		/* 速度うねり判定閾値　1.0rps */
#define SPEED_ABNORMAL_THRESHOLD	(55)	/* 異常速度判定閾値　5.5rps */
#define SPD_STABLE_WAIT_CNT_TH_1MS	(5000)	/* 低速制御中の速度指令到達後の5秒待機カウント閾値 */
#define HOGO_CLR_CNT_TH_1MS	(300000)		/* 保護クリアカウント閾値　5分(1ms単位) */
#define INIT_MODE (1)                       /* 保護解除待機処理 初期化モード */
#define NORMAL_MODE (0)                     /* 保護解除待機処理 通常モード */
#define LOWSPD_CONTROL_START_SPEED (250)    /* 低速制御移行速度閾値 */
#define IIR_ALPHA_Q15   (6554)   /* 固定小数点Q15で、alpha = 0.2相当 */ 
#define IIR_ONE_Q15     (32768)  /* 固定小数点Q15で、1.0相当 */
#define TDC_CORRECT_BUFCNT (13)  /* 機械角周期内で推定速度が最小になった時の推定機械角位置と実際の上死点位置との機械角との差分 */

enum LowSpdCtrlState{
    LOWSPD_STATE_OUT_OF_RANGE,      /* M0 低速制御範囲外 */
    LOWSPD_STATE_ACCEL_DECEL,       /* M1 加減速中 */
    LOWSPD_STATE_WAIT_STABLE,       /* M2 速度安定待ち */
    LOWSPD_STATE_ECO_ENABLE,        /* M3 省エネ制御許可判定 */
    LOWSPD_STATE_PROTECT_HOLD       /* M4 保護保持/解除待ち */
};

/*******************************************
		変数宣言
 *******************************************/

/*******************************************
		プロトタイプ宣言
 *******************************************/
void vLowSpeed_Task_200us(int16_t hElAngle_q15, int16_t hMeasuredSpeed_q0); /* 低速制御中　200μs処理 */
void vLowSpeed_Task_1ms(int16_t Estspeed_q0, int16_t hTargetSpeed);    /* 低速制御中　1ms処理 */

extern uint8_t ucg_TDCflg;  /* 上死点フラグ */
extern uint8_t FirstcallAtEst_flg;  /* 推定処理初回呼び出しフラグ */

#endif /* #ifndef LOWSPEED_CTRL_H */
