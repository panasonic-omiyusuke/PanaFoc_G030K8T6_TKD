/**************************************************************************
<FileName>		:torque.h
<Function>		:torque.c用ヘッダファイル
<Program>		:
<Author>		:
<Date>			:2002-0227
<Update>		:
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/
#ifndef TORQUE_H
#define TORQUE_H
#include "Sys_type.h"
#include "PMD1GPar.h"


/*******************************************
		型宣言
 *******************************************/
enum trq_posmd {
	TRQ_POS1ST = 0,														/* 積算の最大を見つける */
	TRQ_POS2ND = 1,														/* 積算の2番目に大きい値を見つける */
	TRQ_POS3RD = 2,														/* 積算の3番目に大きい値を見つける */
	TRQ_POS_COMPARING = 3,														/* 積算の上位3つが連続で並んでいるか判定する */
	TRQ_POS_MISMATCH = 4,														/* 積算の上位3つが連続で無いときの判定を行う */
	TRQ_POSCLR = 5,														/* 積算結果をクリアし、基準位置または最大位置を確定する */
	TRQ_POSEND = 6														/* 基準位置または最大位置の判定処理が終了した状態を示す */
};

enum eTrqFixPosState {
	kTrqPosNormal,
	kTrqIntgErr1st,
	kTrqIntgErr2nd,
	kTrqIntgErr3rd,
	kTrqPosError,
	kTrqPosFixed
};

//enum eTrq2LvDutyModeTag {
//	kTrqDutyNormal,
//	kTrqDutyChange2Half,
//	kTrqDutyWaitUpdate,
//	kTrqDutyHalf
//};
enum eTorqueCtrlModeTag{
	kErrorTorqueCtrl,            //トルク制御必要速度指令で、エラー状態　位置が確定できない または トルク制御中に位置がずれている
	kNonTorqueUnsyncronizedPos,  //トルク制御不要速度指令 かつ 今回の起動で位置が一度も決まっていない
	kNonTorqueSyncronizedPos,    //トルク制御不要速度指令 かつ　今回の起動で位置が決まったことがある
	//ここまでがトルク制御なし、次からがトルク制御が必要な状態。条件分岐で必要なため状態追加時は要注意
	kPrepareSearchPos,           //トルク制御必要速度指令で速度が低下するまで待っている状態
	kSearchPos,                  //トルク制御必要速度指令で、基準位置を探している状態
	kSyncronizingPos,            //トルク制御必要速度指令で、基準位置が決まったので、トルク制御用位置を合わせている状態
	kTorqueCtrl,                 //トルク制御
};

/*******************************************
		マクロ定義
 *******************************************/
#ifndef TRQ_SUMRANGE
#define TRQ_SUMRANGE	(8)												/* 機械位置間隔を積算する個数 */
#endif
#define TRQ_WAITTM_1ST	(UI16)(30 * 10)									/* 24Hz指令から機械位置判定開始までの時間[s/10] */
#define TRQ_WAITTM_NXT	(UI16)( 5 * 10)									/* 機械位置仮決定後に次の判定開始までの時間[s/10] */
#define TRQ_OFF_LMT_SPD (24)											/* トルク制御OFF時の最低速度 r/s */			/* @002MxDOD[12/04/07] */
#define TRQ_CHKCRCTPOS	(16)											/* 補正量がピークとなる配列の番号 */
#define TRQ_INTEGCNTNUM (200)											/* 位置検出間隔の積算回数 1～253 */

#define TRQ_BASEPOS_NR			(4)										/* 初回決定基準位置との差の許容範囲 */
#define TRQ_DTCTPOS_NR			(2)										/* 基準位置決定時のバラツキ許容範囲 */

#define TRQ_CMPPOSNUM	(3)     //位置検出間隔の積算の上位箇所を格納する数 3=上位3か所
#define TRQ_MAXPOSHSZ	(3+3)   //位置検出間隔の積算の最大位置の履歴保存数
#define TRQ_CHANGE_DUTY	(6)		//右記を6に固定 大きな差なしと判断 6 + ((chgtmng++ & 0x03) == 0x03 ? 0 : 1);
#define TRQ_CHANGE_BASE_DTUY (3)


/*******************************************
		変数宣言
 *******************************************/
extern UI08 ucTRQ_stateflg;												/* トルク制御用フラグ */
extern UI08 ucTRQ_pos;													/* 積算位置検出間隔用の指数変数 */

/* 配列の数を一旦直値。直接外部から参照の必要がなくなるよう修正必要 */
extern UI16 unTRQ_crctarry[18];									/* 各区間のトルク補正量 */

/*******************************************
		関数プロトタイプ
 *******************************************/
UI16 TorqueControlCommutate(unsigned short usMdTime, UI16 now_duty);									/* トルク制御本体 + 仮機械位置決定関数 */
UI08 SetSpeedLimit(UI08 ucbuf);
void InitTrqCtrl(void);
UI08 IsSpeedTorqueControl(void);                          /* 目標速度がトルク制御が必要かどうかを判断 */
UI16 calc_dutyhosei(UI16 now_duty);
void FixMaxPos(void);													/* 機械位置最終決定関数 */
UI08 SpdLimitTrq(UI08 target_speed_rps);
UI08 IsTorqueFindingPos(void);
UI08 IsTorqueRunning(void);
void TorqueControlTc1(void);
#ifdef UNIT_TEST
void SetTorqueCtrlBasePos(void);
void IntgCommutatePeriod(void);
UI08 IsTorqueCtrlBanned(void);
UI08 IsTorquePositionFixed(void);
UI08 IsTorqueTargetDutyZero(void);
UI08 SyncTorquePosition(UI08 pos);
UI08 IsTrqErrorSpdLimit(void);
UI08 IsInvalidFixMaxPosition(void);
UI08 IsTrqPosError(void);
void ClearIntgFixMaxPosition(void);
void CalcIntgTop3(void);
UI08 IsTop3SeriesPosition(void);
enum eTrqFixPosState ErrTorqFixMaxPos(void);
UI08 FinalFixMaxPos(void);
void InitFixMaxPos(void);
UI08 NeedsExceptionFixMaxPos(void);
UI16 SetMaxTorqueDuty(void);
UI16 DecreaseTorqueDuty(UI16 now_max_torque_duty);
UI16 IncreaseTorqueDuty(UI16 now_max_torque_duty);
enum eTorqueCtrlModeTag SwitchTrqueCtrlMode(enum eTorqueCtrlModeTag mode);
#endif
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/16
<Update>	:
**************************************************************************/
/* 差分を計算する */
UI08 func_sub(UI08 a, UI08 b);

#endif //#ifndef TORQUE_H

