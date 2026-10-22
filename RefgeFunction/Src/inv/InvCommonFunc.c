/**************************************************************************
<FileName>		:InvCommonFunc.c
<Function>		:?��ʎY?��\?��t?��g?��?��?��ʊ֐�
<Program>		:
<Author>		:
<Date>			:
<Update>		:2002-0128
<Copyright>		:Panasonic Co. , Ltd.
**************************************************************************/

#include "Sys_type.h"
#include "Timea.h"
#include "InvCommonFunc.h"
#include "angle_control.h"
#include "Trans_Inv.h"
#include "system_ram.h"
#include "RefSpeed.h"
#include "main.h"
#include "PMD1GPar.h"
#include "Torque.h"
#include "motor_process.h"
#include "Output_i.h"

/*******************************************
		プロトタイプ宣言
 *******************************************/

extern sAngleDataTag angle;						/* 通電角に関する情報の構造体 */

extern unsigned8	ucTYPE_position_duty;		/*位置決め方式及び起動Duty	*/
extern unsigned8	ucTYPE_global;				/*海外機種判別確定	*/
extern UI08 ucTYPE_carrier_freq;	/* @250swDOD[23/05/18] */
extern unsigned8	ucTYPE_undvlt_int;			/*直流電圧低下保護仕様	*/

extern UI16 usAGC_cdang;				/* 通電角度：conduction_angleから名称変更 *//* 1536 ~ 2560 */
/* static */ UI08 timer_change_angle;
/* static */ UI16 usAGC_cdang_min;			/* 通電角の下限設定値：conduction_angle_minから名称変更 */
/* static */ UI16 usAGC_cdang_max;			/* 通電角の上限設定値：conduction_angle_maxから名称変更 */
/* static */ UI08 previous_speed_mode;

/* from midware */
unsigned8	ucTYPE_overlap;				/*ｵｰﾊﾞｰﾗｯﾌﾟ及び矩形波ﾍﾞｸﾄﾙ	*/

unsigned8	ucSPD_rpmfgmd;				/*回転数制御用ﾌﾗｸﾞ＆ﾓｰﾄﾞ設定RAM */
		/* ---- -XXX ;回転数制御ﾓｰﾄﾞ0-7  */
		/* ---- 1--- ;unused             */
		/* ---1 ---- ;回転数要減速ﾌﾗｸﾞ   */
		/* --1- ---- ;回転数要加速ﾌﾗｸﾞ   */
		/* -1-- ---- ;unused             */
		/* 1--- ---- :不等幅制御許可ﾌﾗｸﾞ */

const UI08 pitch_table[] = {2, 3, 4, 5, 25, 75, 100, 150};	/* rps偏差に応じた通電角更新周期設定値 */

struct sPamControl pam;

unsigned8	ucPMDi_run2stpfg;			/*ﾋﾟｽﾄﾝ位置固定停止準備完了Fg */	/* @006MwHOM[10/05/25] */
unsigned8	ucSPD_rpmpast;				/*3.3ms(INTTC1周期)前の回転数 from"_PAMHOGO" */

STATIC UI16	uc2LV_dlaytm;	 			/*RY-ON遅延ﾀｲﾏ */

/*from Speedcnt.c start*/
struct tSpeedDataTag speed;
/*from Speedcnt.c end*/

/*from Torque.c start*/
UI16 unTRQ_waittm;														/* 機械位置決定間隔 */
UI08 ucTRQ_mdtmidx;														/* 位置検出間隔保存配列用の指数変数 */
UI16 unTRQ_mdtm[TRQ_SUMRANGE];											/* 位置検出間隔保存用 */
UI16 unTRQ_summdtmN;													/* unTRQ_mdtmの合計 */
UI08 ucTRQ_pos;															/* 積算位置検出間隔用の指数変数 */
UI08 ucTRQ_syncmtbutpt;													/* 通常駆動用のmtbufptと機械位置を同期させるために差分を保持 */
UI08 ucTRQ_fixpos;														/* 最終的に確定した積算位置検出間隔の配列番号 */
UI08 ucTRQ_intgcnt;														/* 0:積算開始待ち、1～TRQ_INTEGCNTNUM:積算中の積算回数、TRQ_INTEGCNTNUM+1～2:積算終了 */
UI32 ulTRQ_intgmdtm[D_MTBUFSZ];											/* 位置検出間隔の過去8回分の和を積算していく */

UI16 unTRQ_crctarry[D_MTBUFSZ];																	/*@012MwRRD[11/05/18]*/ /*@014MwRRD[11/06/01]*/
UI16 unTRQ_crctduty;													/* トルク制御補正量 */	/*@012MwRRD[11/05/18]*/ /*@014MwRRD[11/06/01]*/
UI16 unTRQ_crct_duty_2lv;

enum eTorqueCtrlModeTag torque_ctrl_mode;

UI08 ucTRQ_intgclr_f;													/* 積算結果のクリアが必要かどうかを判定するフラグ */
/*from Torque.c end*/

/*ModeTimer.h start*/
unsigned8	ucPMDi_mtbufpt;				/* Mode time ﾊﾞｯﾌｧﾎﾟｲﾝﾀ. {0  --  (D_MTBUFSZ - 1)} */
unsigned16	unPMDi_mtbuf[D_MTBUFSZ];	/* mode time 平均値算出用 Mode time バッファ. */
extern TIM_HandleTypeDef htim3;
unsigned16  mt_dummy_cnt[D_MTBUFSZ];
/*ModeTimer.h end*/

/*pwm.c*/
UI08 Is1stStart(void);

enum ePamModeTag SwitchPamMode(enum ePamModeTag mode);
enum ePamModeTag pam_mode;
void ClearOrStopKdwatictIfNeeded(enum ePamModeTag mode);
void ClearOrStopRyoffctIfNeeded(enum ePamModeTag mode);
void ClearOrStopKdwatictIfNeeded(enum ePamModeTag mode);
void ClearOrStop2lfsctIfNeeded(enum eFailSafeModeTag mode);
UI08 GetStandardSpeed(void);
UI08 IsVoltBoosted(void);
UI08 IsSpeedResonance(void);
void PrepareStopCtrl(void);

/*from Speedcnt.c start*/
enum SpeedControlModeTag SwitchSpeedControlMode(UI16 target_speed_rp10s);                         /* 増減方向の決定 */
UI08 SwitchSpeedDifferntialMode(UI16 target_speed_rp10s);                     /* 速度差の大きさを決定 */

UI08 IsNowSpeedTarget(void){
    return (speed.differential_mode >= kSpeedDifferentialLittle4); //速度差が1r/s以下
}
UI08 IsNowSpeedOverTarget(void){		//現行速度が速度が目標より1rps以上
    return (hCurMecRps >= (ucSPD_rpmh * 10));
}
UI08 IsSpeedResonance(void){			//共振回転数か
	return (ucSPD_rpmfgmd & 0x40);
}
UI08 GetSpeedDifferentialMode(void){
	return (ucSPD_rpmfgmd & 0x07);
//	return speed_differential_mode;
}
/*from Speedcnt.c end*/

/* Pam_mode.c start */
enum ePamDutyModeTag SwitchPamDutyMode(enum ePamDutyModeTag mode);
/* Pam_mode.c end */

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/12/09
<Update>	:
**************************************************************************/
UI08 IsFullWave(void){
	extern struct sPamControl pam;
	return ( pam.relay_state == kRelayOFF );
}
UI08 IsDoubleVlt(void){
	extern struct sPamControl pam;
	return ( pam.relay_state == kRelayON );
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/12
<Update>	:
**************************************************************************/
enum ePamModeTag SwitchPamMode(enum ePamModeTag mode){
	//タイマ2つは統合しない-> ｋWaitOn2Off から直接ｋWaitStableOnに遷移する可能性があるため
	//利用する直前にタイマを確認する
	ClearOrStopRyoffctIfNeeded(mode);
	ClearOrStopKdwatictIfNeeded(mode);

	if(uc10MSt_intct != 0x0ff){		/* メインループを正常に処理していない場合（≒メインマイコンとの通信確定/非確定を未判断)*//* @324shDOD[25/01/15] */
		return kStopOff;			/* 2Lv制御有無の判断不可のためリレーOFF状態にする *//* @324shDOD[25/01/15] */
	}else{							/* メインループを正常に処理している場合（≒メインマイコンとの通信確定/非確定を判断済み) *//* @324shDOD[25/01/15] */
		/* 何もしない */			 /* 本行以降の2Lv制御モード判定処理を実施する *//* @324shDOD[25/01/15] */
	}								/* @324shDOD[25/01/15] */
	
	if(ucTYPE_2level == 0){			/* 2Level制御なし */ /* @010jxOOM[23/12/13] */
		return kForcedOn;
	}
	if(!IsSpeedDrive()){ 			/* ucSPD_rpmh == 0 && は不要*/
		if(IsFullWave() ){
			mode = kStopOff;
		} else {
			mode =  kWaitOn2Off;
		}
	}
	//2レベルリレー切替の本体
	switch (mode){
	case kWaitOn2Off:			/*倍電圧中にオンからオフへの待機状態 完全に止まってからオフするため時間待機*/
		if( uc100MS_ryoffct >= D_TIME_WAIT_ON2OFF_100MS ){
			mode = kStopOff;
		}
		//no break;				/*kWaitOn2Off から kWaitStableOn を考慮し break なし*/
	case kStopOff:				/*停止中でリレーがオフ状態。運転指令があれば、次の状態に遷移*/
		if( IsSpeedDrive() ){
			mode = kStableOn;
		}
		break;
	case kStableOn:				/*電圧安定待ち 所定時間経過でモータ起動OKな状態に遷移*/
		if( uc100MS_kdwaitct >= D_TIME_WAIT_STABLE_ON_100MS ){
			mode = kForcedOn;
		}
		break;
	case kForcedOn:										/*リレー強制オン状態で駆動中*/
		if( ((ucTRNS_sfrg & 0x06) == 0x00)				/*ではない->騒音測定モード(0x02) または 工程検査モード(0x04)*/
			&& (uc100MS_pumpct >= PUMPCT_END)			/*給油改善済み*/
			&& (pam.fail_safe_mode != kInFailSafe)){	/*フェイルセーフモードではない*/
			mode = kNormalRunningOn;					/*kRunningOff への遷移は kNormalRunningOn を経由*/
		}
		break;
	case kNormalRunningOn:								/*リレーオン状態　目標速度に応じてリレーをオフを許可*/
		if((u8g_SPD_set < D_MINHZRYON) && (hRampMecRps < D_MINHZRYON) && (ePMD_drvmode == D_running)){	/*駆動Hz < 閾値回転数 && D_running parameta仕様に変更 */ /*実回転数の条件を追加 260616*/
			mode = kRunningOff;
		}												/*(uc2LV_enry & 0x01) == 1) mode=kForcedOn;省略 フェールセーフ発生はエラー停止を経由するため運転中の切替は無し*/
		break;
	case kRunningOff:									/*リレーオフ状態　この状態からONした場合に限りフェールセーフ判定を行なう*/
		if((u8g_SPD_set >= D_MINHZRYON) && (hRampMecRps >= (D_MINHZRYON - D_MINHZRYONBUFFER))){
			mode = kNormalRunningOn;
		}
		break;
	default:											/*リレーオンの状態が最も安全のため、リレー強制ON状態とする*/
		mode = kForcedOn;
		break;
	}
	return mode;
}

/**************************************************************************
<Title>			:v2LV_2lflsafe 	;v2LV_Ctrl2lv
<Function>		:2ﾚﾍﾞﾙ･ﾌｪｰﾙｾｰﾌ処理
<Description>	:	ﾘﾚｰonから1秒以内に過電流保護(ucHOG_sttF.4)発生した場合にﾌｪｰﾙｾｰﾌﾌﾗｸﾞuc2LV_enry.0をSET
				:			　起動１秒前にﾘﾚｰはONしているが、”uc100MS_2lfsct(昇圧経過時間ｶｳﾝﾀ)”
				:			がｶｳﾝﾄ開始するのは、ePMD_drvmodeが"D_running"に変化した時となり
				:			D_runningに遷移してから１秒以内にIPMｱﾗｰﾑが発生すると
				:			"ﾘﾚｰONから１秒未満にIPMｱﾗｰﾑが発生した"と誤認識し、
				:			ﾌｪｰﾙｾｰﾌ･ﾓｰﾄﾞﾌﾗｸﾞをｾｯﾄしてしまう。
				:			　そこで、D_stop～D_Forcedの間は強制的に”uc100MS_2lfsct = 10;”して､
				:			D_runningに遷移した瞬間に既にﾘﾚｰONから１秒経過しているように
				:			しておくことで、ﾌｪｰﾙｾｰﾌ･ﾓｰﾄﾞﾌﾗｸﾞをｾｯﾄしないようなｿﾌﾄに変更した｡
<Input>			:uc2LV_enry.0(2LVLﾌｪｰﾙｾｰﾌ制御ﾌﾗｸﾞ)
<Return>		:-Outi	作成
<Author>		:2002-0204
<Date>			:Ver REF02A-0.00 2002-4-25
<Update>		:Ver REF03D-0.00 2002-09-10 -Outi 起動時過電流停止⇒次回ﾉｯﾁ停止までﾌｪｰﾙｾｰﾌ制御動作する不具合対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void v2LV_2lflsafe(enum ePamModeTag mode) {		/*enry 0x01 と 0x04 の組み合わせ*/	/*2lvlﾌｪｰﾙｾｰﾌ制御 */
	ClearOrStop2lfsctIfNeeded(pam.fail_safe_mode);

	switch(pam.fail_safe_mode){
	case kOverTimeMonitor:						/*倍電圧になってから1秒以上経過、起動前や起動直後など疑似的に1秒以上経過として監視対象から外す*/
		if(mode == kRunningOff){
			pam.fail_safe_mode = kUnmonitorFailSafe;
		}
		break;
	case kUnmonitorFailSafe:					/*全波整流でフェイルセーフの監視のが不要*/
		if(mode == kNormalRunningOn){		/*運転中に全波から倍電圧に遷移した場合にのみフェールセーフに移行*/
			pam.fail_safe_mode = kMonitorFailSafe;
		} else if(mode != kRunningOff){		/*全波整流で停止した場合には1秒以上に遷移*/
			pam.fail_safe_mode = kOverTimeMonitor;
		}
		break;
	case kMonitorFailSafe:
		if(mode == kRunningOff){
			pam.fail_safe_mode = kUnmonitorFailSafe;
		} else if(uc100MS_2lfsct >= D_TIME_FAIL_SAFE_MONITOR_100MS) {
			pam.fail_safe_mode = kOverTimeMonitor;
		} else if(ucHOG_sttF & 0x10){	/*pam.ctrl_mode == kNormalRunningOn or kWaitOn2Off*//*何かしらのエラーが発生したとき */
			pam.fail_safe_mode = kInFailSafe;
		}
		break;
	case kInFailSafe:
		if((ucSPD_rpmpast != 0) && (u8g_SPD_set == SPD_ZERO) && (ucHOG_sttF == 0)){ /*保護なしで停止した */	
			pam.fail_safe_mode = kOverTimeMonitor;
		} else {
		}
		break;
	default:
		pam.fail_safe_mode = kOverTimeMonitor;
		break;
	}
	ucSPD_rpmpast = u8g_SPD_set;								/*3.3ms前の駆動Hzをﾒﾓﾘに格納(前回駆動Hz←駆動Hz) */
}

/**************************************************************************
<Title>			:v2LV_Ctrl2lv 	;vMAIN_Inttc1pros
<Function>		:2ﾚﾍﾞﾙ切替制御
<Description>	:	運転ﾓｰﾄﾞとﾒｲﾝﾏｲｺﾝからの回転数指令により､ｺﾝﾌﾟ駆動系への駆動回転数を決定し､その回転数に応じてﾘﾚｰ(2LEVEL)の切替を行う｡
<Input>			:ePMD_drvmode, ucSPD_rpmh
<Return>		:uc2LV_enry.4
<Author>		:-Outi	作成
<Date>			:2002-0128
<Update>		:Ver REF02A-0.00 2002-04-25
				:Ver REF036-0.10 2003-01-07 -hide 新矩形波駆動用運転ﾓｰﾄﾞD_vector新設
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
				:Ver REF052-0.01 2005-05-16 -Outi 起動回転数低回転化対応
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void v2LV_Ctrl2lv(void) {									/*2lvlﾘﾚｰ切換制御 */

	/*vHOG_Shtvlt_judge();*/					/*直流電圧低下検知手段切替処理*/	/* @002MeDOD[10/05/09] *//* vHOG_Shtvlt_judge関数Call箇所注意。昇圧ﾌﾗｸﾞが設定された直後にCallされる必要有*/

    pam.ctrl_mode = SwitchPamMode(pam.ctrl_mode);
    pam_mode = pam.ctrl_mode;

	switch(pam.ctrl_mode){
	case kStopOff:
	case kRunningOff:
		pam.relay_state = kRelayOFF;
//		uc2LV_enry &= ~0x10;
		break;

	case kNormalRunningOn:
	case kStableOn:
	case kForcedOn:
	case kWaitOn2Off:
	default:
		pam.relay_state = kRelayON;
//		uc2LV_enry |= 0x10;		/*ﾘﾚｰ動作ﾌﾗｸﾞ ｾｯﾄ */
		break;
	}
	v2LV_2lflsafe(pam.ctrl_mode);										/*ﾌｪｰﾙｾｰﾌ処理CALL */

#if RLY_2LEVEL == ACTIVE		/* 2ﾚﾍﾞﾙﾘﾚｰ搭載 */													/* @002MxOOM[17/02/21] */
	if(uc10MSt_intct == 0x0ff){ /* メインループを正常に処理している場合（メインループに入る前の処理中は本分岐条件を満たさない) *//* @016jxOOM[24/01/23] */	/* @324shDOD[25/01/15] */
		v2LV_Drvry((UI08)(pam.relay_state));							/*	リレー駆動制御処理 	*/	/* @016jxOOM[24/01/23] */								/* @324shDOD[25/01/15] */
	}																																						/* @324shDOD[25/01/15] */
#endif /* RLY_2LEVEL*/
	pam.duty_mode = SwitchPamDutyMode(pam.duty_mode); /* v2LV_Caldutyから移動 状態が変わるのはここであることv2LV_CaldutyはDuty変更のみとするため */  
}			

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/12
<Update>	    :2026/6 正弦波の基本タイマ周期に合わせて判定時間を変更(2ms) sakamoto    
**************************************************************************/
enum ePamDutyModeTag SwitchPamDutyMode(enum ePamDutyModeTag mode) {

	if(ePMD_drvmode == D_standby){ //モードがずれたと時のための保護
		uc2LV_dlaytm = 0;
		return kBoosted;
	} else if ((mode <= kBoosted) && IsFullWave()) { //昇圧中から昇圧完了 状態で リレーがオフされた
		uc2LV_dlaytm = 0;
		return kDropping;
	} else if ((mode >= kDropping) && IsDoubleVlt()) { //降圧中から降圧完了でリレーがオンされた
		uc2LV_dlaytm = 0;
		return kWaitBoost;
	}

	uc2LV_dlaytm++;
	switch (mode) {
	case kWaitBoost:
		if (uc2LV_dlaytm >= D_2LV_ON_WAIT_TIME) { /*2msごと処理なので、2*4=8ms*/
			mode = kBoosting;
		}
		break;
	case kBoosting:
		mode = kStableBoost;
		break;
	case kStableBoost:
		if (uc2LV_dlaytm >= D_2LV_BOOSTED_TIME) { /*2msごと処理なので、2*250=500*/
			mode = kBoosted;
		}
		break;
	case kDropping:
		if (uc2LV_dlaytm >= D_2LV_DROPPED_TIME) { /*2msごと処理なので、2*250=500*/
			mode = kDropped;
		}	
		break;
	case kBoosted:
	case kDropped:
	default:
		break;
	}

	return mode;
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/12
<Update>	:
**************************************************************************/
void ClearOrStopKdwatictIfNeeded(enum ePamModeTag mode){
	if(mode != kStableOn){
		uc100MS_kdwaitct = 0;
	} else if(uc100MS_kdwaitct >= D_TIME_WAIT_STABLE_ON_100MS){
		uc100MS_kdwaitct = D_TIME_WAIT_STABLE_ON_100MS;
	}
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/12
<Update>	:
**************************************************************************/
void ClearOrStopRyoffctIfNeeded(enum ePamModeTag mode){
	if(mode != kWaitOn2Off){
		uc100MS_ryoffct = 0;
	} else if(uc100MS_ryoffct >= D_TIME_WAIT_ON2OFF_100MS){
		uc100MS_ryoffct = D_TIME_WAIT_ON2OFF_100MS;
	}
}


/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/12
<Update>	:
**************************************************************************/
UI08 IsVoltBoosted(void){ //昇圧完了したか
	return (pam.duty_mode == kBoosted);
}

/**************************************************************************
<Title>			:v2LV_Drvry		;vMAIN_Inttc1pros
<Function>		:�ڰ�쓮����
<Description>	:�ڰ��ON/OFF�쓮
<Input>			:uc2LV_enry
<Return>		:ucCPU_P1DR
<Author>		:-Outi	�쐬
<Date>			:2002-0128
<Update>		:Ver REF02A-0.00 2002-04-25
				:Ver REF081-0.00 2008-04-22 -Outi �Ȼ�MCU�u���Ή�
				:Ver REF1000-0.02 2010-01-20 -Mura 2Level�Ή�
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
#if RLY_2LEVEL == ACTIVE
void v2LV_Drvry(UI08 state) {									/* �����[�쓮���� 					*/
	if ( state == STATE_ON ){						/* リレーON中なら(uc2LV_enry.4)		*/
		  HAL_GPIO_WritePin(DRV_RELAY_GPIO_Port, DRV_RELAY_Pin, GPIO_PIN_SET);
	}else{												/* リレーOFF中なら(uc2LV_enry.4) 	*/
		  HAL_GPIO_WritePin(DRV_RELAY_GPIO_Port, DRV_RELAY_Pin, GPIO_PIN_RESET);
	}
}
#endif /* RLY_2LEVEL*/


/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/12
<Update>	:
**************************************************************************/
void ClearOrStop2lfsctIfNeeded(enum eFailSafeModeTag mode){
	if(mode == kUnmonitorFailSafe){
		uc100MS_2lfsct = 0;
	} else if(uc100MS_2lfsct >= D_TIME_FAIL_SAFE_MONITOR_100MS) {
		uc100MS_2lfsct = D_TIME_FAIL_SAFE_MONITOR_100MS;
	}
}

UI08 GetStandardSpeed(void){
	if(ucTYPE_Dline_point == 1){	/*Dﾗｲﾝ共振ポイント：45～49r/s	 */
		return D_R3_1;						/*H50中指令回転数←D_R3_1 */
	}else {												/*Dﾗｲﾝ共振ポイント：44～47r/s	 */
		return D_R3_0;						/*H50中指令回転数←D_R3_0 */
	}

}

/**************************************************************************
<Title>			:vPMD_Brake 		;vPMDi_Setcomtlap,INTTMR3,INTPDC,INT1,vPMD_stop,vPMDc_TC1
<Function>		:INV停止処理
<Description>	:	異常停止や主局ﾏｲｺﾝなどからの指令により停止する場合の処理
<Input>			:
<Return>		:ePMD_drvmode,unPMD_basect,ucSPD_sfrqfx,ucSPD_rpmh,unPMD_MDOUT,ucPMD_MTCRA,unPMD_CMPU,unPMD_CMP1,unPMD_CMP2,unPMD_CMP3,unPMDc_spdcrnt,unPMDc_dutyn
<Author>		:-Outi	作成
<Date>			:2002-0125
<Update>		:Ver REF02A-0.00 2002-06-03
				:Ver REF036-0.10 2003-01-07 -hide 新矩形波駆動用運転ﾓｰﾄﾞD_vector新設
				:Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				:Ver REF061-0.04 2006-06-01 -Tomo 残骸処理削除
				:Ver REF***-**.* 2007-07-13 -Tomo ルネサスソフト検討用
				:Ver REF090-0.00 2009-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]
				:Ver REF1000-0.01 2010-06-07 -Mura 停止時振動処理Resetバグ対策  @007MwOTT[10/06/07]
				:2023/07/26 シャント電流検出によるうねり抑制制御追加に伴いunZX_CTchkFlgCnt=0処理を削除 瀧田
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vPMD_Brake(void) {							/*強制停止処理 */
}

/**************************************************************************
<Description>	:給油制御ｶｳﾝﾀ処理
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
				:2023/10/10 通電角制御ﾌﾗｸﾞ割当て直し対応 瀧田 @272sxDOD[23/10/10]
 **************************************************************************/
//for running
void PumpCountStopIfNeeded(void){
	if(u8g_SPD_set != SPD_ZERO){				/*運転状態<Not停止中>で */
		if(uc100MS_pumpct >= PUMPCT_END){	/*給油制御済みなら */				/* @012MzDOD[17/02/13] */
			uc100MS_pumpct = PUMPCT_END;	/*給油制御ｶｳﾝﾀ←150 */				/* @012MzDOD[17/02/13] */
		}else{}
/*		if((ucTRNS_sfrg & 0x01) != 0){ */		/* 早送りﾓｰﾄﾞなら */										/* ﾒｲﾝﾏｲｺﾝから早送りﾓｰﾄﾞﾌﾗｸﾞ送信なしのため廃止 *//* @272sxDOD[23/10/10] */
/*			uc100MS_pumpct = PUMPCT_END; */	/* 給油制御済み 給油制御ｶｳﾝﾀ←150 *//* @012MzDOD[17/02/13] */	/* ﾒｲﾝﾏｲｺﾝから早送りﾓｰﾄﾞﾌﾗｸﾞ送信なしのため廃止 *//* @272sxDOD[23/10/10] */
/*		} */																								/* ﾒｲﾝﾏｲｺﾝから早送りﾓｰﾄﾞﾌﾗｸﾞ送信なしのため廃止 *//* @272sxDOD[23/10/10] */
//		if(ucHOG_sttF != 0){				/*保護状態であば */ // 保護が働くため、pumpct は 0 になるため削除。残していると保護が働いて速度が0になってもpumpctが0になるためrpmhが0以外で上書きされることになる
//			uc100MS_pumpct = 0;				/*給油制御ｶｳﾝﾀ←0 */
//		}
	}else{									/*<停止中>なら */
		uc100MS_pumpct = 0;					/*給油制御ｶｳﾝﾀ←0 */
	}

}

/*from ModeTimer.c start*/
/******************************************************************
<Title>			:SetDummyCntBuf
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:竹岡
<Date>			:2023/07/26
<Update>		:2023/07/26 除算切捨値の積算による実回転数算出ズレの修正  @276swRRD[23/11/21] 
<Copyright>		:Panasonic Co. , Ltd.
******************************************************************/
void SetDummyCntBuf(UI16 dummy_cnt){
   mt_dummy_cnt[ucPMDi_mtbufpt] = dummy_cnt; /* 除算切捨値の積算による実回転数算出ズレの修正 *//* @276swRRD[23/11/21] */
}
/*from ModeTimer.c end*/

/*Torque.c*/
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/16
<Update>	:
**************************************************************************/
void IntgCommutatePeriod(void){
	if(un100MSt_trqct < unTRQ_waittm){						/* 起動後もしくは基準位置決定後 にunTRQ_waittm時間だけ経過 */
		return;
	}

	if(ucTRQ_pos == 0){										/* 積算開始は機械位置が0になってからスタート */
		ucTRQ_intgcnt++;										/* 18分割した各機械位置の積算回数をカウントアップ */
	}
	if( ucTRQ_intgcnt > TRQ_INTEGCNTNUM ) {				/* 積算回数が積算を終了するTRQ_INTEGCNTNUM 回を超えた */
		ucTRQ_intgcnt = TRQ_INTEGCNTNUM + 1;					/* 積算終了を示す積算回数を保持。INTEGCNTNUM+1or+2⇒INTEGCNTNUM+1なので、INTEGCNTNUMの最大は253 */
	} else if( ucTRQ_intgcnt >= 1 ){ /* 積算回数が1～TRQ_INTEGCNTNUMの間、積算を実施⇒TRQ_INTEGCNTNUM回ずつ積算 */
		ulTRQ_intgmdtm[ucTRQ_pos] += unTRQ_summdtmN;		/* モードタイムの8回分の和を積算(要素数18) */
	}

	un100MSt_trqct = unTRQ_waittm;						/* 確実に積算を終了させるために経過時間を保持 */															/* ただし速度フィードバック制御で設定される基底Dutyは変更されない*/
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/16
<Update>	:
**************************************************************************/
UI08 IsTorqueCtrlBanned(void){ //トルク制御禁止か？
//	return (!NeedsTorqueControl() || IsAngleLapOrGap() || (ucTRQ_stateflg & 0x10));
//	return (!NeedsTorqueControl() || torque_ctrl_mode == kErrorTorqueCtrl);
	return (torque_ctrl_mode < kPrepareSearchPos);
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/16
<Update>	:
**************************************************************************/
UI08 SyncTorquePosition(UI08 pos){
	pos = pos + ucTRQ_syncmtbutpt;		/* 2011-04-13 竹岡 ucPMDi_mtbufpt に差分を加算 */
	if(pos >= D_MTBUFSZ){							/* 要素数を超えていた場合 */
		pos -= D_MTBUFSZ;							/* 2011-04-13 竹岡 バッファサイズに収まるように再計算 */
	}

	return pos;
}

/**************************************************************************
<Title>			:TrqCtrl
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:竹岡 義典
<Date>			:2010-06-15
<Update>		:
				:Ver REF1100-0.00 2011-06-15 -Mura 工程検査中トルク制御OFF対応 @018MxOTM[11/06/15]
 **************************************************************************/
UI16 TorqueControlCommutate(UI16 usMdTime, UI16 tmp_duty){
#if 1
	if( IsTorqueCtrlBanned() ){
		return tmp_duty;
	}
	ucTRQ_pos = SyncTorquePosition(ucPMDi_mtbufpt);
	tmp_duty += (UI16)(unTRQ_crctarry[ucTRQ_pos]);						//補正結果が100％を超える可能性があるが処理速度優先のため上限チェックなし /* トルク制御用デューティ補正量を加算 トルク制御終了時に徐々に減少させるためにここで処理が必要*//*@014MwRRD[11/06/01]*/

	if( IsNowSpeedTarget() ){							/* 実回転数が24±1r/s以内 */
		ucTRQ_mdtmidx = (ucTRQ_mdtmidx >= TRQ_SUMRANGE - 1 ? 0 : ucTRQ_mdtmidx + 1); /* 位置検出間隔保存配列用の指数変数更新 */
		unTRQ_summdtmN -= unTRQ_mdtm[ucTRQ_mdtmidx];					/* バッファの数を8個として、最も古いデータを和から削除= 7回分 */
		unTRQ_mdtm[ucTRQ_mdtmidx] = usMdTime;							/* 現在のモードタイムを最も古いデータがあった場所に格納 */
		unTRQ_summdtmN += usMdTime;										/* 現在のモードタイムを和に加算=8回分 */

		IntgCommutatePeriod();
	} else {
		un100MSt_trqct = 0;											/* 継続時間を0にクリアする */
		ucTRQ_intgclr_f = 1;										/* 積算時間をクリアする必要があるためクリア処理を実施するフラグをセット */
	}

	return tmp_duty;
#else
	/* 2011-04-13 竹岡 ucTRQ_posのずれを解消するためにucPMDi_mtbufptとの差分から計算 ここから -----------------------> */
	/* ucTRQ_pos ++;														トルク制御バッファポインタ（機械工程位置番号）を進める */
	ucTRQ_pos = ucPMDi_mtbufpt + ucTRQ_syncmtbutpt;						/* 2011-04-13 竹岡 ucPMDi_mtbufpt に差分を加算 */
	if(ucTRQ_pos >= D_MTBUFSZ){											/* 要素数を超えていた場合 */
		/* 	ucTRQ_pos = 0;														指数を0に変更する */
		ucTRQ_pos -= D_MTBUFSZ;											/* 2011-04-13 竹岡 バッファサイズに収まるように再計算 */
	}
	/* 2011-04-13 竹岡 ucTRQ_posのずれを解消するためにucPMDi_mtbufptとの差分から計算 ここまで <----------------------- */


	if(IsTorqueCtrlBanned()){
		un100MSt_trqct = 0;												/* 継続時間を0にクリアする */
		ucTRQ_stateflg &= ~0x02;										/* トルク制御不要速度指令のためフラグクリア */
		ucTRQ_intgclr_f = 1;											/* 積算時間をクリアする必要があるためクリア処理を実施するフラグをセット */
		return TmpDuty;
	}
	TmpDuty += (UI16)(unTRQ_crctarry[ucTRQ_pos]);						/* トルク制御用デューティ補正量を加算 トルク制御終了時に徐々に減少させるためにここで処理が必要*//*@014MwRRD[11/06/01]*/

/* 	if((ucSPD_rpmh <= TRQ_OFF_LMT_SPD) && (!(ucTRQ_stateflg & 0x10))){	指令速度がトルク制御必要速度 ∧ 基準位置エラーでない*/
//	if( NeedsTorqueControl() && IsAngleBasic() && (!(ucTRQ_stateflg & 0x10))){	/* 指令速度がトルク制御必要速度 ∧ 基準位置エラーでない*/
	ucTRQ_stateflg |= 0x02;											/* トルク制御必要フラグセット */

	ucTRQ_mdtmidx = (ucTRQ_mdtmidx >= TRQ_SUMRANGE - 1 ? 0 : ucTRQ_mdtmidx + 1); /* 位置検出間隔保存配列用の指数変数更新 */

	unTRQ_summdtmN -= unTRQ_mdtm[ucTRQ_mdtmidx];					/* バッファの数を8個として、最も古いデータを和から削除= 7回分 */
	unTRQ_mdtm[ucTRQ_mdtmidx] = usMdTime;							/* 現在のモードタイムを最も古いデータがあった場所に格納 */
	unTRQ_summdtmN += usMdTime;										/* 現在のモードタイムを和に加算=8回分 */

	if((ucTRQ_stateflg & 0x05) == 0x04){							/* マックス位置(最大となる8回和の最終位置)が確定 */
		if(ucTRQ_fixpos == ucTRQ_pos){								/* マックス位置とバッファ位置が一致 */
			SetTorqueCtrlBasePos();
		}
	}
	else if(((ucSPD_rpmfgmd & 0x07) >= 3)){							/* 実回転数が24±1r/s以内 */
		IntgCommutatePeriod();
	}
	else {
		un100MSt_trqct = 0;											/* 継続時間を0にクリアする */
		ucTRQ_intgclr_f = 1;										/* 積算時間をクリアする必要があるためクリア処理を実施するフラグをセット */
	}

	return TmpDuty;

		//	}
//	else {
//		un100MSt_trqct = 0;												/* 継続時間を0にクリアする */
////		ucTRQ_stateflg &= ~0x02;										/* トルク制御不要速度指令のためフラグクリア */
//		ucTRQ_intgclr_f = 1;											/* 積算時間をクリアする必要があるためクリア処理を実施するフラグをセット */
//	}
#endif
}
/*Torque.c*/
