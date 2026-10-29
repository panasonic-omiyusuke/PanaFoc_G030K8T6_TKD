/********************************************
		インクルードファイル
 ********************************************/
#include <stdint.h>
#include <stdbool.h>
#include "Sys_type.h"
#include "Hogo.h"								/* */
#include "PMD1GPar.h"							/*PMD系パラメータファイル */
#include "vipdc.h"
#include "Timea.h"
#include "main.h"
#include "Dtyctrl.h"
#include "Trans_Inv.h"
#include "Pmd1G.h"
#include "Pmd_int.h"
#include "Output_i.h"
#include "Torque.h"
#include "Speedcnt.h"
#include "system_ram.h"
#include "RefSpeed.h"
#include "PositionDetect.h"
#include "pwm.h"	/* for vPMD_Brake */
#include "adc.h"	/* for vPMD_Brake */
#include "pwm_i.h"
#include "Adpdc.h"
#include "stm32g0xx_it.h"
#include "Hogo.h"
#include "brake.h"
#include "InvCommonFunc.h"
#include "motor_process.h"
#include "motor_error.h"
#include "led.h"
#include "usart_control.h"
#include "pwm_curr_fdbk.h"

/******************************************************************************/
/*    ROM テーブル                                                            */
/******************************************************************************/
#define HI 1																										/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define LO 0																										/* @007MzDOD[16/12/21] */ /* 差圧起動 */
const UI08 judge_tbl[] = {LO, HI, LO, HI, LO, HI};																	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
const UI08 portsel_tbl[] = {SELTW, SELTV, SELTU, SELTW, SELTV, SELTU};	/* A/D検出ポートの選択 *//* @007MzDOD[16/12/21] */ /* 差圧起動 */
UI32 unPMD_MDOUT;
UI32 ulPMD_CCMR1;
UI32 ulPMD_CCMR2;
UI16 unPMD_CMP1_lap;						/*TmRC転流時機格納変数 */
UI16 unPMD_MCAP;							/*pd間隔格納変数 */
UI16 unPMDi_MCAPdumy;			/*MCAPﾀﾞﾐｰ(過去180ﾟe 分の60ﾟe の平均値) */
UI16 cspk_CMP2_2nd;
UI16 unPMD_CMP1;							/*TmRC転流時機格納変数 */
UI16 unPMD_CMP2;							/*TmRCpd始時機格納変数 */
UI16 unPMD_CMP3;							/*TmRCpd終時機格納変数 */
UI08 renzokuhassei;									/* Spike連続検出不可回数									*/
unsigned8   ucPMD_pdct;                 /* 位置検出回数カウンタ [Forced 状態]: {D_FCDPDCT  --  0} */

unsigned8	ucPMD_mdidx;				/* ﾓｰﾄﾞｲﾝﾃﾞｯｸｽ６通り. {0 -- 5} 位置検出毎に増or減. */
										/*0:UY 1:UZ 2:VZ 3:VX 4:WX 5:WY */
unsigned8	ADC_BRAKE_FLG;
/********************************************/
/*		変数宣言							*/		/* @037MzDOD[14/01/27] */
/********************************************/
/*** global ***/
//from midware
unsigned8	ucTYPE_undvlt_int;			/*直流電圧低下保護仕様	*/

#if SOFT_STOP_SW == ACTIVE
	UI16 free_run_count;
#endif

SI16	snPMD_basect;				/* Pmd処理系基準ﾀｲﾏ(3.3ms)ｶｳﾝﾀ. */
UI08 call_setpwm_flg;								/* @042MzDOD[15/07/27] */

unsigned8	ucPMD_boostmd;				/*(ｺﾝﾌﾟ起動時)boostﾓｰﾄﾞRAM */
		/*	０：ｾﾝｻﾚｽ禁止状態…停止中～強制転流中>強制転流回数ucPMD_fcdprdct ≧18で"ﾓｰﾄﾞ1"へ移行 */
		/*	１：ｾﾝｻﾚｽ移行状態…強制転流完～通常転流>連続位置検出回数 ucPMD_pdct ≧3で"ﾓｰﾄﾞ2"へ移行(同時にePMD_drvmode←D_running) */
		/*	２：ｾﾝｻﾚｽ運転状態…通常転流開始～boost完>42rps到達 or 起動後15秒経過で"ﾓｰﾄﾞ3"へ移行 */
		/*	３：boost完了状態…boost完了～停止>停止時に"ﾓｰﾄﾞ0"へ移行 */
		/*	F0：ﾛｯｸ　異常状態…起動後10～15秒間にDuty10.0%以下状態 */

/*from Pmd_int.c start*/
UI08 ucZX_ptn;													/* 通電パタン把握用 */
/*from Pmd_int.c end*/

/*from vi_phase_differential.c start*/
SI16 nVIC_ave_cur;								/* @263swDOD[23/07/26] */
SI16 nVIC_now_cur;								/* @263swDOD[23/07/26] */
SI16 nVIC_max_cur;								/* @263swDOD[23/07/26] */
SI16 nVIC_hold_cur;								/* @263swDOD[23/07/26] */
UI08 uneri_mode;								/* @263swDOD[23/07/26] */

enum tVctrModeTag {kNoVector, kReadyVector, kDriveVector, kOverMuchVector};//{kNoVector = 0, kReadyVector = 1, kDriveVector = 4, kOverMuchVector = 5};
enum tVctrModeTag ucPMD_VctrMode;
/*from vi_phase_differential.c end*/

/*from adc.c start*/
UI16 ad_start_timing_trc;													/* A/D?X?^?[?g????TRC?l???i?[ */
/*from adc.c end*/

/*from Pmd1G.c start*/
unsigned16	unPMDc_spdcrnt;				/* 実回転周期60ﾟe(但し MCAP 値⇒単位:μs). */
/*from Pmd1G.c end*/

/*soft_stop.c*/
#define START_FREERUN_RPS	(30) /* フリーラン移行速度 *//* @229swHOT[22/09/02] */
//#define IGNORE_CNT 			((UI08)((UI16)D_PWM_FREQUENCY_VECTOR/D_MTBUFSZ/START_FREERUN_RPS))	/* フリーラン移行直後はスパイク後のノイズで誤検出するため、30r/sの1転流機関の半分の時間として設定 */
#define IGNORE_CNT 			(9)	/* フリーラン移行直後はスパイク後のノイズで誤検出するため、30r/sの1転流期間の半分の時間として設定  10^6us/(START_FREERUN_RPS*(D_POLE*3))/pwm周期[us]/2 *//* @229swHOT[22/09/02] */
#define ERROR_COMMTATE_DATA  IGNORE_CNT							/* 1回目の位置検出 IGNORE_CNTの期間によりデータが正常でないため、エラーデータとして無視する *//* @229swHOT[22/09/02] */
#define COMMUTATE_PERIOD_1ST	(ERROR_COMMTATE_DATA + 1)		/* 1回目の正常な位置検出間隔データの取得 *//* @229swHOT[22/09/02] */
#define COMMUTATE_PERIOD_AVE	(COMMUTATE_PERIOD_1ST + 1)		/* 1回目の正常な平均位置検出間隔のデータ取得 *//* @229swHOT[22/09/02] */
#define ACCELERATE_1ST		(COMMUTATE_PERIOD_AVE + 1)			/* 1回目の正常な減速度のデータ取得 *//* @229swHOT[22/09/02] */
#define ACCELERATE_AVE		(ACCELERATE_1ST + 1)				/* 1回目の正常な平均減速度のエータ取得 *//* @229swHOT[22/09/02] */
#define WAIT_1_MACHINE_CYCLE	(ACCELERATE_AVE)				/* 機械角1回転を待って、加速度の最大値を探している状態。1回目の正常な平均減速度の取得委ができた状態からスタートなので、ACCELERATE_AVE と同じ値 *//* @229swHOT[22/09/02] */
//#define WAIT_1_MACHINE_CYCLE	(IGNORE_CNT + 6)				/* @229swHOT[22/09/02] */
#define WAIT_HALF_MACHINE_CYCLE	(WAIT_1_MACHINE_CYCLE + 1)		/* 最大の位置が終わりに近かったため最大位置でない可能性がある場合に追加で最大値を確認中の状態 */
#define WAIT_MATCH_MAX_INDEX	(WAIT_HALF_MACHINE_CYCLE + 1)	/* 1回転目以降の最大位置を確認している状態 ∧速度が閾値以下になるのを待っている状態 */
#define WAIT_LOWER_SPEED		(WAIT_MATCH_MAX_INDEX + 1)		/* 未使用 BRAKE_MOTOR設定するためだけに利用 */
#define BRAKE_MOTOR			(WAIT_LOWER_SPEED + 1)				/* ブレーキ中の状態 *//* @229swHOT[22/09/02] */
#define CARRIER_PERIOD_TR 100								/* キャリアの時間[us] 10kHz前提 クロスタイミングの補正用 */
#define SHORT_CIRCUIT_DUTY ((D_PWMPERIODfrc >> 4) * 15)		/* ブレーキ中のDuty 93.75%設定 コンプごとに見直し必要*/
//#define SHORT_CIRCUIT_DUTY (D_PWMPERIODfrc + 1)
//#define SHORT_CIRCUIT_DUTY (D_PWMPERIODfrc - (UI16)((D_PWMPERIODfrc+5)/10))
//#define BRAKING_SPEED_RPS (19) //3704=15rps　2924=20rps)
#define BRAKING_SPEED_RPS (20)  /* ブレーキ開始速度 rps 3704=15rps　2924=20rps) */
#define BRAKING_SPEED_US (1000000 / (3 * D_POLE) / BRAKING_SPEED_RPS) /* ブレーキ開始検出間隔[us] 3704=15rps　2924=20rps) */
#define D_MTBUF_END_INDEX    (D_MTBUFSZ - 1)		/* 機械位置の最終番号 */
#define D_MAX_INDEX			(D_MTBUFSZ >> 1)	/* 最大位置を固定するときの番号 */
#define D_MAX_INDEX_M1		(D_MAX_INDEX - 1)	/* バラつき判定用 最大位置(固定済) - 1 */
#define D_MAX_INDEX_N1		(D_MAX_INDEX + 1)	/* バラつき判定用 最大位置(固定済) + 1 */
#define D_MACHINE_TOP_INDEX	(D_MAX_INDEX + 1)	 /* 振動低減　上死点 */
/*soft_stop.c*/

UI16 unZX_lrd_tmr;							/* 遅れ進み判定用タイマ 					*/	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
UI08 ucZX_dlychk;							/* U下OFF後に立ち上がっているかﾁｪｯｸ用 		*/	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
UI08 ucZX_pdu;								/* 位置検出タイミング判定用 				*/	/* @007MzDOD[16/12/21] */ /* 差圧起動 */

//enum eFreeRunAdcMode {kFreeRunSelTuw, kFreeRunSelTvw, kFreeRunSelTvu, kFreeRunSelTwu, kFreeRunSelTwv, kFreeRunSelTuv, kFreeRunSelEnd};
enum eFreeRunAdcMode free_run_index; /* 0-5 2021/06/09 takeoka */
static UI08 machine_index;			/* 機械角の番号 0～(3*極数-1) 20deg/digit */
static UI08 max_accel_index;		/* 減速度が最大となる機械位置 都度更新 */
UI08 machine_pos_mode;		/* 停止制御の状態変数 以下の状態をとる
										0~IGNORE_CNT(ERROR_COMMTATE_DATA), COMMUTATE_PERIOD_1ST, COMMUTATE_PERIOD_AVE, ACCELERATE_1ST,
										ACCELERATE_AVE(WAIT_1_MACHINE_CYCLE), WAIT_HALF_MACHINE_CYCLE, WAIT_MATCH_MAX_INDEX, BRAKE_MOTOR   */
static SI16 prev_period_ave;		/* 前回の位置検出間隔の2回平均 減速度を計算する際に利用 減速度＝今回の平均間隔-前回の平均間隔 */
static SI16 prev_period;			/* 前回の位置検出間隔 位置検出間隔の2回平均を算出するのに利用 平均＝(前回+今回)/2 */
static SI16 prev_accel;				/* 前回の減速度 平均減速度を算出するのに利用 平均減速度＝（今回の減速度+前回の平均減速度）/2  */
static SI16 max_accel_ave;			/* 減速度の最大値 今回の1回転の減速度の最大位置を決定するのに利用 */

UI08 pos_detect_cnt;		/* 位置検出回数、1回転ごとにリセット 位置検出するごとにカウントアップ *//* @229swHOT[22/09/02] */
static SI16 prev_max_accel;		/* 前回の1回転の減速度の最大 この減速度を今回の最大が越えると値が前回の最大位置と前回の最大値が更新される */									   /* @229swHOT[22/09/02] */
static UI08 first_capture;		/* 0クロス補正計算用の初回サンプル取得フラグ */
static void InitSfrTrc(UI16 cmp3, UI16 cmp1_lap, UI16 cmp1, UI16 cmp2_2nd);

volatile SI16 prev_small;			/* 前回のAD変換結果 比較した結果が小となることを期待 ⇒ 基本的には prev_small > prev_large */
volatile SI16 prev_large;			/* 前回のAD変換結果 比較した結果が大となることを期待 */

volatile enum Brake_State u8g_brake_state; /*ブレーキ制御用状態*/

#define ADC_TU  	(ADC_CHANNEL_2 >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS)
#define ADC_TV  	(ADC_CHANNEL_3  >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS) /*STm正弦波の出力パターン都合のため。入力側だけVとWを反転*/
#define ADC_TW  	(ADC_CHANNEL_4  >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS) /*STm正弦波の出力パターン都合のため。入力側だけVとWを反転*/
#define ADC_PN  	(ADC_CHANNEL_10  >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS)
#define ADC_CT  	(ADC_CHANNEL_6  >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS)
#define ADC_SHUNT  	(ADC_CHANNEL_1 >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS)
#define ADC_VREFINT	(ADC_CHANNEL_13 >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS)

const static uint32_t ad_seq_u = {					/* ★AD変換順番決定テーブル U相？ */ /*　順番変更　220512サンプリング変更*/
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TU << ADC_REGULAR_RANK_2) |
		(ADC_PN << ADC_REGULAR_RANK_3) |
		(ADC_TU	<< ADC_REGULAR_RANK_4) |
		(ADC_PN << ADC_REGULAR_RANK_5) |
		(ADC_TU	<< ADC_REGULAR_RANK_6) |
		(ADC_PN << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const static uint32_t ad_seq_v = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TV << ADC_REGULAR_RANK_2) |
		(ADC_PN << ADC_REGULAR_RANK_3) |
		(ADC_TV	<< ADC_REGULAR_RANK_4) |
		(ADC_PN << ADC_REGULAR_RANK_5) |
		(ADC_TV	<< ADC_REGULAR_RANK_6) |
		(ADC_PN << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const static uint32_t ad_seq_w = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TW << ADC_REGULAR_RANK_2) |
		(ADC_PN << ADC_REGULAR_RANK_3) |
		(ADC_TW	<< ADC_REGULAR_RANK_4) |
		(ADC_PN << ADC_REGULAR_RANK_5) |
		(ADC_TW	<< ADC_REGULAR_RANK_6) |
		(ADC_PN << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const uint32_t ad_seq_table[] = {
		ad_seq_w,
		ad_seq_v,
		ad_seq_u,
		ad_seq_w,
		ad_seq_v,
		ad_seq_u,
};

#if SOFT_STOP_SW == ACTIVE

const static uint32_t ad_seq_uw = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TU << ADC_REGULAR_RANK_2) |
		(ADC_TW << ADC_REGULAR_RANK_3) |		/* @218swDOD[22/06/01] */
		(ADC_TU	<< ADC_REGULAR_RANK_4) |
		(ADC_TW << ADC_REGULAR_RANK_5) |
		(ADC_TU	<< ADC_REGULAR_RANK_6) |		/* @218swDOD[22/06/01] */
		(ADC_TW << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const static uint32_t ad_seq_vw = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TV << ADC_REGULAR_RANK_2) |
		(ADC_TW << ADC_REGULAR_RANK_3) |		/* @218swDOD[22/06/01] */
		(ADC_TV	<< ADC_REGULAR_RANK_4) |
		(ADC_TW << ADC_REGULAR_RANK_5) |
		(ADC_TV	<< ADC_REGULAR_RANK_6) |		/* @218swDOD[22/06/01] */
		(ADC_TW << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const static uint32_t ad_seq_vu = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TV << ADC_REGULAR_RANK_2) |
		(ADC_TU << ADC_REGULAR_RANK_3) |		/* @218swDOD[22/06/01] */
		(ADC_TV	<< ADC_REGULAR_RANK_4) |
		(ADC_TU << ADC_REGULAR_RANK_5) |
		(ADC_TV	<< ADC_REGULAR_RANK_6) |		/* @218swDOD[22/06/01] */
		(ADC_TU << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const static uint32_t ad_seq_wu = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TW << ADC_REGULAR_RANK_2) |
		(ADC_TU << ADC_REGULAR_RANK_3) |		/* @218swDOD[22/06/01] */
		(ADC_TW	<< ADC_REGULAR_RANK_4) |
		(ADC_TU << ADC_REGULAR_RANK_5) |
		(ADC_TW	<< ADC_REGULAR_RANK_6) |		/* @218swDOD[22/06/01] */
		(ADC_TU << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const static uint32_t ad_seq_wv = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TW << ADC_REGULAR_RANK_2) |
		(ADC_TV << ADC_REGULAR_RANK_3) |		/* @218swDOD[22/06/01] */
		(ADC_TW	<< ADC_REGULAR_RANK_4) |
		(ADC_TV << ADC_REGULAR_RANK_5) |
		(ADC_TW	<< ADC_REGULAR_RANK_6) |		/* @218swDOD[22/06/01] */
		(ADC_TV << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};
const static uint32_t ad_seq_uv = {
		(ADC_SHUNT << ADC_REGULAR_RANK_1) |
		(ADC_TU << ADC_REGULAR_RANK_2) |
		(ADC_TV << ADC_REGULAR_RANK_3) |		/* @218swDOD[22/06/01] */
		(ADC_TU	<< ADC_REGULAR_RANK_4) |
		(ADC_TV << ADC_REGULAR_RANK_5) |
		(ADC_TU	<< ADC_REGULAR_RANK_6) |		/* @218swDOD[22/06/01] */
		(ADC_TV << ADC_REGULAR_RANK_7) |
		(ADC_VREFINT	<< ADC_REGULAR_RANK_8)
};


const uint32_t ad_seq_free_run_table[] = {
		ad_seq_uw,
		ad_seq_vw,
		ad_seq_vu,
		ad_seq_wu,
		ad_seq_wv,
		ad_seq_uv,
};

#endif

/*******************************************
		プロトタイプ宣言
 *******************************************/
void vPMD_ForceStopAndInit(void);
void vPMD_SoftStopDuringFreeRun(void);
void FreeRunPositionDetcted(void); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
static inline UI16 AdjustFreeRunModeTimer(SI16 large, SI16 small, SI16 prev_large_in, SI16 prev_small_in);
SI16 GetNowPeriodAverage(SI16 now_period);
void SearchTopDeadCenter(UI16 adjust, SI16 large, SI16 small);
static void UpdatePrevVoltage(SI16 small, SI16 large);
SI16 GetNowAccelerationAverage(SI16 now_period_ave);
UI08 UpdateMaxAccelIfNeeded(SI16 now_accel_ave);
UI08 UpdateMachineIndex(UI08 index);
UI08 InitMachineIndex(UI08 index);
UI08 SearchModeStateMachine(UI08 mode);
static void MX_BRAKE_ADC1_Init(void);
static void MX_BRAKE_TIM1_Init(void);
static void RestartMcsdkAfterBrakeStop(void);
static void RestoreTim1SineRestartRegisters(void);
static void PrepareTim1ForMcsdkStop(void);

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2020/11/12
<Update>	:
**************************************************************************/
UI08 IsVoltBoosting(void){
	return (pam.duty_mode <= kStableBoost);
}

/* ブレーキ用の初期化 */
void InitShortCircuitBraking(void){
	PresetShortCircuitBrakingMdout(D_STANDBY_INDEX);  /* 短絡ブレーキのためのPWM設定用変数準備 */
	UpdateMdout();							/* 次のPWM出力を選択するレジスタの設定 */
	PWM_wait();                            /* 次のPWMポートへのDuty設定 */
	PWM_out();								/* PWM 出力 */
	htim1.Instance->BDTR &= ~TIM_BDTR_MOE; /* MOE禁止 */
}

/* 停止制御用の変数の初期化 */
void InitFreeRun(void){
	free_run_index = ((ucPMD_mdidx + 0) % 6);
	machine_index = 0;		/* machine degree */
	max_accel_index = 0;	/*top dead center(tdc) machine degree */
	machine_pos_mode = 0; 	/* 停止制御の状態変数の初期化 */
	max_accel_ave = 0;		/* 減速度の最大を格納 */
	prev_period_ave = 0;
	prev_period = 0;
	prev_accel = 0;
	prev_small = 0;  /* smallに比較結果がなることを期待している前回の変換結果  なので、prev_small > prev_large */
	prev_large = 0;  /* largeに比較結果がなることを期待している前回の変換結果 */
}

void InitFreeRunPwm(void){
	extern UI16 unPMDc_PWMprd;
	extern UI16 unPMDc_PWMprd_calc;
	extern UI16 addtct_trdgr[2];
	extern UI16 addtct_trdgr2[2];
	unPMDc_PWMprd = D_PWMPERIODfrc;          /* ブレーキ制御中の キャリアは10kHz */
	unPMDc_PWMprd_calc = D_PWMPERIODfrc;     /* ブレーキ制御中の キャリアは10kHz */
	first_half_timer = SHORT_CIRCUIT_DUTY;   /* ブレーキ中のDuty */
	unPMD_CMPU = SHORT_CIRCUIT_DUTY;		 /* ブレーキ中のDuty */
	PWM_U_TMREG = SHORT_CIRCUIT_DUTY;		 /* ブレーキ中のDuty */
	PWM_V_TMREG = SHORT_CIRCUIT_DUTY;		 /* ブレーキ中のDuty */
	PWM_W_TMREG = SHORT_CIRCUIT_DUTY;		 /* ブレーキ中のDuty */

	addtct_trdgr[0] = addtct_trdgr[1] = addtct_trdgr2[0] = addtct_trdgr2[1] = (D_PWMPERIODfrc >> 1);
	htim1.Instance->CCR4 = (D_PWMPERIODfrc >> 1);
}


void InitFreeRunTrc(void){
    InitSfrTrc(0xffff,0xffff,0xffff,0xffff);
}

/* 停止制御の初期化 */
void InitSoftStop(void){
	InitFreeRun();
	InitFreeRunTrc();
	InitFreeRunPwm();
	InitShortCircuitBraking();
}

/* フリーラン中の位置検出期待値用変数を一つ進める */
void NextFreeRunIndex(void){
	if( free_run_index >= D_POLE - 1 ) {
		free_run_index = 0;
	} else {
		free_run_index++;
	}

}
/**************************************************************************
<Title>			:UpdatePrevVoltage
<Function>		:通信データ用に最初の1回転の中で最も大きいlargeとsmallを見つける
<Description>	:各ブレーキ制御ごとに一番大きい端子電圧large, smallをprev_large, prev_smallにセットする
<Input>			:large, small
<Return>		:
<Author>		:Sakamoto
<Date>			:2026-07-29
<Update>		:
<Copyright>		:Panasonic Co. , Ltd.
 **************************************************************************/
static void UpdatePrevVoltage(SI16 small, SI16 large){
	if (large > prev_large){
		prev_large = large;
		prev_small = small;
	}
}

/*soft_stop.c*/
/* 停止制御のトップ関数 */
void SoftStopControl(SI16 large, SI16 small){
    UI16 adjust;

/*    return; */
    if(machine_pos_mode < IGNORE_CNT){ /* フリーラン移行後のノイズ除去用 */	/* @229swHOT[22/09/02] */
    	machine_pos_mode ++;												/* @229swHOT[22/09/02] */

    } else if(large >= small){ /* 端子電圧クロス発生 */						/* @229swHOT[22/09/02] */
		FreeRunPositionDetcted(); /* 位置検出期待値更新処理 */
		adjust = AdjustFreeRunModeTimer(large, small, prev_large, prev_small); /* 端子電圧クロス計算用のモードタイムの補正量 */
		SearchTopDeadCenter(adjust, large, small);
	}
}

/* 上死点探索処理 */
void SearchTopDeadCenter(UI16 adjust, SI16 large, SI16 small){
	SI16 now_period;
	SI16 now_period_ave;
	SI16 now_accel_ave;

	now_period = htim3.Instance->CNT - adjust; /*キャリア周期内での補正こみの転流間隔を保存*/
	htim3.Instance->CNT = adjust; /* タイマのリセット 既にタイマが補正分は進んでいるのでスタート補正値から  */
	now_period_ave = GetNowPeriodAverage(now_period);
	now_accel_ave = GetNowAccelerationAverage(now_period_ave);

	switch (machine_pos_mode){
	case WAIT_1_MACHINE_CYCLE:      		/* 最大加速区間探索中 */
	case WAIT_HALF_MACHINE_CYCLE:
		max_accel_index = UpdateMaxAccelIfNeeded(now_accel_ave); /* 加速度に応じて最大加速度更新 *//* @229swHOT[22/09/02] */
		machine_index   = UpdateMachineIndex(machine_index); /* それぞれのモードで処理が終わった後にインデックスを進める。インデックスが進んでいない状態を前提に処理を構成しているため *//* @229swHOT[22/09/02] */
		pos_detect_cnt++;									/* @229swHOT[22/09/02] */
		UpdatePrevVoltage(small, large); /* 通信データ用にprev_large, prev_smallを更新 */
		break;

	case WAIT_MATCH_MAX_INDEX:		/* 最大加速度位置が次の1回転でも一致するか確認中 */
		max_accel_index = UpdateMaxAccelIfNeeded(now_accel_ave);/* 加速度に応じて最大加速度更新 *//* @229swHOT[22/09/02] */

		pos_detect_cnt++; /* 一回転カウント用 *//* @229swHOT[22/09/02] */
		if( pos_detect_cnt >= D_MTBUFSZ ){ /*基本的には1回転ごとに最大値を更新。ただし、前の1回転の最大値を越える場合は最大位置が明らかになると考え更新する *//* @229swHOT[22/09/02] */
			pos_detect_cnt = 0;	/* 1回転したので、リセット *//* @229swHOT[22/09/02] */
			machine_index = InitMachineIndex(machine_index); /* はみ出していれば機械位置がずれているのでさらに1回転トライするため機械位置をずらす *//* @229swHOT[22/09/02] */

			/* 次の1回転の最大位置を探すための準備 *//* @229swHOT[22/09/02] */
			prev_max_accel =  max_accel_ave;	/* 次の一回転では前の一回転の最大値を越えたら都度更新。そうでなければ1回転した時点で更新 *//* @229swHOT[22/09/02] */
			max_accel_index = machine_index;	/* 次の一回転の最大位置をスタート位置にリセット⇒現在の位置に設定*//* @229swHOT[22/09/02] */
			max_accel_ave = now_accel_ave;		/* 次の一回転の最大位置をスタート位置の値にリセット *//* @229swHOT[22/09/02] */

		} else if( max_accel_ave > prev_max_accel ){	/* 前の一回転より大きいので、この回転での最大位置を即採用する ? 低速ほど最大位置が確認しやすく誤検出の確率が下がる *//* @229swHOT[22/09/02] */
			machine_index = InitMachineIndex(machine_index); /* 最大位置をD_MAX_INDEXに固定するために、machine_indexをずらす。D_MAX_INDEX±1を比較するのに0やD_MTBUF_SZ-1だとやりにくいため *//* @229swHOT[22/09/02] */
			max_accel_index = D_MAX_INDEX;		/* 最大位置がD_MAX_INDEXになるように更新しているので固定値をセット *//* @229swHOT[22/09/02] */
			prev_max_accel =  max_accel_ave;	/* 次に比較用の値を更新 *//* @229swHOT[22/09/02] */
		}

		if( now_period_ave >= BRAKING_SPEED_US ){ /* 速度閾値以下 */
			
			if( machine_index == D_MACHINE_TOP_INDEX ){ /* 機械位置が上死点 */
				if( !(htim1.Instance->BDTR & (TIM_BDTR_MOE)) ){ /* PWM許可されていないなら */
#if 1
					__HAL_TIM_MOE_ENABLE(&htim1);	/* ブレーキ出力開始 */
#endif
					}
			}
		}
		/* それぞれのモードで処理が終わった後にインデックスを進める。インデックスが進んでいない状態を全体に処理を構成しているため */
		machine_index = UpdateMachineIndex(machine_index);
		break;
	case BRAKE_MOTOR:							/* @229swHOT[22/09/02] */
		break;

	default:
		/* ここではインデックスは0に初期化する。⇒インデックスを進める関数を共通で呼び出せない */
		machine_index = 0;
		max_accel_ave = 0;
		max_accel_index = 0;
		pos_detect_cnt = 0;						/* @229swHOT[22/09/02] */
		break;
	}
	machine_pos_mode = SearchModeStateMachine(machine_pos_mode); /* 状態遷移 */
}

/* 加速度(Periodの差分 正が減速) のフィルタ用 前回の加速度と今回の加速度の平均を取得 */
SI16 GetNowAccelerationAverage(SI16 now_period_ave){
	SI16 now_accel;
	SI16 now_accel_ave;
	static SI16 period_correction;					/* マイナスの減速度の下限を超えた部分を次の転流への補正量として記録 *//* @229swHOT[22/09/02] */
#define LIMIT_ACCEL (25)							/* マイナスの減速度の下限 実験的に決定 *//* @229swHOT[22/09/02] */
	/* 加速度の差分を計算するための準備 */

	/*ノイズ除去のために加速度の平均を計算*/
	now_period_ave += period_correction;					/* 補正量は0か負の数なので、periodは短くなり速度は上がる *//* @229swHOT[22/09/02] */
	now_accel = now_period_ave - prev_period_ave;			/* 加速度を計算 マイナスなら加速、プラスなら減速  *//* @229swHOT[22/09/02] */
	now_accel_ave = (now_accel >> 1) + (prev_accel >> 1);	/* 加速度の平均を計算 */

	if( now_accel_ave + LIMIT_ACCEL < 0 && (machine_pos_mode >= WAIT_1_MACHINE_CYCLE) ){						/* 加速には上限を設ける *//* @229swHOT[22/09/02] */
		period_correction = (now_accel_ave + LIMIT_ACCEL) * 2;	/* 補正量は負。位置検出間隔の補正量を保存。早くなった分遅くなった区間を今回だけでなく、次回の位置検出間隔にも使う *//* @229swHOT[22/09/02] */
		now_period_ave -= period_correction;					/* 負の値を減算するので、periodは増えて、速度は遅くなる。位置検出間隔を訂正 *//* @229swHOT[22/09/02] */
		now_accel -= period_correction; 						/* 負の値を減算するので、accelは増加する。位置検出間隔を訂正した結果で加速度を再計算 *//* @229swHOT[22/09/02] */
		now_accel_ave = -LIMIT_ACCEL;							/* 加速度の平均は下限なのでLIMITに設定 *//* @229swHOT[22/09/02] */
	} else {					/* 補正不要 *//* @229swHOT[22/09/02] */
		period_correction = 0;	/* 補正量は0 *//* @229swHOT[22/09/02] */
	}							/* @229swHOT[22/09/02] */

	/*次回の計算のために今回のデータを保存*/
	prev_period_ave = now_period_ave;
	prev_accel = now_accel;

	/* 戻り値は加速度の平均 */
	return now_accel_ave;

}
/* 機械位置検出用の角度情報 */
UI08 UpdateMachineIndex(UI08 index){

	if (index >= D_MTBUF_END_INDEX){ /* 一回転したら終了 */
		return 0;
	}
	return (index + 1);
}
/* 機械角一回転の中の最大加速度の更新と機械位置の記録 */
UI08 UpdateMaxAccelIfNeeded(SI16 now_accel_ave){/* @229swHOT[22/09/02] */
	UI08 tmp_index = max_accel_index;			/* @229swHOT[22/09/02] */
	/* 加速度の差分の最大を求め、最大の場所が上死点 */
	if( max_accel_ave < now_accel_ave ){
		max_accel_ave = now_accel_ave;
		tmp_index = machine_index;				/* @229swHOT[22/09/02] */
//		HAL_GPIO_TogglePin(debug_SPIN1_GPIO_Port, debug_SPIN1);
	}
	return tmp_index;							/* @229swHOT[22/09/02] */
}

UI08 InitMachineIndex(UI08 index){	/* @229swHOT[22/09/02] */
	return ((index + D_MAX_INDEX + D_MTBUFSZ - max_accel_index) % D_MTBUFSZ); //index = (index - max_accel_index + D_MAX_INDEX) -> max_accel_index = D_MAX_INDEX/* @229swHOT[22/09/02] */
}		
/*soft_stop.c*/

/*vi_phase_differential.c*/
/**************************************************************************
<Title>			:vZX_chkspk
<Function>		:端子電圧状態でゼロクロス判断するためのフラグを立てる
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:2023/07/26 シャント電流検出によるうねり抑制制御追加 @263swDOD[23/07/26]
<Copyright>		:Panasonic Co. , Ltd.
 **************************************************************************/
void vZX_chkspk(void){											/* 端子電圧状態で判断するためのフラグたて関数 */
    switch(ucZX_ptn){													/* 通電パターンで分岐 */
    case DPTN_XOFF_LAP:
    case DPTN_UON_LAP:
        if(ucZX_pdu == TVLT_CLR){                                       /* スパイク検出なので初回のW相OFF後の立下りのみ分岐内の処理 *//* @263swDOD[23/07/26] */
            ucZX_pdu = TVLT_XOFF_SPK;																								/* @263swDOD[23/07/26] */
        }																															/* @263swDOD[23/07/26] */
        break;
    case DPTN_UOFF_LAP:													/* uvw xyz = 010 001 */
        /* 上下OFF区間で立下りを検出した */
        if(ucZX_pdu & TVLT_UOFF_EDG){									/* U相立下り検出後 誘起電圧0クロスはリンギングがあるため最終値立下りを取得 */
            ucZX_pdu |= TVLT_UOFF_PDU;									/* 誘起電圧0クロス検出 */
            unZX_lrd_tmr = unPMD_MCAP;								/* 誘起電圧0クロスの時間取得 *//*"unPMD_MCAP * 2"でなくてよいか要確認！　★瀧*/
        }
        else {															/* この区間で1回目の立下り */
            ucZX_pdu |= TVLT_UOFF_EDG;									/* U相立下り検出 */
        }
        /* この区間では誘起電圧割込みのリンギング発生時の最終を取得するために
		   U相上OFFの後の立下り後のみではなく毎回許可                        */
        break;

    case DPTN_XON_LAP:													/* uvw xyz = 010 101 */
    case DPTN_XON:														/* uvw xyz = 010 100 */
        if( !(ucZX_pdu & TVLT_UOFF_EDG) ){																/* @008MzDOD[16/12/21] */
            ucZX_pdu |= TVLT_UOFF_EDG;																	/* @008MzDOD[16/12/21] */
        }																								/* @008MzDOD[16/12/21] */
        else if( !(ucZX_pdu & TVLT_UOFF_PDU) ){															/* @008MzDOD[16/12/21] */
            ucZX_pdu |= TVLT_XON_EDG;										/* X相ON立下り検出 */		/* @008MzDOD[16/12/21] */
        }																								/* @008MzDOD[16/12/21] */
        break;

    default:
        break;
    }
}

/**************************************************************************
<Description>	:
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
 **************************************************************************/
//for D_forced
void SetIntLevelTrc(UI08 int_level){
#ifdef RENESAS_TO_ST32
	//割り込み優先レベルはツールを使った初期設定で実施
#else
    ucCPU_TRCIC = int_level;                                     /* ﾀｲﾏRC 割り込み優先ﾚﾍﾞﾙ = 4 */
#endif
}
/**************************************************************************
<Description>	:
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
 **************************************************************************/
void ClearTrc(void){
	htim3.Instance->CNT = 0;
	htim3.Instance->SR = 0x0000;
}
/**************************************************************************
<Description>	:
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
 **************************************************************************/
void StartTrc(void){
	htim3.Instance->CR1 |= TIM_CR1_CEN;
}
/**************************************************************************
<Description>	:
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
 **************************************************************************/
void ModeTime2Trcgra(UI16 mode_time){
	htim3.Instance->ARR = mMDTM2TRC(mode_time);
}
/**************************************************************************
<Description>	:
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
 **************************************************************************/
void ModeTime2Trcgrb(UI16 mode_time){
	htim3.Instance->CCR1 = mMDTM2TRC(mode_time);
}
/**************************************************************************
<Description>	:
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
 **************************************************************************/
void ModeTime2Trcgrc(UI16 mode_time){
	htim3.Instance->CCR2 = mMDTM2TRC(mode_time);
}

/**************************************************************************
<Description>	:
**************************************************************************/
void ModeTime2Trcgrd(UI16 mode_time){
	htim3.Instance->CCR3 = mMDTM2TRC(mode_time);
}



/**************************************************************************
<Title>			:IsBreakControlEnabled
<Function>		:ブレーキ制御が有効かどうかを判定する
<Description>   : BREAK制御有無判定関数
                  ucTRNS_stopctrlの下位5bitを参照し、
                  18未満ならBREAK制御あり(TRUE)、
                  18以上ならBREAK制御なし(FALSE)を返す
<Author>        : Takeoka
<Date>          :
<Update>        : 新規作成
**************************************************************************/
bool IsBreakControlEnabled(uint8_t ucTRNS_stopctrl_in) /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
{
	bool isEnabled; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	isEnabled = (ucTRNS_stopctrl_in < 18);
	return isEnabled;
}

/**************************************************************************
<Description>	:
<Author>		:Takeoka
<Date>			:
<Update>		:新規作成
 **************************************************************************/
static void InitSfrTrc(UI16 cmp3, UI16 cmp1_lap, UI16 cmp1, UI16 cmp2_2nd){
	htim3.Instance->CR1 &= ~TIM_CR1_CEN;
	ClearModeTimerIntRequest(TIM_IT_UPDATE);
	htim3.Instance->DIER = (MDTM_INT_ALL_ENABLE & ~MDTM_INT_LAP_ENABLE);

    ModeTime2Trcgra(cmp3);
    ModeTime2Trcgrb(cmp1_lap);
    ModeTime2Trcgrc(cmp1);
    ModeTime2Trcgrd(cmp2_2nd);

    SetIntLevelTrc(4);                                     /* ﾀｲﾏRC 割り込み優先ﾚﾍﾞﾙ = 4 */
    ClearTrc();                                     /* ﾀｲﾏ値ｸﾘｱ */       /* @013MwOTH[10/09/16] */
    StartTrc();
}

void ReEnableAdConvertIfNeed(void){
	uint32_t tmp_cr;
	
	tmp_cr = hadc1.Instance->CR;
	if( (tmp_cr &  (ADC_CR_ADDIS | ADC_CR_ADEN)) != ADC_CR_ADEN ) {
		while((hadc1.Instance->CR & ADC_CR_ADDIS) == ADC_CR_ADDIS );
		hadc1.Instance->ISR = ADC_ISR_ADRDY;
		hadc1.Instance->CR |= ADC_CR_ADEN;
		while((hadc1.Instance->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY);
	}
}

static void GuardBrakeAdTriggerCcr4(void)
{
	if (ADC_BRAKE_FLG != 0U)
	{
		if ((htim1.Instance->CCR4 == 0U) || ((htim1.Instance->CCER & TIM_CCER_CC4E) == 0U))
		{
			htim1.Instance->CCR4 = 1U;
			htim1.Instance->CCER |= TIM_CCER_CC4E;
			htim1.Instance->EGR |= TIM_EGR_UG;
		}
	}
}

static void RestoreTim1SineRestartRegisters(void)
{
	/*
	 * ブレーキ制御後の再始動で、正弦波復帰時に必要なTIM1主要レジスタを
	 * 既知の正常値へ明示復帰する。
	 * MC_StopMotor1() 内の R1_SwitchOffPWM() は、
	 * TIM1 がセンターアラインドでカウント継続している前提で
	 * 方向反転待ちを実施するため、ブレーキ用設定から戻しておく。
	 */
	TIM1->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);
	TIM1->CR1 |= TIM_CR1_CMS_0;
	TIM1->CNT = 0U;
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;
	
	TIM1->CR1 = 0x00000141U; /* 正弦波運転の前提となるTIM1基本モード(CEN/CMS/ARPE)を固定復元 */
	TIM1->RCR = 0x00000001U; /* Update発生周期を正弦波制御時の想定値(1)へ復元 */ 
	TIM1->CCR4 = 0x00000000U; /* CH4比較値を復元し、ブレーキ経路から復帰後のADC外部トリガ基準を合わせる */
	TIM1->CCR5 = 0x00000E8DU; /* CH5比較値(3725)を復元し、電流サンプリング位相を正弦波運転値へ戻す */
	TIM1->CCR6 = 0x00000FE6U; /* CH6比較値(4070)を復元し、電流サンプリング位相を正弦波運転値へ戻す */
	TIM1->EGR |= TIM_EGR_UG; /* 更新イベント(UG)を強制発生 */
}

/**************************************************************************
 * <Function Name> : vPMD_ForceStopAndInit
 * <Overview>      : インバータ強制停止処理（第1段階）
 * <Description>   :
 *   モータ駆動中に呼び出され、PWM出力を即座に停止し、
 *   インバータを安全に停止状態へ遷移させる。
 *   あわせて、電流・制御系パラメータを初期化し、
 *   再起動可能な状態へ復帰させる。
 *
 *   主な処理内容：
 *     ・運転モードを STOP 状態へ遷移
 *     ・PWM出力の停止
 *     ・ADC / DMA の停止およびリセット
 *     ・電流関連バッファのクリア
 *     ・各種制御変数の初期化
 *     ・タイマおよびADCの再初期化と再起動
 *
 * <Input>         : なし
 * <Output>        : なし（内部状態を更新）
 * <Author>        : Sakamoto
 * <Date>          : 2026/05/07
 * <Note>          :
 *   本関数は停止処理の開始（ハード停止）を担当する。
 *   必要に応じて後段処理でソフト停止制御を実施する。
 **************************************************************************/
void vPMD_ForceStopAndInit(void) {							/*強制停止処理 */
	ReEnableAdConvertIfNeed(); /* vPMD_PwmDisable()内で vPMD_Brake()の再帰条件を成立させないために実施  */
		/*DriveMode追加(2003/01/07-hide) */
		ePMD_drvmode = D_stop;					/*各種運転状態 -> stop 運転状態←＜Break＞ */
		snPMD_basect = D_BREAKLEN;				/*基準ﾀｲﾏｶｳﾝﾀ←強制停止期間3.3ms */
		ClearCurrentBuf();						/* 電流検出結果をｸﾘｱ */	/* @304swDOD[24/09/02] */
		current_wire = 0;						/* ｺﾝﾌﾟ電流平均値用変数をｸﾘｱ *//* @313syRRD[24/09/05] */
		ucPMD_boostmd = 0;

		/* ===== PWM停止 ===== */        
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);		
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		/* CH4はADC外部トリガ源として維持するため停止しない */
		HAL_TIM_Base_Stop(&htim1);
		__HAL_TIM_MOE_DISABLE(&htim1);
		htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E |
									 TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

		/* ===== DMA停止（最優先）===== */
		HAL_ADC_Stop_DMA(&hadc1);
		/* TIM1の割り込み/DMA要求設定は維持し、DMAチャネル（CH4/UP）のみ停止 */

        /* DMA1 の Channel5 を停止 */
	    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
		
		/* ===== DMA無効 ===== */
		hdma_adc1.Instance->CCR &= ~DMA_CCR_EN;

		/* ===== DMAフラグクリア ===== */
		DMA1->IFCR =
		DMA_IFCR_CGIF1 |
		DMA_IFCR_CGIF2 |
		DMA_IFCR_CGIF3 |
		DMA_IFCR_CGIF4 |
		DMA_IFCR_CGIF5 |
		DMA_IFCR_CGIF6 |
		DMA_IFCR_CGIF7;

		/* ===== ADC停止 ===== */
		HAL_ADC_Stop(&hadc1);

		/* ===== DMA先アドレス指定 ===== */
		/* DMA dedicated to ADC conversion */
		LL_DMA_SetMemoryAddress(DMA1, PWM_Handle_M1.pParams_str->DMA_ADC_DR_ChannelX, (uint32_t)ad_data_arr);
		//cstat !MISRAC2012-Rule-11.4
		LL_DMA_SetPeriphAddress(DMA1, PWM_Handle_M1.pParams_str->DMA_ADC_DR_ChannelX, (uint32_t)&ADC1->DR); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
		LL_DMA_SetDataLength(DMA1, PWM_Handle_M1.pParams_str->DMA_ADC_DR_ChannelX, ADC_SERIES_NUM);

		/* ===== ADC再初期化 ===== */
		MX_BRAKE_ADC1_Init();
		hadc1.Instance->CFGR1 |= (ADC_CFGR1_CHSELRMOD); /*bit21: 1*/
		hadc1.Instance->CFGR1 |= (ADC_CFGR1_EXTEN_0); /*bit10: 1*/
		hadc1.Instance->CFGR1 &= ~(ADC_CFGR1_SCANDIR); /*bit2: 0*/
		
		/* EXTEN[1:0] = 10 (Rising edge trigger) */
		hadc1.Instance->CFGR1 &= ~ADC_CFGR1_EXTEN;
		hadc1.Instance->CFGR1 |= ADC_CFGR1_EXTEN_0;

		__HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

		/* ===== DMA再設定 ===== */
		InitDtc();
		
		/* ===== ADC有効化 ===== */
		if (!(ADC1->CR & ADC_CR_ADEN))
		{
			ADC1->CR |= ADC_CR_ADEN;
			while (!(ADC1->ISR & ADC_ISR_ADRDY));
		}

		/* ===== TIM1再初期化 ===== */
		MX_BRAKE_TIM1_Init();
		AddInitPwmTimer();
		htim1.Instance->CCR4 = 1;

        call_setpwm_flg = 0;
#if SOFT_STOP_SW == ACTIVE
		if(u8g_MOTOR_ctrlflg & MOTOR_CTRL_FCT){
			free_run_count = FREE_RUN_COUNT_MAX_FCT; /*FCT検査13秒以内に*/
		}else{
			free_run_count = FREE_RUN_COUNT_MAX_NORMAL; /*2msタイマに変更*/
		}
        htim1.Instance->DIER &= ~TIM_DIER_BIE;
        htim1.Instance->EGR |= TIM_EGR_B2G;
#else
        htim1.Instance->DIER &= ~TIM_DIER_BIE;
        htim1.Instance->EGR |= TIM_EGR_B2G;
#endif
		/* ===== タイマ・ADC再起動 ===== */
		ADC1->CFGR1 |= (ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG);
		/*
		 * 直前の停止シーケンスで残る ADC ステータスをクリアする。
		 * OVR が残っていると変換は進んでも DMA 転送要求が止まる場合がある。
		 */
		ADC1->ISR = (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_EOSMP | ADC_ISR_OVR);

		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ad_data_arr, ADC_SERIES_NUM) != HAL_OK)
		{
			/*
			 * DMA 状態が不整合な場合に備えて再設定し、1回だけ再試行する。
			 */
			InitDtc();
			ADC1->ISR = (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_EOSMP | ADC_ISR_OVR);
			if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ad_data_arr, ADC_SERIES_NUM) != HAL_OK)
			{
				/*
				 * 再試行でも失敗する場合、DMA/ADC の再開条件を明示的に再アームする。
				 * 外部トリガ駆動のため、ADSTART を再発行して待機状態へ戻す。
				 */
				hdma_adc1.Instance->CCR |= DMA_CCR_TCIE;
				hdma_adc1.Instance->CCR |= DMA_CCR_EN;
				hadc1.Instance->CR |= ADC_CR_ADSTART;
			}
		}
		htim1.Instance->CCR5 = (D_PWMPERIODfrc >> 1); /* shunt電圧のトリガーブレーキ制御中のトリガ設定 */
		
		HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
		HAL_TIM_Base_Start(&htim1);
		GuardBrakeAdTriggerCcr4();
}

static void RestartMcsdkAfterBrakeStop(void)
{
	RestoreTim1SineRestartRegisters();
	MC_StopMotor1();

	/* 制御周期が止まるのを待つ */
	HAL_Delay(1);

	__disable_irq(); /* main loop用 */

	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_DeInit(&hadc1);

	HAL_TIM_PWM_DeInit(&htim1);
	HAL_TIM_Base_DeInit(&htim1);

	/* ADC1 */
	__HAL_RCC_ADC_FORCE_RESET();
	__HAL_RCC_ADC_RELEASE_RESET();

	/* DMA1 */
	__HAL_RCC_DMA1_FORCE_RESET();
	__HAL_RCC_DMA1_RELEASE_RESET();

	/* TIM1 */
	__HAL_RCC_TIM1_FORCE_RESET();
	__HAL_RCC_TIM1_RELEASE_RESET();

	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_MotorControl_Init();

	/* 念のため割り込みを再アーム */
	HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);
	HAL_NVIC_ClearPendingIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

	MC_AcknowledgeFaultMotor1();
	RestoreTim1SineRestartRegisters();
	MC_StopMotor1();

	/* ソフト側エラー状態を再始動可能状態へ復帰 */
	u16g_MOTOR_errflg = F_ERR_NOERR;
	u8g_ERR_remain = FALSE;
	u8g_RUN_state = STATE_NORMAL;
	u8g_LED_err = LED_ERR_NONE;

	__enable_irq(); /* main loop用 */
	chk_spike = kWaitZeroCross;
	ADC_BRAKE_FLG = 0;
}

/**************************************************************************
 * <Function Name> : vPMD_SoftStopEntry
 * <Overview>      : ブレーキ制御用
 * <Description>   :
 *   vPMD_Brake1 により強制停止された後に呼び出され、
 *   フリーラン中のモータ状態を監視しながらソフトブレーキ制御を行う。
 *   PWMを再利用して逆起電力（ゼロクロス）を検出し、
 *   滑らかにモータを停止させる。
 *
 *   主な処理内容：
 *     ・PWMブレーク機能の無効化（ブレーキ制御へ移行）
 *     ・転流／検出用タイミングレジスタのリセット
 *     ・フリーラン中の監視（free_run_count による制御）
 *     ・初回のみ停止制御の初期化処理を実施
 *         - PWMタイマの再スタート
 *         - ADCトリガ設定（PWM同期）
 *         - ゼロクロス検出モードへ移行
 *         - ソフトストップ制御の初期化
 *         - PWM出力の再有効化（制御用）
 *
 * <Input>         : なし
 * <Output>        : なし（内部状態を更新）
 * <Author>        : 成（変更・整理）
 * <Date>          : 2026/05/07
 * <Note>          :
 *   本関数は停止処理の後段（ソフト停止）を担当する。
 *   自然減速中の逆起電力を利用して回転状態を検出し、
 *   振動や逆転を抑えながら安全に停止させる。
 **************************************************************************/
void vPMD_SoftStopEntry(void){
    DisarmPwmTimerBreak();					/*ﾌﾞﾚｰｷ制御のためにBRK,BRK2を非実装化 */
    unPMD_CMP1_lap = 0xffff;
    unPMD_CMP1 = 0xffff;					/*転流ﾀｲﾐﾝｸﾞ格納ﾚｼﾞｽﾀ←ｸﾘｱ */
    unPMD_CMP2 = 0xffff;					/*位置検出ｻﾝﾌﾟﾘﾝｸﾞ開始ﾀｲﾐﾝｸﾞ格納ﾚｼﾞｽﾀ←ｸﾘｱ */
    unPMD_CMP3 = 0xffff;					/*強制転流ﾀｲﾐﾝｸﾞ格納ﾚｼﾞｽﾀ←ｸﾘｱ */
    GuardBrakeAdTriggerCcr4();
    /*__disable_irq();*//*割り込みの中では呼ばない*/					/* 処理開始前に念のため割り込み禁止 */	/*割込禁止 */
    //StartPwmTimer();						/* 処理チャンスを増やすため、PWMはあらかじめスタートしておく */

    init_addtct_trdgr();					/* AD変換をPWM周期で発生させるために設定 */

    chk_spike = kWaitFreeRunZeroCross;		/* モードを変更し、INTADDTCで停止制御を実行のため */
    InitSoftStop();
    ucPMD_mdidx = 0;
    FreeRunPositionDetcted();				/* 位置検出状態の初期化用に呼び出し */
    vPMD_PwmEnable();

    htim3.Instance->DIER &= ~MDTM_INT_ALL_ENABLE;
    /*__enable_irq();*/						/*割込禁止解除 */
    unPMDc_spdcrnt = 0xffff;				/*現60ﾟe周期←無限大 51rpm相当  01/01/15 */
    unPMDc_dutyn = 0;						/*Duty←ｸﾘｱ		~02-0206 -Outi */
    uc100MS_pumpct = 0;						/* 停止状態であるために設定必要 */				/*給油制御ｶｳﾝﾀ←0 */	/* @007MwOTT[10/06/07] */

    ADC_BRAKE_FLG = 1; /*ブレーキ制御用のADC変換設定のときに1*/
    GuardBrakeAdTriggerCcr4();

    TIM1->CNT = 0U; /*TIM1のカウンタをリセット*/
    htim1.Instance->EGR |= TIM_EGR_COMG; //update pwm out
}

/**************************************************************************
 * <Function Name> : vPMD_SoftStopDuringFreeRun
 * <Overview>      : インバータ停止制御処理（第2段階：ソフトブレーキ）
 * <Description>   :
 *   vPMD_Brake1 により強制停止された後に呼び出され、
 *   フリーラン中のモータ状態を監視しながらソフトブレーキ制御を行う。
 *   PWMを再利用して逆起電力（ゼロクロス）を検出し、
 *   滑らかにモータを停止させる。
 *
 *   主な処理内容：
 *     ・フリーラン中の監視（free_run_count による制御）
 *     ・初回のみ停止制御の初期化処理を実施
 *         - ソフトストップ制御の初期化
 *         - PWM出力の再有効化（制御用）
 *
 * <Input>         : なし
 * <Output>        : なし（内部状態を更新）
 * <Author>        : 成（変更・整理）
 * <Date>          : 2026/05/07
 * <Note>          :
 *   本関数は停止処理の後段（ソフト停止）を担当する。
 *   自然減速中の逆起電力を利用して回転状態を検出し、
 *   振動や逆転を抑えながら安全に停止させる。
 **************************************************************************/
void vPMD_SoftStopDuringFreeRun(void){
    if( free_run_count > 0 ) { /* ブレーキ制御中か判断 ダウンカウンタ 処理の最後でデクリメント */			
        free_run_count--;
    }else if (u8g_brake_state == BRAKE_STOPPING){
        /*ブレーキ制御完了(停止)*/
        u8g_brake_state = BRAKE_FINISHED;
        /*再起動に向けて正弦波用のMX_ADC_Init()を呼ぶ*/
        if(ADC_BRAKE_FLG){
            RestartMcsdkAfterBrakeStop();
        }
        u8g_brake_state = BRAKE_BANNED;
    }
}


/* フリーラン中の位置検出処理 */
void FreeRunPositionDetcted(void) {
	NextFreeRunIndex();
#if 0 /* 今後の検討で必要になる可能性ありのためｺﾒﾝﾄとして本モジュールを残す */
	if ((hadc1.Instance->CR & (ADC_CR_ADDIS | ADC_CR_ADEN)) != ADC_CR_ADEN) { /*ADが許可されているかチェック*/
		vPMD_Brake();
		return;
	}
	if ((hadc1.Instance->CR & (ADC_CR_ADSTP | ADC_CR_ADSTART)) == ADC_CR_ADSTART) {  /* AD停止信号を受け付ける状態か確認 */
		hadc1.Instance->CR |= ADC_CR_ADSTP;  /* 要検討 */
	}
#endif
	if(bStopAD() == 0){ //AD変換異常なら 基本的には発生しない
		chk_spike = kStopCheckSpike; //フリーランをvPMD_Brake内で、停止制御を再開するためには必要
		vPMD_Brake(); //AD再設定+停止制御へ移行
		return;
	}
	while(hadc1.Instance->CR & ADC_CR_ADSTART){  /* AD停止完了まで待機 */
		;
	}
	hadc1.Instance->ISR = ADC_ISR_CCRDY;  /*CHSELR変更前に必要*/
	hadc1.Instance->CHSELR = ad_seq_free_run_table[free_run_index]; /*新しい期待値取得のためのAD変換順を登録*/
	while(hadc1.Instance->ISR & ADC_ISR_CCRDY){ /*登録完了をチェック*/
		;
	}
	/*
	 * CHSELRを切り替えた直後はDMAリングバッファ位置が旧シーケンス側に残り、
	 * ad_data_arr[]の添字とランクの対応がずれることがあるため再同期する。
	 */
	ReinitDma();

}

/*soft_stop.c*/
/* 実際の端子電圧クロスタイミングを計算するための補正量[us]を計算する */
static inline UI16 AdjustFreeRunModeTimer(SI16 large, SI16 small, SI16 prev_large_in, SI16 prev_small_in){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	SI32 numerator;		/* 補正量計算の分子 */
	SI32 denominator;	/* 補正量計算の分母 */
	SI32 adjust;		/* 補正量 戻り値 */

	switch(free_run_index){
	case kFreeRunSelTuw:
	case kFreeRunSelTvu:
	case kFreeRunSelTwv:
	case kFreeRunSelTvw:
	case kFreeRunSelTwu:
	case kFreeRunSelTuv:
		// y = (large - prev_large_in)/CARRIER_PERIOD_TR*x + prev_large_in
		// y = (small - prev_small_in)/CARRIER_PERIOD_TR*x + prev_small_in
		// x = (prev_small_in - prev_large_in) * CARRIER_PERIOD_TR / ((large - prev_large_in) - (small - prev_small_in))
		// adjust = CARRIER_PERIOD_TR - x
		denominator = large - prev_large_in - small + prev_small_in;  /* 分母 Largeの変化量とSmallの変化量の絶対値の合計を分母 実際の値は絶対値とはならない可能性はあるが簡略化のため理論的に実装 */
		if(denominator == 0){
			adjust = 0;
			break;
		}

		numerator   = (large - small) * CARRIER_PERIOD_TR; 		/* 分子 LargeとSmallの差×キャリア時間[us] */
		adjust = numerator / denominator; 						/* モードタイムの補正量 検出タイミングから減算用 */
		//limit
		if( adjust > CARRIER_PERIOD_TR){
			adjust = CARRIER_PERIOD_TR;
		} else if( adjust < 0 ){
			adjust = 0;
		}
		break;
	case kFreeRunSelEnd: /* 異常処理 */
	default:
		adjust = 0;
		break;
	}
	return adjust;
}

/* クロス間隔のフィルタ用 前回のクロス間隔と今回のクロス間隔の平均を取得 */
SI16 GetNowPeriodAverage(SI16 now_period){
	SI16 now_period_ave;
	now_period_ave = (prev_period >> 1) + (now_period >> 1);
	prev_period = now_period;
	return now_period_ave;
}


/* 停止制御の状態遷移 */
UI08 SearchModeStateMachine(UI08 mode){
	/* 停止開始位置のオフセットがなければ、ブレーキ制御なし */
	if(!HasStopOffset()){ /* 0指令は受信後rpmhに反映されるまで時間がかかるため、オフセットあり->0指令になる。設定のタイミングで順番が逆転することは無い */
		return 0; /* mode=0 */
	}
	switch(mode){
	case WAIT_1_MACHINE_CYCLE:    /*機械角360度経過を待っている状態*/
		if(machine_index == 0){	/* 360度経過判定 */
			if( (max_accel_index >= (D_MTBUF_END_INDEX - 3)) || (max_accel_index <= 1)){  /* 加速度が最も遅いポイントが直近3回に含まれているか確認 *//* @229swHOT[22/09/02] */
				/* 速度が遅くなるほど減速が大きくなるため、
				 * 終わり際に最大点がある場合は減速区間がすべて含まれていない可能性がある。
				 * 減速区間全体が含まれるよう更に機械角60度判定を追加 */
				mode = WAIT_HALF_MACHINE_CYCLE;
			} else { /* 減速区間全体が含まれているのでここで機械位置探索終了 */
				mode = WAIT_MATCH_MAX_INDEX;
			}
		}
		break;
	case WAIT_HALF_MACHINE_CYCLE: /*追加の機械位置探索中*/
		if(machine_index >= 3){ /* 追加の60度が経過したか？ */
			mode = WAIT_MATCH_MAX_INDEX;
		}
		break;
	case WAIT_MATCH_MAX_INDEX: /* 上死点再確認中 次の上死点タイミングまで同じ値が維持されていたら確定 */
		if( htim1.Instance->BDTR & TIM_BDTR_MOE ){
			mode = BRAKE_MOTOR; //over write/* @229swHOT[22/09/02] */
		
		}else {
			mode = WAIT_MATCH_MAX_INDEX; //over write/* @229swHOT[22/09/02] */
		}
		break;
	case BRAKE_MOTOR:		/* @229swHOT[22/09/02] */
		mode = BRAKE_MOTOR;	/* @229swHOT[22/09/02] */
		break;
	default: /*フリーラン開始時のデータがそろうまでの待機状態 */
		if( mode >= (WAIT_1_MACHINE_CYCLE - 1) ){/* @229swHOT[22/09/02] */
			mode = WAIT_1_MACHINE_CYCLE;
		} else {
			mode++; /* accel　求めるために3つのperiodデータが必要 */
		}
		break;
	}
	return mode;
}
/*soft_stop.c*/


/*from main.c start*/
static void MX_BRAKE_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10666 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;	/* BKINの立上りを検知するためactive lo設定 *//* loアクティブ仕様 *//* @004lxOOM[24/06/12] */
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_DISABLE;			/* BRK2ではBKINをﾄﾘｶﾞにしないよう設定(※ﾚｼﾞｽﾀｵﾌｾｯﾄのままでは有効設定のため) */
  //  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim1.Init.Period / 2;						/* @183swDOD[21/11/12] */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;/* @004lxOOM[24/06/12] */
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;  /* @002lxORM[24/06/12] */
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;  /* @002lxORM[24/06/12] */
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
  sConfigOC.OCMode = TIM_OCMODE_PWM1; 
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;		/* ﾌｨﾙﾀ後のBRK入力に対してactive high設定 */
  sBreakDeadTimeConfig.BreakFilter = 0xe;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;			/* BRK2は使用しない(OSSI = OSSR = 0とするため) *//* @224swRRD[22/06/09] */
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;	/* ﾌｨﾙﾀ後のBRK2入力に対してactive high設定 */
  sBreakDeadTimeConfig.Break2Filter = 0xe;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

static void MX_BRAKE_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_CC4;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;/* @215swDOD[22/05/20] */
//  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;/* @215swDOD[22/05/20] */
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/*from main.c start*/
