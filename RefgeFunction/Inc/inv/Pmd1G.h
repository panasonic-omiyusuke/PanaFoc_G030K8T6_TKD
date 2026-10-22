/**************************************************************************
<FileName>		:PMD1G.h
<Function>		:PMD1G.c用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0125
<Update>		:Ver REF02A-0.00 2002-06-03
				:Ver REF045-1.00 2004-06-02 -Outi SPK検知仕様の追加
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
				:Ver REF052-1.00 2005-06-13 -Outi ﾊﾞｸﾞ修正
				:Ver REF052-1.01 2005-06-15 -Outi 起動時回転加速度UP変更
				:Ver REF070-0.01 2007-04-09 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.01 2008-06-05 -Outi 信号検出処理修正	@000ahOTM[08/06/05](発覚08/05/21)
				:Ver REF1000-0.01 2010-04-01 -Mura 2Level対応	@000MwDOD[10/04/01]
				:Ver REF1000-0.02 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/05]
				:Ver REF1000-0.03 2010-06-22 -Mura 停止オフセット受取りタイミング修正  @012MzORT[10/06/22]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/

#ifndef PMD1G_H
#define PMD1G_H
/********************************************
		インクルードファイル
 ********************************************/
//#include    "com_def.h"
#include "Sys_type.h"
#include "pwm.h"

/*******************************************
		マクロ定義
 *******************************************/
/*０停止(D_stop) */
#ifdef RENESAS_TO_ST32
#define  D_BREAKPTN  (T_MDOUTptn[D_STOP_INDEX])
#define  D_STNBYPTN  (T_MDOUTptn[D_STANDBY_INDEX])
#else
#define  D_BREAKPTN  0x0000					/*全出力停止ﾊﾟﾀﾝ */
#define  D_STNBYPTN  0x00e0					/*充電安定時出力パタン */
#endif

#define  D_BREAKLEN  1						/*2ms → 3.3ms　TC1が3.3msとなるため'1'は同じ(2002/12/27-hide) */
/*１充電安定(D_standby) */
#define  D_BOOTLEN	14						/*充電安定時間46.67ms(2002/12/27-hide) */
/*２位置決め制御(D_position) */
#define  D_POSPREPRD  1						/*位置決め制御前準備期間 2ms → 3.3ms */
#define  D_POS1STPRD  30					/*100ms(=30*3.333) (2002/12/27-hide) */
#define  D_POS2NDPRD  270					/*900ms(=270*3.333) (2002/12/27-hide) */
#define  D_POS1STPTN  0x262C				/*位置決め@ﾊﾟﾀﾝ VW-X (0x268a → 0x262C)*/	/* @010MzDOD[17/02/04] */
#define  D_POS2NDPTN  0x2428				/*位置決めAﾊﾟﾀﾝ  W-X (0x2488 → 0x2428)*/	/* @010MzDOD[17/02/04] */
#define  D_POS2ND_IDX 4
#define  D_POS1ST_IDX ((D_POS2ND_IDX + 5) % 6)


/*----------------------------
  143T用設定値(GINV2)
------------------------------*/
#define  D_POS1STDUTY00  50					/*Duty:  3.5%　位置決めDuty(巻込：143T) */	/* @008MzDOD[13/04/3] */
#define  D_POS2NDDUTY00  86					/*Duty:  5.9%　位置決めDuty(巻込：143T) */	/* @008MzDOD[13/04/3] */
#define  D_DUTYS00  117						/*Duty:  9.9 %　起動Duty(巻込：143T) */		/* @008MzDOD[13/04/3] */

/*----------------------------
  176T用設定値（GINVでは未使用)
------------------------------*/
#define  D_POS1STDUTY30  24					/*Duty:  2.8%　位置決めDuty(巻込：176T) */	/* @002axOOM[10/05/06] */
#define  D_POS2NDDUTY30  41					/*Duty:  4.8%　位置決めDuty(巻込：176T) */	/* @002axOOM[10/05/06] */
#define  D_DUTYS30  68						/*Duty:  8.0%　起動Duty(巻込：176T) */	/* @002axOOM[10/05/06] */

/*３強制転流制御(D_Forced) */
#define  D_FCDAMPPRD 3						/*10ms(2002/12/27-hide) */

/*#define  D_FCDAMPLAST  500	*/			/*Duty: 50% 強制転流緊急停止Duty */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define  D_FCDAMPLAST  800					/*Duty: 80% 強制転流緊急停止Duty */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
/*４転流120≦θ＜150制御(D_running) */
#define  D_LAPINI  0						/*ｵｰﾊﾞｰﾗｯﾌﾟ角 */
#define  D_LDANGINI  7						/*進角 */
#define  D_LDANGKYUYU 4						/*進角No. 給油時(ucPMD_boostmd >= 3 かつ、uc100MS_pumpct < PUMPCT_END)*/
#define  D_LDANGSPD	 3						/*進角 5 → 3 */		/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define  D_LDANGTDK	 4						/*進角 4 全電圧制御時 */
#define  D_LDANG_PWM_OFF 7                  /*PWMオフでの位置検出の際の進角 */
#define  D_LDCHNGSPD   29					/*進角変更指令回転数29r/s */
#define  D_LDCHNGDTY   600					/*進角変更duty 600‰(ﾌﾞﾛｺﾝtrip時65%を参考) */
#ifdef ECO_LDANG
#define  D_LDANGLOSPD0 4					/*低速時進角0 指令回転数≦D_LDCHNGSPD(pumpct<150) */
#define  D_LDANGLOSPD1 3					/*低速時進角1 指令回転数≦D_LDCHNGSPD(pumpct≧150∧dutyn＞D_LDCHNGDTY) */
#define  D_LDANGLOSPD2 2					/*低速時進角2 指令回転数≦D_LDCHNGSPD(pumpct≧150∧dutyn≦D_LDCHNGDTY) */
#define  D_LDANGHISPD  4					/*高速時進角  指令回転数＞D_LDCHNGSPD */
#else
#define  D_LDANGLOSPD0 3					/*低速時進角0 指令回転数≦D_LDCHNGSPD(pumpct<150) */
#define  D_LDANGLOSPD1 D_LDANGLOSPD0		/*低速時進角1 指令回転数≦D_LDCHNGSPD(pumpct≧150∧dutyn＞D_LDCHNGDTY) */
#define  D_LDANGLOSPD2 D_LDANGLOSPD0		/*低速時進角2 指令回転数≦D_LDCHNGSPD(pumpct≧150∧dutyn≦D_LDCHNGDTY) */
#define  D_LDANGHISPD  4					/*指令回転数D_LDCHNGSPD以上の時 (追加　2002/10/08-hide) */
#endif

#define  Sft_DUTY_120_TIME 13
/*#define  D_REMVRING  (40 * D_fTRD)*/				/*ringing除去時間 40us  40*D_fTRD[clk] */
#define  D_VLOWLMT2		79							/* 直流電圧下限ﾘﾐｯﾀ(全電圧) 218.1V (AC154V<AC220V-30%>相当) */
/*#define  D_VLOWLMT	23	*/						/* 直流電圧下限ﾘﾐｯﾀ(倍電圧)  69.5V (AC 50V相当) */
/*#define  D_VHILMT		127	*/						/* 直流電圧上限ﾘﾐｯﾀ(全電圧) 325.3V (AC115V相当) */
/*#define  D_VHILMT2	159	*/						/* 直流電圧上限ﾘﾐｯﾀ(全電圧) 439.0V (AC310V相当) */
#define  D_VCHKCNT	((D_VHILMT2 - D_VLOWLMT) / 2)	/* 起動時母線電圧検出回数(【検出速度:2[digit/3.3ms]】で最大変動幅を検出するための回数) */

#define D_HYSDUTY (5)														/* ｷｬﾘｱ切替用のdutyﾋｽﾃﾘｼｽ[%] */						/* @257swDOD[23/06/27] */
#define D_DUTY4HICARRIER0 ((100 * D_PWMONMIN / D_PWMPERIODHi0) + D_HYSDUTY)	/* 高ｷｬﾘｱ設定時に最小PWMｵﾝ幅を確保できるﾋｽﾃﾘｼｽを加えたDuty[%] *//* @257swDOD[23/06/27] */
#define D_DUTY4HICARRIER1 ((100 * D_PWMONMIN / D_PWMPERIODHi1) + D_HYSDUTY)	/* 高ｷｬﾘｱ設定時に最小PWMｵﾝ幅を確保できるﾋｽﾃﾘｼｽを加えたDuty[%] *//* @257swDOD[23/06/27] */
#define D_PWMON4HICARRIER0 (D_DUTY4HICARRIER0 * D_PWMPERIODLo / 100)		/* 高ｷｬﾘｱへ切替時のPWMｵﾝ幅閾値 */					/* @257swDOD[23/06/27] */
#define D_PWMON4HICARRIER1 (D_DUTY4HICARRIER1 * D_PWMPERIODLo / 100)		/* 高ｷｬﾘｱへ切替時のPWMｵﾝ幅閾値 */					/* @257swDOD[23/06/27] */
#define NUM_CHG2HICARRIER (10) 												/* PWMｵﾝ幅最小状態継続判定時間 10ms * 10 = 100ms */	/* @257swDOD[23/06/27] */
#define NUM_CHG2LOCARRIER (10) 												/* 高ｷｬﾘｱへ切替可能判定時間 10ms * 10 = 100ms */	/* @257swDOD[23/06/27] */

/*******************************************
		プロトタイプ宣言
 *******************************************/
void vPMDc_TC1(void);									/*TC1ﾀｲﾏ周期処理 */
void vPMD_start(void);
void vPMD_stop(void);
void vPMD_init(void);					/*PMD系初期化処理 */
void vPMD_setldang(void);										/*進角設定処理(2002/10/08-hide) */
void UpdatePwmPeriod(void);
/*******************************************
		変数宣言
 *******************************************/

extern UI08 call_setpwm_flg;								/* @042MzDOD[15/07/27] */
//extern eStateTag  ePMD_drvmode;			/* State of Pmd1G */
extern UI08	ucPMD_mdidx;				/* ﾓｰﾄﾞｲﾝﾃﾞｯｸｽ６通り. {0 -- 5} 位置検出毎に増or減. *//*0:UY 1:UZ 2:VZ 3:VX 4:WX 5:WY */

extern UI08	ucPMD_ldang; 				/*進角 */
extern UI16 unPMDc_spdcrnt;				/* 実回転周期60ﾟe(但し MCAP 値⇒単位:μs). */
extern UI08	ucPMD_boostmd;				/*(ｺﾝﾌﾟ起動時)boostﾓｰﾄﾞRAM */
extern SI16 snPMD_basect;				/* Pmd処理系基準ﾀｲﾏ(3.3ms)ｶｳﾝﾀ. */
//extern unsigned8	ucPMD_fcdprdct;				/*3rpsでの強制同期運転カウンター*/

//from midware
//extern UI08	ucTYPE_uart;				/* 通信機種判別確定	*/
extern UI08	ucTYPE_global;				/* 海外機種判別確定	*/

extern UI08 ucTYPE_position_duty;		/*位置決め方式及び起動Duty	*/

#endif
