#ifndef PMD1GPAR_H
#define PMD1GPAR_H

/**************************************************************************
<FileName>		:PMD1GPar.h
<Function>		:ﾊﾟﾗﾒｰﾀ設定用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0125
<Update>		:Ver REF045-2.xx 2004-06-12 -Outi 回転数ﾀﾞｳﾝ仕様ﾀﾞﾐｰ追加
				:Ver REF04C-0.xx 2005-01-23 -Outi VCT駆動中の通電角調整仕様の変更
				:Ver REF052-0.xx 2005-04-26 -Outi VCT中LAP制御改善
				:Ver REF052-0.xx 2005-05-16 -Outi 起動回転数低回転化対応
				:Ver REF052-1.xx 2005-06-09 -Outi 気柱共鳴音回避制御廃止対応
				:Ver REF052-1.xx 2005-06-15 -Outi 起動時回転加速度UP変更
				:Ver REF052-1.xx 2005-06-15 -Outi Dﾗｲﾝ共振点回避制御追加
				:Ver REF052-1.xx 2005-06-26 -Outi ｳﾈﾘ状態信号送信仕様追加
				:Ver REF058-0.xx 2005-07-05 -Outi サンケンＩＰＭ対応
				:Ver REF061-0.xx 2006-04-13 -Outi 2ﾚﾍﾞﾙﾘﾚｰ廃止対応(R1回転数廃止)
				:Ver REF070-0.01 2007-04-09 -Tomo 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.01 2008-05-30 -Outi OUTptn(MDOUT)適正化
				:Ver REF081-0.02 2008-06-05 -Outi 信号検出処理修正	@000ahOTM[08/06/05](発覚08/05/21)
				:Ver REF090-0.00 2009-04-03 -Outi 起動改善対応(SSL移行位置検回数18回化) @003awOOM[09/04/03]
				:Ver REF090-0.01 2009-05-11 -Outi IEC対応(60分統一<断続運転周期/通信異常判定期間>含む) @006axOOM[09/05/11]
				:Ver REF1000-0.00 2010-05-06 -Outi 176T対応 @002axOOM[10/05/06]
				:Ver REF1000-0.01 2010-05-25 -Mura 停止時振動対策 メイン通信処理追加  @008MwHOM[10/06/11]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/


/********************************************
		インクルードファイル
 ********************************************/
#include "vipdc.h"														/* 2008-06-12 竹岡 */

/*******************************************
		マクロ定義
 *******************************************/
/*基本パラメータ */
#define  D_fc  16							/*発振子周波数 16MHz */
#define  D_f32 32							/*クロック分周32分周 */
#define  D_f8  8							/*クロック分周 8分周 */
#define  D_f4  4							/*クロック分周 4分周 */
#define  D_f2  2							/*クロック分周 2分周 */
#define  D_f1  1							/*クロック分周 1分周(分周ﾅｼ) */
#define  D_fTRD		(32)					/* 32より大きくなる場合、OFFTIMEの変更必要★ */
#define  D_fTRC		(1)			/*TRC�ۯ�  2MHz �N���b�N���� 8���� */
/*位置検出機能関連 */
#define  D_MODECLOCK D_fTRC					/*ModeTimerClock用ﾚｼﾞｽﾀ設定値  */
#define  D_INTHWDLY  20						/*割込発生からmodule呼出迄のHW処理遅れ時間20clk(20/D_fc[usec]) */
#define  D_ENPDDLY   160					/*TRDmod呼出～位置検出許可迄のSW処理遅れ時間160clk(160/D_fc[usec]) */
#define  D_ENPDDLY2  128					/*位置検出許可処理のみのSW処理遅れ時間130clk(130/D_fc[usec])	@000ahOTM[08/06/05](発覚08/05/21) */

#define  D_REMVRING  (UI16)(14 * D_fTRD)			/*ringing除去時間 14us  14*D_fTRD[clk] */
#define  D_POSDELAY  (D_REMVRING - D_INTHWDLY - D_ENPDDLY)	/* IGBTｵﾝ遅れ補正時間 */
#define  D_POSDELY2  (D_REMVRING - D_ENPDDLY2)				/* IGBTｵﾝ遅れ&位置検許可処理遅れ補正時間 */
#define  D_RIngINTtm (D_REMVRING + (12 * D_fTRD))			/* TRD1割込(信号未確定時)処理時間 26us @000ahOTM[08/06/05](発覚08/05/21)*/
#define  D_LVLCHK2ON (D_PDLVLCHK2 + D_RIngINTtm)			/* 2度目のPDﾚﾍﾞﾙ判定許可pwm最小ｵﾝ幅	  @000ahOTM[08/06/05](発覚08/05/21)*/
/* #define  D_PWMONMIN (D_REMVRING + (10 * D_fTRD))	PWMｵﾝ幅許容最小時間 */	/* @006axOOM[09/05/11] */

#define  D_PDLVLCHK2 ((15 * D_fTRD) + 32)	/*2度目のPDﾚﾍﾞﾙ割込発生時機 trd1≡CMPU-D_PDLVLCHK2(Judge迄時間11us+最大命令clk数27clkを考慮)	@000ahOTM[08/06/05](発覚08/05/21)*/

#define  D_POSSAMPL  0						/*位置検知カウント方法 */
/*モータ定数関連 */
#define  D_POLE  6							/*ﾓｰﾀ極数 6 */
//#define  D_POLE  4                          //for test Jiaxipera GMCC
#define  D_MTBUFSZ  18						/*mode time ﾊﾞｯﾌｧﾉｻｲｽﾞ(機械角１回転当たりの極数：3*D_POLE) */
/*PWM制御関連(ｷｬﾘｱ＆Duty) */
#define D_PWM_FREQUENCY_DEFAULT (3000)												/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define D_PWM_FREQUENCY_LOW     D_PWM_FREQUENCY_DEFAULT								/* @007MzDOD[16/12/21] */ /* 差圧起動 */

/* #define D_PWM_FREQUENCY_HIGH    (6500)	*/											/* @007MzDOD[16/12/21] */ /* 差圧起動 */
/* #define D_PWM_FREQUENCY_HIGH    (10000)	*/											/* 20PBFｺﾝﾌﾟｷｬﾘｱ騒音対応、20PBFｺﾝﾌﾟ機種のみで10kHz設定 *//* @247swHOM[23/04/06] */
#define D_PWM_FREQUENCY_HIGH0    (10000)												/* ucTRNS_typeﾌﾗｸﾞによるｷｬﾘｱ切替え仕様実装 *//* @250swDOD[23/05/18] */
#define D_PWM_FREQUENCY_HIGH1    (6500)													/* ucTRNS_typeﾌﾗｸﾞによるｷｬﾘｱ切替え仕様実装 *//* @250swDOD[23/05/18] */

#define D_PWM_FREQUENCY_VECTOR  (10000)												/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define D_PWM_FREQUENCY_ECO     (2000)

//#define D_PWMPERIOD    (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_DEFAULT)	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
//#define D_PWMPERIODLo  (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_LOW)		/* @007MzDOD[16/12/21] */ /* 差圧起動 */
//#define D_PWMPERIODHi  (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_HIGH)		/* @007MzDOD[16/12/21] */ /* 差圧起動 */
//#define D_PWMPERIODfrc (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_VECTOR)	/* @007MzDOD[16/12/21] */ /* 差圧起動 */

/*通信送付用キャリア周波数*/
#define D_PWM_FREQ_DEFAULT_COM (D_PWM_FREQUENCY_DEFAULT/100)
#define D_PWM_FREQ_LOW_COM     (D_PWM_FREQUENCY_LOW/100)
#define D_PWM_FREQ_HIGH_COM    (D_PWM_FREQUENCY_HIGH/100)
#define D_PWM_FREQ_VECTOR_COM  (D_PWM_FREQUENCY_VECTOR/100)


//#ifdef TRQ_CTL										/* @012MzDOD[17/02/13] */
//#define  D_PWMPERIODTRQ D_PWMPERIODLo				/* トルク制御用キャリア */
//#define  D_PWMPERIODCHK D_PWMPERIODHi				/* トルク制御用キャリア */
//#endif

#define  D_DUTYMIN  50						/*Duty:  約5%　最小Duty(4.88%) */
#define  D_DUTYMAX  1000					/*Duty: 100.0%　最大Duty */
/*０停止(D_stop) */
/*１充電安定(D_standby) */
/*２位置決め制御(D_position) */
/*３強制転流制御(D_Forced) */
#define  D_FCDPRD  18518
#define  D_FCDPDCT  200						//250以下に抑えること //割込み処理から3.3msにモード移項条件を移したため、255に近い値では速度と極数によって正常動作にもかかわらずINTPDCで異常判定の可能性あり、 250以下に抑えれば理論上6極80r/sでも問題なし/*強制転流⇒通常転流遷移のための連続位置検出回数  200回 */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */

/*４転流120≦θ＜150制御(D_running) */
#define  D_PUMPAMPPRD 3						/*10ms (2002/12/27-hide) */
#define  D_PUMPAMPINC  5					/*Duty:  0.5%　給油改善制御時Dutyｱｯﾌﾟ率 5[‰/10ms] *//* TKD91E20PBF用設定 *//* @144swDOD[21/05/27] */
#define  D_PUMPAMPINC2 5					/*Duty:  0.5%　給油改善制御時Dutyｱｯﾌﾟ率 5[‰/10ms] */ /*~05-0615 -Outi 起動時回転加速度UP変更*/ /* 午前 09:56 REF052-1.xx */ /* @023awOTT[05/06/15] */
#define  D_PUMPRPM  33						/* ~03-0327 -Outi 給油改善制御中回転数  33回転（通常時）REF03Da(2U技術試作から変更) */
#define  D_PUMPMES  27						/* 圧縮機停止時振動抑制制御 停止基準位置検出用起動回転数 */ /* @008MwHOM[10/06/11] */
#define  D_BOOSTSPD	(1000000/D_PUMPRPM/3/ D_POLE)	/*~02-1231 -Outi給油改善制御中回転周期（通常時）  */
#define  D_PUMPINI	27						/* ~03-0630 -Outi 給油改善制御中回転数  27回転（ｲﾆｼｬﾙ時）_setrpm()AtPAM_MODE.sにて使用 */
#define  D_BOOSTINI	(1000000/D_PUMPINI/3/ D_POLE)	/*~02-1231 -Outi 給油改善制御中回転周期（ｲﾆｼｬﾙ時） */
#define  D_RUNPRD  1						/*通常運転中  Dutyﾁｪｯｸ周期  3.3ms ~02-0213 -Outi */
#define  D_ECORPMMAX  25					/*省エネ運転中最大回転数 ~03-0630 -Outi ｲﾆｼｬﾙ起動回転数変更 */
#define PUMPCT_END	(UI08)(150)				/* 給油改善制御時間*/	/* @012MzDOD[17/02/13] */

/*５強制θ＝150転流制御(D_vector) */
/*#define  D_SSLmaxlap	3*/	/*≧VCTMINLAP+2*/	/*11.25° SSL中最大LAP角 */
//#define  D_VCTMINLAP	1	/*＞0*/				/* 3.75°  VCT中最小LAP角 */	/*Overlap.c */
/*#define  D_VCTmaxlap	12*/	/*＞VCTMINLAP*/	/*45°  VCT中最大LAP角 */		/*Overlap.c */
/*#define  D_VCTWIDLAP	8*/	/*＞VCTMINLAP*/		/*30°  VCT中広角角判定ﾅﾝﾊﾞ */

/* MTCRA: Mode timer 制御レジスタ A */
#define  D_MTCRAini  (0x10 + D_MODECLOCK * 32)	/*~04-0317 -Outi 47F=>41Uﾏｲｺﾝ置換*/
#define  D_MTCRAssl  (0x12 + D_MODECLOCK * 32)	/*~04-0317 -Outi 47F=>41Uﾏｲｺﾝ置換*/
/* 起動時duty電圧変動対応*/
/*#define  D_VLOWLMT2	79	*/				/* 直流電圧下限ﾘﾐｯﾀ(全電圧) 218.1V (AC154V<AC220V-30%>相当) */
#define  D_VLOWLMT	23						/* 直流電圧下限ﾘﾐｯﾀ(倍電圧)  69.5V (AC 50V相当) */
#define  D_VHILMT		127					/* 直流電圧上限ﾘﾐｯﾀ(全電圧) 325.3V (AC115V相当) */
#define  D_VHILMT2		159					/* 直流電圧上限ﾘﾐｯﾀ(全電圧) 439.0V (AC310V相当) */
/*vSPD_H50sfrqfx() */
#define  D_TRERRJUDGETM  60					/*通信異常判定時間 */	/* @006axOOM[09/05/11] */
#if H50_DANZOKU == ACTIVE					/*H50断続運転アリ */	/* @006axOOM[09/05/11] */
 #define  D_H50DANZOKOFF  18				/*H50断続運転off時間 */	/* @006axOOM[09/05/11] */
#else /* H50_DANZOKU */						/*H50断続運転ナシ */	/* @006axOOM[09/05/11] */
 #define  D_H50DANZOKOFF  D_TRERRJUDGETM	/*H50断続運転off時間 */	/* @006axOOM[09/05/11] */
#endif /* H50_DANZOKU */
#define  D_H50DANZOKPRD  D_TRERRJUDGETM		/*H50断続運転周期 */	/* @006axOOM[09/05/11] */

#define  kHOGOKEIZOKU	13					/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間 異常判定値 */	/* @021ywDOD[18/01/25] */
											/* 要注意：ｲﾆｼｬﾙの通信確定待ちに10分待ちがある。この値より小さくすると、通信異常時に意図せずﾘｾｯﾄがかかる */
#define   DTC_CMPSTOP	0					/* @108MzDOD[14/05/27] */

#define	TRNSFAULTNUM  (100)					/* ｽﾀﾝﾊﾞｲ異常判定回数 */	/*ﾒｲﾝからｽﾀﾝﾊﾞｲ100回以上連続受信する場合は異常と判断 */	/* @086sxDOD[20/04/10] */
											/* ﾒｲﾝのEEPROM記憶処理時間(920msec) << ｽﾀﾝﾊﾞｲ100回連続送信時間(約16sec(=160msec×100)) となるように設定 */

#define D_BASIC_ANGLE_MODE (0)
#define D_GAP_ANGLE_MODE  (1)
#define D_LAP_ANGLE_MODE  (2)

#define SAMPLINGTIME	(2.844)   /* 要検討 名前変更 AD_CONV_TIME */ /*AD検討 220520*/
#define NUM_DTCVPN	 (3)													/* PN間電圧を検出するためのDTC繰り返し回数 */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define NUM_DTCVT	 (3)													/* 端子電圧を検出するためのDTC繰り返し回数 */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define NUM_DTCSHUNT (1)																									/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define NUM_DTCETC (1)															/* MAIN ｾﾝｻ系AD変換繰り返し回数 */
#define ETC_ADNUM (5)								/* MAIN ｾﾝｻ系ADﾎﾟｰﾄCH数 */	/* @002MzDOD[16/09/13] */


/*******************************************
		変数宣言
 *******************************************/

/*******************************************
		プロトタイプ宣言
 *******************************************/

/********************************************/
/*		外部変数参照						*/	/* @086sxDOD[20/04/10] */
/********************************************/

extern unsigned char	ucg_TR_stbyct;		/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀ(ｽﾀﾝﾊﾞｲ受信後のACK1送信ｶｳﾝﾀ) */	/* @086sxDOD[20/04/10] */

#endif //#ifndef PMD1GPAR_H
