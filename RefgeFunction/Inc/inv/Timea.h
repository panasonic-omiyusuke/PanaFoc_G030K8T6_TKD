/**************************************************************************
<FileName>		:TIMEA.h
<Function>		:TIMEA.c用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
				:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
				:Ver REF070-0.01 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-05-17 -Tomo 山田ＧＭご指摘対応、機種切換（通信）ループにタイムセーフ追加
<Copyright>		:Matsushita Refrigerator Co. , Ltd. 
**************************************************************************/
#ifndef TIMEA_H
#define TIMEA_H

/********************************************
		インクルードファイル
 ********************************************/
#include	"Sys_type.h"

/*******************************************
		マクロ定義
 *******************************************/

/*******************************************
		変数宣言
 *******************************************/
extern	UI08	uc100MS_pumpct;				/*給油改善制御期間規定用ｶｳﾝﾀ(150,15s) */
extern	UI16	un100MSt_trqct;				/* 機械位置判定用ﾀｲﾏ 	 */		/* @012MzDOD[17/02/13] */
extern	UI08	uc10MSt_dlnhgct;	 		/*Dﾗｲﾝ共振保護制御用ｶｳﾝﾀ */
extern	UI08	ucMINtm_dlnrlsct; 			/*Dﾗｲﾝ共振保護解除用ｶｳﾝﾀ */
extern	UI08	uc10MS_2lhgvltct;			/*２ﾚﾍﾞﾙ保護電圧継続時間ｶｳﾝﾀ(1秒,10ms) */
extern	UI08	uc100MS_2lhgct;			/*２ﾚﾍﾞﾙ保護開始時間ｶｳﾝﾀ(15秒 100ms) */
extern	UI08	uc100MS_2lfsct;				/*昇圧経過時間ﾀｲﾏ(10,1s) */
extern  UI16    un10MSt_2lhgct;
extern	UI08 	uc100MS_kdwaitct;			/*ﾘﾚｰON起動待機ﾀｲﾏ(10,1s) */
extern	UI08 	uc100MS_ryoffct;			/*(ｺﾝﾌﾟOFF後)ﾘﾚｰ切待機ﾀｲﾏ(30,3s) */
extern	UI08 	ucMINtm_trerct;				/*通信異常１時間計測ｶｳﾝﾀ<60,1時間>AtvTRNS_Cnttrns() */
//extern	UI08 	uc6Stmr_rsnntct;			/*気柱共鳴音回避制御ｶｳﾝﾀ(16,96秒) */
extern	UI08 	uc10MSt_intct;				/*main確定(10ms毎)ｶｳﾝﾀ */
extern	UI08 	uc10MSt_hzchkct;			/*指令Hz確定用ｶｳﾝﾀ(50,500ms) */
extern	UI08	ucTC1tm_dtyctrct;			/*Duty補正制御用ﾀｲﾏ(250,500ms) */
extern	UI08	uc6Stmr_hgrlsct;			/*保護動作ﾌﾗｸﾞｸﾘｱ用ｶｳﾝﾀ(50,５分) */
extern	UI08	uc6Stmr_wirehgct;			/*ｺﾝﾌﾟ巻線保護動作ﾌﾗｸﾞSET用ｶｳﾝﾀ(10,60秒) *//* @304swDOD[24/09/02] */
extern	UI08	uc10MSt_svhgct;				/*直流電圧低下保護確定時間ｶｳﾝﾀAtHOGO.c(10,100ms) */
extern  UI08	ucTC1tm_VctrCtr;	/* 矩形波ベクトル 転流周期カウンタ */
extern  unsigned8	ucTC1tm_spdctrlct;			/*目標回転数更新用ｶｳﾝﾀ  */
//extern  unsigned8	ucTC1tm_RckDtctCtr;			/*ﾛｯｸ検出用ｶｳﾝﾀ */	/*~04-0602 -Outi  午後 11:02 REF045-1.00 */
extern  unsigned8	ucMINtm_H50ct;				/*通信異常時強制off/on制御用ｶｳﾝﾀ（通信異常６０分継続時にｶｳﾝﾄUpするｶｳﾝﾀ）*/
extern  unsigned8	ucMINtm_HOGOct;				/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間の計測ｶｳﾝﾀ */	/* @021ywDOD[18/01/25] */
/* extern  UI08 ucZX_CTjudgeCnt2; */								/* CT回路異常確定用ｶｳﾝﾀ2(異常FLGA継続時間180分計数用ｶｳﾝﾀ)  *//* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
extern unsigned8	ucTC1tm_SPKrfrshct;			/*SPKﾘﾌﾚｯｼｭ用ｶｳﾝﾀ*/	/*~04-0602 -Outi  午後 07:54 REF045-1.00 */
extern unsigned8	ucTC1tm_VctrHogoCtr;		/*Vctor→Sensorless移行保護タイマカウンタ(2003/02/04-hide)REF036-0.12 */
//extern unsigned8	uc100MS_chgfgkepct;			/*通電角変更中Fg保持用ｶｳﾝﾀ*/	/*~04-0604 -Outi  午後 03:21 REF045-1.02 */
extern unsigned8	uc6Stmr_wdr15ct;			/*WDﾘｾｯﾄ発生用(６秒毎)ｶｳﾝﾀAtTIMEA.c */
extern unsigned16	un100MSt_typect;			/*機種切換（通信）ﾙｰﾌﾟﾀｲﾑｾｰﾌ(6000,600s)ｶｳﾝﾀ */
extern unsigned8	uc10MSt_sdobsvct;			/*連続送信監視ﾀｲﾏｶｳﾝﾄAtvTRNS_Cnttrns() (68,680ms) */
extern unsigned8	ucTC1tm_tmc;				/*通信ﾀｲﾏｶｳﾝﾀ AtvTRNS_Cnttrns */
extern  unsigned8	ucMINtm_stbyct;				/*ｽﾀﾝﾊﾞｲ連続異常計測ﾀｲﾏｶｳﾝﾀ	*//* @088swRRD[20/04/22] */

//from midware
/* ucMAIN_10upはPmd_midware.cでは参照していないため、別で持つ方が良い */
extern volatile UI08 ucMAIN_10up;				/*10ms経過ﾌﾗｸﾞ群     */
					/*   ---- ---1 : 	//10ms経過ﾌﾗｸﾞ（主） */
					/*   ---- --1- : 	//10ms経過ﾌﾗｸﾞ(従1） */
					/*   ---- -1-- : 	unused.              */
					/*   ---- 1--- : 	unused.              */
					/*   ---1 ---- : 	//10ms経過ﾌﾗｸﾞ(従4） */
					/*   --1- ---- : 	unused.              */
					/*   -1-- ---- : 	unused.              */
					/*   1--- ---- : 	//10ms経過ﾌﾗｸﾞ(従7） */

extern volatile UI08 ucg_MAIN_40up;
extern volatile uint8_t ucg_2ms_flg;


//extern unsigned8	ucTC1tm_UNEclrct;			/*UNE(ct)ｸﾘｱｶｳﾝﾀ*/	/*~04-0609 -Outi  午後 03:05 REF045-1.03 */

/*ついかはじめ*/
#define BASE_TIMER_COUNT 2 /*Base=2ms*/ 

#define	count_100ms	50	/*2ms*50=100ms*/
#define	count_40ms	20	/*2ms*20=40ms*/
#define	count_20ms	10	/*2ms*10=20ms*/
#define	count_10ms	(10/BASE_TIMER_COUNT)	/*2ms*5=10ms*/ /*除算結果が整数になるように*/
/*ついかおわり*/

/*******************************************
		プロトタイプ宣言
 *******************************************/
void vTIME_Timea(void);				/*ﾀｲﾏｶｳﾝﾀ基本処理 */
#endif
