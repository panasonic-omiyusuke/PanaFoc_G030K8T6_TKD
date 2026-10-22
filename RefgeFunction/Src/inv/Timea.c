/**************************************************************************
<FileName>		:TIMEA.c
<Function>		:ﾀｲﾏｶｳﾝﾀ･ｶｳﾝﾄUP制御
<Program>		:vTIME_Timea(), _TC1TMR(), _10MSTMR(), _100MSTMR(), _6STMR(), _1MINTMR(), _CNT10MS()
<Author>		:大内山 智則 -Outi
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
				:Ver REF036-0.00 2002-10-04 -hide VCT時転流周期更新用ｶｳﾝﾀ新設
				:Ver REF036-0.10 2003-02-04 -hide VCT⇒SSL帰還時保護用ｶｳﾝﾀ新設
				:Ver REF036-1.00 2002-12-27 -hide 基本ﾀｲﾏ2ms⇒3.3ms変更
				:Ver REF03D-0.00 2002-12-28 -Outi ﾀﾞｳﾝｶｳﾝﾀ仕様修正
				:Ver REF03D-0.01 2003-03-27 -Outi VCT⇒SSL帰還時保護用ｶｳﾝﾀﾊﾞｸﾞ修正
				:Ver REF03D-1.00 2003-06-05 -Outi 気柱共鳴音回避制御ｶｳﾝﾀ新設
				:Ver REF045-1.00 2004-06-02 -Outi SPKﾘﾌﾚｯｼｭ用ｶｳﾝﾀ 追加
				:Ver REF045-1.01 2004-06-02 -Outi ﾛｯｸ検出用ｶｳﾝﾀ 追加
				:Ver REF045-1.02 2004-06-03 -Outi 通電角変更中Fg保持用ｶｳﾝﾀ 追加
				:Ver REF045-1.03 2004-06-09 -Outi UNEchkｸﾘｱｶｳﾝﾀ 追加
				:Ver REF045-3.00 2004-07-06 -Outi 通信異常時強制運転仕様追加
				:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
				:Ver REF070-0.01 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-05-17 -Tomo 山田ＧＭご指摘対応、機種切換（通信）ループにタイムセーフ追加
				:Ver REF090-0.00 2008-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]
<Copyright>		:Matsushita Refrigerator Co. , Ltd. 
**************************************************************************/


/********************************************
		インクルードファイル
 ********************************************/
#include "vipdc.h"           /*2019-02-05 竹岡 add*/
#include  "Timea.h"						/*関数・変数定義 */
#include  "PMD1GPar.h"					/*PMD系パラメータファイル */

#include "Speedcnt.h"
#include "main.h"
/*さくじょはじめ*/
/* #include "Pmd1G.h" */
/*さくじょおわり*/

/*******************************************
		マクロ定義
 *******************************************/

/********************************************/
/*		変数宣言							*/		/* @037MzDOD[14/01/27] */
/********************************************/
/*** global ***/
unsigned8	ucTIME_tc1ct;				/*基準ﾀｲﾏｶｳﾝﾀ */
unsigned8	ucTIME_100msct;				/*0.1sﾀｲﾏｶｳﾝﾀ */
unsigned8	ucTIME_1sct;				/*１秒ﾀｲﾏｶｳﾝﾀ */
unsigned8	ucTIME_tmrcallct;			/*vTIME_10mstmr(),vMAIN_Cnt10ms()呼出用ｶｳﾝﾀ */
unsigned8	ucTIME_1stmrcallct;			/*_1stmr()呼出用ｶｳﾝﾀ */ /*＃ 午後 08:36 REF052-0.00 */
unsigned8	ucTC1tm_dtyctrct;			/*Duty補正制御用ﾀｲﾏ(250,500ms) */
unsigned8	ucTC1tm_tmc;				/*通信ﾀｲﾏｶｳﾝﾀ AtvTRNS_Cnttrns */
unsigned8	ucTC1tm_SPKrfrshct;			/*SPKﾘﾌﾚｯｼｭ用ｶｳﾝﾀ*/	/*~04-0602 -Outi  午後 07:54 REF045-1.00 */
/*unsigned8	ucTC1tm_UNEclrct;			UNE(ct)ｸﾘｱｶｳﾝﾀ	~04-0609 -Outi  午後 03:05 REF045-1.03 */
unsigned8	uc10MSt_hzchkct;			/*指令Hz確定用ｶｳﾝﾀ(50,500ms) */
unsigned8	uc10MSt_intct;				/*main確定(10ms毎)ｶｳﾝﾀ */
unsigned8	uc10MSt_svhgct;				/*直流電圧低下保護確定時間ｶｳﾝﾀAtHOGO.c(10,100ms) */
unsigned8	uc10MSt_sdobsvct;			/*連続送信監視ﾀｲﾏｶｳﾝﾄAtvTRNS_Cnttrns() (68,680ms) */
unsigned8	uc10MSt_dlnhgct;	 		/*Dﾗｲﾝ共振保護制御用ｶｳﾝﾀ */
unsigned8	uc100MS_kdwaitct;			/*ﾘﾚｰON起動待機ﾀｲﾏ(10,1s) */
unsigned8	uc100MS_pumpct;				/*給油改善制御期間規定用ｶｳﾝﾀ(150,15s) */
unsigned8	uc10MS_2lhgvltct;			/*２ﾚﾍﾞﾙ保護電圧継続時間ｶｳﾝﾀ(1秒,10ms) */
unsigned8	uc100MS_2lhgct;			/*２ﾚﾍﾞﾙ保護開始時間ｶｳﾝﾀ(15秒 100ms) */
unsigned8	uc100MS_2lfsct;				/*昇圧経過時間ﾀｲﾏ(10,1s) */
unsigned16	un10MSt_2lhgct;
unsigned8	uc100MS_ryoffct;			/*(ｺﾝﾌﾟOFF後)ﾘﾚｰ切待機ﾀｲﾏ(30,3s) */
/*unsigned8	uc100MS_chgfgkepct;			通電角変更中Fg保持用ｶｳﾝﾀ	~04-0604 -Outi  午後 03:21 REF045-1.02 */
unsigned16	un100MSt_typect;			/*機種切換（通信）ﾙｰﾌﾟﾀｲﾑｾｰﾌ(6000,600s)ｶｳﾝﾀ */
unsigned8	uc6Stmr_wdr15ct;			/*WDﾘｾｯﾄ発生用(６秒毎)ｶｳﾝﾀAtTIMEA.c */
unsigned8	uc6Stmr_hgrlsct;			/*保護動作ﾌﾗｸﾞｸﾘｱ用ｶｳﾝﾀ(50,５分) */
unsigned8	uc6Stmr_wirehgct;			/*ｺﾝﾌﾟ巻線保護動作ﾌﾗｸﾞSET用ｶｳﾝﾀ(10,1分) *//* @304swDOD[24/09/02] */
/*unsigned8	uc6Stmr_rsnntct;			気柱共鳴音回避制御ｶｳﾝﾀ(16,96秒) */
unsigned8	ucMINtm_trerct;				/*通信異常１時間計測ｶｳﾝﾀ<60,1時間>AtvTRNS_Cnttrns() */
unsigned8	ucTC1tm_VctrHogoCtr;		/*Vctor→Sensorless移行保護タイマカウンタ(2003/02/04-hide)REF036-0.12 */
unsigned8	ucMINtm_dlnrlsct; 			/*Dﾗｲﾝ共振保護解除用ｶｳﾝﾀ */
#if TRQ_CTL == ACTIVE
unsigned16	un100MSt_trqct;				/* 機械位置判定用ﾀｲﾏ 	 */		/* @012MzDOD[17/02/13] */
#endif /* TRQ_CTL */
unsigned8	ucMINtm_stbyct;				/*ｽﾀﾝﾊﾞｲ連続異常計測ﾀｲﾏｶｳﾝﾀ	*/							/* @088swRRD[20/04/22] */
										/*通信確立待ち時間をH50状態時間に反映するために必要	*/	/* @088swRRD[20/04/22] */
										/*通信異常とｽﾀﾝﾊﾞｲ連続を別で考える設計				*/	/* @088swRRD[20/04/22] */
/* 以下はここで宣言が妥当 */
/*unsigned8   ucTC1tm_VctrCtr;					Vector時の転流更新カウンタ(2002/10/04-hide) */
/*unsigned8	ucTC1tm_spdctrlct;			目標回転数更新用ｶｳﾝﾀ */
/*unsigned8	ucTC1tm_RckDtctCtr;			ﾛｯｸ検出用ｶｳﾝﾀ ~04-0602 -Outi  午後 11:02 REF045-1.00 */
unsigned8	ucMINtm_H50ct;				/*通信異常時強制off/on制御用ｶｳﾝﾀ（通信異常６０分継続時にｶｳﾝﾄUpするｶｳﾝﾀ）*/
/*unsigned8	ucMINtm_HOGOct;				 ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間の計測ｶｳﾝﾀ @021ywDOD[18/01/25] */
/*unsigned8   ucZX_CTjudgeCnt2;								 CT回路異常確定用ｶｳﾝﾀ2(異常FLG 継続時間180分計数用ｶｳﾝﾀ) */
volatile unsigned8	ucMAIN_10up;				/*10ms経過ﾌﾗｸﾞ群     */
					/*   ---- ---1 : 	//10ms経過ﾌﾗｸﾞ（主） */
					/*   ---- --1- : 	//10ms経過ﾌﾗｸﾞ(従1） */
					/*   ---- -1-- : 	unused.              */
					/*   ---- 1--- : 	unused.              */
					/*   ---1 ---- : 	//10ms経過ﾌﾗｸﾞ(従4） */
					/*   --1- ---- : 	unused.              */
					/*   -1-- ---- : 	unused.              */
					/*   1--- ---- : 	//10ms経過ﾌﾗｸﾞ(従7） */
volatile unsigned8 	ucg_MAIN_40up;
uint8_t volatile  ucg_2ms_flg;

/*** static ***/



/********************************************/
/*		プロトタイプ						*/
/********************************************/
void vMAIN_Cnt10ms(void);					/*10ms割込ｶｳﾝﾀ処理 */


/********************************************/
/*		外部変数参照						*/		/* @037MzDOD[14/01/27] */
/********************************************/
/*extern	unsigned8	ucMAIN_10up;		 */
/*extern	unsigned8	ucTC1tm_VctrCtr;	Vector時の転流更新カウンタ(2002/10/04-hide) */
/*extern	unsigned8	ucTC1tm_spdctrlct;	目標回転数更新用ｶｳﾝﾀ */
/*extern	unsigned8	ucTC1tm_RckDtctCtr;	ﾛｯｸ検出用ｶｳﾝﾀ ~04-0602 -Outi  午後 11:02 REF045-1.00 */
/*extern	unsigned8	ucMINtm_H50ct;		通信異常時強制off/on制御用ｶｳﾝﾀ（通信異常６０分継続時にｶｳﾝﾄUpするｶｳﾝﾀ）*/
/*extern	unsigned8	ucMINtm_HOGOct;		ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間ｶｳﾝﾀ */
/*extern UI08 ucZX_CTjudgeCnt2;			CT回路異常確定用ｶｳﾝﾀ2(異常FLG継続時間180分計数用ｶｳﾝﾀ) */


/**************************************************************************
<Title>			:CNT10MS 		;vTIME_Timea
<Function>		:10msｶｳﾝﾄ処理
<Description>	:	10ms毎にvTIME_Timea()から呼び出され10ms従ﾌﾗｸﾞを４つ全部SET
<Input>			:
<Return>		:ucMAIN_10up 10ms経過ﾌﾗｸﾞ
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vMAIN_Cnt10ms(){					/*10ms割込ｶｳﾝﾀ処理 */
										/* */
	ucMAIN_10up |= 0x92;				/*10ms経過ﾌﾗｸﾞ←全ｾｯﾄ */
} 										/* */

/**************************************************************************
<Title>			:vTIME_Tc1tmr 		;vTIME_Timea
<Function>		:±【基準タイマINTTC1】程度の精度が必要なﾀｲﾏのｶｳﾝﾄ処理
<Description>	:	TC1毎にﾀｲﾏ制御ﾒｲﾝﾙｰﾁﾝ”Timea()”から呼び出される
				:カウンタ名	ucTC1tm_dtyctrct	duty補正制御用ﾀｲﾏ(250,500ms)
				:			ucTC1tm_tmc			通信ﾀｲﾏｶｳﾝﾀ @vTRNS_Cnttrns
				:			ucTC1tm_VctrCtr		Vector時の転流更新カウンタ(2002/10/04-hide)
				:			ucTC1tm_VctrHogoCtr	Vector→Sensorless移行保護カウンタ(2003/02/04-hide)
				:			ucTC1tm_spdctrlct	目標回転数更新用ｶｳﾝﾀ~03-1228 -Outi
				:			ucTC1tm_SPKrfrshct	SPKﾘﾌﾚｯｼｭ用ｶｳﾝﾀ
				:			ucTC1tm_RckDtctCtr	ﾛｯｸ検出用ｶｳﾝﾀ
				:			ucTC1tm_UNEclrct	UNE(ct)ｸﾘｱｶｳﾝﾀ
<Input>			:
<Return>		:ucTC1tm_dtyctrct,ucTC1tm_tmc,ucTC1tm_VctrCtr,ucTC1tm_VctrHogoCtr,ucTC1tm_spdctrlct,ucTC1tm_SPKrfrshct,ucTC1tm_UNEclrct,ucTC1tm_RckDtctCtr
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
                 Ver REF036-0.00 2002-10-04 -hide VCT時転流周期更新用ｶｳﾝﾀ新設
                 Ver REF036-0.10 2003-02-04 -hide VCT⇒SSL帰還時保護用ｶｳﾝﾀ新設
                 Ver REF03D-0.00 2002-12-28 -Outi ﾀﾞｳﾝｶｳﾝﾀ仕様修正
                 Ver REF03D-0.01 2003-03-27 -Outi VCT⇒SSL帰還時保護用ｶｳﾝﾀﾊﾞｸﾞ修正
                 Ver REF045-1.00 2004-06-02 -Outi SPKﾘﾌﾚｯｼｭ用ｶｳﾝﾀ 追加
                 Ver REF045-1.01 2004-06-02 -Outi ﾛｯｸ検出用ｶｳﾝﾀ 追加
                 Ver REF045-1.03 2004-06-09 -Outi UNEchkｸﾘｱｶｳﾝﾀ 追加
                 Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
                 Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vTIME_Tc1tmr(void) {				/*ﾀｲﾏ1割込・ｶｳﾝﾀ処理 */

	ucTC1tm_dtyctrct ++;				/*Duty補正制御用ﾀｲﾏ(150,500ms) */
	ucTC1tm_tmc ++;						/*通信ﾀｲﾏｶｳﾝﾀ @vTRNS_Cnttrns */
	ucTC1tm_VctrCtr ++;					/*Vector時の転流更新カウンタ(2002/10/04-hide) */
	if(ucTC1tm_VctrHogoCtr < 255){		/*VCT⇒SSL移行時保護ｶｳﾝﾀ＜255なら	~03-0327 -Outi bug修正 */	/* 午後 04:51 REF045-4.00 */
		ucTC1tm_VctrHogoCtr ++;			/*Vector→Sensorless移行保護カウンタ(2003/02/04-hide) */
	}									/* 午後 04:51 REF045-4.00 */
	ucTC1tm_spdctrlct ++;				/*~02-1228 -Outi目標回転数更新用ｶｳﾝﾀ(X_time,???ms) */
	ucTC1tm_SPKrfrshct ++;				/*SPKﾘﾌﾚｯｼｭ用ｶｳﾝﾀ*//*~04-0602 -Outi  午後 10:58 REF045-1.00 */
	/*	ucTC1tm_RckDtctCtr ++;				ﾛｯｸ検出用ｶｳﾝﾀ ~04-0602 -Outi  午後 10:58 REF045-1.00 */
	/*	ucTC1tm_UNEclrct ++;				UNE(ct)ｸﾘｱｶｳﾝﾀ ~04-0609 -Outi  午後 03:05 REF045-1.03 */
}										/* */

/**************************************************************************
<Title>			:vTIME_10mstmr 		;vTIME_Timea
<Function>		:±【10ms】程度の精度が必要なﾀｲﾏのｶｳﾝﾄ処理
<Description>	:	10ms毎にﾀｲﾏ制御ﾒｲﾝﾙｰﾁﾝ”Timea()”から呼び出される
				:カウンタ名	uc10MSt_hzchkct		指令Hz確定用ﾀｲﾏｶｳﾝﾀ(50,500ms)
				:			uc10MSt_intct		main確定(10ms毎)ｶｳﾝﾀ@TIMEA.c
				:			uc10MSt_svhgct		直流電圧低下保護確定時間ｶｳﾝﾀ@HOGO.c(10,100ms)
				:			uc10MSt_sdobsvct	連続送信監視ﾀｲﾏｶｳﾝﾄ@vTRNS_Cnttrns() (68,680ms)
				:			un10MSt_2lhgct		２ﾚﾍﾞﾙ保護開始時間ｶｳﾝﾀ(1500,15秒/10,100ms)
<Input>			:
<Return>		:uc10MSt_hzchkct, uc10MSt_intct, uc10MSt_svhgct, uc10MSt_sdobsvct, un10MSt_2lhgct
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
				:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
				:Ver REF070-0.01 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vTIME_10mstmr(void) {						/*10msﾀｲﾏ・ｶｳﾝﾀ処理 */

	uc10MSt_hzchkct ++;							/*指令Hz確定用ﾀｲﾏ(50,500ms) */
	if(uc10MSt_intct < 250){ uc10MSt_intct ++;}	/*main確定(10ms毎)ｶｳﾝﾀ@TIMEA.c */	/* 午後 04:50 REF045-4.00 */
	uc10MSt_svhgct ++;							/*直流電圧低下保護確定時間ｶｳﾝﾀ@HOGO.c(10,100ms) */
	uc10MSt_sdobsvct ++;						/*連続送信監視ﾀｲﾏｶｳﾝﾄ@vTRNS_Cnttrns() (68,680ms) */
	uc10MS_2lhgvltct ++;							/*２ﾚﾍﾞﾙ保護電圧継続時間ｶｳﾝﾀ(1秒,10ms) */
	uc10MSt_dlnhgct ++; 					/*カウンタの値の監視はそれぞれの利用モジュールの中で行うため、保護があるならばという分岐を廃止*/ /*Dﾗｲﾝ共振保護制御用ｶｳﾝﾀ(250,2.5s) */
	un10MSt_2lhgct++;
}												/* */

/**************************************************************************
<Title>			:vTIME_100mstmr 		;vTIME_Timea
<Function>		:±【100ms】程度の精度が必要なﾀｲﾏのｶｳﾝﾄ処理
<Description>	:	100ms毎にﾀｲﾏ制御ﾒｲﾝﾙｰﾁﾝ”Timea()”から呼び出される
				:カウンタ名	uc100MS_pumpct		給油改善制御ﾀｲﾏｶｳﾝﾀ(150,15秒)
				:			uc100MS_kdwaitct	(ﾘﾚｰon後)ｺﾝﾌﾟ起動待機ﾀｲﾏ(10,1秒)
				:			uc100MS_ryoffct		(ｺﾝﾌﾟoff後)ﾘﾚｰ切待機ﾀｲﾏ(30,3秒)
				:			uc100MS_2lfsct		昇圧経過時間ﾀｲﾏ(10,1秒)
				:			uc100MS_chgfgkepct	通電角変更中Fg保持用ｶｳﾝﾀ(00,0秒)
<Input>			:
<Return>		:uc100MS_pumpct, uc100MS_kdwaitct, uc100MS_ryoffct, uc100MS_2lfsct, uc100MS_chgfgkepct
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
				:Ver REF045-1.02 2004-06-03 -Outi 通電角変更中Fg保持用ｶｳﾝﾀ 追加
				:Ver REF070-0.04 2007-05-17 -Tomo 山田ＧＭご指摘対応、機種切換（通信）ループにタイムセーフ追加
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vTIME_100mstmr(void) {				/*100msﾀｲﾏ・ｶｳﾝﾀ処理 */

	uc100MS_pumpct ++;					/*給油改善制御ﾀｲﾏｶｳﾝﾀ(150,15秒) */
	uc100MS_kdwaitct ++;				/*(ﾘﾚｰON後)ｺﾝﾌﾟ起動待機ﾀｲﾏ(10,1秒) */
	uc100MS_ryoffct ++;					/*(ｺﾝﾌﾟOFF後)ﾘﾚｰ切待機ﾀｲﾏ(30,3秒) */
	uc100MS_2lfsct ++;					/*昇圧経過時間ﾀｲﾏ(10,1秒) */
	uc100MS_2lhgct++;			/*２ﾚﾍﾞﾙ保護開始時間ｶｳﾝﾀ(15秒 100ms) */
	/*	uc100MS_chgfgkepct ++;				通電角変更中Fg保持用ｶｳﾝﾀ(00,0秒) ~04-0604 -Outi  午後 03:21 REF045-1.02 */
	un100MSt_typect ++;					/*機種切換（通信）ﾙｰﾌﾟﾀｲﾑｾｰﾌ(6000,600s)ｶｳﾝﾀ */ /* @011axORT[07/05/17] */
#if TRQ_CTL == ACTIVE
	un100MSt_trqct++;					/* 機械位置判定用ﾀｲﾏ 			*/		/* @012MzDOD[17/02/13] */
#endif /* TRQ_CTL */
}										/* */

/**************************************************************************
<Title>			:vTIME_1stmr 		;未使用<~02-0424時点 "vTIME_Timea()" で呼び出していない>
<Function>		:±【1s】程度の精度が必要なﾀｲﾏのｶｳﾝﾄ処理
<Description>	:	1s毎にﾀｲﾏ制御ﾒｲﾝﾙｰﾁﾝ”Timea()”から呼び出される
<Input>			:
<Return>		:
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vTIME_1stmr(void) {				/*1sﾀｲﾏ・ｶｳﾝﾀ処理 */

/* *********************************************************** */
/* >>>注意！！<<< ~02-0221時点 "vTIME_Timea()" で呼び出していない。 */
/* *********************************************************** */
}										/* */

/**************************************************************************
<Title>			:vTIME_6stmr 		;vTIME_Timea
<Function>		:±【6s】程度の精度が必要なﾀｲﾏのｶｳﾝﾄ処理
<Description>	:	6s毎にﾀｲﾏ制御ﾒｲﾝﾙｰﾁﾝ”Timea()”から呼び出される
				:カウンタ名	uc6Stmr_wdr15ct		WDﾘｾｯﾄ発生用(6s毎)ｶｳﾝﾀ(150,15分)
				;			uc6Stmr_hgrlsct		保護動作ﾌﾗｸﾞｸﾘｱ用ｶｳﾝﾀ(50,５分)
				;			uc6Stmr_rsnntct		気柱共鳴音回避制御ｶｳﾝﾀ(16,96秒)
<Input>			:
<Return>		:uc6Stmr_wdr15ct, uc6Stmr_hgrlsct, uc6Stmr_rsnntct
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
				:Ver REF03D-1.00 2003-06-05 -Outi 気柱共鳴音回避制御ｶｳﾝﾀ新設
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vTIME_6stmr(void) {				/*6sﾀｲﾏ・ｶｳﾝﾀ処理 */

	uc6Stmr_wdr15ct ++;					/*WDﾘｾｯﾄ発生用(6s毎)ｶｳﾝﾀ(150,15分) */
	uc6Stmr_hgrlsct ++;					/*保護動作ﾌﾗｸﾞｸﾘｱ用ｶｳﾝﾀ(50,５分) */
	/*	uc6Stmr_rsnntct ++;					~03-0605 -Outi 気柱共鳴音回避制御ｶｳﾝﾀ(16,96秒) */
	uc6Stmr_wirehgct ++;				/*ｺﾝﾌﾟ巻線保護動作ﾌﾗｸﾞSET用ｶｳﾝﾀ(10,60秒) *//* @304swDOD[24/09/02] */
}										/* */

/**************************************************************************
<Title>			:vTIME_1mintmr 		;vTIME_Timea
<Function>		:±【1分】程度の精度が必要なﾀｲﾏのｶｳﾝﾄ処理
<Description>	:	1分毎にﾀｲﾏ制御ﾒｲﾝﾙｰﾁﾝ”Timea()”から呼び出される
				:カウンタ名	ucMINtm_trerct		通信異常１時間計測ｶｳﾝﾀ<60,1時間>@vTRNS_Cnttrns()
				:			ucMINtm_H50ct		通信異常時off/on制御用ｶｳﾝﾀ<50,50分間>@_H50sfrqfx()
				:			ucMINtm_dlnrlsct 	Dﾗｲﾝ共振保護解除用ｶｳﾝﾀ(40,40分)>@vSPD_DlineJudge()
				:			ucZX_CTjudgeCnt2	CT回路異常確定用ｶｳﾝﾀ2(異常FLG@確定用ｶｳﾝﾀ)←ｲﾝｸﾘ@004axOOM[09/05/08]
<Input>			:
<Return>		: ucMINtm_trerct, ucMINtm_H50ct
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
				:Ver REF045-3.00 2004-07-06 -Outi 通信異常時強制運転仕様追加
				:Ver REF070-0.01 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF090-0.00 2008-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]
				:180125 F604WPX ｲﾝﾊﾞｰﾀ保護継続ｶｳﾝﾀのｶｳﾝﾄ処理追加 吉田
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vTIME_1mintmr(void) {				/*1分ﾀｲﾏ・ｶｳﾝﾀ処理 */

	ucMINtm_trerct ++;					/*通信異常１時間計測ｶｳﾝﾀ<60,1時間>@vTRNS_Cnttrns() */
	ucMINtm_H50ct ++;					/*通信異常時off/on用ｶｳﾝﾀ<60,60分間>@_H50sfrqfx() */
	ucMINtm_HOGOct++;					/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間の計測ｶｳﾝﾀ */			/* @021ywDOD[18/01/25] */
	ucMINtm_stbyct ++;					/*ｽﾀﾝﾊﾞｲ連続異常計測ﾀｲﾏｶｳﾝﾀ *//*通信確立待ち時間をH50状態時間に反映するために必要*/	/* @088swRRD[20/04/22] */
	if(ucTYPE_Dline == 1){	/* Ｄライン共振保護制御有 */
		ucMINtm_dlnrlsct ++; 			/*Dﾗｲﾝ共振保護解除用ｶｳﾝﾀ(40,40分) */
	}else{								/* Ｄライン共振保護制御無 */
	}

/*	if(ucZX_CTjudgeCnt2 < D_H53TIME) */	/* CT回路異常確定用ｶｳﾝﾀ2＜H53確定時間 */				/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	{								 */																						/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		ucZX_CTjudgeCnt2 ++;		 */	/* CT回路異常確定用ｶｳﾝﾀ2(異常FLG@確定用ｶｳﾝﾀ)←ｲﾝｸﾘ */	/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	}else{}							 */	/* CT回路異常確定用ｶｳﾝﾀ2≧H53確定回数⇒未処理 */		/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
}										/* */

/**************************************************************************
<Title>			:vTIME_Timea 		;vMAIN_Inttc1pros
<Function>		:ﾀｲﾏｶｳﾝﾄ処理のﾒｲﾝﾙｰﾁﾝ 
<Description>	:ｶｳﾝﾄUPﾓｼﾞｭｰﾙの呼出を行うﾀｲﾏ制御ﾒｲﾝﾙｰﾁﾝ
				:vMAIN_Inttc1pros()から呼び出される
				:	サブルーチン	vTIME_Tc1tmr()
				:					vTIME_10mstmr()
				:					vTIME_100mstmr()
				:					vTIME_1stmr()	<未使用>
				:					vTIME_6stmr()
				:					vTIME_1mintmr()
<Input>			:
<Return>		:
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-24
				:Ver REF036-1.00 2002-12-27 -hide 基本ﾀｲﾏ2ms⇒3.3ms変更
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd. 
**************************************************************************/
void vTIME_Timea(void) {				/*ﾀｲﾏｶｳﾝﾀ基本処理 */

	vTIME_Tc1tmr();						/*精度±基準ﾀｲﾏｶｳﾝﾀ処理 */
	ucTIME_tc1ct ++;					/*基準ﾀｲﾏｶｳﾝﾀのｶｳﾝﾄｱｯﾌﾟ */
	ucTIME_tmrcallct ++;				/*subﾓｼﾞｭｰﾙ呼出用ｶｳﾝﾀ ++ */
	if(ucTIME_tmrcallct >= count_10ms){ /*10msごとに*/
		ucTIME_tmrcallct = 0;			/*subﾓｼﾞｭｰﾙ呼出用ｶｳﾝﾀ←ｸﾘｱ */ /*＃ 午後 02:50 REF052-0.00 */
		vTIME_10mstmr();				/*精度±0.1sﾀｲﾏｶｳﾝﾀ処理 */
		vMAIN_Cnt10ms();
	}									/*10ms毎処理終了 */
	if(ucTIME_tc1ct >= count_100ms){	/*(2002/12/27-hide) */
		ucTIME_tc1ct = 0;				/*基準ﾀｲﾏｶｳﾝﾀのｸﾘｱ */
		vTIME_100mstmr();				/*精度±0.1sﾀｲﾏｶｳﾝﾀ処理 */
		ucTIME_100msct ++;				/*0.1sﾀｲﾏｶｳﾝﾀのｶｳﾝﾄｱｯﾌﾟ */
		ucTIME_1stmrcallct ++;			/*1stmrﾓｼﾞｭｰﾙ呼出用ｶｳﾝﾀ ++ */
		if(ucTIME_1stmrcallct >= 10){	/*１秒毎に･･･ */
			ucTIME_1stmrcallct = 0;		/*1stmrﾓｼﾞｭｰﾙ呼出用ｶｳﾝﾀ⇒ｸﾘｱ */ /*＃ 午後 08:31 REF052-0.00 */
			ucTIME_1sct ++;				/*１秒ﾀｲﾏｶｳﾝﾀのｶｳﾝﾄｱｯﾌﾟ */
		}								/*0.1s毎処理終了 */
	}									/*１秒毎処理終了 */
										/* */
	if(ucTIME_100msct >= 60){			/*６秒毎に･･･ */
		ucTIME_100msct = 0;				/*0.1sﾀｲﾏｶｳﾝﾀのｸﾘｱ */
		vTIME_6stmr();					/*精度±６秒ﾀｲﾏｶｳﾝﾀ処理 */
	}									/*６秒毎処理終了 */
										/* */
	if(ucTIME_1sct >= 60){				/*１分毎に･･･ */
		ucTIME_1sct = 0;				/*１秒ﾀｲﾏｶｳﾝﾀのｸﾘｱ */
		vTIME_1mintmr();				/*精度±１分ﾀｲﾏｶｳﾝﾀ処理 */
	}									/*１分毎処理終了 */
}										/* */
