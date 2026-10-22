/*
 * RefSpeed.c
 *
 *  Created on: 2020/10/20
 *      Author: 2993017
 */
#include "Sys_type.h"
#include "vipdc.h"
#include "RefSpeed.h"
#include "PMD1GPar.h"
#include "Timea.h"
#include "Pmd_int.h"
#include "Trans_Inv.h"
#include "system_ram.h"
#include "Hogo.h"
#include "Speedcnt.h"
#include "InvCommonFunc.h"
#include "Output_i.h"
#include "mc_api.h"


unsigned8	ucSPD_sfrqtp_b;				/*指令Hz(格納値)<BCD> */
unsigned8	ucSPD_sfrqfx;				/*指令Hz(確定値)<dec> */
unsigned8	ucSPD_rpmh;					/*駆動Hz		<dec> */
/*unsigned8	ucMINtm_H50ct;	*/			/*通信異常時強制off/on制御用ｶｳﾝﾀ（通信異常６０分継続時にｶｳﾝﾄUpするｶｳﾝﾀ）*/
unsigned8	ucSPD_mtbufpt_stop;			/*転流停止ﾊﾞｯﾌｧﾎﾟｲﾝﾀ値(ｵﾌｾｯﾄ加算)*/	/* @012MzORT[10/06/22] */
unsigned8	ucSPD_initff;					/*イニシャル判定用ＦＦ */
unsigned8	ucSPD_enstopfg;				/*強制停止許可Fg */		/* @006MwHOM[10/05/25] */
unsigned8	ucPMDi_mtbufpt_detmax;		/* 検出最大BUFPT */		/* @012MzORT[10/06/22] */

/*** static ***/
static	unsigned8	ucSPD_H50frqfx;				/*強制運転時回転数（通信異常発生前の最終運転回転数格納メモリ）*/
static	unsigned8	ucSPD_stop_ofs;				/*転流停止ｵﾌｾｯﾄ値*/		/* @012MzORT[10/06/22] */
/* 複数箇所で関数を呼ぶとstaticが呼び出し箇所毎に別変数となるため、正常に動作しない。INLINE内staticはwarning対象 *//* @010ywOOM[17/12/15] */
static	unsigned16	unPMDi_360deg;									/*今回360ﾟe */						/* @006MwHOM[10/05/25]_STA */
static	unsigned16	unPMDi_360deg_max;								/*最大360ﾟe */
static	unsigned16	unPMDi_60deg[D_POLE];							/*今回60ﾟeBUF */
static	unsigned8	ucPMDi_60bfpt;									/*今回60ﾟeBUF用pt */
static	unsigned8	ucPMDi_maxbufpt;								/*今回最大360ﾟe時BUFPT */
static	unsigned8	ucPMDi_maxbufpt_pst;							/*前回最大360ﾟe時BUFPT */
static	unsigned8	ucPMDi_max360degct;								/*最大360ﾟe時BUFPT連続一致ｶｳﾝﾀ */
/*	static	unsigned8	ucPMDi_stop_ofs;		*/						/*転流停止ｵﾌｾｯﾄ値*/					/* @008MwHOM[10/06/11]_STA */ /* @012MzORT[10/06/22] */
static	unsigned8	ucPMDi_series_num;								/*停止基準位置判定連続一致回数*/	/* @008MwHOM[10/06/11]_END */
/* D_SERIES_NUMの設定について																	   */
/* 	　設定値を大きくし過ぎるとﾋﾟｽﾄﾝ位置抽出処理実行期間内に抽出不可能となる。小さくし過ぎると、	   */
/* 	抽出位置にﾊﾞﾗﾂｷが生じてしまう。即ち、必ず抽出可能であり、かつ、ﾊﾞﾗﾂｷが小さくなる(bufptﾊﾞﾗﾂｷ	   */
/* 	目標±1)ように設定すべき。																	   */
/* 	　また、ﾋﾟｽﾄﾝ位置抽出処理実行期間の設定は、期間内でucPMDi_mtbufpt≡0となる機会が確実に		   */
/* 	(D_SERIES_NUM+2)回以上になる期間とする。更に、期間外で確実に関連変数を全ｸﾘｱできる期間とする。  */
#define  D_PICKOUT_TM	(5)					/*ﾋﾟｽﾄﾝ位置抽出時間(単位:s) */							/* @006MwHOM[10/05/25]_STA */
#define  D_POUT_START	(150-(D_PICKOUT_TM*10))	/*ﾋﾟｽﾄﾝ位置抽出開始時機 */
#define  D_SERIES_NUM	(3)					/*360ﾟe最大pt一致連続回数 */
#define  D_OFFSET_BUFPT	(0)					/*360ﾟe最大ptからの補正量 */							/* @006MwHOM[10/05/25]_END */

void vSPD_H50sfrqfx(void);
void SelectStopPosition(void);
unsigned16 CalculateAndGetRpsFOC(void);
//void vSPD_Resonanttmr(void);
//void vSPD_Resonantrpm(void);

/**************************************************************************
<Title>			:CalculateAndGetRpsFOC
<Function>		:平均回転数の計算(rpm, 16bit /小数点第一位まで)
<Description>	:MC_GetMecSpeedAverageMotor1()を取得
				:ローパスをかけて平均化処理を行う
<Input>			:None
<Return>		:hCurMecRps
<Author>		:Sakamoto	作成
<Date>			:2026-06-15
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
unsigned16 CalculateAndGetRpsFOC(void)
{
    static unsigned16 s_u16LpfRps = 0;
    static unsigned8  s_u8LpfInit = 0;

    unsigned16 u16RawRps = (unsigned16)MC_GetMecSpeedAverageMotor1();

    if (s_u8LpfInit == 0) {
        s_u8LpfInit = REFSPD_LPF_INIT;
        s_u16LpfRps = u16RawRps;
    } else {
        s_u16LpfRps = (unsigned16)(((UI32)s_u16LpfRps * REFSPD_LPF_PREV_COEF + u16RawRps) / REFSPD_LPF_DIVISOR);
    }

    hCurMecRps = s_u16LpfRps;
    return hCurMecRps;
}

/**************************************************************************
<Title>			:_H50rpm 		;vSPD_Setrpm
<Function>		:通信異常時強制off/on制御
<Description>	:	通信異常発生継続後の駆動回転数を制御する
				:		 ○運転率70%でoff/on(18分/42分)運転を行う
				:		 ○on中の回転数は通信正常時の最終運転回転数
				:		 ○ｲﾆｼｬﾙでH50発生の場合は回転数をD_R3する
				:	＿＿＿＿＿＿＿＿＿＿＿＿＿＿ _＿ _＿＿＿
				:	  sfrqfx	58				X 0 X  58   X
				:	￣￣￣￣￣￣￣￣↑￣￣￣￣￣|￣~|￣￣￣~↑
				:			  通信異常←-60分-→|18m|←42m→WDR作動(Softr)
				:	＿＿＿＿_＿＿＿ ＿＿＿＿＿＿＿＿＿＿＿＿
				:	  sfrqfx	58 X  0 			X  58   X
				:	￣￣￣￣~￣￣￣ ↑~￣￣￣￣￣￣~|￣￣￣~↑
				:			  通信異常←---78分---→|←42m→WDR作動(Softr)
				:	＿＿＿＿＿＿＿＿＿＿＿ _＿＿＿
				:	  sfrqfx	   0      X D_R3  X
				:	￣↑￣￣￣￣￣￣↑￣￣|￣￣￣~↑
				:ﾏｲｺﾝﾘｾｯﾄ	  通信異常 18m|←42m→WDR作動(Softr)
				:
<Input>			:ucMINtm_trerct	(通信異常ｶｳﾝﾀ)
<Return>		:ucSPD_sfrqfx	(指令回転数)
<Author>		:-Outi	作成
<Date>			:2004-0705
<Update>		:Ver REF045-3.00 2004-07-06 -Outi 通信異常時強制運転仕様追加
				:Ver REF052-1.00 2005-06-26 -Outi ｳﾈﾘ状態信号送信仕様追加
				:Ver REF070-0.02 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.05 2007-04-22 -Outi 機種切り替え対応
				:Ver REF090-0.00 2009-05-11 -Outi IEC対応(D_TRERRJUDGETM分統一<断続運転周期/通信異常判定期間>含む) @006axOOM[09/05/11]
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vSPD_H50sfrqfx(void){										/*通信異常時強制off/on制御	 */

/*	if(ucMINtm_trerct < D_TRERRJUDGETM){	*/					/*通信異常継続時間＜異常確定 */
	if((ucMINtm_trerct < D_TRERRJUDGETM) 						/*通信異常継続時間60分未満なら */	/* @018MzDOD[17/03/04] */
		&& (ucg_TR_stbyct < TRNSFAULTNUM)){						/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀがTRNSFAULTNUM回未満なら */	/* @086sxDOD[20/04/10] */
		ucMINtm_H50ct = 0;										/*H50中off/on制御用ｶｳﾝﾀ⇒ｸﾘｱ */
		if(ucSPD_sfrqfx != 0){									/*指令回転数≠０なら		 */
			ucSPD_H50frqfx = ucSPD_sfrqfx;						/*H50中指令回転数←指令回転数*/
		}
		if(ucSPD_H50frqfx == 0){								/*H50中指令回転数:0(ﾘｾｯﾄ後initial通信異常)なら */	/* @006axOOM[09/05/11] */
			if(ucMINtm_trerct > 1){								/*通信異常継続時間＞1分(通信異常状態確定)なら */	/* @006axOOM[09/05/11] */
				ucMINtm_H50ct = ucMINtm_trerct;					/*H50中off/on制御用ｶｳﾝﾀ←1(既に1分経過状態) */		/* @006axOOM[09/05/11] */
				ucMINtm_trerct = D_TRERRJUDGETM;				/*通信異常継続時間計数ｶｳﾝﾀ←異常確定(断続開始) */	/* @006axOOM[09/05/11] */
			}else{}
			if(ucMINtm_stbyct > 1){								/*ｽﾀﾝﾊﾞｲ異常継続時間＞1分 なら */												/* @088swRRD[20/04/22] */
				ucMINtm_H50ct = ucMINtm_stbyct;					/*H50中off/on制御用ｶｳﾝﾀ←通信確立待ち時間(既に通信確立待ち時間分経過状態) */	/* @088swRRD[20/04/22] */
				ucg_TR_stbyct = TRNSFAULTNUM;					/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀ←TRNSFAULTNUM←ｽﾀﾝﾊﾞｲ異常確定(断続開始) */					/* @088swRRD[20/04/22] */
			}else{}												/*ｽﾀﾝﾊﾞｲ異常継続時間0分 なら 何もしない */										/* @088swRRD[20/04/22] */
		}else{}
	}else{														/*通信異常継続時間≧異常確定(断続開始)	 */
		if(ucSPD_H50frqfx == 0){								/*H50中指令回転数:0なら				 */
			ucSPD_H50frqfx = GetStandardSpeed();
//			if(ucTYPE_Dline_point == 1){	/*Dﾗｲﾝ共振ポイント：45～49r/s	 */
//				ucSPD_H50frqfx = D_R3_1;						/*H50中指令回転数←D_R3_1 */
//			}else {												/*Dﾗｲﾝ共振ポイント：44～47r/s	 */
//				ucSPD_H50frqfx = D_R3_0;						/*H50中指令回転数←D_R3_0 */
//			}
		}
		if(ucMINtm_H50ct <= D_H50DANZOKOFF){					/*H50中off/on制御用ct≦ｵﾌ時間		 */				/* @006axOOM[09/05/11] */
			ucSPD_sfrqfx = 0;									/*指令回転数←ｸﾘｱ */
		}else if(ucMINtm_H50ct <= D_H50DANZOKPRD){				/*ｵﾌ時間＜off/on制御ct≦断続運転周期 */				/* @006axOOM[09/05/11] */
			ucSPD_sfrqfx = ucSPD_H50frqfx;						/*指令回転数←H50中指令回転数*/						/* @006axOOM[09/05/11] */
		}else{													/*H50中off/on制御用ct＞断続運転周期	 */
			ucMINtm_H50ct = D_H50DANZOKPRD;						/*H50中off/on制御用ct←断続周期(wdrflgｾｯﾄ待ち) */	/* @006axOOM[09/05/11] */
		}
	}
}

/**************************************************************************
<Title>			:vSPD_Resonanttmr 		;vSPD_Setrpm
<Function>		:気柱共鳴音回避制御(ﾀｲﾏ)
<Description>	:	起動から一定時間(秒)経過までは指令回転数よりｎ回転低い回転数で運転する
				:		一定時間：	D_RSNNTTM	(6秒～25分30秒)
<Input>			:
<Return>		:uc6Stmr_rsnntct
<Author>		:-Outi	作成
<Date>			:2002-0605
<Update>		:Ver REF03D-1.00 2003-06-05 -Outi 気柱共鳴音回避制御ﾓｼﾞｭｰﾙ新設
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
#if 0 /* 今後の検討で必要になる可能性ありのためｺﾒﾝﾄとして本モジュールを残す */
void vSPD_Resonanttmr(void){									/*気柱共鳴音回避制御(ﾀｲﾏ) */

	if(ePMD_drvmode != D_stop){								/*運転状態<Not停止中>で */
		if(uc6Stmr_rsnntct >= D_RSNNTTM){					/*1.6分経過していれば */
			uc6Stmr_rsnntct = D_RSNNTTM;					/*共鳴回避制御ｶｳﾝﾀ←D_RSNNTTM */
		}													/* */
/*		if((ucTRNS_sfrg & 0x01) != 0){ */						/* 早送りﾓｰﾄﾞなら */			/* ﾒｲﾝﾏｲｺﾝから早送りﾓｰﾄﾞﾌﾗｸﾞ送信なしのため廃止 *//* @272sxDOD[23/10/10] */
/*			uc6Stmr_rsnntct = D_RSNNTTM; */						/*共鳴回避制御ｶｳﾝﾀ←D_RSNNTTM */	/* ﾒｲﾝﾏｲｺﾝから早送りﾓｰﾄﾞﾌﾗｸﾞ送信なしのため廃止 *//* @272sxDOD[23/10/10] */
/*		} */													/* */							/* ﾒｲﾝﾏｲｺﾝから早送りﾓｰﾄﾞﾌﾗｸﾞ送信なしのため廃止 *//* @272sxDOD[23/10/10] */
		if(ucHOG_sttF != 0){								/*保護状態であば */
			uc6Stmr_rsnntct = 0;							/*共鳴回避制御ｶｳﾝﾀ←0 */
		}													/* */
	}else{													/*<停止中>なら */
		uc6Stmr_rsnntct = 0;								/*共鳴回避制御ｶｳﾝﾀ←0 */
	}														/* */
}															/* */
#endif

/**************************************************************************
<Title>			:vSPD_Bcd2nrm(unsigned8 ucBcd) 	;vSPD_Setrpm
<Function>		:BCD ⇒標準表示変換
<Description>	:	指令rpsを通信用のBCD表示から16進数表示に変換
<Input>			:ucBcd 回転数指令値(BCD値)
<Return>		:ucbuf 回転数指令値(10進値)
<Author>		:-Outi	作成
<Date>			:2002-0130
<Update>		:Ver REF02A-0.00 2002-04-25
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
unsigned8 vSPD_Bcd2nrm(unsigned8 ucBcd) {					/*BCD⇒標準表示変換 */
	unsigned8    ucbuf;										/*ﾃﾞｰﾀ格納用ﾃﾝﾎﾟﾗﾘ(8bit) */
	unsigned8    ucSyou;									/*商・格納用ﾃﾝﾎﾟﾗﾘ(8bit) */
	unsigned8    ucAmari;									/*あまり・格納用ﾃﾝﾎﾟﾗﾘ(8bit) */

	ucSyou  = ucBcd / SPD_BCD_DECIMAL_BASE;									/* ucSyou = ucSPD_sfrqtp_b / 16 */
	ucAmari = ucBcd % SPD_BCD_DECIMAL_BASE;									/* ucAmari = ucSPD_sfrqtp_b % 16 */
	ucbuf = ucSyou * SPD_DECIMAL_BASE + ucAmari;							/*ﾃﾝﾎﾟﾗﾘ←10進 */

	return ucbuf;											/*16進表示値return */
}

/**************************************************************************
<Title>			:_Spd-rpm(unsigned16) 		;vSPD_Setrpm
<Function>		:駆動rps変換 ⇔ ModeTime
<Description>	:	ModeTime《電気角60ﾟ 周期[us]》→実rps《圧縮機モータの回転数[s~(-1)]》の変換処理
				:Caution>双方向変換可能だが駆動rpsをreturnする場合は8bit
<Input>			:unSpd2rpm	駆動rps or ModeTime(16bit)
<Return>		:unbuf		ModeTime or 駆動rps(16bit)
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-25
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
unsigned16 vSPD_Spdrpm(unsigned16 unSpd2rpm) {				/*周期⇒回転数変換処理 */
	unsigned16	unbuf;										/*ﾃﾝﾎﾟﾗﾘ(16bit) */

	unbuf  = ((unsigned16)(1000000 / (D_POLE * 3)));		/* 55,556←(1/1[Hz]) / (3*6[極数]) / 1[us] */
	unbuf += (unSpd2rpm >> 1);								/* 戻り値の四捨五入準備。 */
	if(unSpd2rpm > 0){	unbuf /= unSpd2rpm;					/* 55,556 を引数で割る。 */
	}else{				unbuf = 0xffff;		}				/* */

	return unbuf;											/*戻り値は16bit */
}
/**************************************************************************
<Description>	:
<Author/Date>	:
<Update>	:
**************************************************************************/
UI08 Is1stStart(void){
    return (ucSPD_initff == 0xff);
}

/**************************************************************************
<Title>			:vSPD_Getrpmh		;vMAIN_Inttc1pros
<Function>		:駆動回転数設定
<Description>	:駆動回転数(ucSPD_rpmh)を設定
<Input>			:ucSPD_sfrqfx
<Return>		:ucSPD_rpmh
<Author>		:-Mura	作成
<Date>			:2010-0616
<Update>		:Ver REF1000-0.01 2010-06-11 -Mura 停止時振動 @008MwHOM[10/06/11]
				:Ver REF1000-0.02 2010-06-16 -Mura Sub関数化  @009MwORM[10/06/16]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
 void vSPD_Getrpmh(void){																			/* @009MwORM[10/06/16]_STA */
	unsigned8	ucbuf;												/*ﾃﾝﾎﾟﾗﾘ(8bit) */
//	unsigned16	unbuf;												/*ﾃﾝﾎﾟﾗﾘ(16bit) */

	if(uc100MS_pumpct >= PUMPCT_END){							/*給油改善制御済みなら */	/* @012MzDOD[17/02/13] */

		ucbuf = SetSpeedLimit(ucSPD_sfrqfx);												/* @012MzDOD[17/02/13] */

/*------------------------------------------------
	割込禁止
------------------------------------------------*/				/*＠~07-0422 -Outi 機種切り替え対応 */
		ucSPD_rpmh = ucbuf;										/*駆動回転数←ﾊﾞｯﾌｧ */ /*＃ 午前 08:42 REF070-0.05 */ /* @001agORM[07/04/22] */
/*------------------------------------------------
	割込許可
------------------------------------------------*/
		ucSPD_initff = 0;										/*ｲﾆｼｬﾙ判定FF←ｸﾘｱ(初起動終了) */


		//	}else if(uc100MS_pumpct != 0){								/*給油改善中(起動から15s以内)なら */
	}else if(ePMD_drvmode != D_stop){								/*給油改善中(起動から15s以内)なら */
		if( Is1stStart() ){								/*初回起動ならば */
            ucSPD_rpmh = D_PUMPINI;                         /*駆動Hz←給油中Hz(27Hz) */
		}else{													/*初回起動終了していれば */			/* @008MwHOM[10/06/11]_END */
            ucSPD_rpmh = D_PUMPMES;                     /*駆動Hz←給油中Hz(27Hz)      */  /* @015MzDOD[17/02/28] */
		}														/* */								/* @008MwHOM[10/06/11] */
	}
	else{														/*<停止中>なら */
		ucSPD_rpmh = 0;											/*駆動Hz←０Ｈｚ */
	}
}																									/* @009MwORM[10/06/16]_END */

UI08 HasStopOffset(void){
	return ((ucTRNS_stopctrl & 0x1F) <= (D_MTBUFSZ - 1)); /* オフセットあり　-> ucTRNS_stopctrl & 0x1F が 0 ～ (極数×3-1)、 極数×3 はオフセットなし*/
}
UI08 NeedsPrepareStopCtrl(UI08 direction_spd){
	return (((ucTRNS_stopctrl & 0x1F) <= 17) && (ucSPD_rpmh != 0) && (direction_spd == 0) && (ucPMDi_run2stpfg));
}
void PrepareStopCtrl(void){
	ucSPD_stop_ofs   = (ucTRNS_stopctrl & 0x1F);				/*転流停止ｵﾌｾｯﾄ値格納 ← 転流停止制御用データ(下位5bit) */			/* @012MzORT[10/06/22]_STA */
	ucSPD_mtbufpt_stop  = ucPMDi_mtbufpt_detmax + ucSPD_stop_ofs;	/*停止時BUFPT更新 */
	if(ucSPD_mtbufpt_stop >= D_MTBUFSZ){	/*停止時BUFPT>= D_MTBUFSZ */
			ucSPD_mtbufpt_stop -= D_MTBUFSZ;	/*停止時BUFPT更新 */
	}else{}									/*未処理 */																				/* @012MzORT[10/06/22]_END */
	ucSPD_enstopfg = 0x01;										/*強制停止許可Fg←SET */
}

/**************************************************************************
<Title>			:vSPD_Setrpm 		;vMAIN_Inttc1pros
<Function>		:回転数設定
<Description>	:	指令rpsと起動開始からの経過時間を基にｺﾝﾌﾟﾚｯｻ回転数を設定
<Input>			:ucTRNS_sfrq_b, ePMD_drvmode
<Return>		:ucSPD_rpmh
<Author>		:-Outi	作成
<Date>			:2002-0201
<Update>		:Ver REF02A-0.00 2002-04-25
				:Ver REF035-0.00 2003-02-22 -Outi ﾊﾞﾘﾊﾞﾘ音対策仕様新設
				:Ver REF036-0.00 2002-11-27 -hide ﾛｯｸ状態ﾁｪｯｸ機能新設
				:Ver REF036-0.01 2002-12-12 -hide Forced 駆動での最高回転数設定のため追加
				:Ver REF036-0.11 2003-02-04 -hide ucr_RckCheckの自動変数化
				:Ver REF03D-0.10 2003-06-05 -Outi ｽｲｰﾌﾟ試験用ﾓｼﾞｭｰﾙ"vSPD_TEST"新設
				:Ver REF03D-1.00 2003-06-05 -Outi 気柱共鳴音回避制御ﾓｼﾞｭｰﾙ新設
				:Ver REF045-0.00 2004-04-16 -Outi SWP処理(vSPD_TEST)改良
				:Ver REF045-1.00 2004-06-07 -Outi ｽﾑｰｽ帰還仕様追加に伴う変更
				:Ver REF045-2.00 2004-06-12 -Outi 回転数ﾀﾞｳﾝ仕様ﾀﾞﾐｰ追加
				:Ver REF045-2.01 2004-06-19 -Outi VCT中はunSPD_spdtrgt算出を禁止(VCT中はINTTMR3内でﾃｰﾌﾞﾙ参照値を格納)
				:Ver REF045-2.02 2004-06-20 -Outi 回転数ﾀﾞｳﾝ仕様ﾀﾞﾐｰ改良
				:Ver REF045-2.03 2004-06-29 -Outi 回転数ﾀﾞｳﾝ仕様ﾀﾞﾐｰ廃止
				:Ver REF045-3.00 2004-07-06 -Outi 通信異常時強制運転仕様追加
				:Ver REF052-0.01 2005-05-16 -Outi 起動回転数低回転化対応
				:Ver REF061-0.00 2006-04-14 -Outi 2ﾚﾍﾞﾙﾘﾚｰ廃止対応(R1回転数廃止)←Ｒ１運転可能と判断し本仕様変更は削除(2006/5/25)
				:Ver REF061-0.01 2006-04-28 -Outi P67切替仕様変更(IPM判別に利用)
				:Ver REF070-0.01 2007-04-09 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.05 2007-04-22 -Outi 機種切り替え対応
				:Ver REF070-0.06 2007-04-25 -Outi 指令回転数更新処理修正
				:Ver REF070-0.07 2007-04-25 -Tomo ソフト設計ＤＲ指摘対応
				:Ver REF1000-0.00 2010-05-25 -Mura 停止時振動抑制対応(ﾋﾟｽﾄﾝ位置固定停止仕様) @006MwHOM[10/05/25]
				:Ver REF1000-0.01 2010-06-11 -Mura 停止時振動対策 メイン通信処理追加  @008MwHOM[10/06/11]
				:Ver REF1000-0.02 2010-06-16 -Mura Setrpm関数のSub関数化  @009MwORM[10/06/16]
				:Ver REF1000-0.03 2010-06-22 -Mura 停止オフセット受取りタイミング修正  @012MzORT[10/06/22]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void vSPD_Setrpm(void) {										/*回転数設定処理 */
unsigned8	ucbuf;												/*ﾃﾝﾎﾟﾗﾘ(8bit) */
																/*↑静的変数にする必要がない為変更する */
	if(uc10MSt_intct >= 255){									/*ﾒｲﾝﾙｰﾁﾝ確定なら */
//		ucSPD_rpmpast = ucSPD_rpmh;								/*3.3ms前の駆動Hzをﾒﾓﾘに格納(前回駆動Hz←駆動Hz) */
	}else{														/*ｲﾆｼｬﾙ処理ﾙｰﾌﾟ中なら */
		ucSPD_initff = 0xff;									/*ｲﾆｼｬﾙ判定FF←ｾｯﾄ(ｲﾆｼｬﾙ状態) */
	}
	if ( ucSPD_sfrqtp_b != ucTRNS_sfrq_b ){						/*指令Hz(格納値)≡指令Hz(通信値)なら */
		uc10MSt_hzchkct = 0;									/*指令Hz確定用ｶｳﾝﾀｸﾘｱ */
		if( ucTYPE_uart != 0 ){									/*通信機種判別確定 */ /* @014MzDOD[17/02/27] */
			ucSPD_sfrqtp_b = ucTRNS_sfrq_b;						/*回転数設定 更新 */
		}else{													/*通信機種判別"未"確定 */
			ucSPD_sfrqtp_b = 0;									/*停止設定 */
		}														/* */ /*＃ 午後 12:56 REF070-0.05 */
	} else if ( uc10MSt_hzchkct >= 50 ){							/*指令が一致　500ms連続判定 00-0503 -AK */
		uc10MSt_hzchkct = 0;								/*指令Hz確定によりｶｳﾝﾀｸﾘｱ */
		ucbuf = vSPD_Bcd2nrm(ucSPD_sfrqtp_b);				/*ﾃﾝﾎﾟﾗﾘ←標準表示変換結果 */
																/*BCD→標準表示変換 ex.58H(58Hz)→58D(3CH) */
		if(ucHOG_sttF == 0){								/*保護状態でなければ */
																/*要注意！！++++++++++++++++++++++++++++++++*/
																/* vMAIN_Input(main.c)でP60-P63はbitﾏｽｸ状態なので*/
																/*SWP処理する場合はbitﾏｽｸを外す必要がある	*/
																/* 要注意２：インバータ異常判定の前提条件にucSPD_sfrqfxを使っているものがあるため、ｲﾆｼｬﾙ時など特に、確定処理時間の早いほうが優先される */
			if( ucTYPE_uart == 0 ){							/*通信機種判別"未"確定 */
				ucSPD_sfrqfx = 0;							/*停止設定 */
			}
			/*通信機種判別確定 */ 	/* @014MzDOD[17/02/27] */
			else if(NeedsPrepareStopCtrl(ucbuf)){	/*転流停止制御許可(ｵﾌｾｯﾄ17以下) ∧ 運転中∧ 停止指令受領中∧ ﾋﾟｽﾄﾝ位置固定停止準備完了 */		/* @006MwHOM[10/05/25]_END */ /* @008MwHOM[10/06/11] */
				PrepareStopCtrl();
			} else {															/*停止中∨運転指令受領中∨ﾋﾟｽﾄﾝ位置固定停止準備未完 */		/* @006MwHOM[10/05/25]_STA */
				ucSPD_sfrqfx = ucbuf;					/*回転数確定 */
			}

			//vSPD_H50sfrqfx();/* 午前 09:37 REF045-3.00 */	/*通信異常時強制運転制御 */
		}else{												/*保護状態なら */
			ucSPD_sfrqfx = 0;								/*停止設定 */
		}
	} else {
/* */
	}
//	PumpCountStopIfNeeded();							//20201021 takeoka カウントの停止はPMD1g.cの方が意味のつながりが強いと判断で移動
//	vSPD_Resonanttmr();											/*気柱共鳴音回避制御(ﾀｲﾏ) CALL */
/*------------------------------------------------
	駆動Hzセット処理
------------------------------------------------*/

	/*u8g_SPD_cmdをセットする*/
	/*vSPD_Getrpmh();*/									/*駆動回転数セット*/	/* @009MwORM[10/06/16] */

//	if(ePMD_drvmode != D_vector){	/* 午後 05:15 REF045-2.01	VCT運転中でなければ*/
//		unSPD_spdtrgt = vSPD_Spdrpm((unsigned16) ucSPD_rpmh);}	/*目標速度60ﾟe←駆動Hz変換処理 */
//	    unSPD_spdtrgt = RotationPerSecond2CommutatePeriod((unsigned16) ucSPD_rpmh);
//	}																/* */

}


#if 0
void ClearRefSpeed(void){
	ucSPD_sfrqfx = 0;						/*駆動Hz←ｸﾘｱ	~02-0206 -Outi */
	ucSPD_rpmh = 0;							/*指令Hz←ｸﾘｱ	~02-0206 -Outi */
}
#endif

UI08 IsSpeedDrive(void){
	/*return (ucSPD_sfrqfx > 0);*/
	return (u8g_SPD_set>0);
}

/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void SelectStopPosition(void){
    UI08 i;																/* 初期化用自動変数 */
    UI08 k;																/* 初期化用自動変数 */				/* @008MwHOM[10/06/11] */
    UI16 unDmy360;													/*ﾀﾞﾐｰ(16bit)の定義 */				/* @006MwHOM[10/05/25]_END */
    UI16 unBuf;													/*ﾃﾝﾎﾟﾗﾘ(16bit)の定義 */

    if( uc100MS_pumpct < D_POUT_START ){
        unPMDi_360deg = 0;									/*今回360ﾟeをｸﾘｱ */
        unPMDi_360deg_max = 0;								/*最大360ﾟeをｸﾘｱ */
        for(i = 0; i < D_POLE; i++){						/*今回60ﾟeBUF初期化for-loop */
            unPMDi_60deg[i] = 0;							/*今回60ﾟeBUFをｸﾘｱ */
        }
        ucPMDi_60bfpt = 0;									/*今回60ﾟeBUF用pt */
        ucPMDi_maxbufpt = 0;								/*今回最大360ﾟe時BUFPTをｸﾘｱ */
        ucPMDi_maxbufpt_pst = 0;							/*前回最大360ﾟe時BUFPTをｸﾘｱ */
        ucPMDi_max360degct = 0;								/*最大360ﾟe時BUFPT連続一致ｶｳﾝﾀをｸﾘｱ */					/* @012MzORT[10/06/22] */
        /* ucPMDi_mtbufpt_stop = 0;	*/						/*停止時BUFPTをｸﾘｱ */									/* @012MzORT[10/06/22] */
        ucPMDi_mtbufpt_detmax =0;							/* 検出最大BUFPT */
        ucSPD_enstopfg = 0;									/*強制停止許可Fg←CLR */
        ucPMDi_run2stpfg = 0;								/*ﾋﾟｽﾄﾝ位置固定停止準備完了Fg←SET */
        return;
    }

    /* ucPMDi_stop_ofs   = (ucTRNS_stopctrl & 0x1F);	*/	/*転流停止ｵﾌｾｯﾄ値格納 ← 転流停止制御用データ(下位5bit) */				/* @008MwHOM[10/06/11]_STA */	/* @012MzORT[10/06/22] */
    k = ((ucTRNS_stopctrl >> 5) & 0x03);				/*停止基準位置判定連続一致回数切替 ← 転流停止制御用データ(6-5bit)*/	/* @008MwHOM[10/06/11]_END */
    ucPMDi_series_num = (k * 2) + 3;					/*停止基準位置判定連続一致回数を設定 (3回/5回/7回/9回）  */

    unBuf = unPMD_MCAP;
    unDmy360 = unPMDi_360deg - unPMDi_60deg[ucPMDi_60bfpt];	/*演算用に格納 */
    unPMDi_60deg[ucPMDi_60bfpt] = unBuf;				/*今回60ﾟeBUFに格納 */
    unPMDi_360deg = unBuf + unDmy360;					/*今回360ﾟeを更新 */
    if( (ucPMDi_60bfpt = ucPMDi_60bfpt + 1) >= D_POLE ){ /*今回60ﾟeBUFptを更新(ｲﾝｸﾘ) */
        ucPMDi_60bfpt = 0;								/*今回60ﾟeBUF用ptを更新(ｸﾘｱ) */
    }
    else{}												/*未処理 */

    if( unPMDi_360deg_max < unPMDi_360deg ){				/*最大＜今回360ﾟeなら */
        unPMDi_360deg_max = unPMDi_360deg;				/*最大360ﾟeを更新 */
        ucPMDi_maxbufpt = ucPMDi_mtbufpt;				/*最大360ﾟe時BUFPTを更新 */
    }
    else{}												/*未処理 */

    if ( ucPMDi_mtbufpt == 0 ){							/*機械角360ﾟ毎チェック */
        unPMDi_360deg_max = unPMDi_360deg;				/*最大360ﾟeを更新 */
        if( ucPMDi_maxbufpt == ucPMDi_maxbufpt_pst ){	/*最大360ﾟe時BUFPTが前回と一致 */
            ucPMDi_max360degct ++;						/*最大360ﾟe時BUFPT連続一致ｶｳﾝﾀ←計数 */
            /* if(ucPMDi_max360degct >= D_SERIES_NUM)	*/	/*最大360ﾟe時BUFPT連続一致ｶｳﾝﾀ≧360ﾟe最大pt一致連続回数 */	/* @008MwHOM[10/06/11]_STA */
            if( ucPMDi_max360degct >= ucPMDi_series_num ){	/*最大360ﾟe時BUFPT連続一致ｶｳﾝﾀ≧360ﾟe最大pt一致連続回数 */	/* @008MwHOM[10/06/11]_END */
                ucPMDi_max360degct = 0;					/*最大360ﾟe時BUFPT連続一致ｶｳﾝﾀ←ｸﾘｱ */

#if 0 /* 今後の検討で必要になる可能性ありのためｺﾒﾝﾄとして本モジュールを残す */
                ucPMDi_mtbufpt_stop = ucPMDi_maxbufpt + ucPMDi_stop_ofs;	/*停止時BUFPT更新 */					/* @012MzORT[10/06/22]_STA */

                if(ucPMDi_mtbufpt_stop >= D_MTBUFSZ)	/*停止時BUFPT>= D_MTBUFSZ */
                {
                    ucPMDi_mtbufpt_stop -= D_MTBUFSZ;	/*停止時BUFPT更新 */
                }else{}									/*未処理 */													/* @012MzORT[10/06/22]_END */
#endif
                ucPMDi_mtbufpt_detmax = ucPMDi_maxbufpt; /*検出最大BUFPT更新 */									/* @012MzORT[10/06/22] */

                ucPMDi_run2stpfg = 0x01;				/*ﾋﾟｽﾄﾝ位置固定停止準備完了Fg←SET */
            }
            else{}										/*未処理 */
        }
        else{											/*最大360ﾟe時BUFPTが前回と不一致 */
            ucPMDi_max360degct = 0;						/*最大360ﾟe時BUFPT連続一致ｶｳﾝﾀ←ｸﾘｱ */
            ucPMDi_maxbufpt_pst = ucPMDi_maxbufpt;		/*前回最大360ﾟe時BUFPTを更新 */
        }
    }
    else{}												/*未処理 */
#if SOFT_STOP_SW == ACTIVE												/* @229swHOT[22/09/02] */
    ucPMDi_run2stpfg = 0x01; /* 転流直後に停止開始するために強制ON */	/* @229swHOT[22/09/02] */
#endif																	/* @229swHOT[22/09/02] */
}
