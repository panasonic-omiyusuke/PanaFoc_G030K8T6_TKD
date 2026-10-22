/**************************************************************************
#ifdef DOC
File Name		：MAIN.c
Function		：メインルーチン処理
Description		：
Program			：
Author			：田中　正之
Date			：07/03/21 B32M1 UPD780501
Update			：07/05/08 REF071-1.01 B32M1 UPD78F0501 記憶ﾃﾞｰﾀ領域出力処理削除,ｽﾀｯｸ領域変更
				：07/05/10 REF071-1.02 B32M1 A/D確定のためﾒｲﾝﾙｰﾌﾟ直前3秒待機
				：09/01/07 REF071-1.02 B32M2 連続冷却削除,ﾒｲﾝﾙｰﾌﾟ直前1秒待機
				：10/10/21 BW465	暫定削除項目対応	村上 @007MwDOD[10/10/21]
				：Ver REF1010-0.01 2010-12-13 -Mura 通信関数名変更 @038MwDOD[10/12/13]
Copyright		：Matsushita Refrigeration Co.,Ltd.
#endif
**************************************************************************/

/********************************************/
/*		    インクルードファイル			*/
/********************************************/
#include	"Sys_type_main.h"								/* @018MzDOD[13/07/11] */	/*必要 CLR SET 定義*/
#include	"Sys_func.h"									/* @018MzDOD[13/07/11] */	/*必要 タイマ、ヒータ、ＦＣＴ関数*/
#include	"Ram.h"											/* @018MzDOD[13/07/11] */	/*必要 カウント変数*/
#include	"Trans_Inv.h"									/* 多重定義ｴﾗｰ対応 *//* @259swDOD[23/06/30] */
#include 	"PMD1GPar.h"									/* @018MzDOD[17/03/04] *//*必要 D_TRERRJUDGETM */
#include 	"Main_Pros.h"									/* @037MzDOD[14/01/27] *//*必要 H_CYCLEW、H50時ヒータ通電率 */
#include 	"main.h"

/*さくじょはじめ*/
// #include "Softr.h"	/*必要 ポートリセット・リセット処理 */
// #include "Com_def.h"/*必要 割込み禁止＆許可*/
/*さくじょおわり*/


/********************************************/
/*		変数宣言							*/		/* @037MzDOD[14/01/27] */
/********************************************/
/*** global ***/
UCHAR	ucg_MAIN_ptest;		/* 検査制御ﾌﾗｸﾞ*//*070425*/

USHORT	ung_MAIN_DEFHon;		/* ﾃﾞﾌﾛｽﾄﾋｰﾀ 	ON時間 	*/
USHORT	ung_MAIN_PCEVAHon;		/* PCEVAﾋｰﾀ 		ON時間	*/
/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/

UCHAR	ucg_INT_DEFH;	 		/* ﾃﾞﾌﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
UCHAR	ucg_INT_PCEVAH;	 	/* PCEVAﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/
UCHAR	ucg_INT_PULSE;			/* 発振制度確認ﾊﾟﾙｽ出力ﾌﾗｸﾞ  */

/*** static ***/
/* static UCHAR	ucg_INT_intc; */		/* 初期ﾃﾞｰﾀ ｶｳﾝﾀ*//* @271swDOD[23/10/10] */
/*static UCHAR	ucg_INT_10cn;*/		/* 10msｶｳﾝﾄ*/
/*static UCHAR	ucg_MAIN_intc;*/		/* 初期ﾃﾞｰﾀ ｶｳﾝﾀ*//*070425*/


/********************************************/
/*		外部変数参照						*/
/********************************************/
extern  UCHAR	ucMINtm_trerct;					/*通信異常時間計測ｶｳﾝﾀ*/

/*******************************************
		プロトタイプ宣言
 *******************************************/

/**************************************************************************
#ifdef DOC
Title		：Main制御用メイン処理
Function	：
Description	：
Input		：
Retrn		：
Author		：村上
Date		：10/8/26
Update		：10/10/21 BW465	暫定削除項目対応	村上 @007MwDOD[10/10/21]
			：Ver REF1010-0.01 2010-12-13 -Mura 通信関数名変更 @038MwDOD[10/12/13]
			：Ver REF1010-0.02 2010-12-15 -Mura HC冷媒漏洩検知CALL追加 @049MwDOD[10/12/15]
			: 14/04/07 BL7/BR7  エコナビ処理移設  Hieu (村上)	@082MzDOD[14/04/07]

#endif
**************************************************************************/
void vMAIN_MainPros (void){
/*	UCHAR ret;	*//* 不要な変数のため削除 */
		ucg_MAIN_ptest = 0;						/* P板検査制御ﾌﾗｸﾞ ｸﾘｱ		*/
		vMAIN_Heater();							/* ﾋｰﾀON/OFF処理 			*/	/* @005MzDOD[16/10/04] */

		vTIME_hour ();								/* 時間ﾀｲﾏ				*/
		vTIME_min ();								/* 分ﾀｲﾏ				*/
		vTIME_sec ();								/* 秒ﾀｲﾏ				*/
}

/**************************************************************************
<Title>			:vMAIN_Heater
<Function>		:デフロスト/PCEVAヒータのＯＮ／ＯＦＦ判定
<Description>	:通信状態と通電率に応じてヒータのＯＮ／ＯＦＦを設定する
<Input>			:なし
<Return>		:なし
<Input(EXT)>	:ucMINtm_trerct, ucg_TR_stbyct, ucTRNS_DEFHrate, ucTRNS_PCEVAHrate,
				 stg_TIMES_heatertm.word, stg_TIMEM_heatertm.word
<Output(EXT)>	:ucg_INT_DEFH, ucg_INT_PCEVAH, ung_MAIN_DEFHon, ung_MAIN_PCEVAHon,
				 stg_TIMES_heatertm.word, stg_TIMEM_heatertm.word
<Author>		:村上 和宏
<Date>			:
<Update>		:
<Copyright>		:Panasonic Refrigeration Co.,Ltd.
**************************************************************************/
void vMAIN_Heater(void){
UCHAR heat1,heat2;
static UCHAR	ucl_offDEFH = CLR;		/* ﾃﾞﾌﾛｽﾄﾋｰﾀ強制OFFﾌﾗｸﾞ	*//* @057sxDOD[19/09/10] */
	/* ﾀｲﾏｸﾘｱ処理 */	/* 100%DUTY出力実現のために関数冒頭にｸﾘｱ処理実行 */
	if ( stg_TIMES_heatertm.word  >= H_CYCLEW ){								/*	ﾋｰﾀﾀｲﾏ10s以上なら			*/
		stg_TIMES_heatertm.word = CLR;											/*	ﾋｰﾀﾀｲﾏｸﾘｱ					*/
	}

/*	if(ucMINtm_trerct < D_TRERRJUDGETM){	*/									/*通信異常継続時間1分未満なら 	*/	/* @018MzDOD[17/03/04] */
/*	if((ucMINtm_trerct < D_TRERRJUDGETM)	*/ 										/*通信異常継続時間60分未満なら */	/* @018MzDOD[17/03/04] */
/*		&& (ucg_TR_stbyct < TRNSFAULTNUM)){	*/										/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀｶｳﾝﾀがTRNSFAULTNUM回未満なら */	/* @086sxDOD[20/04/10] */
	if(u8g_MOTOR_ctrlflg == MOTOR_CTRL_ONLINE){                                 /*通信異常でないorｽﾀﾝﾊﾞｲ連続異常でないなら*/
		heat1 =	ucTRNS_DEFHrate;												/* 	正常時 ﾃﾞﾌﾛｽﾄﾋｰﾀ設定	  	*/
		heat2 =	ucTRNS_PCEVAHrate; 											/* 	正常時 ﾌﾚﾝﾁﾋｰﾀ設定	  		*/
		/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/
	}else{																		/*通信異常継続時間1分以上なら	*/
		heat1 =	H50_RATE_DEFH;													/* 	H50時 ﾃﾞﾌﾛｽﾄﾋｰﾀ設定	  		*/
		heat2 =	H50_RATE_PCEVAH;												/* 	H50時 ﾌﾚﾝﾁﾋｰﾀ設定	  		*/
		/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/
	}

	/*---- ﾃﾞﾌﾛｽﾄﾋｰﾀ ----*/
	if ( ucTRNS_DEFHrate == CLR ){												/*	通電率0%なら				*//* @057sxDOD[19/09/10] */
		stg_TIMEM_heatertm.word = CLR;											/*	ﾋｰﾀﾀｲﾏ(分)ｸﾘｱ				*//* @057sxDOD[19/09/10] */
		ucl_offDEFH = CLR;														/*	ﾃﾞﾌﾛｽﾄﾋｰﾀ強制OFFﾌﾗｸﾞ CLR	*//* @057sxDOD[19/09/10] */
	}
	if ( stg_TIMEM_heatertm.word >= DEFH_CYCMAX){								/*	ﾋｰﾀMAXON時間以上なら	*/	/* @057sxDOD[19/09/10] */
		ucl_offDEFH = SET;														/*	 ﾃﾞﾌﾛｽﾄﾋｰﾀ強制OFFﾌﾗｸﾞ SET	*//* @057sxDOD[19/09/10] */
	}
	if (ucl_offDEFH == SET){													/*	 ﾃﾞﾌﾛｽﾄﾋｰﾀ強制OFF状態なら	*//* @057sxDOD[19/09/10] */
		ucg_INT_DEFH = CLR;														/*	 ﾋｰﾀ OFF					*//* @057sxDOD[19/09/10] */
	}else{																		/*	 ﾃﾞﾌﾛｽﾄﾋｰﾀ強制OFF状態でない	*//* @057sxDOD[19/09/10] */
		ung_MAIN_DEFHon = unFUNC_heatontimeset( heat1,(USHORT)H_CYCLE );			/* ﾋｰﾀ通電率 → ﾋｰﾀON時間換算	*/
		if ( stg_TIMES_heatertm.word  >= ung_MAIN_DEFHon ){							/*	ON時間以上なら				*/
			ucg_INT_DEFH = CLR;														/*	 ﾋｰﾀ OFF					*/
		}else{																		/*	ON時間未満なら				*/
		 	ucg_INT_DEFH = SET;														/*	 ﾋｰﾀ ON						*/
		}																			
	}																			


	/*---- PCEVAﾋｰﾀｰ ----*/
	ung_MAIN_PCEVAHon = unFUNC_heatontimeset( heat2,(USHORT)H_CYCLE );			/* ﾋｰﾀ通電率 → ﾋｰﾀON時間換算 	*/
	if ( stg_TIMES_heatertm.word  >= ung_MAIN_PCEVAHon ){						/*	ON時間以上なら				*/
		ucg_INT_PCEVAH = CLR;													/*	 ﾋｰﾀ OFF					*/
	}else{																		/*	ON時間未満なら				*/
	 	ucg_INT_PCEVAH = SET;													/*	 ﾋｰﾀ ON						*/
	}

	/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/

}

/**************************************************************************
<Title>			:unFUNC_heatontimeset
<Function>		:ヒータＯＮ時間タイマ比較値設定
<Description>	:ヒータ通電率と通電サイクル時間からからＯＮ時間タイマ比較値を設定する
<Input>			:unl_timedt(分ﾃﾞｰﾀ)
<Return>		:unl_tmdthm(ﾀｲﾏ比較値)
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:-
<Author>		:小柳 英二郎
<Date>			:06/12/04
<Update>		:07/05/24 QAC指摘対応 小柳
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
USHORT unFUNC_heatontimeset( USHORT unl_heatp,USHORT unl_heatcycle ){
	USHORT unl_heatondt;

	unl_heatondt = ((((unl_heatp*unl_heatcycle)/100)<<8) | ((unl_heatp*unl_heatcycle)%100));	/*ON時間設定*/	/* @069bwOTM[07/05/24] */
	unl_heatondt = unFUNC_max_un(unl_heatondt, (unl_heatcycle<<8));			/* ON時間 MAX制限*/
	return ( unl_heatondt );
}


/**************************************************************************
<Title>			:unFUNC_max_un
<Function>		:USHORT版MAX制限
<Description>	:
<Input>			:unl_dat1(判定ﾃﾞｰﾀ)
<Return>		:unl_max(MAX値)
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:-
<Author>		:山崎(電住研)
<Date>			:06/11/08
<Update>		:
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
USHORT unFUNC_max_un(USHORT unl_dat1, USHORT unl_max){	/* MAX制限 */

	if ( unl_dat1 >= unl_max ){				/* MAX値より大? */
 		unl_dat1 = unl_max;					/* MAX値をｾｯﾄ */
 	}
 	return(unl_dat1);
}
