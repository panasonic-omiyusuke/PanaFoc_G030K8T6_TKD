/**************************************************************************
#ifdef DOC
File Name		：INTTM51.c
Function		：ＩＮＴＴＭ５１割込み処理(3.3ms割込み処理)
Description		：
Program			：
Author			：小柳　英二郎
Date			：05/08/22
Update			：07/03/21 B32M1 UPD780501
				：07/04/23 REF071 B32M1 UPD78F0501 P板検査ﾓｰﾄﾞ用入力処理追加
				：07/04/26 REF071 B32M1 UPD78F0501 ｽﾀｯｸ領域変更(170byte確保)
												   P板検査ﾓｰﾄﾞ用ﾎﾟｰﾄ入力処理追加
				：07/05/07 REF071-1.01 B32M1 UPD78F0501 ADCR→ADCRH修正 @001agOTT[07/05/08]
														ｽﾀｯｸ領域変更,TEST1/TEST2入替え
				：07/05/16 REF071-1.02 B32M1 UPD78F0501 ﾌｧﾝ電圧切替修正 @006agOTT[07/05/16]
				：07/07/18 REF071-1.03 B32M1 UPD78F0501 H51時に発振周波数ｶｳﾝﾀをH固定
				：09/12/24 BU343 要求仕様実装 				村上 (KEY:BU343_000)
				：10/02/25 BU343 ｺﾝﾌﾟﾚｯｻのﾎﾟｰﾄ出力を反転	岩崎 @004IxDOD[10/02/25]
				：10/02/26 BU343 多重割込み禁止				岩崎 @005IxDOD[10/02/26]
				：10/10/22 BW465 AD値取得処理整備 			村上 @008MwDOD[10/10/22]
				：10/10/21 BW465 暫定削除項目対応			村上 @007MwDOD[10/10/21]
				：10/11/01 BW465 三菱IPM基板用IO対応 		村上 @013MwDOD[10/11/01]
				：10/11/01 BW465 A/D入力選択ﾚｼﾞｽﾀ設定変更	岩崎 @014IwDOD[10/11/01]
				: 10/12/08 BW465 デバッグ用出力変数追加 	村上 @037MwDOD[10/12/08]
				: 11/06/06 BW465 ADch定義修正			 	村上 @073MwDOD[11/06/06]
Copyright		：Matsushita Refrigeration Co.,Ltd.
#endif
**************************************************************************/

/********************************************
		インクルードファイル
 ********************************************/
#include    "Sys_type.h"
#include	"Sys_type_main.h"								/* @018MzDOD[13/07/11] */
#include	"Sys_func.h"									/* @018MzDOD[13/07/11] */
#include    "vipdc.h"
#include	"Ram.h"											/* @018MzDOD[13/07/11] */

#ifdef RENESAS_TO_ST32
#include 	"main.h"
#else /* RENESAS_TO_ST32 */
#ifdef UNIT_TEST
#include "sfr_r835c_dummy.h"
#else
#include "sfr_r835c.h"
#endif
#endif /* RENESAS_TO_ST32 */

//#include	"Adpdc.h"
#include	"PMD1GPar.h"
#include 	"Inttm51.h"							/* @072MzDOD[14/02/10] */
#include	"Pcbchk.h"

/********************************************/
/*		変数宣言							*/		/* @037MzDOD[14/01/27] */
/********************************************/

/******* GLOBAL *******/
UCHAR	ucg_MAIN_20up;		/* 20msﾀｲﾑUP*/		/* @000IzDOD[10/09/13]_03 */
UCHAR	ucg_IN_add[ETC_ADNUM];	/* ANﾃﾞｰﾀ 格納*/	/* @002MxOOM[17/02/21] */
/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi1;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi2;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi3;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi4;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi5;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi6;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi7;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi8;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdi9;		/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/	/* @112MzDOD[14/06/03] */
UCHAR	ucg_INT_ptdi10;	/*ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/	/* @000MzDOD[14/12/17] */

/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt1;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt2;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt3;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt4;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt5;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt6;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt7;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt8;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/
UCHAR	ucg_INT_ptdt9;		/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/	/* @112MzDOD[14/06/03] */
UCHAR	ucg_INT_ptdt10;	/*10msec前ﾎﾟｰﾄ入力ﾃﾞｰﾀ*/	/* @000MzDOD[14/12/17] */
//UCHAR test;

/*ﾎﾟｰﾄｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn1;		/*入力1ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn2;		/*入力2ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn3;		/*入力3ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn4;		/*入力4ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn5;		/*入力5ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn6;		/*入力6ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn7;		/*入力7ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn8;		/*入力8ｶｳﾝﾀ*/
UCHAR	ucg_INT_ptcn9;		/*入力9ｶｳﾝﾀ*/	/* @112MzDOD[14/06/03] */
UCHAR	ucg_INT_ptcn10;	/*入力10ｶｳﾝﾀ*/	/* @000MzDOD[14/12/17] */


/*入力確定データ*/
UCHAR	ucg_INTf_testpa;		/* testa 入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INTf_testpb;		/* testb 入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INTf_test0;		/* test0 入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INTf_test1;		/* 強制ﾃﾞﾌ 入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INTf_test2;		/* 連続冷却 入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INf_pcdr;			/* PCﾄﾞｱ 入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INf_rx;			/* Rxd 入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INf_kisyu;		/* 機種切替入力確定ﾃﾞｰﾀ */
UCHAR	ucg_INf_fo;			/* FOﾎﾟｰﾄ(P板検査用)入力確定		*/	/* @100zDOD[14/05/12] */
UCHAR	ucg_INf_dumper;	/* ダンパ切り替え入力確定				*/	/* @000MzDOD[14/12/17] */

/*出力データ*/
UCHAR	 ucg_OUTf_defh;		/* DEFﾋｰﾀ出力		*/	/* @097MzDOD[14/05/06] */
UCHAR	 ucg_OUTf_fcfan;		/*  ﾌｧﾝ出力 		*/	/* @097MzDOD[14/05/06] */
UCHAR	 ucg_OUTf_fnchg;		/* ﾌｧﾝ切替出力		*/	/* @097MzDOD[14/05/06] */
UCHAR	 ucg_OUTf_plamp;		/* 庫内灯 		*/	/* @097MzDOD[14/05/06] */
UCHAR	 ucg_OUTf_tx;		/* ﾏｲｺﾝ通信Tx		*/	/* @097MzDOD[14/05/06] */

/******* STATIC *******/
/* static UCHAR	ucg_ADC_admd; */						/* A/Dﾓｰﾄﾞ*//*070424*//* @271swDOD[23/10/10] */
/* static UCHAR	ucg_ADC_adwt; */						/* A/Dｲﾆｼｬﾙﾌﾗｸﾞ*//*070424*//* @271swDOD[23/10/10] */
static UCHAR	ucg_INT_40cn;		/* 40msｶｳﾝﾄ*/
static UCHAR	ucg_INT_20cn;		/* 20msｶｳﾝﾄ*/
static UCHAR	ucg_INT_10md;		/* 10msｶｳﾝﾀ(3.3ms*3回)*/
static UCHAR	ucg_INT_20md;		/* 20msｶｳﾝﾀ(10ms*2回)*/

/********************************************/
/*		     プロトタイプ宣言				*/
/********************************************/
void vINT_pcbchkiniout(void);	/* P板検査制御出力処理 */	/* @004AxDOD[15/06/30] */
void	vINT_uarttm(void);						/*通信ﾀｲﾏｶｳﾝﾄ処理*//* @073HxDOD[11/05/11] */

/**************************************************************************
#ifdef DOC
Title		：vINT_inttm51
Function	：ＩＮＴＴＭ５１割込み処理(3.3mS割込み処理)
Description	：
Input		：
Retrn		：
Author		：
Date		：05/08/22
Update		：07/03/21 B32M1 UPD780501
			：07/04/24 規約対応　Y.Tsujii
			：07/04/26 B32M1 UPD780501 P板検査追加により入力処理変更 仕様変更対応
			: 09/06/09 BY551SM ﾎﾟｰﾄ対応 				村上 IND551_000[09/06/09]
			: 09/06/10 BY551SM 光野菜LED制御変更		村上 IND551_008[09/06/10]
			: 09/06/15 BY551SM FCﾌｧﾝ切替⇔PCﾄﾞｱSW修正	村上 IND551_012[09/06/15]
			: 09/06/17 BY551SM Port割当て修正			村上 @001MxDOD[09/06/17]
			: 09/12/24 BU343 要求仕様実装 				村上 (KEY:BU343_000)
			: 10/02/25 BU343 ｺﾝﾌﾟﾚｯｻのﾎﾟｰﾄ出力を反転	岩崎 @004IxDOD[10/02/25]
			: 10/02/26 BU343 多重割込み禁止				岩崎 @005IxDOD[10/02/26]
			: 10/10/22 BW465 AD値取得処理整備 			村上 @008MwDOD[10/10/22]
			: 10/10/21 BW465 暫定削除項目対応			村上 @007MwDOD[10/10/21]
			: 10/11/01 BW465 三菱IPM基板用IO対応 		村上 @013MwDOD[10/11/01]
			: 10/11/02 BW465 ﾃﾞﾌﾋｰﾀ出力反転				岩崎 @015IwDOD[10/11/02]
			: 10/11/04 BW465 発振周波数出力判定変更		岩崎 @016IwDOD[10/11/04]
			: 10/12/08 BW465 デバッグ用出力変数追加 	村上 @037MwDOD[10/12/08]
			: 11/01/17 BW465 デバッグ用出力変数追加 	村上 @055MwDOD[11/01/17]
@055MwDOD[11/01/17]
#endif
**************************************************************************/
 void vMAIN_MainInttm51(void){																			/* @000MzDOD[10/08/26]_03 */

/*-------------------------------------------
   ﾒｲﾝﾙｰﾌﾟ用ｶｳﾝﾀ処理
-------------------------------------------*/
	ucg_INT_40cn++;											/*40msｶｳﾝﾀ+1*/
	if ( ucg_INT_40cn >= 20 ){								/*40msｶｳﾝﾀ≧20 (2ms×20=40ms)*/
		ucg_INT_40cn = 0;									/*40msｶｳﾝﾀｸﾘｱ*/
		ucg_MAIN_40up = SET;								/*40msﾌﾗｸﾞｾｯﾄ*/
	}else{													/*40msｶｳﾝﾀ＜20*/
		ucg_MAIN_40up = CLR;								/*40msﾌﾗｸﾞｸﾘｱ*/
	}

/*-------------------------------------------
   P板検査ﾙｰﾌﾟ用ｶｳﾝﾀ処理
-------------------------------------------*/				/* @000IzDOD[10/09/13]_03 */
	ucg_INT_20cn++;											/*20msｶｳﾝﾀ+1*/
	if ( ucg_INT_20cn >= 10 ){								/*20msｶｳﾝﾀ≧10 (2ms×10≒20ms)*/
		ucg_INT_20cn = 0;									/*20msｶｳﾝﾀｸﾘｱ*/
		ucg_MAIN_20up = SET;								/*20msﾌﾗｸﾞｾｯﾄ*/
	}else{													/*20msｶｳﾝﾀ＜10*/
		ucg_MAIN_20up = CLR;								/*20msﾌﾗｸﾞｸﾘｱ*/
	}

	/*vINT_potinput();*/										/*入力処理*/
	vINT_potoutput();										/*出力処理*/
}

/**************************************************************************
#ifdef DOC
Title			：vINT_potinput
Function		：入力処理
Description	：
Input		：PORT_PCB2
			：PORT_PCB1
			：PORT_TEST0
			：PORT_TEST0
			：PORT_TEST0
			：PORT_DOOR
			：PORT_RX
Retrn		：
Author		：村上 和宏
Date		：14/05/12
Update		：
#endif
**************************************************************************/
void vINT_potinput(void){																/* @099zDOD[14/05/12] */

	ucg_INT_10md++;										/*10msﾓｰﾄﾞ+1*/
	if ( ucg_INT_10md >= 3 ){									/*10msﾓｰﾄﾞ≧3(3.3ms×3≒10ms)*/
		ucg_INT_10md = 0;									/*10msﾓｰﾄﾞｸﾘｱ*/

#if 0
//		ucg_INT_ptdi3 = PORT_RX;						/* TEST0 切替 			*/	/* @002MzDOD[16/09/13] *//*要否検討のこと　USART2RXのピンを充てるのがよさそう 瀧*/
		ucg_INT_ptdi4 = HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin);						/* TEST1入力(強制DEF)	*/
//		ucg_INT_ptdi5 = PORT_KISYU;						/* 機種切替ﾎﾟｰﾄ入力		*/	/* @013MzDOD[17/02/24] *//*要否検討のこと*/

		vINT_potinset(&ucg_INT_ptdi1,	&ucg_INT_ptdt1,	&ucg_INT_ptcn1,	&ucg_INTf_testpa,	INPUT_SETDT);		/* TESTA(P板検査) 入力確定	*/
		vINT_potinset(&ucg_INT_ptdi2,	&ucg_INT_ptdt2,	&ucg_INT_ptcn2,	&ucg_INTf_testpb,	INPUT_SETDT);		/* TESTB(P板検査) 入力確定	*/
//		vINT_potinset(&ucg_INT_ptdi3,	&ucg_INT_ptdt3,	&ucg_INT_ptcn3,	&ucg_INf_rx,		INPUT_SETDT);		/* 通信Rx(P板検査)入力確定	*/	/* @002MzDOD[16/09/13] */
		vINT_potinset(&ucg_INT_ptdi4,	&ucg_INT_ptdt4,	&ucg_INT_ptcn4,	&ucg_ITf_fo,		INPUT_SETDT);		/* FOﾎﾟｰﾄ(P板検査)入力確定	*/	/* @002MzDOD[16/09/13] */
//		vINT_potinset(&ucg_INT_ptdi5,	&ucg_INT_ptdt5,	&ucg_INT_ptcn5,	&ucg_ITf_kisyu,		INPUT_SETDT);		/* 機種切替ﾎﾟｰﾄ	  入力確定	*/	/* @013MzDOD[17/02/24] */
	}
#endif
	}
}

/**************************************************************************
<Title>			:vINT_potinset
<Function>		:入力ポートデータ確定処理
<Description>	:入力ポートデータと前回データの比較
				:設定回数連続一致で確定
<Input>			:*ptdi(現入力ﾃﾞｰﾀ)
				:*ptdt(前回入力ﾃﾞｰﾀ)
				:*ptcn(入力確定ｶｳﾝﾀ)
				:ucl_cndt(一致判定回数)
<Return>		:*ptdt(前回入力ﾃﾞｰﾀ)
				:*ptcn(入力確定ｶｳﾝﾀ)
				:*ptda(入力確定ﾃﾞｰﾀ)
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:-
<Author>		:小柳　英二郎
<Date>			:06/11/16
<Update>		:06/11/29 修正 小柳
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
void vINT_potinset(UCHAR *adrw,UCHAR *adrx,UCHAR *adry,UCHAR *adrz,UCHAR ucl_cndt){
	if ( *adrw == *adrx ){							/*ﾎﾟｰﾄ入力値と10msec前ﾎﾟｰﾄ入力値比較*/
		*adry += 1;									/*ﾁｬﾀﾘﾝｸﾞｶｳﾝﾀ+1*/
		*adrx = *adrw;								/*ﾎﾟｰﾄ入力値更新*/
		if ( *adry >= ucl_cndt ){					/*60msec経過*/
			*adry = CLR;							/*ﾁｬﾀﾘﾝｸﾞｶｳﾝﾀｸﾘｱ*/
			*adrz = *adrw;							/*ﾎﾟｰﾄ入力確定ﾃﾞｰﾀ=ﾎﾟｰﾄ入力*/
		}
	}else{
		*adry = CLR;								/*ﾁｬﾀﾘﾝｸﾞｶｳﾝﾀｸﾘｱ*/
		*adrx = *adrw;								/*ﾎﾟｰﾄ入力値更新*/
	}
}

/**************************************************************************
Title			：vINT_potoutput
Function		：出力処理
Description	：
Input		：
Output		：
Author		：村上 和宏
Date		：14/05/12
Update		：18/10/03 マイコンコード2byte化対応 吉田
			：18/10/31 マイコンコード出力を、4bitから8bitに変更 吉田
			：22/01/18 STマイコン対応 瀧田
			：23/03/31 HV1 ﾋｰﾀ変更 瀧田  @246swDOD[23/03/31] 
**************************************************************************/
void vINT_potoutput(void){

	ucg_INT_20md++;											/* 20msﾓｰﾄﾞ+1					*/
	if ( ucg_INT_20md >= 10 ){									/* 20msﾓｰﾄﾞ≧10	(2ms×10=20ms)*/
		ucg_INT_20md = 0;										/* 20msﾓｰﾄﾞｸﾘｱ					*/
		
		/*--- ヒータ出力 --- */
		HAL_GPIO_WritePin(HEAT1_GPIO_Port, HEAT1_Pin, ucg_INT_DEFH);	/* ﾃﾞﾌﾋｰﾀｰ 				*/
		HAL_GPIO_WritePin(HEAT2_GPIO_Port, HEAT2_Pin, ucg_INT_PCEVAH);	/* PCEVAﾋｰﾀｰ 			*/
	}
}

/**************************************************************************
<Function>		:オプションバイト確認処理
<Author>		:瀧田
<Date>			:2022/04/11
<Update>		:
 **************************************************************************/
int vMAIN_IsOptionByteCheck(void){
	
	if( ((mREADOPBYTE(OPBYTEADDRS1) & OPBYTECHK1_MASK) == (OPBYTECHK1 & OPBYTECHK1_MASK)) /* 予約bitはマスクで無視 */
	&& ((mREADOPBYTE(OPBYTEADDRS4) & OPBYTECHK4_MASK) == (OPBYTECHK4 & OPBYTECHK4_MASK))            
	&& ((mREADOPBYTE(OPBYTEADDRS5) & OPBYTECHK5_MASK) == (OPBYTECHK5 & OPBYTECHK5_MASK)) ){		/* オプションバイトが内部データと一致の場合 */
		return 1;
	}else{												/* オプションバイトが内部データと不一致の場合 */
		return 0;
	}

}

