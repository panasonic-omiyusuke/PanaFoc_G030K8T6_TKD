/**************************************************************************
#ifdef DOC
File Name		：PCBCHK.C
Function		：P板検査制御
Description		：
Program			：
Author			：田中　正之
Date			：07/04/20
Update			：07/04/23 REF071 B32M1 UPD78F0501 FANﾁｪｯｸ用入力処理追加
				：07/04/26 規約対応　Y.Tsujii
				：07/05/11 ﾏｲｺﾝｺｰﾄﾞ表示同期出力設定追加
				：10/02/08 BU343 UPD78F0512 PCC変更
Copyright		：Matsushita Refrigeration Co.,Ltd.
#endif
**************************************************************************/
/* #pragma	sfr	*/			/* @000MzDOD[10/08/26]_06 */
/********************************************
		インクルードファイル
 ********************************************/

#include	"Sys_type_main.h"								/* @018MzDOD[13/07/11] */
#include	"Sys_func.h"									/* @018MzDOD[13/07/11] */
#include	"Ram.h"											/* @018MzDOD[13/07/11] */
#include 	"vipdc.h"										/* @040MwDOD[10/12/13] */
/* #include 	"Pcbchk.h" */										/* @037MzDOD[14/01/27] */
/* #include 	"sfr_r835c.h" */										/* @112MzDOD[14/06/03] */
#include 	"Pcbchk.h"										/* @262swDOD[23/07/05] */
#include 	"main.h"										/* @262swDOD[23/07/05] */
#include	"Adpdc.h"
#include "InvCommonFunc.h"

/********************************************/
/*		変数宣言							*/	/* @037MzDOD[14/01/27] */
/********************************************/
/*** global ***/
UCHAR ucg_TEST_chckcnt;						/* IGBT出力検査用ｶｳﾝﾀ(ﾋｰﾀに使用しない) */	/* @262swDOD[23/07/05] */
UCHAR ucg_TEST_chcksumi;					/* IGBT検査済みﾌﾗｸﾞ(ﾋｰﾀに使用しない) */		/* @262swDOD[23/07/05] */
UCHAR ucg_TEST_chcksumiall;					/* IGBT検査全相済みﾌﾗｸﾞ */					/* @262swDOD[23/07/05] */
UCHAR ucg_TEST_phasechckcnt;				/* 1相出力ごとのｶｳﾝﾀ */						/* @262swDOD[23/07/05] */
UCHAR ucg_TEST_heatchckcnt;					/* 出力ﾋｰﾀカウンタ			*/	/* @262swDOD[23/07/05] */
UCHAR ucg_TEST_heatsel;						/* 出力ﾋｰﾀ選択ﾌﾗｸﾞ			*/	/* @024MzDOD[17/03/09] */
UCHAR ucg_TEST_heatflg;						/* ﾋｰﾀ出力許可ﾌﾗｸ			*/	/* @024MzDOD[17/03/09] */
UCHAR ucg_TEST_heatsumi;					/* ﾋｰﾀ出力済みﾌﾗｸ			*/	/* @030MyOTM[17/04/04] */
UI16  ucg_TEST_adbuf[ARRNUM_ADAVEBUF];		/* AD値平均用リングバッファ	 */						

/*** static ***/
static UCHAR ucg_TEST_mcucodeoutputState = Output_Non;
static UCHAR ucg_TEST_bufFullflg;
static UCHAR ucg_TEST_adbufcnt;
UCHAR	ucMAIN_wdrfg;		/*ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄ発生用ﾌﾗｸﾞ */

/********************************************/
/*		外部変数参照						*/
/********************************************/
extern UCHAR	ucg_MAIN_ptest;		/* 検査制御ﾌﾗｸﾞ*/

// extern UCHAR	ucg_IN_add[ETC_ADNUM];	/* ANﾃﾞｰﾀ 格納*/	/* @002MxOOM[17/02/21] */  

extern UCHAR	ucg_INTf_testpa;		/*ﾎﾟｰﾄ入力確定ﾃﾞｰﾀ*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INTf_testpb;		/*ﾎﾟｰﾄ入力確定ﾃﾞｰﾀ*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INTf_test0;		/*  TEST0端子入力	*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INTf_test1;		/*  TEST1端子入力	*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INTf_test2;		/*  TEST2端子入力	*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INf_pcdr;			/*PCﾄﾞｱ入力確定ﾃﾞｰﾀ*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INf_rx;			/*RX入力確定ﾃﾞｰﾀ*/	/* @099zDOD[14/05/12] */
extern uint16_t ad_data_arr[ADC_SERIES_NUM]; 	/* adc データの格納用の変数（外部参照）　シャント用のADポート検査のためにアクセス */

extern union byte_def P0_addr;
extern union byte_def P1_addr;
extern union byte_def P2_addr;
extern union byte_def P4_addr;
extern union byte_def INTCMP_addr;

extern UCHAR ucg_INT_DEFH;	 		/* ﾃﾞﾌﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_PCEVAH;		/* PCEVAﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/
/* extern UCHAR ucPDC_ADVPN[];*/			/* 電流AD値 母線電圧AD	*/
extern UCHAR ucg_INf_fo;			/* FOﾎﾟｰﾄ(P板検査用)入力確定*/

/********************************************/
/*		     プロトタイプ宣言				*/
/********************************************/

/**************************************************************************
#ifdef DOC
Title		：vTEST_pcbchk
Function	：P板検査制御
Description	：
Input		：
Retrn		：
Author		：田中　正之
Date		：07/04/20
Update  	：07/04/26 規約対応　Y.Tsujii
			：09/06/16 BY551SM 検査制御変更 	村上 IND551_013[09/06/16]
			：09/06/17 BY551SM P板検査 仕様変更 村上 @000MxDOD[09/06/17]
			: 09/06/17 BY551SM Port割当て修正	村上 @001MxDOD[09/06/17]
			: 09/06/26 BY551SM P板検査仕様変更	村上 @003MxDOD[09/06/26]
			: 09/06/26 BY551SM P板検査ﾓｰﾄﾞ対応	村上 @008MwOTT[09/07/29]
			: 09/12/24 BU343   要求仕様実装 	村上 (KEY:BU343_000)
			: 10/03/16 BU343   P板検査 仕様変更 岩崎 @007IxDOD[10/03/16]
			: 10/11/02 BW465   P板検査入力判定処理追加 岩崎 @015IwDOD[10/11/02]
			: 17/12/20 3.5V 電流検知回路廃止に伴い、関連処理削除 吉田
			: 23/07/05 WX1 ﾊﾟﾜｰ素子検査仕様追加 瀧田 @262swDOD[23/07/05]
#endif
**************************************************************************/
void vTEST_pcbchk(void){										/* P板検査制御 */

/*-----------------------------------------------
   Ｐ板検査 入力確認	@015IwDOD[10/11/02]
-----------------------------------------------*/
	if ( (ucg_INTf_testpa == CLR) && (ucg_INTf_testpb == CLR) )		{	/* P板検査A＆P板検査BがL？ 			*/	/* @092MzDOD[14/04/24] */	/* @038MzDOD[15/07/13] */
		ucg_MAIN_ptest = 1;												/*	P板検査ﾌﾗｸﾞSET 					*/
	}else{																/* 上記以外のとき 					*/
		ucMAIN_wdrfg = 1;												/* 	WDTによるﾘｾｯﾄ⇒P板検査終了		*/
	}																	/*									*/
/* @016yxDOD[17/12/20] */
#if 0
	if((ucPDC_ADVPN[ADSTNUM_SHUNT] > IMAX) || (ucPDC_ADVPN[ADSTNUM_SHUNT] < IMIN)){		/* 電流検知回路異常のとき	*/	/* @030MyOTM[17/04/04] */
		vTEST_HEATERoff();																/* 	ﾋｰﾀOFF			 		*/
		return;																			/* 	ﾓｼﾞｭｰﾙを抜ける			*/
	}																					/*							*/
#endif
	/* ------ﾋｰﾀ出力検査------ */
	if(ucg_INf_fo == FO_INIT_STATE){												/* FOピンが初期値のままなら出力しない *//* @008lwORM[24/06/21] */
		vTEST_HEATERoff();													/* 	ﾋｰﾀOFF			 		*/
		ucg_TEST_heatsumi = 0;												/*	ﾋｰﾀ出力終了ﾌﾗｸﾞ			*/	/* F0信号H⇔Lで複数回ﾋｰﾀ出力検査する場合を想定 */
		ucg_TEST_heatsel = 0;												/* 	ﾋｰﾀ出力ﾓｰﾄﾞｸﾘｱ			*/
		ucg_TEST_heatchckcnt = 0;											/* 	ﾋｰﾀ切替ｶｳﾝﾀｸﾘｱ			*/
	}else if(ucg_TEST_heatsumi == SET){									/* ﾋｰﾀ出力済の場合(⋀ FOピンが真条件（Acitve条件）の時) *//* @008lwORM[24/06/21] */
		vTEST_HEATERoff();													/* 	ﾋｰﾀOFF			 		*/
	}else{																/* ﾋｰﾀ出力未 ⋀ FOピンが真条件（Active条件）の時 *//* @225swDOD[22/06/17] *//* @008lwORM[24/06/21] */
		vTEST_HEATERchk();													/* ﾋｰﾀ出力処理 */
	}
	
	/* ------IGBT出力検査------ */
	if(ucg_INf_fo == FO_INIT_STATE){												/* FOピンが初期値のままなら出力しない 		*//* @008lwORM[24/06/21] */
		vTEST_IPM_OutputOFF();												/*  出力停止 						*/				/* @262swDOD[23/07/05] */
		ucg_TEST_chckcnt = 0;												/*	IGBT出力ｶｳﾝﾀｸﾘｱ					*/				/* @262swDOD[23/07/05] */
		ucg_TEST_chcksumi = CLR;											/*	IGBT検査出力済みﾌﾗｸﾞ ｸﾘｱ		*/				/* @262swDOD[23/07/05] */
		ucg_TEST_chcksumiall = CLR;											/*  IGBT検査全相出力済みﾌﾗｸﾞ ｸﾘｱ 	*/				/* @262swDOD[23/07/05] */
		ucg_TEST_phasechckcnt = 0;											/* 1相出力ごとのｶｳﾝﾀ */	
	}else{																/* IGBT検査全相出力未 ⋀ FOピンが真条件（Acitve条件）の時		*/	/* @262swDOD[23/07/05] *//* @008lwORM[24/06/21] */
		vTEST_IGBTchk();													/* IPM検査出力処理 					*/				/* @262swDOD[23/07/05] */
	}
}


/**************************************************************************
#ifdef DOC
Title			：vTEST_HEATERon
Function		：ヒータ出力処理
Description	：
Input		：
Retrn		：
Author		：村上 和宏
Date		：17/03/06
Update	  	：
#endif
**************************************************************************/
void	vTEST_HEATERchk(){	/* ﾋｰﾀ切替処理 */

  /*-- ﾋｰﾀ切替時間ｶｳﾝﾄ --*/
	if(ucg_TEST_heatchckcnt >= T_HEATON){							/* ﾋｰﾀ切替ｶｳﾝﾀが1s経過したとき		*/
		ucg_TEST_heatsel++;											/* 	ﾋｰﾀ出力ﾓｰﾄﾞ ｲﾝｸﾘﾒﾝﾄ				*/
		ucg_TEST_heatchckcnt = 0;									/* 	ﾋｰﾀ切替ｶｳﾝﾀｸﾘｱ					*/
	}else{															/* 未経過							*/
		/* ｶｳﾝﾄ中 */
	}																/*									*/

  /*-- ﾋｰﾀ出力 --*/
	vTEST_HEATERon(ucg_TEST_heatsel);								/* ﾋｰﾀ出力ﾓｰﾄﾞに応じてﾋｰﾀ出力		*/

  /*-- ﾋｰﾀ出力 終了判定 --*/
	if(ucg_TEST_heatsel >= NUM_SELMOD){								/* ﾋｰﾀ出力 所定回数到達したとき		*/
		ucg_TEST_heatsumi =SET;										/* ﾋｰﾀ出力済ﾌﾗｸﾞｾｯﾄ					*/
		ucg_TEST_heatsel = 0;										/* 	ﾋｰﾀ出力ﾓｰﾄﾞｸﾘｱ					*/
		ucg_TEST_heatchckcnt = 0;									/* 	ﾋｰﾀ切替ｶｳﾝﾀｸﾘｱ					*/
	}																/*									*/

	ucg_TEST_heatchckcnt++;											/* 	出力 カウントアップ				*/
}

/**************************************************************************
#ifdef DOC
Title			：vTEST_HEATERon
Function		：ヒータ出力処理
Description	：
Input		：
Retrn		：
Author		：村上 和宏
Date		：17/03/06
Update	  	：24/12/03 HY2EY2 FCT仕様変更 親泊 @014lxHOM[24/12/03]
#endif
**************************************************************************/
void vTEST_HEATERon(UCHAR sel){	/* HEATER出力処理 */

 /*-- 一旦ﾋｰﾀ出力変数ｸﾘｱ --*/
	ucg_INT_DEFH	= 0;				/* ﾃﾞﾌﾛｽﾄﾋｰﾀｰ 			*/
	ucg_INT_PCEVAH = 0;	 			/* PCEVAﾋｰﾀｰ 				*/
/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/

/*-- ﾋｰﾀ出力処理 --*/
	switch(sel){/* @014lxHOM[24/12/03] */
		case	0:	ucg_INT_DEFH = SET;			/* ﾃﾞﾌﾛｽﾄﾋｰﾀ ON		*/
					ucg_INT_PCEVAH = SET;		/* PCEVAﾋｰﾀ ON		*/
/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/
			break;
		default:								/* ﾋｰﾀ出力OFF区間	*//* @014lxHOM[24/12/03] */
			break;
	}
}

/**************************************************************************
#ifdef DOC
Title		：vTEST_HEATERoff
Function	：ヒータ出力停止処理
Description	：
Input		：
Retrn		：
Author		：村上 和宏
Date		：17/03/06
Update	  	：
#endif
**************************************************************************/
void vTEST_HEATERoff(void){	/* HEATER停止処理 */

		ucg_INT_DEFH	= 0;				/* ﾃﾞﾌﾛｽﾄﾋｰﾀｰ 			*/
		ucg_INT_PCEVAH = 0;	 			/* ﾌﾚﾝﾁｰﾀｰ 				*/
}

/**************************************************************************
#ifdef DOC
Title		：vTEST_IPM_OutputOFF
Function	：IPM検査用出力停止処理
Description	：
Input		：
Retrn		：
Author		：Takida
Date		：23/07/05    @262swDOD[23/07/05] 
Update	  	：24/05/21    @008lwORM[24/06/21] 極性を意識せず出力できる定義値を使うよう変更
#endif
**************************************************************************/
void vTEST_IPM_OutputOFF(void)
{
  /*Configure GPIO pin Output Level */	/*極性を意識しない出力処理に変更*//* @008lwORM[24/06/21] */
    /*上アーム*/
	HAL_GPIO_WritePin(PWM_U_HIGH_GPIO_Port,PWM_U_HIGH_Pin,UPPER_ARM_OFF);/*ポートの定義値変更で対応できるよう各相のポート毎に設定する*/ /* @009lwORM[24/06/24] */
	HAL_GPIO_WritePin(PWM_V_HIGH_GPIO_Port,PWM_V_HIGH_Pin,UPPER_ARM_OFF);/* @009lwORM[24/06/24] */
	HAL_GPIO_WritePin(PWM_W_HIGH_GPIO_Port,PWM_W_HIGH_Pin,UPPER_ARM_OFF);/* @009lwORM[24/06/24] */
    /*下アーム*/
    HAL_GPIO_WritePin(PWM_U_LOW_GPIO_Port,PWM_U_LOW_Pin,LOWER_ARM_OFF);/* @009lwORM[24/06/24] */
	HAL_GPIO_WritePin(PWM_V_LOW_GPIO_Port,PWM_V_LOW_Pin,LOWER_ARM_OFF);/* @009lwORM[24/06/24] */
	HAL_GPIO_WritePin(PWM_W_LOW_GPIO_Port,PWM_W_LOW_Pin,LOWER_ARM_OFF);/* @009lwORM[24/06/24] */
}
/**************************************************************************
#ifdef DOC
Title		：vTEST_IPM_GPIO_Init
Function	：IPM検査用GPIO設定処理
Description	：
Input		：
Retrn		：
Author		：Takida
Date		：23/07/05    @262swDOD[23/07/05] 
Update	  	：
#endif
**************************************************************************/
void vTEST_IPM_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pins : Uout_Pin, Vout_Pin, Wout_Pin */
  GPIO_InitStruct.Pin = PWM_U_HIGH_Pin|PWM_V_HIGH_Pin|PWM_W_HIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 mDisableInterrupt();						/* 割り込み禁止 */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 mEnableInterrupt();						/* 割り込み許可 */
	
  /*Configure GPIO pins : Xout_Pin, Yout_Pin, Zout_Pin */
  GPIO_InitStruct.Pin = PWM_U_LOW_Pin|PWM_V_LOW_Pin|PWM_W_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 mDisableInterrupt();						/* 割り込み禁止 */
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 mEnableInterrupt();						/* 割り込み許可 */
}
/**************************************************************************
#ifdef DOC
Title			：vTEST_IGBTchk
Function		：IGBT検査制御
Description	：1入力トリガで3相分の検査出力する
Input		：
Retrn		：
Author		：Takida
Date		：23/07/05    @262swDOD[23/07/05] 
Update	  	：
#endif
**************************************************************************/
void vTEST_IGBTchk(void){										/* IGBTテスト出力 */
	
	if(ucg_TEST_chcksumiall == CLR){	/* IGBT検査全相出力未の場合 */	/* @262swDOD[23/07/05] */
		switch(ucg_TEST_phasechckcnt){
			case 0:
				vTEST_IGBTon(PWM_U_LOW_GPIO_Port, PWM_U_LOW_Pin, PWM_U_HIGH_GPIO_Port, PWM_U_HIGH_Pin, PWM_V_LOW_GPIO_Port, PWM_V_LOW_Pin);	/* U-Y出力 (充電Xオン) */
				if(ucg_TEST_chcksumi == SET){			/* 1相出力完了？ */
					ucg_TEST_phasechckcnt++;				/* 1相出力ごとにｶｳﾝﾀｲﾝｸﾘ */
					ucg_TEST_chcksumi = CLR;			/*	IGBT出力ｶｳﾝﾀｸﾘｱ			*/
				}
				break;
			case 1:
				vTEST_IGBTon(PWM_W_LOW_GPIO_Port, PWM_W_LOW_Pin, PWM_W_HIGH_GPIO_Port, PWM_W_HIGH_Pin, PWM_U_LOW_GPIO_Port, PWM_U_LOW_Pin);	/* W-X出力 (充電Zオン) */
				if(ucg_TEST_chcksumi == SET){			/* 1相出力完了？ */
					ucg_TEST_phasechckcnt++;				/* 1相出力ごとにｶｳﾝﾀｲﾝｸﾘ */
					ucg_TEST_chcksumi = CLR;			/*	IGBT出力ｶｳﾝﾀｸﾘｱ			*/
				}
				break;
			case 2:
				vTEST_IGBTon(PWM_V_LOW_GPIO_Port, PWM_V_LOW_Pin, PWM_V_HIGH_GPIO_Port, PWM_V_HIGH_Pin, PWM_W_LOW_GPIO_Port, PWM_W_LOW_Pin);	/* V-Z出力 (充電Yオン) */
				if(ucg_TEST_chcksumi == SET){			/* 1相出力完了？ */
					ucg_TEST_phasechckcnt++;				/* 1相出力ごとにｶｳﾝﾀｲﾝｸﾘ */
					ucg_TEST_chcksumi = CLR;			/*	IGBT出力ｶｳﾝﾀｸﾘｱ			*/
				}
				break;
			case 3:
				vTEST_IPM_OutputOFF();					/*  出力停止*/
				ucg_TEST_chcksumiall = SET;				/*	IGBT出力完了ﾌﾗｸﾞセット */
				ucg_TEST_phasechckcnt = 0;				/* ｶｳﾝﾀｸﾘｱ */
				break;
			default:
				vTEST_IPM_OutputOFF();					/*  出力停止*/
				ucg_TEST_phasechckcnt = 0;				/* ｶｳﾝﾀｸﾘｱ */
				break;
		}
	}else{								/* IGBT検査全相出力済みの場合 */	/* @262swDOD[23/07/05] */
		vTEST_IPM_OutputOFF();							/*  出力停止*/
	}
}
/**************************************************************************
#ifdef DOC
Title			：vTEST_IGBTon
Function		：IGBT出力処理
Description	：
Input		：
Retrn		：
Author		：Takida
Date		：23/07/05    @262swDOD[23/07/05] 
Update	  	：
#endif
**************************************************************************/ 
void	vTEST_IGBTon(GPIO_TypeDef *PORT_CHARGE,uint16_t PIN_CHARGE,GPIO_TypeDef *PORT_UPPER,uint16_t PIN_UPPER,GPIO_TypeDef *PORT_LOWER,uint16_t PIN_LOWER){	/* IGBT出力P板検査 */
		/*極性を意識しない出力処理に変更 @001lxORM[24/06/12] */
		if(ucg_TEST_chcksumi == CLR){ 											/* IGBT出力終了未なら										 */
			ucg_TEST_chckcnt++;												/* 	IGBT出力 カウントアップ								*/

			if(ucg_TEST_chckcnt <= T_CHARGE){									/* 充電処理：0ms～60ms (60ms間)							*/
				HAL_GPIO_WritePin(PORT_CHARGE, PIN_CHARGE, LOWER_ARM_ON);		/* 	充電用下アームON	(下ｱｰﾑON   HiActiveなので対象ﾋﾞｯﾄをｾｯﾄ) *//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_UPPER, PIN_UPPER, UPPER_ARM_OFF);		/* 	出力用上アームOFF	(上ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */	
				HAL_GPIO_WritePin(PORT_LOWER, PIN_LOWER,LOWER_ARM_OFF );		/*	出力用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
			}else if(ucg_TEST_chckcnt <=(T_CHARGE+T_WAIT)){					/* 上下短絡防止：60ms～80ms (20ms間)						*/
				HAL_GPIO_WritePin(PORT_CHARGE, PIN_CHARGE, LOWER_ARM_OFF);		/*	充電用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_UPPER, PIN_UPPER,UPPER_ARM_OFF );		/* 	出力用上アームOFF	(上ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ) *//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_LOWER, PIN_LOWER, LOWER_ARM_OFF);		/*	出力用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
			}else if(ucg_TEST_chckcnt <= (T_CHARGE+T_WAIT+T_OUT)){			/* 80ms～180ms (100ms間)									*/	
				HAL_GPIO_WritePin(PORT_CHARGE, PIN_CHARGE,LOWER_ARM_OFF);		/* 	充電用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_UPPER, PIN_UPPER, UPPER_ARM_ON);			/* 	出力用上アームON	(上ｱｰﾑON   HiActiveなので対象ﾋﾞｯﾄをｾｯﾄ)	*//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_LOWER, PIN_LOWER, LOWER_ARM_ON);			/* 	出力用下アームON	(下ｱｰﾑON   HiActiveなので対象ﾋﾞｯﾄをｾｯﾄ)	*//* @008lwORM[24/06/21] */
			}else if(ucg_TEST_chckcnt < (T_CHARGE+T_WAIT+T_OUT+T_STOP)){	/* 180ms～220ms (40ms間)									*//* @262swDOD[23/07/05] */
				HAL_GPIO_WritePin(PORT_CHARGE, PIN_CHARGE, LOWER_ARM_OFF);		/* 充電用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	 *//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_UPPER, PIN_UPPER,UPPER_ARM_OFF );		/* 出力用上アームOFF	(上ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_LOWER, PIN_LOWER, LOWER_ARM_OFF);		/* 出力用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
			}else{	/* 220ms～240ms (20ms間)									*//* @262swDOD[23/07/05] */
				HAL_GPIO_WritePin(PORT_CHARGE, PIN_CHARGE, LOWER_ARM_OFF);		/* 充電用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ) *//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_UPPER, PIN_UPPER, UPPER_ARM_OFF);		/* 出力用上アームOFF	(上ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
				HAL_GPIO_WritePin(PORT_LOWER, PIN_LOWER, LOWER_ARM_OFF);		/* 出力用下アームOFF	(下ｱｰﾑOFF  HiActiveなので対象ﾋﾞｯﾄをｸﾘｱ)	*//* @008lwORM[24/06/21] */
				ucg_TEST_chckcnt = 0;											/*	IGBT出力カウンタクリア									*/
				ucg_TEST_chcksumi = SET;										/*	IGBT出力終了フラグ セット(1回のﾄﾘｶﾞ検知で出力は1回のため)	*/	
			}
		}
}
/**************************************************************************
#ifdef DOC
Title			：vTEST_adbufmax
Function		：リングバッファカウンタのMAX判定を行う
Description	：
Input		：
Retrn		：
Author		：Shimpaku
Date		：24/10/29
Update	  	：
#endif
**************************************************************************/ 
void vTEST_adbufmax(void){
	ucg_TEST_adbufcnt++;
	if(ucg_TEST_adbufcnt >= ARRNUM_ADAVEBUF ){
		ucg_TEST_adbufcnt = 0;/* リングバッファカウンタをクリア */
		ucg_TEST_bufFullflg = SET;/* 以降はCLRに設定されない */
	}
}

/**************************************************************************
#ifdef DOC
Title			：vTEST_get_ShuntADAverageCalc
Function		：平均値計算用のリングバッファにシャント用ADポート値を代入、平均する
Description	：
Input		：
Retrn		：ucg_TEST_adresult（平均結果）
Author		：Shimpaku
Date		：24/10/29
Update	  	：
#endif
**************************************************************************/ 
UI16 vTEST_get_ShuntADAverageCalc(void){
	UI16 ucl_TEST_adresult;/* AD平均値格納用変数 */
	ucg_TEST_adbuf[ucg_TEST_adbufcnt] = ad_data_arr[ADSTNUM_SHUNT];
	vTEST_adbufmax();/* リングバッファの要素数MAX判定 */

	if(ucg_TEST_bufFullflg == SET){/* 加算平均許可 */
		ucl_TEST_adresult = (ucg_TEST_adbuf[0] + ucg_TEST_adbuf[1] + ucg_TEST_adbuf[2] + ucg_TEST_adbuf[3]) / ARRNUM_ADAVEBUF;/* 平均値 */
	}else{/* 加算平均禁止 */
		ucl_TEST_adresult = ucg_TEST_adbuf[0];/* 初期値 */
	}
	return (ucl_TEST_adresult);
}

/**************************************************************************
#ifdef DOC
Title			：vTEST_Set_McuCodeoutputState
Function		：取得したAD値を処理して、その結果からマイコンコード出力ステータスを決める
Description	：
Input		：
Retrn		：
Author		：Shimpaku
Date		：24/10/29
Update	  	：
#endif
**************************************************************************/ 
void vTEST_Set_McuCodeoutputState(void){
	UI16 shuntAD;
	shuntAD = vTEST_get_ShuntADAverageCalc();
	if(shuntAD > SHUNTADPORT_TH2){
		ucg_TEST_mcucodeoutputState = Outout_LB;/* 下位4bit出力 */
	}else if(shuntAD > SHUNTADPORT_TH1){
		ucg_TEST_mcucodeoutputState = Output_HB;/* 上位4bit出力 */
	}else{
		ucg_TEST_mcucodeoutputState = Output_Non;/* 出力なし */
	}
}

/**************************************************************************
#ifdef DOC
Title			：vTEST_Get_McuCodeOutputState
Function		：現在のマイコンコード出力ステータスを返す
Description	：
Input		：
Retrn		：ucg_TEST_mcucodeoutputState
Author		：Shimpaku
Date		：24/10/29
Update	  	：
#endif
**************************************************************************/ 
UI08 vTEST_Get_McuCodeOutputState(void){
	return (ucg_TEST_mcucodeoutputState);
}
