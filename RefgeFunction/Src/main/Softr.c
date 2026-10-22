/**************************************************************************
<FileName>		:SOFTR.c
<Function>		:ｿﾌﾄﾘｾｯﾄ処理
<Program>		:vMAIN_Softr()
<Author>		:大内山 智則 -Outi
<Date>			:2002-0219
<Update>		:Ver REF02A-0.00 2002-06-15
				;Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				;Ver REF045-3.00 2004-07-13 -Outi TOLS置換bug修正
				;Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				;Ver REF058-0.00 2005-07-05 -Outi サンケンＩＰＭ対応
				;Ver REF061-0.00 2006-04-27 -Outi IPM論理判定処理追加
				;Ver REF061-0.01 2006-06-01 -Tomo IPM機種別判定二重チェック削除
				:Ver REF070-0.01 2007-04-16 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-04-22 -Outi 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.01 2008-05-30 -Outi 保護処理流用率up
				:Ver REF081-0.02 2008-06-28 -Outi ﾙﾈｻｽ指摘0627対応(機能向上) @005awORM[08/06/27]
				:Ver REF090-0.00 2009-05-11 -Outi IEC対応 @006axOOM[09/05/11]
				:Ver REF090-0.01 2009-05-13 -Outi POR実装 @010axOOM[09/05/13]
				:Ver REF1010-0.01 2010-11-01 -Mura 三菱IPM基板用IO対応 @013MwDOD[10/11/01]
				:Ver REF1010-0.02 2010-12-13 -Mura メイン制御テスト用処理 @039MwDOD[10/12/13]
				:18/9/14 ロック保護継続時にリセット発生させるよう変更(3GT不冷対応) 吉田@101ywOTT[18/09/14]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/


/********************************************
		インクルードファイル
 ********************************************/

#include "Softr.h"
#include "PMD1GPar.h"
#include "Trans_Inv.h"
#include "Speedcnt.h"
#include "Timea.h"
#include "Hogo.h"
#include "Com_def.h"
#include "RefSpeed.h"
#include "stm32g0xx_it.h"
#include "stm32g030xx.h"
#include "stm32g0xx_hal.h"
#include "main.h"
#include "motor_process.h"
#include "motor_error.h"

/*******************************************
		マクロ定義
 *******************************************/
#define LOCKRESETNUM	2	/* ウォッチドッグリセット発生させるロック保護継続回数[回]			*/		/* @101ywOTT[18/09/14] */
#define LOCKRESETDELAY	30	/* ロック継続でウォッチドッグリセット発生させる際の遅延時間[*6s]	*/	/* @101ywOTT[18/09/14] */


/*******************************************
		変数宣言
 *******************************************/


/*******************************************
		プロトタイプ宣言
 *******************************************/


/**************************************************************************
<Title>			:vMAIN_Softr 		;main
<Function>		:ｿﾌﾄﾘｾｯﾄ処理
<Description>	:機能ﾎﾟｰﾄ等のﾃﾞｰﾀﾚｼﾞｽﾀ＆制御ﾚｼﾞｽﾀをﾘﾌﾚｯｼｭする｡
				: ﾏｲｺﾝ暴走･多重割込を検出しﾏｲｺﾝｿﾌﾄﾘｾｯﾄ用ﾌﾗｸﾞをSETする｡
<Input>			:ucSPD_sfrqtp_b, ucTRNS_ifrq_b, ucSPD_rpmfgmd, ucSPD_rpmfgmd, uc6Stmr_wdr15ct,
				:ePMD_drvmode, uc100MS_pumpct, ucMINtm_H50ct, ucMINtm_HOGOct, ucg_HOGO_lockct, 
				:uc6Stmr_hgrlsct, ucMAIN_chkint, uc10MSt_intct
<Return>		:ucMAIN_wdrfg
<Author>		:-Outi	作成
<Date>			:2002-0219
<Update>		:Ver REF02A-0.00 2002-06-15
				;Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				;Ver REF045-3.00 2004-07-13 -Outi TOLS置換bug修正
				;Ver REF061-0.00 2006-04-27 -Outi IPM論理判定処理追加
				;Ver REF061-0.01 2006-06-01 -Tomo IPM機種別判定二重チェック削除
				:Ver REF070-0.04 2007-04-22 -Outi 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.01 2008-05-30 -Outi 保護処理流用率up
				:Ver REF090-0.00 2009-05-11 -Outi IEC対応 @006axOOM[09/05/11]
				:Ver REF090-0.01 2009-05-13 -Outi POR実装 @010axOOM[09/05/13]
				:180125 F604WPX ｲﾝﾊﾞｰﾀ保護継続時にﾏｲｺﾝﾘｾｯﾄする仕様を追加(修正) 吉田
				:230615 F65WX1 割込み禁止時間短縮変更 瀧田 @254swDOD[23/06/15]
				:HTAで正弦波駆動導入に伴いリセット処理見直し 2026/08/03 Takida
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void vMAIN_Softr(void){							/*ｿﾌﾄﾘｾｯﾄ処理 */
/*------------------------------------------------
	入出力ﾓｰﾄﾞ ｾｯﾄ
------------------------------------------------*/
	vMAIN_GPIO_Reinit();						/* GPIOポート入出力方向 PP/PD 再設定処理 */
/*------------------------------------------------
	インバータマイコン・外部強制リセット
------------------------------------------------*/
	if(IsResetByHogo5minStop() == SET)	        /*保護停止による5min待ちが3回以上発生なら */
	{                                           /*保護が3回以上(≒保護停止による強制停止が合計15分以上)の場合は各ポート設定異常発生の可能性を無くすためにもリセット */
		ucMAIN_wdrfg = 1;					    /*ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄﾌﾗｸﾞ←ｾｯﾄ */
	}else{										/*保護停止による5min待ちが3回未満の場合 */
		ucMAIN_wdrfg = 0;						/*ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄﾌﾗｸﾞ←ｸﾘｱ */
	}											/* */
    /*断続運転時リセット用処理*/
	if(ucMINtm_H50ct >= MTR_CTRL_OFFLINE_TOTAL_MINUTE){		/*H50断続運転1周期継続で */											/* @006axOOM[09/05/11] */
		ucMAIN_wdrfg = 1;						/*ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄﾌﾗｸﾞ←ｾｯﾄ */											/* @006axOOM[09/05/11] */
	}else{}
/*------------------------------------------------
	外部割込みチェック(メインループのタイマ監視)
------------------------------------------------*/
	if ( ucMAIN_chkint >= 0xFF ){				/*割り込みｶｳﾝﾀ >= 255回 ﾁｪｯｸ */	/* 午後 04:39 REF045-4.00 */
		ucMAIN_wdrfg = 1;						/*ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄﾌﾗｸﾞ←ｾｯﾄ */
	}											/* 午後 04:39 REF045-4.00 */										/* @006axOOM[09/05/11] */
	else if ( uc10MSt_intct >= 0xFF )			/*ﾒｲﾝﾙｰﾌﾟ確定 */													/* @006axOOM[09/05/11] */
	{
		ucMAIN_chkint ++;						/*割り込みﾁｪｯｸｶｳﾝﾀ ++ */											/* @006axOOM[09/05/11] */
	}else{}										/*ﾒｲﾝﾙｰﾌﾟ未確定時⇒未処理 */										/* @006axOOM[09/05/11] */
}												/* */


/**************************************************************************
<Function>		:GPIOポート入出力方向 PP/PD 再設定処理
<Author>		:瀧田
<Date>			:2022/02/07
<Update>		:HTシリーズ用に更新。ヒータ出力ポートのみ定期リフレッシュする。 2026/08/03 Takida
 **************************************************************************/
void vMAIN_GPIO_Reinit(void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* ========== ポート入出力方向 PP/PD 設定 ==========> */
  /*Configure GPIO pins : HEAT1_Pin */
  GPIO_InitStruct.Pin = HEAT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 mDisableInterrupt();						/* 割り込み禁止 */
  HAL_GPIO_Init(HEAT1_GPIO_Port, &GPIO_InitStruct);
 mEnableInterrupt();						/* 割り込み許可 */

  /*Configure GPIO pins : HEAT2_Pin */
  GPIO_InitStruct.Pin = HEAT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 mDisableInterrupt();						/* 割り込み禁止 */
  HAL_GPIO_Init(HEAT2_GPIO_Port, &GPIO_InitStruct);
 mEnableInterrupt();						/* 割り込み許可 */

/* <========== ポート入出力方向 PP/PD 設定 ========== */
}
