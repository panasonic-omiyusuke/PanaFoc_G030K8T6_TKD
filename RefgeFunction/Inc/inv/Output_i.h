/**************************************************************************
<File Name>		:OUTPUT_I.h
<Function>		:OUTPUT_I.c用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0128
<Update>		:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				:Ver REF052-1.00 2005-06-26 -Outi ｳﾈﾘ状態信号送信仕様追加
				:Ver REF070-0.00 2007-04-22 -Outi 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF1000-0.01 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/22]
				:Ver REF1000-0.02 2010-05-25 -Mura 重複テーブル削除(シュリンク対応）@005MwHOM[10/05/25]
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
#ifndef OUTPUT_I_H
#define OUTPUT_I_H

/********************************************
		インクルードファイル
 ********************************************/

/*******************************************
		マクロ定義
 *******************************************/
/* GetRoundedMecSpeed() 用定数（0.1rps -> rps 四捨五入） */
#define RPS_X10_ROUND_OFFSET     (5u)
#define RPS_X10_DIVISOR          (10u)
/*******************************************
		変数宣言
 *******************************************/
extern UI08 sfrq_d;	/*ﾃﾝﾎﾟﾗﾘ←現行Hz少数１位以下四捨五入 */
extern unsigned8 bCurMecSpeedRps;
/*******************************************
		プロトタイプ宣言
 *******************************************/
void vTRNS_Outputi(void);
extern unsigned8 vSPD_Bcd2nrm(unsigned8 ucBcd);

#endif /* #ifndef OUTPUT_I_H */

