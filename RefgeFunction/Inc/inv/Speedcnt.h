/**************************************************************************
<FileName>		:SPEEDCNT.h
<Function>		:SPEEDCNT.c用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0323
<Update>		:Ver REF02A-0.00 2002-05-11
				:Ver REF045-4.02 2004-08-04 -Outi 関数宣言の適正化(void型への変更)
				:Ver REF052-1.00 2005-06-15 -Outi 起動時回転加速度UP変更
				:Ver REF070-0.01 2007-04-09 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-04-22 -Outi 機種切り替え対応
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/
#ifndef SPEEDCNT_H
#define SPEEDCNT_H

/********************************************
		インクルードファイル
 ********************************************/
#include	"Sys_type.h"

/*******************************************
		マクロ定義
 *******************************************/
#define  D_DLNHGLOLMT_1	450		/*共振保護制御用ｶｳﾝﾀ加算許可回転数（下限）	*/	/*＠~07-0422 -Outi 機種切り替え対応*/
#define  D_DLNHGHILMT_1	490		/*共振保護制御用ｶｳﾝﾀ加算許可回転数（上限）	*/
#define  D_DLNHGLOLMT_0	440		/*共振保護制御用ｶｳﾝﾀ加算許可回転数（下限）	*/
#define  D_DLNHGHILMT_0	470		/*共振保護制御用ｶｳﾝﾀ加算許可回転数（上限）	*/	/*＃ 午前 11:58 REF070-0.04 */

/*******************************************
		変数宣言
 *******************************************/
extern struct tSpeedDataTag speed;
extern UI16	hCurMecRps;				/*現行Hz(少数第１位/16bit) */
extern UI08	ucSPD_rpmfgmd;				/*回転数制御用ﾌﾗｸﾞ＆ﾓｰﾄﾞ設定RAM */
extern UI08	ucTYPE_Dline;				/* Dﾗｲﾝ共振保護制御	*/
extern UI08	ucTYPE_Dline_point;			/* Dﾗｲﾝ共振ポイント	*/

/*******************************************
		プロトタイプ宣言
 *******************************************/
void vSPD_Ctrlspd(void);								/*回転速度制御 */
void InitPumpampRate(UI08 prd, UI08 duty_up);
UI08 IsNowSpeedTarget(void);
UI08 IsNowSpeedOverTarget(void);
UI08 SpdLimitDline(UI08 target_speed_rps);									/* D_R3_0、D_R3_1を使用 モジュールをPam_mode.cに移動すれば別定義は解消 */
unsigned16 unPMDc_getspd(void);			/*実回転周期（60ﾟe）取得 */
UI08 GetSpeedDifferentialMode(void);
UI08 GetStandardSpeed(void);
#endif
