/*
 * RefSpeed.h
 *
 *  Created on: 2020/10/20
 *      Author: 2993017
 */

#ifndef REFSPEED_H_
#define REFSPEED_H_
#include "vipdc.h"
#include "Sys_type.h"

/*******************************************
		マクロ定義
 *******************************************/
#define  D_R2  		35						/* 35rpsﾒｲﾝﾏｲｺﾝが指令する３段階目の回転数 */	/*＠~06-0413 -Outi 2ﾚﾍﾞﾙﾘﾚｰ廃止対応(R1回転数廃止)＃午後 09:51 REF061-0.xx*/
#ifdef UNIT_TEST
#define  D_R3_0		42						/* 42rpsﾒｲﾝﾏｲｺﾝが指令する３段階目の回転数 */	/* Dﾗｲﾝ共振ポイント：44～47r/s */
#define  D_R3_1		43						/* 44rpsﾒｲﾝﾏｲｺﾝが指令する３段階目の回転数 */	/* Dﾗｲﾝ共振ポイント：45～49r/s */
#else
#define  D_R3_0		42						/* 42rpsﾒｲﾝﾏｲｺﾝが指令する３段階目の回転数 */	/* Dﾗｲﾝ共振ポイント：44～47r/s */
#define  D_R3_1		42						/* 44rpsﾒｲﾝﾏｲｺﾝが指令する３段階目の回転数 */	/* Dﾗｲﾝ共振ポイント：45～49r/s */
#endif
#define  D_R4  		58						/* 58rpsﾒｲﾝﾏｲｺﾝが指令する４段階目の回転数 */	/*~04-0612 -Outi  午後 09:29 REF045-2.00 */
#define  D_R5  		69						/* 69rpsﾒｲﾝﾏｲｺﾝが指令する５段階目の回転数 */	/*~04-0612 -Outi  午後 09:29 REF045-2.00 */
#define  D_R6  		80						/* 80rpsﾒｲﾝﾏｲｺﾝが指令する６段階目の回転数 */	/*~04-0612 -Outi  午後 09:29 REF045-2.00 */

/* FOC speed LPF: y = (LPF_PREV_COEF * y + x) / LPF_DIVISOR */
#define REFSPD_LPF_PREV_COEF   (3u)
#define REFSPD_LPF_DIVISOR     (4u)
#define REFSPD_LPF_INIT        (1u)

#define SPD_BCD_DECIMAL_BASE   16
#define SPD_DECIMAL_BASE	   10
/*******************************************
		プロトタイプ宣言
 *******************************************/
/*void ClearRefSpeed(void);*/
UI08 IsSpeedDrive(void);
void vSPD_Setrpm(void);
unsigned8 vSPD_Bcd2nrm(unsigned8 ucBcd);					/*BCD⇒標準表示変換 */
UI08 HasStopOffset(void);

/*******************************************
		変数宣言
 *******************************************/
extern UI08 ucTYPE_Dline;
extern UI08 ucTYPE_Dline_point;
extern unsigned8	ucSPD_sfrqtp_b;				/*指令Hz(格納値)<BCD> */
extern unsigned8	ucSPD_sfrqfx;				/*指令Hz(確定値)<dec> */
extern UI08 ucSPD_rpmh;

extern void SelectStopPosition(void);
extern unsigned16 CalculateAndGetRpsFOC(void);

#endif /* REFSPEED_H_ */
