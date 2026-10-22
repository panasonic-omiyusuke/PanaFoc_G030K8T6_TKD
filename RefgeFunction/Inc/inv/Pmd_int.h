/**************************************************************************
<FileName>		:PMD_INT.h
<Function>		:PMD_INT.c用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0227
<Update>		:Ver REF02A-0.00 2002-06-03
				:Ver REF045-2.09 2004-06-29 -Outi 回転数ﾀﾞｳﾝ仕様廃止
				:Ver REF04C-0.00 2004-09-07 -Outi ﾗｯﾌﾟ転流仕様見直し
				:Ver REF04C-0.01 2004-01-23 -Outi VCT駆動中の通電角調整仕様の変更
				:Ver REF070-0.01 2007-04-09 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-13 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.03 2008-05-30 -Outi 保護処理流用率up
				:Ver REF081-0.04 2008-06-05 -Outi 信号検出処理修正	@000ahOTM[08/06/05](発覚08/05/21)
				:Ver REF1000-0.01 2010-04-01 -Mura 2Level対応	@000MwDOD[10/04/01]
				:Ver REF1000-0.02 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/05]
				:Ver REF1000-0.03 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/22]
				:Ver REF1000-0.04 2010-05-25 -Mura 重複テーブル削除(シュリンク対応）@005MwHOM[10/05/25]
				:Ver REF1000-0.05 2010-05-25 -Mura 停止時振動対策インプリ  @006MwHOM[10/05/25]
				:Ver REF1000-0.06 2010-06-11 -Mura 停止時振動対策 メイン通信処理追加  @008MwHOM[10/06/11]
				:Ver REF1000-0.07 2010-06-22 -Mura 停止オフセット受取りタイミング修正  @012MzORT[10/06/22]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/
#ifndef PMD_INT_H
#define PMD_INT_H


/********************************************
		インクルードファイル
 ********************************************/
#include "Sys_type.h"
#include "PMD1GPar.h"
#include "angle_control.h"

/*******************************************
		マクロ定義
 *******************************************/
#define DTCT_NUM	(3)
//#define WEIGHTING_BIT_SHIFT (1) //速度のLPFの現在と過去分の重みづけのためのビットシフト数
//#define D_INSTANTint	(15)				/*即TRC割込発生用CLK数=10*/  							/* @008MzDOD[16/12/21] */
//#define MAX_PERIOD (0xffff - D_MTBUFSZ / 2) //転流の最大周期
//#define TRCGRA_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRBの設定時の最小値 オーバフロー対策は未実施 */
//#define TRCGRB_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRBの設定時の最小値 オーバフロー対策は未実施 */
//#define TRCGRC_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRCの設定時の最小値 オーバフロー対策は未実施 */
//#define TRCGRD_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRDの設定時の最小値 オーバフロー対策は未実施 */

/*******************************************
		変数宣言
 *******************************************/
extern UI08 ucZX_ptn;					/* 通電パタン把握用 */
extern UI08	ucPMDi_run2stpfg;			/* ﾋﾟｽﾄﾝ位置固定停止準備完了Fg */	/* @006MwHOM[10/05/25] */

extern UI32 unPMD_MDOUT;
extern UI32 ulPMD_CCMR1;
extern UI32 ulPMD_CCMR2;
extern UI16 cspk_CMP2_2nd;				/* CMP2、1転流中に2回目設定用。スパイク検出タイムアウト用。コンペアが多ければ同じコンペアを上書きする必要なし */
extern UI16 unPMD_CMP1;							/*TmRC転流時機格納変数 */
extern UI16 unPMD_CMP1_lap;                         /*TmRC転流時機格納変数 */
extern UI16 unPMD_CMP3;							/*TmRCpd終時機格納変数 */
extern UI16 unPMD_CMP2;							/*TmRCpd始時機格納変数 */

extern unsigned8    ucPMD_pdct;                 /* 位置検出回数カウンタ [Forced 状態]: {D_FCDPDCT  --  0} */
extern const UI32 T_MDOUTptn[8];			/* uy	   uz	   vz	   vx	   wx	   wy	*/
extern const UI32 T_MDOUTlap[8];
extern const UI32 T_MDOUTgap[8];
extern UI08 ucPMD_mdidx;

//from midware
extern UI16 unZX_cap[DTCT_NUM];							/* 2 でOK CTとスパイクのみ 0は実際には位相差確保用に利用 	*///実態はPmd_midware.cにあるが、Pmd_int.cでのみ利用 実態を移動した方が良い
extern UI16	unPMD_MCAP;					/* pd間隔格納変数 */
extern UI08 renzokuhassei;				/* Spike連続検出不可回数									*/



/*******************************************
		プロトタイプ宣言
 *******************************************/
//unsigned16 unPMD_GetMCAPdumy(void);
void INTPDC(void);
void vPMD_PdcPtnSet(void);												/*信号検期待値設定*/
void vZX_chkspk(void);
//void UpdateTrcgra(UI16 mode_time);
void INTTMR(void);
void INTSAGV(void);
void INTEMG(void);
void INTOVRV(void);

#endif
