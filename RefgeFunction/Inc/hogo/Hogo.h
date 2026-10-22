#ifndef HOGO_H
#define HOGO_H
/**************************************************************************
<FileName>		:HOGO.h
<Function>		:HOGO.c用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0510
<Update>		:Ver REF02A-0.00 2002-05-24
				:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				:Ver REF061-0.01 2006-04-27 -Outi IPM論理判定処理追加
				:Ver REF070-0.00 2007-01-09 -Outi Ver.4試作基板対応
				:Ver REF070-0.01 2007-04-09 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF1000-0.01 2010-01-20 -Mura 2Level対応	@000MwDOD[10/01/20]
				:Ver REF1000-0.02 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/05]
				:Ver REF1000-0.03 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/22]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/

/********************************************
		インクルードファイル
 ********************************************/
#include "Pmd_int.h"
#include "system_ram.h"
#ifdef UNIT_TEST
#define INLNE
#else
#define INLIN inline
#endif
/*******************************************
		マクロ定義
 *******************************************/
/*from Hogo.h start*/
#define  D_DLNHGTM		250		/*共振保護制御動作時間（250）	*/
#define  D_DLNRLSTM		40		/*共振保護制御解除時間（ 40）	*/
/*from Hogo.h end*/
#define SDFAILCNT	 (100)												/* 駆動停止するスパイク検出の連続失敗回数 */
#define  D_HOGSHT1	28						/* 直流電圧低下保護保護ﾚﾍﾞﾙ(全電圧) 80V */
#define  D_RLSSHT1	33						/* 直流電圧低下保護解除ﾚﾍﾞﾙ(全電圧) 90V */
#define  D_HOGSHT2	54						/* 直流電圧低下保護保護ﾚﾍﾞﾙ(倍電圧)160V(57)→148V(54) ※(BX468/IPD/43℃/AC113V)で不足電圧保護発生を確認*/ /* @002MzDOD[16/06/20] */
#define  D_RLSSHT2	62						/* 直流電圧低下保護解除ﾚﾍﾞﾙ(倍電圧)180V(66)→168V(62) */													 /* @002MzDOD[16/06/20] */
#define D_HOGO_CLR_TIME	(50) 				/* 50 = (5min = 300s) / 6s */
#define  D_HOG2LHG	73						/* ２ﾚﾍﾞﾙ電圧異常保護ﾚﾍﾞﾙ(全電圧)  200V */
#define  D_RLS2LHG	65						/* ２ﾚﾍﾞﾙ電圧異常解除ﾚﾍﾞﾙ(全電圧)  180V */

#define FO_ACTIVE			!(FO_POLALITY^SET)/*POLALITY=LoActive(0)ならLo(0)入力 POLALITY=HiActive(1)ならHi(1)入力 *//* @008lwORM[24/06/21] */
#define FO_INACTIVE			!(FO_POLALITY^CLR)/*POLALITY=LoActive(0)ならHi(1)入力 POLALITY=HiActive(1)ならLo(0)入力 *//* @008lwORM[24/06/21] */


/*vHOG_Wirehg_FOC*/
#define POWER_PROTECTWIRE_THRESHOLD   300   /* 保護ON（W） */
/********************************************/
/*		型宣言  							*/
/********************************************/
enum eShortVltType {kShortVltAdLo, kShortVltAdHi, kShortVltSkip,kShortVltInt};
enum ePamProtectModeTag {kDisablePamProtect, kEnablePamProtect};

/*******************************************
		変数宣言
 *******************************************/
extern UI08	ucHOG_movF;					/*保護動作格納ﾒﾓﾘ */
extern UI08	ucHOG_sttF;					/*保護状態格納ﾒﾓﾘ */
extern enum eShortVltType uc2LV_scantyvlt_judge;	/*for inline function*/ /*直流電圧低下検知手段切替ﾌﾗｸﾞ*/	/*@001MwDOD[10/04/05]*/
extern unsigned8	uc2LV_scantyvlt_skip;					/*直流電圧低下保護Skipﾌﾗｸﾞ*/ 		/*@001MwDOD[10/04/22]*/
extern unsigned8	ucMINtm_HOGOct;				/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間の計測ｶｳﾝﾀ */	/* @021ywDOD[18/01/25] */
extern unsigned8	ucg_HOGO_lockct;			/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀﾛｯｸ保護(FO信号含む)継続回数ｶｳﾝﾀ */	/* @101ywOTT[18/09/14] */
extern unsigned16	current_wire;
extern void CheckDline(void);

/*from midware*/
extern UI08 ucTYPE_undvlt_int;			/*for main*/ /*直流電圧低下保護仕様	*/

static inline UI08 IsFailPeriodTooShort(UI16 commutate_period){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    if( (commutate_period < 300) && (ePMD_drvmode != D_vector) ){							/* ロック検知レベル以下（180Hz以上)なら */	/*検知レベル変更(2003/01/20-hide) */
        return 1;												/* */
    }														/* */
	return 0;
}

static inline UI08 IsFailVector2Running(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	extern UI08 ucTC1tm_VctrHogoCtr;
	if(ucTC1tm_VctrHogoCtr >= 255){
		return 1;
	}
	return 0;
}

static inline UI08 IsFailDetctSpikeEnd(UI08 series_fail_cnt){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    if( series_fail_cnt >= SDFAILCNT ){				/* 連続検出不可回数が所定値以上だったら */
        return 1;
    }
    return 0;
}



static inline UI08 IsFailShortVltInt(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    if((uc2LV_scantyvlt_judge == kShortVltInt) && (uc2LV_scantyvlt_skip ==0)){		/* [直流電圧低下保護(外部割込み仕様)]∧[不足電圧保護Skip無]なら */ /*5V処理INTHOGOでの分岐処理を追加*/	/* @023MzDOD[17/03/06] */
        return 1;
    }
    return 0;
}
/*******************************************
		プロトタイプ宣言
 *******************************************/
void vHOG_Shtvlt_judge(void);				/*直流電圧低下検知手段切替処理 */		/* @001MwDOD[10/04/05]*/
UI08 IsForcedLockError(UI16 duty);
void vHOG_Hogo(void);								/* 保護制御ﾒｲﾝﾙｰﾁﾝ処理 */
void ErrMotorLock(void);
void ErrTimeOut(void);
void ErrShortVlt(void);
UI08 SetSpeedLimit(UI08 ucbuf);
void ClearErrorFlags(void);
void vPMD_Brake(void);				/* ブレーキ制御 */
void DisarmPwmTimerBreak(void);
extern void vHOG_Wirehg(void);
extern void vHOG_5minwt(void);
#ifdef UNIT_TEST
void ErrMotorLock(void);
void ErrTimeOut(void);
void ErrShortVlt(void);
void vHOG_Shtvlt(void);
void vHOG_Shtvlt_int(void);					/*直流電圧低下保護  */
void vHOG_Ovrvlt_int(void);						/* 過電圧保護 */
void vHOG_Pamhogo(void);						/*2ﾚﾍﾞﾙ保護制御	01-0606 -outi 変更 */
void vHOG_5minwt(void);
#endif
#endif
