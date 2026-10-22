#ifndef VI_PHASE_DIFFERENTIAL
#define VI_PHASE_DIFFERENTIAL

#include "Sys_type.h"
#include "vipdc.h"

/*******************************************
		定数定義
 *******************************************/
//from midware
/* 0クロス検出方法 */
#define DTCT_OFF	(0)													/* 0クロス検出しない */
#define DTCT_SPK	(1)													/* スパイク検出 */
#define DTCT_CT		(2)													/* CT検出 */
#define DTCT_CRNT	(3)													/* 現在設定されている0クロス検出方法参照用 2008-10-10 竹岡 chg 0 -> 3*/

/* 0クロス検出状態 */
#define ZX_WAIT		(0)													/* 0クロス検出待ち */
#define ZX_DTCT		(1)													/* 0クロス検出済み */
#define ZX_STOP		(2)													/* 0クロス検出禁止 */

#define DPTN_UON_LAP	(1)												/* U相 ラップ通電 開始 uvw xyz = 101 010 */
#define DPTN_UON		(2)												/* U相 120deg通電 開始 uvw xyz = 100 010 */
#define DPTN_UOFF		(5)												/* U相 120deg通電 終了 uvw xyz = 110 001 */
#define DPTN_UOFF_LAP	(6)												/* U相 ラップ通電 終了 uvw xyz = 010 001 */
#define DPTN_XON_LAP	(7)												/* X相 ラップ通電 開始 uvw xyz = 010 101 */
#define DPTN_XON		(8)												/* X相 120deg通電 開始 uvw xyz = 010 100 */
#define DPTN_XOFF		(11)											/* X相 120deg通電 終了 uvw xyz = 001 110 */
#define DPTN_XOFF_LAP	(0)												/* X相 ラップ通電 終了 uvw xyz = 001 010 */

/* 端子電圧状態 */
#define TVLT_CLR		(0x00)											/* 端子電圧状態クリア用 */
#define TVLT_XOFF_SPK	(0x01)											/* X相OFF後のスパイク立下りを検出
																			  ┌↓  ┌─────┐┌─┐ X 
																			─┘└─┘ U        └┘  └───── */
#define TVLT_UOFF_EDG	(0x02)											/* U相OFFと同期した立下りを検出
																			  ┌┐  ┌─────↓┌─┐ X
																			─┘└─┘ U        └┘  └───── */
#define TVLT_UOFF_PDU	(0x04)											/* U相OFF後の誘起電圧0クロス検出
																			  ┌┐  ┌─────┐┌↓┐ X
																			─┘└─┘ U        └┘└└───── */
#define TVLT_XON_EDG	(0x08)											/* X相ONと同期した立下り検出
																			  ┌┐  ┌─────┐┌─↓ X
																			─┘└─┘ U        └┘  └───── */
#define TVLT_STOP_SPK	(0x80)											/* スパイク検出禁止 */

/* 補正処理 */
#define ZX_CORRECT		(12)											/* 補正量の大きさ (位相差)/ZX_CORRECT */


#define ZX_OFS_CNTNUM			(251)									/* オフセット取得回数 */
#define ZX_OFS_CNTEND			(ZX_OFS_CNTNUM + 1)						/* オフセット取得終了 */
#define ZX_OFS_CNTOLD			(ZX_OFS_CNTEND + 1)						/* オフセット取得後に駆動実績有り */

#define ZX_UNERECORD	(0x8000)										/* ウネリ発生履歴フラグ */
#define ZX_UNEFLGMASK	(0x7fff)										/* フラグ部分をマスクする */
#define ZX_UNECNT		(UI16)(1000/3.3)								/* カウント回数 */

#define D_UNERI_THRESHOLD 166 /* = 300[mA] / 1.8117[mA/LSB] *//* @263swDOD[23/07/26] */


/*******************************************
		プロトタイプ宣言
 *******************************************/
void ClearVIPhaseDifferentialControl(void);
void VIPhaseDifferentialControl(UI08 rpmh);
void InitVIPhaseDifferentialControl(void);
void InitializeVIPDCDrive(void);
void Shift2Running(void);
void Shift2Vector(void);

UI08 SpdLimitViPhaseDiffCtrl(UI08 target_speed_rps);

void ClearAdCHG(void);
void vZX_initproc(void);
void vZX_Inttc1pros(void);												/* zx用定期処理 */
void SetCommutateVector(void);
UI08 NeedShift2Vector(void);
UI08 IsVectorNormal(void);

/*******************************************
		変数宣言
 *******************************************/
extern UI16 unPMDi_CMP1dumy;

extern UI16 unZX_uneri;					/* ウネリフラグ(b15)/ウネリカウンタ(b0-b14)					*/
extern UI08 ucZX_ofs_cnt;				/* オフセット取得時の回数カウント用 */
extern SI16 nZX_ofs;					/* 2008-03-07 竹岡 U相電流オフセット 						*/
extern UI08 ucZX_dcf;										/* 制御方法を記憶 											*/
//★extern UI08 ucZX_kp_deftm;				//Pmd_intで1回のみ参照					/* 位相差確保時間 											*/
extern UI16 unZX_leadtm;				//Pmd_intで1回のみ参照					/* 進角目標時間 											*/
//★extern UI08 ucZX_ctpkidx;									/* ctpk1ec指定用 											*/
//★extern UI16 unZX_ctpk1ec[3];								/* 1電気ｻｲｸﾙの中の最大電流値を格納  3(=極数/2)回分 = 1機械角 *///配列数に本来はD_POLEを利用したい
/* extern UI08 ucZX_CTerrorFlgs; */								/* CT回路異常判断FLG群 										*/	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
													/* 	bit0:異常FLG(0:起動時正常/1:回路異常) 				*/
													/*	bit1:異常FLG(0:正常回転運転中/1:成行き回転運転中) 	*/
													/* 	bit2:異常FLG(0:vct時正常/1:回路異常) 					*/
/* extern UI08 ucZX_CTjudgeCnt1; */								/* CT回路異常確定用ｶｳﾝﾀ1(異常FLG10回計数用ｶｳﾝﾀ) 			*//* H53判定用ｶｳﾝﾀ *//* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
enum tVctrModeTag {kNoVector, kReadyVector, kDriveVector, kOverMuchVector};//{kNoVector = 0, kReadyVector = 1, kDriveVector = 4, kOverMuchVector = 5};
extern enum tVctrModeTag ucPMD_VctrMode;

extern UI16 unZX_lcnt;						/* 連続進み回数カウント用 *//* @263swDOD[23/07/26] */
extern UI16 unZX_dcnt;						/* 連続遅れ回数カウント用 *//* @263swDOD[23/07/26] */
extern SI16 nVIC_max_cur;
extern SI16 nVIC_now_cur;
extern SI16 nVIC_hold_cur;
extern UI08 is_skip_check_spike;			/* ｽﾊﾟｲｸ確認ｽｷｯﾌﾟﾌﾗｸﾞ */	/* @263swDOD[23/07/26] */

/**************************************************************************
<Title>			:NeedShift2Running
<Function>		:（★機能内容を記載のこと★）
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright>		:Panasonic Co. , Ltd.
 **************************************************************************/
static inline UI08 NeedShift2Running(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	return ( ucPMD_VctrMode == kOverMuchVector );
}
/**************************************************************************
<Title>			:UpdateMaxCurrent
<Function>		:電流最大値を更新する
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright>		:Panasonic Co. , Ltd.
 **************************************************************************/
static inline void UpdateMaxCurrent(SI16 current){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    nVIC_now_cur = current;
    if(nVIC_max_cur < nVIC_now_cur){
        nVIC_max_cur = nVIC_now_cur;
    }else{
    	/* 何もしない */
    }
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
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
static inline void UpdateHoldCurrent(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    nVIC_hold_cur = nVIC_max_cur; /* 最大の判定の最終タイミングなのでここで保持電流を更新 */
    nVIC_max_cur = 0;               /* 電流の最大値判定の開始位置のため0クリア */
    StoreCurrentBuf(nVIC_hold_cur);	/* 電流検出値の保存処理 *//* @304swDOD[24/09/02] */
}
#endif
