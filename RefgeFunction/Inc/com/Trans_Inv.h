/**************************************************************************
<FileName>		:Hw_trns.h
<Function>		:Hw_trns.c用ヘッダファイル(型定義／関数定義／パラメータ定義／SFR定義／EXTERN)
<Program>		:
<Author>		:甲田 篤志 -AK
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
				:Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				:Ver REF070-0.01 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-25 -Tomo ソフト設計ＤＲ指摘対応
				:Ver REF1000-0.01 2010-05-25 -Mura 停止時振動対策 メイン通信処理追加  @008MwHOM[10/06/11]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/
#ifndef HWTRNS_H
#define HWTRNS_H


/********************************************
		インクルードファイル
 ********************************************/
#include "Sys_type.h"
#include "Sys_type_main.h"			/* 宣言箇所最適化 *//* @260swDOD[23/06/30] */
/*******************************************
		マクロ定義
 *******************************************/

/*送信ﾊﾞｯﾌｧ数 */
#if  (defined(NOTCHTEST) || defined(STARTUP_TEST))
#define cTDBAFAC (1+1+16+2)	/* Labviewでの評価用*/
#else
#define SEND_DATA_NO	18							/* 送信データバイト数 */	/* @104yxDOD[18/10/03] */
#define	cTDBAFAC		( 2 + SEND_DATA_NO + 2 )	/* 通常 */					/* @104yxDOD[18/10/03] */
#endif

/*受信ﾊﾞｯﾌｧ数 */
#define cRDBAFAC 31

/* ------------------ Renesas ------------------> */
#define cBRG    0x01				/* 転送クロック選択 */
#define cTEND   0x08				/* 送信終了フラグ */

#define cPE     0x40				/* パリティ付加 */
#define cEVEN   0x20				/* 偶数パリティ */
#define cSTBT   0x10				/* 送信ストップビット長 */

#define ucCPU_UARTSR ucCPU_U1C1		/*ＵＡＲＴステータスレジスタ */	/* @001MzDOD[13/03/15] */
#define cTXE    0x01				/* 送信動作 */
#define cTBEP   0x02				/* 送信バッファエンプティフラグ */
#define cRXE    0x04				/* 受信動作 */
#define cRBFL   0x08				/* 受信バッファフルフラグ */
#define unCPU_RDBUF ucCPU_U1RB		/*ＵＡＲＴ受信データバッファ */	/* @001MzDOD[13/03/15] */
#define cOERR   0x1000				/* オーバーランエラーフラグ */
#define cFERR   0x2000				/* フレーミングエラーフラグ */
#define cPERR   0x4000				/* パリティエラーフラグ */
#define cESUM   0x8000				/* パリティエラーフラグ */

#define ucCPU_RDBUF ucCPU_U1RBL		/*ＵＡＲＴ受信データバッファ */	/* @001MzDOD[13/03/15] */
#define ucCPU_TDBUF ucCPU_U1TBL		/*ＵＡＲＴ送信データバッファ */	/* @001MzDOD[13/03/15] */

#define cSendUART (cSTBT | cEVEN | cPE | 0x05)
#define cRcveUART (/*cRXE  |*/ 0x1a)
#define cRcvUART2 (cEVEN | cPE | 0x05)
#define cRcvUART3 (cBRG | 0x28)
#define cErrUART  (cOERR | cFERR | cPERR)
/* <------------------ Renesas ------------------ */

/* ------------------ STM32G0 ------------------> */
#define cBRG    0x01				/* 転送クロック選択 */
#define cTEND   0x08				/* 送信終了フラグ */

#define cPE     0x40				/* パリティ付加 */
#define cEVEN   0x20				/* 偶数パリティ */
#define cSTBT   0x10				/* 送信ストップビット長 */

#define ucCPU_UARTSR ucCPU_U1C1		/*ＵＡＲＴステータスレジスタ */	/* @001MzDOD[13/03/15] */
#define cTXE    0x01				/* 送信動作 */
#define cTBEP   0x02				/* 送信バッファエンプティフラグ */
#define cRXE    0x04				/* 受信動作 */
#define cRBFL   0x08				/* 受信バッファフルフラグ */
#define unCPU_RDBUF ucCPU_U1RB		/*ＵＡＲＴ受信データバッファ */	/* @001MzDOD[13/03/15] */
#define cOERR   0x1000				/* オーバーランエラーフラグ */
#define cFERR   0x2000				/* フレーミングエラーフラグ */
#define cPERR   0x4000				/* パリティエラーフラグ */
#define cESUM   0x8000				/* パリティエラーフラグ */

//#define ucCPU_RDBUF ucCPU_U1RBL		/*ＵＡＲＴ受信データバッファ */	/* @001MzDOD[13/03/15] */	/*STM32では不要*/
//#define ucCPU_TDBUF ucCPU_U1TBL		/*ＵＡＲＴ送信データバッファ */	/* @001MzDOD[13/03/15] */	/*STM32では不要*/

//#define cSendUART (cSTBT | cEVEN | cPE | 0x05)	/*STM32では不要*/
//#define cRcveUART (/*cRXE  |*/ 0x1a)			/*STM32では不要*/
//#define cRcvUART2 (cEVEN | cPE | 0x05)		/*STM32では不要*/
//#define cRcvUART3 (cBRG | 0x28)				/*STM32では不要*/
#define cErrUART  (cOERR | cFERR | cPERR)
/* <------------------ STM32G0 ------------------ */


/*列挙型変数の定義 */
enum TrmdTag		/*通信メッセージ */
{
	cInit,			/* イニシャルモード */
	cSend,			/* 送信モード */
	cRcve			/* 受信モード */
};

enum SemdTag		/*送信メッセージ */
{
	cSendNak,		/* ＮＡＫコマンドの送信設定 */
	cSendAck1,		/* ＡＣＫ１コマンドの送信設定 */
	cSendAck2,		/* ＡＣＫ２コマンドの送信設定 */
	cSendData,		/* データの送信設定 */
	cSendAck4,		/* ＡＣＫ４コマンドの送信設定 */
	cSendAck5,		/* ＡＣＫ５コマンドの送信設定 */
	cSendWait,		/* コマンド間隔ウェイト */
	cSendExec		/* 送信処理の実行 */
};

enum RcmdTag		/*受信メッセージ */
{
	cRcveExec,		/* 受信処理の実行 */
	cRcveCmd,		/* コマンド受信 */
	cRcveData,		/* データ受信 */
	cRcveErr		/* 受信エラー */
};


/*通信データ構造体の定義 */
struct TrnsTag
{
	enum TrmdTag X_Trmd;		/*通信メッセージ */
	enum SemdTag X_Semd;		/*送信メッセージ */
	enum RcmdTag X_Rcmd;		/*受信メッセージ */
	unsigned char X_Tmc;		/*通信タイマカウンタ */
};

/*送信データ構造体の定義 */
struct SendTag
{
	unsigned char X_Buff[cTDBAFAC];	/*送信バッファ */
	unsigned char X_SetCnt;			/*送信バイト数カウンタ */
	unsigned char X_EndCnt;			/*送信済バイト数カウンタ */
	unsigned char X_Flag;			/*送信フラグ */
};

/*受信データ構造体の定義 */
struct RcveTag
{
	unsigned char X_Buff[cRDBAFAC];	/*受信バッファ */
	unsigned char X_Cnt;			/*受信カウンタ */
	unsigned char X_Flag;			/*受信フラグ */
	unsigned char X_Err;			/*受信エラーフラグ */
};


/*構造体型のエイリアス定義 */
typedef struct TrnsTag TRNS;
typedef struct SendTag SEND;
typedef struct RcveTag RCVE;


/* 外部変数 */
extern uint8_t u8g_SPD_cmd;		/*motorDriver用 外部指令Hz   At通信系ﾓｼﾞｭｰﾙ[DEC] */
extern uint8_t u8g_SPD_set;		/*motorDriver用 内部指令Hz   At通信系ﾓｼﾞｭｰﾙ[DEC] */

extern unsigned8	ucTRNS_ifrq_b;				/*実回転数 At通信系ﾓｼﾞｭｰﾙ[BCD] */
extern unsigned8	ucTRNS_sfrq_b;				/*指令Hz   At通信系ﾓｼﾞｭｰﾙ[BCD] */
extern unsigned8	ucTRNS_sfrg;				/*通信受信ﾃﾞｰﾀ */
		/*   ---- ---1 ;TEST3動作状態	0:通常状態 1:TEST3 */
		/*   ---- --1- ;騒音測定ﾓｰﾄﾞ	0:通常状態 1:騒音測定ﾓｰﾄﾞ中 */
		/*   ---- -1-- ;unused */
		/*   ---- 1--- ;ｺﾝﾌﾟ保護状態	0:通常状態 1:ｺﾝﾌﾟ保護制御中 */
		/*   ---1 ---- ;unused */
		/*   --1- ---- ;停止基準位置検出用起動回転数切替 0：33rps 1：27rps */
		/*   -1-- ---- ;unused */
		/*   1--- ---- ;unused */

extern unsigned8    ucTRNS_basepos;
extern unsigned16   unTRNS_crctduty;
extern unsigned8    ucTRNS_stopctrl;	/*転流停止制御用データ*/ /* @008MwHOM[10/06/11] */
extern unsigned8	ucTRNS_wirehgflg;	/* ｺﾝﾌﾟﾚｯｻ巻線保護ﾌﾗｸﾞﾃﾞｰﾀ *//* @305swDOD[24/09/02] */
extern unsigned8	ucTRNS_ista;				/*異常状態ﾃﾞｰﾀ  */
		/*   ---- ---1 ;過電流保護状態	0:正常 1:過電流保護状態 */
		/*   ---- --1- ;２ﾚﾍﾞﾙ保護状態	0:正常 1:２ﾚﾍﾞﾙ保護状態 */
		/*   ---- -1-- ;直電圧低下保護状態	0:正常 1:直流電圧低下保護状態 */
		/*   ---- 1--- ;ロック保護状態	0:正常 1:ロック保護状態 */
		/*   ---1 ---- ;ﾕﾆｯﾄ動作状態	0:ﾕﾆｯﾄ動作中 1:ﾕﾆｯﾄ動作停止中 */
		/*   --1- ---- ;ｺﾏﾝﾄﾞ実行状態	0:ｺﾏﾝﾄﾞ実行中 1:ｺﾏﾝﾄﾞ実行終了 */
		/*   -1-- ---- ;ﾕﾆｯﾄ正/異常状態	0:ﾕﾆｯﾄ正常 1:ﾕﾆｯﾄ異常 */
		/*   1--- ---- ;unused */
extern unsigned8	ucTRNS_duty;				/*Duty[%]  At通信系ﾓｼﾞｭｰﾙ[Nrm] */
extern unsigned8	ucTRNS_pwr;				/*平均電力[W]  At通信系ﾓｼﾞｭｰﾙ[Nrm] */
extern unsigned16	unTRNS_iq_decia;		/*q軸電流[A]*10 At通信系ﾓｼﾞｭｰﾙ[DEC] */
extern unsigned8	ucTRNS_prev_large_dec;	/*端子電圧A(上位2桁BCD)送信用 */
extern unsigned8	ucTRNS_prev_small_dec;	/*端子電圧B(上位2桁BCD)送信用 */
extern unsigned8	ucg_TRNS_angle;			/* 送信用通電角 */		/* @104yxDOD[18/10/03] */
extern unsigned8	ucTRNS_ista2;				/*異常状態ﾃﾞｰﾀ2  *//* @104yxDOD[18/10/03] */
		/*   ---- ---1 ;unused */
		/*   ---- --1- ;unused */
		/*   ---- -1-- ;unused */
		/*   ---- 1--- ;unused */
		/*   ---1 ---- ;unused */
		/*   --1- ---- ;unused */
		/*   -1-- ---- ;unused *//* CT不使用のためCT回路異常判断なし*//* @263swDOD[23/07/26] */
		/*   1--- ---- ;unused */
extern unsigned8	ucg_TRNS_PWMfreq;		/* 送信用ｷｬﾘｱ周波数 */	/* @104yxDOD[18/10/03] */
extern unsigned8	ucTRNS_trer;				/*通信異常関連ﾒﾓﾘ */
extern unsigned8	ucTRNS_type;				/*通信受信ﾃﾞｰﾀ(機種判別) */

extern UCHAR	ucTRNS_carrier;  			/* キャリア周波数 		*//* 宣言箇所最適化 *//* @260swDOD[23/06/30] */
extern UCHAR	ucTRNS_pos1duty; 			/* 起動duty 位置決め1 	*//* 宣言箇所最適化 *//* @260swDOD[23/06/30] */
extern UCHAR	ucTRNS_pos2duty;			/* 起動duty 位置決め2 	*//* 宣言箇所最適化 *//* @260swDOD[23/06/30] */
extern UCHAR	ucTRNS_forcedduty;  		/* 起動duty 強制転流 	*//* 宣言箇所最適化 *//* @260swDOD[23/06/30] */
extern UCHAR	ucTRNS_DlineHi;  			/* Dライン共振回転数(上限) *//* 宣言箇所最適化 *//* @260swDOD[23/06/30] */
extern UCHAR	ucTRNS_DlineLow; 			/* Dライン共振回転数(下限) *//* 宣言箇所最適化 *//* @260swDOD[23/06/30] */
extern UCHAR	ucTRNS_DEFHrate;  			/* デフヒータ通電率 	*/
extern UCHAR	ucTRNS_PCEVAHrate;  		/* PCエバヒータ 		通電率	*/


extern eStateTag  ePMD_drvmode;					/* State of Pmd1G */

extern UART_HandleTypeDef huart1;

/* prtotype */
void vTRNS_Hw_trns(void);

extern UI08 IsFullWave(void);


#endif	/* !defined( __HWTRNS_H ) */
