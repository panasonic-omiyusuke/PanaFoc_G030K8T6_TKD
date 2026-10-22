/**************************************************************************
#ifdef DOC
File Name		：TIME_M.C
Function		：分タイマモジュール
Description		：
Program			：
Author			：小柳　英二郎
Date			：05/04/20
Update			：05/05/17 REF051-1.01 低外気ﾃﾞﾌ終了判定ﾀｲﾏ、ﾃﾞﾌ後ﾌｧﾝ遅延ﾀｲﾏ削除、実省エネﾄﾞｱ開ﾀｲﾏ追加
				：05/06/10 REF051-1.02 ﾌﾟﾘｸｰﾙﾀｲﾏ追加
				：05/06/21 REF051-1.03 工程検査Cﾌｧﾝ遅延ﾀｲﾏ追加
				：05/07/08 REF051-1.04 実省エネPCﾄﾞｱ開ﾀｲﾏ、C法Cﾌｧﾝ用起動ﾀｲﾏ削除
				：05/07/11 REF051-1.05 ｺﾝﾌﾟ回転数DOWNﾀｲﾏ追加
				：05/09/05 REF05A-2.01 実省エネPCﾄﾞｱﾀｲﾏ追加
				：06/09/04 REF068-1.00 OFF/ON制御ﾀｲﾏ追加
Copyright		：Matsushita Refrigeration Co.,Ltd.
#endif
**************************************************************************/
//#pragma SECTION program dtf_program		/* 本命令以降の関数をDataFlash領域へ配置 */	/* @011MzDOD[17/02/08] */

/********************************************
		インクルードファイル
 ********************************************/
#include	"Sys_type_main.h"							/* @018MzDOD[13/07/11] */
#include	"Sys_func.h"								/* @018MzDOD[13/07/11] */
#include	"Ram.h"										/* @018MzDOD[13/07/11] */
#include	"time.h"									/* @018MzDOD[13/07/11] */

/*******************************************
		マクロ定義
 *******************************************/
/*******************************************
		変数宣言
 *******************************************/
st_2btm	stg_TIMEM[TMMCT];					/*分ﾀｲﾏ*/

UCHAR	ucg_TIME_timem_ms;	/*分ｶｳﾝﾄ用ﾀｲﾏ(ms)*/

/*******************************************
		プロトタイプ宣言
 *******************************************/

/**************************************************************************
<Title>			:vTIME_min
<Function>		:毎分カウントアップタイマ
<Description>	:	1分以上の定時間用タイマ。
				:	1秒毎にカウントアップし続る。
				:	タイマスタート及び終了の判定は
				:	各モジュールにて行ない、使用時以外は
				:	1分、1秒カウンタをクリアしておく。
				:	本ルーチン内で更に別のサブルーチンをコールする。
				:	（分カウント	vTIME_cnt）
				:	（msカウント	ucTIME_mcntmstmup）
<Input>			:-
<Return>		:-
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:ung_TIMEM(分ﾀｲﾏ)
<Author>		:小柳　英二郎
<Date>			:06/11/15
<Update>		:06/11/30 ﾀｲﾏ変更 小柳
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
void vTIME_min(void){

	if ( ucTIME_mcntmstmup() == SET ){
		vTIME_cnt(&stg_TIMEM[0],TMMCT,MHCNTDT,MHMAXDT);		/*時間ﾀｲﾏｶｳﾝﾄ処理*/
	}
}

/**************************************************************************
<Title>			:ucTIME_mcntmstmup
<Function>		:分タイマｍｓカウントアップタイマ
<Description>	:分タイマ用ｍｓタイマカウントタイムＵＰ判定
<Input>			:-
<Return>		:ucl_timeup(msﾀｲﾏﾀｲﾑUP)
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:ucg_TIME_timem_ms(10msecｶｳﾝﾀ)
<Author>		:小柳　英二郎
<Date>			:06/11/15
<Update>		:
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
UCHAR ucTIME_mcntmstmup(void){
	UCHAR ucl_timeup = CLR;

	ucg_TIME_timem_ms += MSCNTDT;							/* 10msecｶｳﾝﾀ+4					*/
	if( ucg_TIME_timem_ms >= MSMAXDT){						/* 10msecｶｳﾝﾀ≧100				*/
		ucg_TIME_timem_ms = CLR;							/* 10msecｶｳﾝﾀﾘｾｯﾄ				*/
		ucl_timeup = SET;									/*msﾀｲﾏﾀｲﾑUP*/
	}
	return ( ucl_timeup );
}
