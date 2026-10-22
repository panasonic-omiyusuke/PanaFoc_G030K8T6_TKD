/**************************************************************************
#ifdef DOC
File Name		：TIME_H.C
Function		：時間タイマモジュール
Description		：
Program			：
Author			：小柳　英二郎
Date			：05/04/21
Update			：05/05/17 REF051-1.01 実省エネ待機ﾓｰﾄﾞﾀｲﾏ追加
				：05/06/03 REF051-1.02 IPD時ﾀﾞﾝﾊﾟﾀｲﾏ削除
				：05/06/21 REF051-1.03 温度履歴書込み判定ﾀｲﾏ追加
				：05/07/13 REF051-1.04 庫内高負荷制御ﾀｲﾏ追加
				：05/09/13 REF068-1.00 OFF/ONﾀｲﾑｾｰﾌ60分とならずﾀｲﾏ数追加修正
				：07/04/24 REF071 B32M1 UPD78F0501 ﾌｧﾝ低速回転連続ﾀｲﾏ追加
				：07/07/04 B32M1 過負荷検知判定追加
				：10/09/13 ﾀｲﾏ処理変更
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
st_2btm	stg_TIMEH[TMHCT];					/*時間ﾀｲﾏ*/
UCHAR	ucg_TIME_timeh_ms;	/*時間ｶｳﾝﾄ用ﾀｲﾏ(ms)*/	/* @000IzDOD[10/09/13]_04 */
UCHAR	ucg_TIME_timeh_s;	/*時間ｶｳﾝﾄ用ﾀｲﾏ(s)*/	/* @000IzDOD[10/09/13]_04 */


/*******************************************
		プロトタイプ宣言
 *******************************************/

/**************************************************************************
<Title>			:vTIME_hour
<Function>		:毎時間カウントアップタイマ
<Description>	:	1時間以上の定時間用タイマ。
				:	1分毎にカウントアップし続ける。
				:	タイマスタート及び終了の判定は各モジュールにて行ない、使用時以外は
				:	1時間,1分カウンタをクリアしておく。
				:	本ルーチン内で更に別のサブルーチンをコールする。
				:（時間カウント　vTIME_cnt）
				:（時間カウント分タイマタイムＵＰ	ucTIME_hcntmtmup）
<Input>			:-
<Return>		:-
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:ung_TIMEH(時間ﾀｲﾏ)
<Author>		:小柳　英二郎
<Date>			:06/11/15
<Update>		:06/11/30 ﾀｲﾏ変更 小柳
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
void vTIME_hour(void){

	if ( ucTIME_hcntmtmup() == SET ){						/*分ﾀｲﾏﾀｲﾑUP*/
		vTIME_cnt(&stg_TIMEH[0],TMHCT,MHCNTDT,MHMAXDT);		/*時間ﾀｲﾏｶｳﾝﾄ処理*/
	}
}

/**************************************************************************
<Title>			:ucTIME_hcntmtmup
<Function>		:時間タイマ秒カウントアップタイマ
<Description>	:	時間タイマ用秒タイマカウントタイムＵＰ判定
<Input>			:-
<Return>		:ucl_timeup(分ﾀｲﾏﾀｲﾑUP)
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:ucg_TIME_timem_ms(msecｶｳﾝﾀ)
				:ucg_TIME_timem_s(secｶｳﾝﾀ)
<Author>		:小柳　英二郎
<Date>			:06/11/15
<Update>		:
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
UCHAR ucTIME_hcntmtmup(void){
	UCHAR ucl_timeup = CLR;

	ucg_TIME_timeh_ms += MSCNTDT;							/* 10msecｶｳﾝﾀ+4					*/
	if ( ucg_TIME_timeh_ms >= MSMAXDT){						/* 10msecｶｳﾝﾀ≧100				*/
		ucg_TIME_timeh_ms = CLR;							/* 10msecｶｳﾝﾀﾘｾｯﾄ				*/
		ucg_TIME_timeh_s += MHCNTDT;						/* 1秒ｶｳﾝﾀ+1					*/
		if( ucg_TIME_timeh_s >= MHMAXDT){					/* 1秒ｶｳﾝﾀ≧60					*/
			ucg_TIME_timeh_s = CLR;							/* 1秒ｶｳﾝﾀﾘｾｯﾄ					*/
			ucl_timeup = SET;								/*秒ﾀｲﾏﾀｲﾑUP*/
		}
	}
	return ( ucl_timeup );
}
