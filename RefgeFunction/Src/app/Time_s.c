/**************************************************************************
#ifdef DOC
File Name		：TIME_S.C
Function		：秒タイマモジュール
Description		：
Program			：
Author			：小柳　英二郎
Date			：05/04/21
Update			：05/05/17 REF051-1.01 ｻｰﾋﾞｽ制御ﾏｲｺﾝｺｰﾄﾞ表示ﾀｲﾏ削除
				：05/05/26 REF051-1.02 ｻｰﾋﾞｽ制御ﾏｲｺﾝｺｰﾄﾞ表示ﾀｲﾏ復活
				：05/06/24 REF051-1.03 製氷LED点滅用ﾀｲﾏ削除
Copyright		：Matsushita Refrigeration Co.,Ltd.
#endif
**************************************************************************/
//#pragma SECTION program dtf_program		/* 本命令以降の関数をDataFlash領域へ配置 */	/* @011MzDOD[17/02/08] */

/********************************************
		インクルードファイル
 ********************************************/
#include	"Sys_type_main.h"							/* @018MzDOD[13/07/11] */	/*必要 CLR SET 定義*/
#include	"Sys_func.h"								/* @018MzDOD[13/07/11] */
#include	"Ram.h"										/* @018MzDOD[13/07/11] *//*必要*/
#include	"time.h"									/* @018MzDOD[13/07/11] *//*必要*/

/*******************************************
		マクロ定義
 *******************************************/
/*******************************************
		変数宣言
 *******************************************/
st_2btm	stg_TIMES[TMSCT];					/*秒ﾀｲﾏ*/

/*******************************************
		プロトタイプ宣言
 *******************************************/

/**************************************************************************
<Title>			:vTIME_sec
<Function>		:毎秒カウントアップタイマ
<Description>	:	1秒以上の定時間用タイマ。
				:	40msec毎にカウントアップし続る。
				:	タイマスタート及び終了の判定は各モジュールにて行ない、
				:	使用時以外は1秒、40msecカウンタをクリアしておく。
				:	本ルーチン内で更に別のサブルーチンをコールする。
				:	（秒カウント	vTIME_cnt）
<Input>			:-
<Return>		:-
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:ung_TIMES(秒ﾀｲﾏ)
<Author>		:小柳　英二郎
<Date>			:06/10/10
<Update>		:06/11/30 ﾀｲﾏ変更 小柳
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
void vTIME_sec(void){

	vTIME_cnt(&stg_TIMES[0],TMSCT,MSCNTDT,MSMAXDT);				/*秒ﾀｲﾏｶｳﾝﾄ処理*/
}

/**************************************************************************
<Title>			:vTIME_cnt
<Function>		:秒カウント
<Description>	:	40msec毎のカウントアップを受け1秒を形成する。
<Input>			:*adry(ﾀｲﾏ)
				:ucl_timesct(ﾀｲﾏ数)
				:ucl_cntdt(ﾀｲﾏｶｳﾝﾄ値)
				:ucl_lomax(下位ﾀｲﾏmax値)
<Return>		:-
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:-
<Author>		:小柳　英二郎
<Date>			:06/11/15
<Update>		:06/11/30 ﾀｲﾏ変更 小柳
				:06/12/05 修正 小柳
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
void vTIME_cnt(st_2btm *time,UCHAR ucl_timesct,UCHAR ucl_cntdt,UCHAR ucl_lomax ){
	UCHAR k;
	st_2btm *tmadr;

	for( k = 0; k < ucl_timesct; k++ ){
		tmadr = time;									/*先頭ｱﾄﾞﾚｽ*/
		tmadr += k;										/*先頭ｱﾄﾞﾚｽ +k */
		tmadr->word += ucl_cntdt;						/*+ｶｳﾝﾄ値*/
		if( tmadr->byte.lo >= ucl_lomax ){				/* max値を超えたら次のﾒﾓﾘ	*/
			tmadr->byte.lo = CLR;						/* ﾀｲﾏｸﾘｱ	*/
			if( tmadr->byte.hi < CNTMAXDT){				/* ｶｳﾝﾄ上限まで */
				tmadr->byte.hi += MHCNTDT;				/*+1ｶｳﾝﾄ*/
			}
		}
	}
}
