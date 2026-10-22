/*
 * PositionDetect.h
 *
 *  Created on: 2020/10/21
 *      Author: 2993017
 */

#ifndef INV_POSITIONDETECT_H_
#define INV_POSITIONDETECT_H_

#include "Sys_type.h"
extern UI16 unPMDi_MCAPdumy;
extern UI16 unPMD_MCAP;
extern UI08 ucPMD_pdct;

void vPMDi_Setcomtptn(void);
void DummyPositionDetect(void);

/**************************************************************************
<Title>			:unPMD_GetMCAPdumy 		;vHOG_Rockhg
<Function>		:ucr_MCAPdumy の取得
<Description>	:	本関数の呼出で、転流用モードタイム(ucr_MCAPdumy)を参照する
<Input>			:
<Return>		:ucr_MCAPdumy
				:	備考     0:ロック確認不要	1:ロック確認必要
<Author>		:-hide	作成
<Date>			:2002-1111
<Update>		:Ver REF036-0.07 2002-11-11
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline UI16 unPMD_GetMCAPdumy(void){										/*転流用ﾓｰﾄﾞﾀｲﾑﾃﾞｰﾀ取得(Wordﾃﾞｰﾀ) */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	return unPMDi_MCAPdumy;											/*転流用ﾓｰﾄﾞﾀｲﾑﾃﾞｰﾀを返す */
}


static inline void ClearForcedPosDetectCnt(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    ucPMD_pdct = 0;                                             /*UPカウンタ化 */
}

#endif /* INV_POSITIONDETECT_H_ */
