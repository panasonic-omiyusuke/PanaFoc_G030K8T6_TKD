/*
 * soft_stop.h
 *
 *  Created on: 2021/06/09
 *      Author: 2993017
 */

#ifndef INV_SOFT_STOP_H_
#define INV_SOFT_STOP_H_

#include "Sys_type.h"
#include "adc_i.h"

extern enum eFreeRunAdcMode free_run_index; /* 0-5 2021/06/09 takeoka */

/* フリーラン中の位置検出用にAD変換するレジスタを選択  2021/06/09 takeoka */
/* ADINSEL <- free_run_src_adinsl_tbl[free_run_index];  2021/06/09 takeoka */
//extern UI08 free_run_src_adinsl_tbl[6][READSTART_NUM];
/* adc result for free run 2021/06/09 takeoka */
/* ucCPU_SAR_GETVT <- free_run_src_portreg_tbl_large[free_run_index] 2021/06/09 takeoka */
//extern UI08 * const free_run_src_portreg_tbl_large[6];
/* ucCPU_SAR_GETVPN <- free_run_src_portreg_tbl_small[free_run_index] 2021/06/09 takeoka */
//extern UI08 * const free_run_src_portreg_tbl_small[6];


void InitSoftStop(void);						 /* 停止処理の初期化 */
void FreeRunPositionDetcted(void);			 /* 停止のフリーラン中の位置検出処理 */
void SoftStopControl(SI16 large, SI16 small); /* ブレーキ停止処理の起点 AD変換結果が引数 期待値が large ＞ small */


#endif /* INV_SOFT_STOP_H_ */
