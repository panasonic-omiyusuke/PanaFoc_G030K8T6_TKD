#ifndef BRAKE_H
#define BRAKE_H

/********************************************
		インクルードファイル
 ********************************************/
#include "Pmd_int.h"
#include "system_ram.h"

#define D_STANDBY_INDEX  (7)
#define FREE_RUN_POS	(0x04)	/* 停止指令時にPMDブレーキ制御を有効化 */
#define FREE_RUN_COUNT_MAX_FCT	6500
#define FREE_RUN_COUNT_MAX_NORMAL	2000

#define PWMOUT_MASK (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE)

/*vi_phase_differential.c*/
#define DPTN_UON_LAP	(1)												/* U相 重なり期間 ON: uvw xyz = 101 010 */
#define DPTN_UON		(2)												/* U相 120deg期間 ON: uvw xyz = 100 010 */
#define DPTN_UOFF		(5)												/* U相 120deg期間 OFF: uvw xyz = 110 001 */
#define DPTN_UOFF_LAP	(6)												/* U相 重なり期間 OFF: uvw xyz = 010 001 */
#define DPTN_XON_LAP	(7)												/* X相 重なり期間 ON: uvw xyz = 010 101 */
#define DPTN_XON		(8)												/* X相 120deg期間 ON: uvw xyz = 010 100 */
#define DPTN_XOFF		(11)											/* X相 120deg期間 OFF: uvw xyz = 001 110 */
#define DPTN_XOFF_LAP	(0)												/* X相 重なり期間 OFF: uvw xyz = 001 010 */

/* 停止検出ステータス */
#define TVLT_CLR		(0x00)											/* 停止検出ステータス初期値 */
#define TVLT_XOFF_SPK	(0x01)											/* X相OFFスパイク検出 */
#define TVLT_UOFF_EDG	(0x02)											/* U相OFFエッジ検出 */
#define TVLT_UOFF_PDU	(0x04)											/* U相OFF時のDutyゼロ検出 */
#define TVLT_XON_EDG	(0x08)											/* X相ONエッジ検出 */
#define TVLT_STOP_SPK	(0x80)											/* 停止判定スパイク検出 */
/*vi_phase_differential.c*/

#define BRAKE_START_ANGLE 14563 /* ブレーキ開始角度: 4/9周期 (32768*(4/9)) */

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UI08 ucPMD_mdidx;			/* PMDモードインデックス */
extern const UI08 judge_tbl[];
extern const uint32_t ad_seq_table[];
extern const uint32_t ad_seq_free_run_table[6];
extern void vPMD_ForceStopAndInit(void);
extern void vPMD_SoftStopEntry(void); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
extern void vPMD_SoftStopDuringFreeRun(void);
extern void InitSoftStop(void);
extern bool IsBreakControlEnabled(uint8_t ucTRNS_stopctrl);

extern unsigned8 ADC_BRAKE_FLG;
extern UI16 free_run_count;

extern volatile SI16 prev_small;			/* 前回AD値（常に prev_small > prev_large） */
extern volatile SI16 prev_large;			/* 前回AD値 */

enum Brake_State {
	BRAKE_READY, /* ブレーキ開始可能状態 */
    BRAKE_STOPPING, /* ブレーキ停止中状態 */
    BRAKE_FINISHED, /* ブレーキ完了状態 */
	BRAKE_BANNED /* ブレーキ禁止状態 */
};

extern volatile enum Brake_State u8g_brake_state; /* ブレーキ制御状態 */

#endif /* BRAKE_H */
