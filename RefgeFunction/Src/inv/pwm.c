#include "Sys_type.h"
#include "vipdc.h"
#include "dtc_i.h"

#include "pwm.h"
#include "pwm_i.h"

#include "PMD1GPar.h"

#include "angle_control.h"  /* for IsAngleBasic */
#include "Pmd_int.h"

#include "dtc.h"
#include "Com_def.h"
#include "system_ram.h"
#include "Pmd1G.h"

#include "main.h"


/* extern UI08 IsDutyFull(void); */
/* extern UI08 IsDrvmodeNormalRun(void); */
extern UI08 IsMdoutPosition1st(void);
extern UI08 IsMdoutPosition2nd(void);
extern UI08 IsMdoutStop(void);

extern UI08 ucTRNS_sfrg;
extern TIM_HandleTypeDef htim1;


#ifdef UNIT_TEST
/* UI08 ucTYPE_LARM_drive; */
/* eStateTag ePMD_drvmode; */
/* UI08 ucPMD_mdidx; */
/* UI08 ucPMD_boostmd; */
/* UI16 unPMDc_dutyn; */
/* UI08 ucTRNS_3kflag; */
/* UI08 ucSPD_rpmh; */
/* UI08 call_setpwm_flg; */
/* sAngleDataTag angle; */                        /* 通電角に関する情報の構造体 */
/* UI16 unPMD_MDOUT; */
/* void SetDtcChangeDuty(UI08 *dtc_addr, UI08 ctrl_reg, UI08 block_size, UI16 *src_addr, UI16 *dest_addr){} */
/* void RegistDtcChangeDuty(void){} */
/* void ResetDtcCountChangeDuty(void){} */
/* void SetDtcAddrChangeDuty(UI16* crr_sw_dst_addr, UI16* nxt_sw_dst_addr,UI16* nnxt_sw_src_addr, UI16* nnxt_sw_dst_addr,UI16* rest_dst_addr){} */
/* UI08 IsDrvmodeNormalRun(void){return 0;} */
/* UI08 IsDrvmodeStop(void){return (ePMD_drvmode == D_stop);} */
/* UI08 IsDutyFull(void){return (unPMDc_dutyn >= D_DUTYMAX );} */
/* UI08 IsMdoutPosition1st(void){return (unPMD_MDOUT == D_POS1STPTN);} */
/* UI08 IsMdoutPosition2nd(void){return (unPMD_MDOUT == D_POS2NDPTN);} */
/* UI08 IsMdoutStop(void){ return (unPMD_MDOUT == D_BREAKPTN);} */


extern UI08 ucPMD_mdidx;
extern UI08 ucPMD_boostmd;
extern UI08 ucTYPE_uart;
extern UI16 unPMDc_dutyn;
extern UI08 ucTRNS_3kflag;
extern UI08 ucSPD_rpmh;
extern UI08 call_setpwm_flg;


#undef  static inline /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#define static inline

#else
unsigned8  ucTRNS_3kflag;				/*3kHz強制切替FG  */					/* @018awDOD[14/05/16] */

extern UI08 ucPMD_mdidx;
extern UI08 ucTRQ_stateflg;
extern UI08 ucPMD_boostmd;
extern UI08 ucTYPE_uart;
extern UI16 unPMDc_dutyn;

extern UI08 ucSPD_rpmh;
extern UI08 call_setpwm_flg;
#endif
UI08 ucPMD_p2ptn;
UI08 ucPMD_p2ptn_full;
UI08 uc_PMC_p2ptnL_pol;
UI08 detect_pwm_mode;
UI08 detect_pwm_mode_calc;
UI08 assigned_flg;
UI08 ucTYPE_carrier_freq;	/* @250swDOD[23/05/18] */
UI16 unPWMPeriodHi;			/* @250swDOD[23/05/18] */

/* UI16 spike_detection_timing[2]; */
/* UI16 spike_detection_timing; */
/* UI16 spike_detection_timing2; */
UI08 inv_sw_mode;
UI16 hysteresis_duty_trd_detect_pwm_mode;

/* volatile UI32 *const rest_tbl_ptr; */
const UI08 *pwmint_en_trd0_dtctbl_ptr;
const UI08 *pwmint_en_trd1_dtctbl_ptr;
const UI08 *addtct_tbl3_ptr;
const UI08 *addtct_tbl4_ptr;

/* #define D_PWMPERIOD    (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_DEFAULT) */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
/* #define D_PWMPERIODLo  (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_LOW)	 */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
/* #define D_PWMPERIODHi  (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_HIGH)	 */	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
/* #define D_PWMPERIODfrc (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_VECTOR)	 *//* @007MzDOD[16/12/21] */ /* 差圧起動 */
/* #define D_PWMPERIOD_ECO (UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_ECO)	 */



/* PWMポート出力の選択用 */
#define TRDOER1_B0	(0x20)
#define TRDOER1_C0	(0x40)
#define TRDOER1_D0	(0x80)
#define TRDOER1_B1	(0x02)
#define TRDOER1_C1	(0x04)
#define TRDOER1_D1	(0x08)

/* TRD割込み許可ﾚｼﾞｽﾀ(TRDIER0,TRDIER1) ﾋﾞｯﾄ割付 */
#define TRDIER0_B0	(0x02)
#define TRDIER0_C0	(0x04)
#define TRDIER0_D0	(0x08)
#define TRDIER0_B1	(0x00)
#define TRDIER0_C1	(0x00)
#define TRDIER0_D1	(0x00)

#define TRDIER1_B0	(0x00)
#define TRDIER1_C0	(0x00)
#define TRDIER1_D0	(0x00)
#define TRDIER1_B1	(0x02)
#define TRDIER1_C1	(0x04)
#define TRDIER1_D1	(0x08)

/* DTC起動許可ﾚｼﾞｽﾀ(DTCEN3) ﾋﾞｯﾄ割付 */
#define DTCEN3_B0	(0x10)
#define DTCEN3_C0	(0x08)
#define DTCEN3_D0	(0x04)
#define DTCEN3_B1	(0x01)
#define DTCEN3_C1	(0x00)
#define DTCEN3_D1	(0x00)

#define DTCEN4_B0	(0x00)
#define DTCEN4_C0	(0x00)
#define DTCEN4_D0 	(0x00)
#define DTCEN4_B1	(0x00)
#define DTCEN4_C1	(0x80)
#define DTCEN4_D1	(0x40)

#define  D_PWMON2SAMPLING (UI16)(200 * D_fTRD)				/* 2回サンプリングできるかのDuty閾値 */
#define  D_2SMPL_TIME_TRD (UI16)(200 * D_fTRD)				/* 2回サンプリングできるかのDuty閾値 */

#ifdef RENESAS_TO_ST32
const uint32_t output_mode_table12[] = {
		(MR1_U_X_ON  | MR1_V_Y_PWM | MR1_W_Z_PWM), //U-Y
		(MR1_U_X_PWM | MR1_V_Y_PWM | MR1_W_Z_ON ), //U-Z
		(MR1_U_X_PWM | MR1_V_Y_ON  | MR1_W_Z_PWM), //V-Z
		(MR1_U_X_ON  | MR1_V_Y_PWM | MR1_W_Z_PWM), //V-X
		(MR1_U_X_PWM | MR1_V_Y_PWM | MR1_W_Z_ON ), //W-X
		(MR1_U_X_PWM | MR1_V_Y_ON  | MR1_W_Z_PWM), //W-Y
		(MR1_U_X_ON  | MR1_V_Y_ON  | MR1_W_Z_ON ), //stop
		(MR1_U_X_ON  | MR1_V_Y_ON  | MR1_W_Z_ON ), //standby
		};
const uint32_t output_mode_table3[]  = {
		(MR2_U_X_ON  | MR2_V_Y_PWM | MR2_W_Z_PWM), //U-Y
		(MR2_U_X_PWM | MR2_V_Y_PWM | MR2_W_Z_ON ), //U-Z
		(MR2_U_X_PWM | MR2_V_Y_ON  | MR2_W_Z_PWM), //V-Z
		(MR2_U_X_ON  | MR2_V_Y_PWM | MR2_W_Z_PWM), //V-X
		(MR2_U_X_PWM | MR2_V_Y_PWM | MR2_W_Z_ON ), //W-X
		(MR2_U_X_PWM | MR2_V_Y_ON  | MR2_W_Z_PWM), //W-Y
		(MR2_U_X_ON  | MR2_V_Y_ON  | MR2_W_Z_ON ), //stop
		(MR2_U_X_ON  | MR2_V_Y_ON  | MR2_W_Z_ON ), //standby
		};

const uint32_t output_mode_lap_table12[] = {
		(MR1_U_X_PWM | MR1_V_Y_ON  | MR1_W_Z_PWM), //UW-Y or Y
		(MR1_U_X_ON  | MR1_V_Y_PWM | MR1_W_Z_PWM), //U-YZ or U
		(MR1_U_X_PWM | MR1_V_Y_PWM | MR1_W_Z_ON ), //UV-Z or Z
		(MR1_U_X_PWM | MR1_V_Y_ON  | MR1_W_Z_PWM), //V-XZ or V
		(MR1_U_X_ON  | MR1_V_Y_PWM | MR1_W_Z_PWM), //VW-X or X
		(MR1_U_X_PWM | MR1_V_Y_PWM | MR1_W_Z_ON ), //W-XY or W
		(MR1_U_X_ON  | MR1_V_Y_ON  | MR1_W_Z_ON ), //stop
		(MR1_U_X_ON  | MR1_V_Y_ON  | MR1_W_Z_ON ), //standby
		};
const uint32_t output_mode_lap_table3[]  = {
		(MR2_U_X_PWM | MR2_V_Y_ON  | MR2_W_Z_PWM), //UW-Y or Y
		(MR2_U_X_ON  | MR2_V_Y_PWM | MR2_W_Z_PWM), //U-YZ or U
		(MR2_U_X_PWM | MR2_V_Y_PWM | MR2_W_Z_ON ), //UV-Z or Z
		(MR2_U_X_PWM | MR2_V_Y_ON  | MR2_W_Z_PWM), //V-XZ or V
		(MR2_U_X_ON  | MR2_V_Y_PWM | MR2_W_Z_PWM), //VW-X or X
		(MR2_U_X_PWM | MR2_V_Y_PWM | MR2_W_Z_ON ), //W-XY or W
		(MR2_U_X_ON  | MR2_V_Y_ON  | MR2_W_Z_ON ), //stop
		(MR2_U_X_ON  | MR2_V_Y_ON  | MR2_W_Z_ON ), //standby
		};

const uint32_t output_mode_all_pwm_12 = (MR1_U_X_PWM | MR1_V_Y_PWM | MR1_W_Z_PWM);
const uint32_t output_mode_all_pwm_3  = (MR2_U_X_PWM | MR2_V_Y_PWM | MR2_W_Z_PWM);

UI32 next_mdout;
UI32 next_ccmr1;
UI32 next_ccmr2;

const uint32_t T_MDOUTptn[] = {
		(TIM_CCER_CCE_U                                                     | TIM_CCER_CCNE_Y                  ), //U-Y
		(TIM_CCER_CCE_U                                                                       | TIM_CCER_CCNE_Z), //U-Z
		(                 TIM_CCER_CCE_V                                                      | TIM_CCER_CCNE_Z), //V-Z
		(                 TIM_CCER_CCE_V                  | TIM_CCER_CCNE_X                                    ), //V-X
		(                                  TIM_CCER_CCE_W | TIM_CCER_CCNE_X                                    ), //W-X
		(                                  TIM_CCER_CCE_W                   | TIM_CCER_CCNE_Y                  ), //W-Y
		(0),
		(                                                   TIM_CCER_CCNE_X | TIM_CCER_CCNE_Y | TIM_CCER_CCNE_Z), //standby
};

const uint32_t T_MDOUTlap[] = {
		(TIM_CCER_CCE_U                  | TIM_CCER_CCE_W                   | TIM_CCER_CCNE_Y                  ), //UW-Y
		(TIM_CCER_CCE_U                                                     | TIM_CCER_CCNE_Y | TIM_CCER_CCNE_Z), //U-YZ
		(TIM_CCER_CCE_U | TIM_CCER_CCE_V                                                      | TIM_CCER_CCNE_Z), //UV-Z
		(                 TIM_CCER_CCE_V                  | TIM_CCER_CCNE_X                   | TIM_CCER_CCNE_Z), //V-XZ
		(                 TIM_CCER_CCE_V | TIM_CCER_CCE_W | TIM_CCER_CCNE_X                                    ), //VW-X
		(                                  TIM_CCER_CCE_W | TIM_CCER_CCNE_X | TIM_CCER_CCNE_Y                  ), //W-XY
		(0),
		(                                                   TIM_CCER_CCNE_X | TIM_CCER_CCNE_Y | TIM_CCER_CCNE_Z), //standby
		};
const uint32_t T_MDOUTgap[] = {
		(                                                                     TIM_CCER_CCNE_Y                  ), //UW-Y
		(TIM_CCER_CCE_U                                                                                        ), //U-YZ
		(                                                                                       TIM_CCER_CCNE_Z), //UV-Z
		(                 TIM_CCER_CCE_V                                                                       ), //V-XZ
		(                                                   TIM_CCER_CCNE_X                                    ), //VW-X
		(                                  TIM_CCER_CCE_W                                                      ), //W-XY
		(0), //dummy
		(                                                   TIM_CCER_CCNE_X | TIM_CCER_CCNE_Y | TIM_CCER_CCNE_Z), //dummy
		};


volatile UI32 *const nnxt_phase_sw_tbl_head[]																			/* @007MzDOD[16/12/21] */ /* �����N�� */
  = {&unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U};				/* @007MzDOD[16/12/21] */ /* �����N�� */
volatile UI32 *const nxt_phase_sw_tbl_head[]																				/* @007MzDOD[16/12/21] */ /* �����N�� */
  = {&unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y};				/* @007MzDOD[16/12/21] */ /* �����N�� */
volatile UI32 *const crr_phase_sw_tbl_head[]																				/* @007MzDOD[16/12/21] */ /* �����N�� */
  = {&unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W};				/* @007MzDOD[16/12/21] */ /* �����N�� */
volatile UI32 *const crr_phase_full_tbl_head[]																			/* @007MzDOD[16/12/21] */ /* �����N�� */
  = {&unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X};				/* @007MzDOD[16/12/21] */ /* �����N�� */

volatile UI32 *const nnxt_phase_full_tbl_tail[]
  = {&unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U};
volatile UI32 *const nnxt_phase_sw_tbl_tail[]
  = {&unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y};
volatile UI32 *const nxt_phase_sw_tbl_tail[]
  = {&unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W};
volatile UI32 *const crr_phase_sw_tbl_tail[]
  = {&unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X};

#if 0 /* 今後の検討で必要になる可能性ありのためｺﾒﾝﾄとして本モジュールを残す */
volatile UI32 *const crr_phase_sw_tbl_pos[]
  = {&unCPU_TRDGR_U, &unCPU_TRDGR_V, &unCPU_TRDGR_V, &unCPU_TRDGR_W, &unCPU_TRDGR_W, &unCPU_TRDGR_U};
volatile UI32 *const crr_phase_sw_lap_tbl_pos[]
  = {&unCPU_TRDGR_V, &unCPU_TRDGR_U, &unCPU_TRDGR_W, &unCPU_TRDGR_V, &unCPU_TRDGR_U, &unCPU_TRDGR_W};
volatile UI32 *const crr_full_tbl_pos[]
  = {&unCPU_TRDGR_Z, &unCPU_TRDGR_Z, &unCPU_TRDGR_X, &unCPU_TRDGR_X, &unCPU_TRDGR_Y, &unCPU_TRDGR_Y};
#endif


volatile UI32 * const rest_tbl_ptr
  = &(TIM1->CCR4);

volatile UI32 *crr_phase_sw;				/* ���݂̃I���f�q ��										*/
volatile UI32 *nxt_phase_sw;				/* ���̃I���f�q ��											*/
volatile UI32 *nnxt_phase_sw;				/* ���̃I���f�q ��											*/
volatile UI32 *nnxt_full;
volatile UI32 *nxt_full;
volatile UI32 *crr_full;


#else

#define unCPU_TRDGR_U 	unCPU_TRDGRB0
#define PWM_SELECT_U	TRDOER1_B0
#define PWMINTEN0_U		TRDIER0_B0
#define PWMINTEN1_U		TRDIER1_B0
#define DTCEN3_U		DTCEN3_B0
#define DTCEN4_U		DTCEN4_B0

#define unCPU_TRDGR_V 	unCPU_TRDGRC0
#define PWM_SELECT_V	TRDOER1_C0
#define PWMINTEN0_V		TRDIER0_C0
#define PWMINTEN1_V		TRDIER1_C0
#define DTCEN3_V		DTCEN3_C0
#define DTCEN4_V		DTCEN4_C0

#define unCPU_TRDGR_W 	unCPU_TRDGRD0
#define PWM_SELECT_W	TRDOER1_D0
#define PWMINTEN0_W		TRDIER0_D0
#define PWMINTEN1_W		TRDIER1_D0
#define DTCEN3_W		DTCEN3_D0
#define DTCEN4_W		DTCEN4_D0

#define unCPU_TRDGR_X 	unCPU_TRDGRB1
#define PWM_SELECT_X	TRDOER1_B1
#define PWMINTEN0_X		TRDIER0_B1
#define PWMINTEN1_X		TRDIER1_B1
#define DTCEN3_X		DTCEN3_B1
#define DTCEN4_X		DTCEN4_B1

#define unCPU_TRDGR_Y 	unCPU_TRDGRC1
#define PWM_SELECT_Y	TRDOER1_C1
#define PWMINTEN0_Y		TRDIER0_C1
#define PWMINTEN1_Y		TRDIER1_C1
#define DTCEN3_Y		DTCEN3_C1
#define DTCEN4_Y		DTCEN4_C1

#define unCPU_TRDGR_Z 	unCPU_TRDGRD1
#define PWM_SELECT_Z	TRDOER1_D1
#define PWMINTEN0_Z		TRDIER0_D1
#define PWMINTEN1_Z		TRDIER1_D1
#define DTCEN3_Z		DTCEN3_D1
#define DTCEN4_Z		DTCEN4_D1

#define SELECT_UVW (PWM_SELECT_U | PWM_SELECT_V| PWM_SELECT_W)																							/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define SELECT_XYZ (PWM_SELECT_X | PWM_SELECT_Y| PWM_SELECT_Z)	/* @007MzDOD[16/12/21] */ /* 差圧起動 */

/* for vPMD_PdcPtnSet */
volatile UI16 * const rest_tbl_head[]
  = {&unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V};
/* DTCスタート用のTRDタイマの選択 */
const UI08 pwmint_en_trd0_dtctbl_head[] = {PWMINTEN0_X, PWMINTEN0_W, PWMINTEN0_Y, PWMINTEN0_U, PWMINTEN0_Z, PWMINTEN0_V};
const UI08 pwmint_en_trd1_dtctbl_head[] = {PWMINTEN1_X, PWMINTEN1_W, PWMINTEN1_Y, PWMINTEN1_U, PWMINTEN1_Z, PWMINTEN1_V};
/* TRDで起動するDTCの許可 */
const UI08 addtct_tbl3_head[] = {DTCEN3_X, DTCEN3_W, DTCEN3_Y, DTCEN3_U, DTCEN3_Z, DTCEN3_V};
const UI08 addtct_tbl4_head[] = {DTCEN4_X, DTCEN4_W, DTCEN4_Y, DTCEN4_U, DTCEN4_Z, DTCEN4_V};

volatile UI16 *const nnxt_phase_sw_tbl_head[]																			/* @007MzDOD[16/12/21] */ /* 差圧起動 */
  = {&unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U};				/* @007MzDOD[16/12/21] */ /* 差圧起動 */
volatile UI16 *const nxt_phase_sw_tbl_head[]																				/* @007MzDOD[16/12/21] */ /* 差圧起動 */
  = {&unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y};				/* @007MzDOD[16/12/21] */ /* 差圧起動 */
volatile UI16 *const crr_phase_sw_tbl_head[]																				/* @007MzDOD[16/12/21] */ /* 差圧起動 */
  = {&unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W};				/* @007MzDOD[16/12/21] */ /* 差圧起動 */
volatile UI16 *const crr_phase_full_tbl_head[]																			/* @007MzDOD[16/12/21] */ /* 差圧起動 */
  = {&unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X};				/* @007MzDOD[16/12/21] */ /* 差圧起動 */
const UI08 cls_mask_full_tbl_head[] = {SELECT_UVW, SELECT_XYZ, SELECT_UVW, SELECT_XYZ, SELECT_UVW, SELECT_XYZ};			/* @007MzDOD[16/12/21] */ /* 差圧起動 */
const UI08 cls_mask_sw_tbl_head[] = {SELECT_XYZ, SELECT_UVW, SELECT_XYZ, SELECT_UVW, SELECT_XYZ, SELECT_UVW};			/* @007MzDOD[16/12/21] */ /* 差圧起動 */

volatile UI16 * const rest_tbl_tail[]
  = {&unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z};
/* DTCスタート用のTRDタイマの選択 */
const UI08 pwmint_en_trd0_dtctbl_tail[] = {PWMINTEN0_V, PWMINTEN0_X, PWMINTEN0_W, PWMINTEN0_Y, PWMINTEN0_U, PWMINTEN0_Z};
const UI08 pwmint_en_trd1_dtctbl_tail[] = {PWMINTEN1_V, PWMINTEN1_X, PWMINTEN1_W, PWMINTEN1_Y, PWMINTEN1_U, PWMINTEN1_Z};
/* TRDで起動するDTCの許可 */
const UI08 addtct_tbl3_tail[] = {DTCEN3_V, DTCEN3_X, DTCEN3_W, DTCEN3_Y, DTCEN3_U, DTCEN3_Z};
const UI08 addtct_tbl4_tail[] = {DTCEN4_V, DTCEN4_X, DTCEN4_W, DTCEN4_Y, DTCEN4_U, DTCEN4_Z};

volatile UI16 *const nnxt_phase_full_tbl_tail[]
  = {&unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U};
volatile UI16 *const nnxt_phase_sw_tbl_tail[]
  = {&unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W, &unCPU_TRDGR_Y};
volatile UI16 *const nxt_phase_sw_tbl_tail[]
  = {&unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X, &unCPU_TRDGR_W};
volatile UI16 *const crr_phase_sw_tbl_tail[]
  = {&unCPU_TRDGR_W, &unCPU_TRDGR_Y, &unCPU_TRDGR_U, &unCPU_TRDGR_Z, &unCPU_TRDGR_V, &unCPU_TRDGR_X};

const UI08 cls_mask_full_tbl_tail[] = {SELECT_XYZ, SELECT_UVW, SELECT_XYZ, SELECT_UVW, SELECT_XYZ, SELECT_UVW};
const UI08 cls_mask_sw_tbl_tail[] = {SELECT_UVW, SELECT_XYZ, SELECT_UVW, SELECT_XYZ, SELECT_UVW, SELECT_XYZ};
volatile UI16 *crr_phase_sw;				/* 現在のオン素子 上										*/
volatile UI16 *nxt_phase_sw;				/* 次のオン素子 上											*/
volatile UI16 *nnxt_phase_sw;				/* 次のオン素子 下											*/
volatile UI16 *nnxt_full;
volatile UI16 *nxt_full;
volatile UI16 *crr_full;
#endif

UI16 unPMDc_PWMprd;				/*PWM周期 */
UI16 unPMDc_PWMprd_calc;

volatile UI16	unPMD_CMPU;							/*U相PWMｵﾝ時間格納変数 */
UI16	unPMD_CMPV;							/*V相PWMｵﾝ時間格納変数 */
//UI16	unPMD_CMPW;							/*W相PWMｵﾝ時間格納変数	@000ahOTM[08/06/05](発覚08/05/21) */
//UI16 unPMD_CMPW_nxt;
UI16 first_half_timer;

UI08 pwmint_enable_trd0;
UI08 pwmint_enable_trd1;
UI08 dtctiming;
const UI08 src_pwm_change_finish = 0x00;  								/* @027MyOTM[17/03/28] */
UI16 addtct_trdgr[2];											/* 実際のA/Dスタートタイミングを格納 DTCで反映させる *//* @015ywDOD[17/12/19] */
		/* ↑本来、volatile宣言をしないと、他のファイルからうまく更新されない可能性あり */
UI16 addtct_trdgr2[2];											/* A/Dスタートタイミングが格納されている */


#ifdef RENESAS_TO_ST32
UI08 IsMdoutPosition1st(void){
	return (ucPMD_mdidx == D_POS1ST_IDX);
}
UI08 IsMdoutPosition2nd(void){
	return (ucPMD_mdidx == D_POS2ND_IDX);
}
#else
UI08 IsMdoutPosition1st(void){
	return (unPMD_MDOUT == D_POS1STPTN);
}
UI08 IsMdoutPosition2nd(void){
	return (unPMD_MDOUT == D_POS2NDPTN);
}
#endif
UI08 IsMdoutStop(void){
	return (unPMD_MDOUT == D_BREAKPTN);
}

void SelectLowArmPolarity(void){
 #if(LOWER_ARM == LO_ACTIVE)
	uc_PMC_p2ptnL_pol = 0xe0;/* 上アームの極性定義に伴い下アームで使っていた変数設定を削除  *//* @008lwORM[24/06/21] */
 #elif (LOWER_ARM == HI_ACTIVE)
	 uc_PMC_p2ptnL_pol = 0x00;/* 上アームの極性定義に伴い下アームで使っていた変数設定を削除  *//* @008lwORM[24/06/21] */
 #endif /* LOWER_ARM */
}

void DisablePwmOut(void){
#ifdef RENESAS_TO_ST32
	htim1.Instance->CCER &= ~PWMOUT_MASK;
#else
	ucCPU_TRDOER1 = 0xff;					/* output disable */
#endif
}

void StopPwmTimer(void){
#ifdef RENESAS_TO_ST32
	htim1.Instance->CR1 &= ~TIM_CR1_CEN;
	htim1.Instance->CNT = 0;
#else
	ucCPU_TRDSTR = 0x0c;					/* Timer RD counter stop */
	unCPU_TRD0 = 0x0000;					/* Timer RD counter 0 */	/* @013MwOTH[10/09/16] */
	unCPU_TRD1 = 0x0000;					/* Timer RD counter 1 */	/* @013MwOTH[10/09/16] */
#endif
}

void StartPwmTimer(void){
#ifdef RENESAS_TO_ST32
	htim1.Instance->CR1 |= TIM_CR1_CEN;
#else
	ucCPU_TRDSTR = 0x0f;					/* Timer RD counter stop */
#endif
}




/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
void ResetFirstHalfTimer(void){
	first_half_timer = D_FIRSTHALF;
}

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
void InitPwm(void){
//	UI32 tmp_bdtr;
	detect_pwm_mode = 1;
	detect_pwm_mode_calc = 1;
	hysteresis_duty_trd_detect_pwm_mode = D_PWMPERIOD / HYSTERESIS_DIV_NUM;
	assigned_flg = 0;
//	unPMD_CMPW_nxt = 0;
	unPMDc_PWMprd = D_PWMPERIOD;						/*起動時のキャリア初期化(2002/12/18-hide) */
	unPMDc_PWMprd_calc = D_PWMPERIOD;
	SelectSwitchingHeadOrTail(SWITCHING_TAIL);

	htim1.Instance->SR &= (~(TIM_SR_BIF | TIM_SR_B2IF));/* BRK割込ﾌﾗｸﾞとBRK2割込ﾌﾗｸﾞをｸﾘｱ */
	htim1.Instance->DIER = TIM_DIER_BIE;				/* BRK機能の有効化 */

}
void SelectSwitchingHeadOrTail(UI08 head_or_tail){
#ifdef RENESAS_TO_ST32
	inv_sw_mode = SWITCHING_TAIL;
#else
	if( head_or_tail == SWITCHING_HEAD ){
		inv_sw_mode = SWITCHING_HEAD;
		rest_tbl_ptr              = rest_tbl_head;				/* 自由に使える相を指定 */
		pwmint_en_trd0_dtctbl_ptr = pwmint_en_trd0_dtctbl_head;
		pwmint_en_trd1_dtctbl_ptr = pwmint_en_trd1_dtctbl_head;
		addtct_tbl3_ptr           = addtct_tbl3_head;			/* dtcの起動許可の指定を格納 */
		addtct_tbl4_ptr           = addtct_tbl4_head;
	}
	else {
		inv_sw_mode = SWITCHING_TAIL;
		rest_tbl_ptr              = rest_tbl_tail;				/* 自由に使える相を指定 */
		pwmint_en_trd0_dtctbl_ptr = pwmint_en_trd0_dtctbl_tail;
		pwmint_en_trd1_dtctbl_ptr = pwmint_en_trd1_dtctbl_tail;
		addtct_tbl3_ptr           = addtct_tbl3_tail;			/* dtcの起動許可の指定を格納 */
		addtct_tbl4_ptr           = addtct_tbl4_tail;
	}
#endif
}
/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
void InitPwmCompareReg(void){
	DisablePwmOut();
#ifdef RENESAS_TO_ST32
	htim1.Instance->ARR = D_PWMPERIOD;
	htim1.Instance->CCR1
	= htim1.Instance->CCR2
	= htim1.Instance->CCR3
	= (D_PWMPERIOD >> 1);
	htim1.Instance->CCR4 = addtct_trdgr[0];
#else
	unCPU_TRDGRA0 = unCPU_TRDGRA1 = D_PWMPERIOD;
	unCPU_TRDGRB0 = unCPU_TRDGRC0 = unCPU_TRDGRD0 = DTC_CMPSTOP;				/* @108MzDOD[14/05/27] */
	unCPU_TRDGRB1 = unCPU_TRDGRC1 = unCPU_TRDGRD1 = (D_PWMPERIOD >> 1);
#endif
}

void SetStandbyDuty(void){ //要検討：ポート出力なので、InitPwmCompareRegで対応できる？ 割込みの発生の確認必要
	DisablePwmOut(); //ポート出力とするためPWM出力しない duty自体に意味はない
#ifdef RENESAS_TO_ST32
	htim1.Instance->CCR1
	= htim1.Instance->CCR2
	= htim1.Instance->CCR3
	= 0;
	htim1.Instance->CCR4 = addtct_trdgr[0];
#else
	unCPU_TRDGRB0 = unCPU_TRDGRC0 = unCPU_TRDGRD0 = DTC_CMPSTOP;	/* ﾌﾞｰﾄｽﾄﾗｯﾌﾟ充電処理(duty:0%) */	/* @108MzDOD[14/05/27] */
	unCPU_TRDGRB1 = unCPU_TRDGRC1 = unCPU_TRDGRD1 = unCPU_TRDGRA0 + 1;	/* ﾌﾞｰﾄｽﾄﾗｯﾌﾟ充電処理(duty:0%) */
#endif
}


/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
void InitPwmCompare(void){
//	unPMD_CMPU = unPMD_CMPV = unPMD_CMPW = unPMD_CMPW_nxt = 1;
	unPMD_CMPU = unPMD_CMPV = 1;
}



/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
UI08 IsPwmFreqVector(void){
#ifdef RENESAS_TO_ST32
	return (htim1.Instance->ARR == D_PWMPERIODfrc);
#else
	return (unCPU_TRDGRA0 == D_PWMPERIODfrc);
#endif
}

/**************************************************************************
<Function>		:通電角に必要なキャリア周波数に変更されたか確認用(倍電圧のみ必要)
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
UI08 IsPwmFreqAngleControl(void){					
	return (htim1.Instance->ARR == D_PWMPERIODfrc);
}

UI08 IsPwmUpdated(void){
	return (unPMD_CMPU == unPMD_CMPV);
}

UI08 GetTRNS_PWMfreq(void){
#if 0 /* 今後の検討で必要になる可能性ありのためｺﾒﾝﾄとして本モジュールを残す */
	switch(unPMDc_PWMprd){
	case D_PWMPERIOD:					/* ﾃﾞﾌｫﾙﾄｷｬﾘｱで動作中？ */
		return D_PWM_FREQ_DEFAULT_COM;	/* ﾃﾞﾌｫﾙﾄｷｬﾘｱ値を格納 	*/

/* D_PWMPERIODと同じため、一旦削除 */
//	case D_PWMPERIODLo:					/* 低ｷｬﾘｱで動作中？ 	*/
//		return D_PWM_FREQ_LOW_COM;		/* 低ｷｬﾘｱ値を格納 		*/

	case D_PWMPERIODHi:					/* 高ｷｬﾘｱで動作中？ 	*/
		return D_PWM_FREQ_HIGH_COM;		/* 高ｷｬﾘｱ値を格納 		*/

	case D_PWMPERIODfrc:				/* 特殊ｷｬﾘｱで動作中？ 	*/
		return D_PWM_FREQ_VECTOR_COM;	/* 特殊ｷｬﾘｱ値を格納 	*/

	default:
		return 0;			/* 0を格納 */
	}
#else
	return (10000 / (unPMDc_PWMprd /D_fTRD));
#endif
}

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
void init_addtct_trdgr(void){
    addtct_trdgr[0] = addtct_trdgr[1] = 1;
    //spike_detection_timing = spike_detection_timing2 = 1;
}




#if 0 /* 今後の検討で必要になる可能性ありのためｺﾒﾝﾄとして本モジュールを残す */

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
static inline  void CorrectNowFullPhase(SI16 correction_time){
    /* �x�^ON���̃^�C�}�R���y�A���v�Z */
    first_half_timer = CalcFirstHalfTimer(correction_time);

    /* �␳���ʂ�TRD�^�C�}���ׂ��ꍇ��(first_half_timer < TRD0 < *crr_full=���̃R���y�A�l)���`�F�b�N */
    if( (*crr_full > unCPU_TRD0) && (first_half_timer < (unCPU_TRD0 + 4*16 + 24)) ){
		/* �t�����̃R���y�A�l��TRD���ׂ� */
        *crr_full = unCPU_TRD0 + 24;        /* �R���y�A��v����ŏ��l���i�[ */
        mWaitCompareTimerFull();			/*TRD����v����܂ő҂�(nop = 24�ɑ���)*/
    }

    /*(���) TRD�����݂̃R���y�A�l�ȏ�ɂȂ�܂���*/

    *crr_full = first_half_timer;           /* �x�^ON���̕␳���ʂ��i�[ */ /* ���̷�ر�ŕ␳�l�����f����ēd�� */
}
#endif

#ifdef UNIT_TEST
void InitDtcPwm(void){
}
void setDTC_CHGCRR(void){
}
#else
void InitDtcPwm(void){
#ifdef RENESAS_TO_ST32
	//STでは、DTCを使ってPWMを変更しない
#else
	/* �L�����A�ύX �� �f���[�e�B�ύX �������� ----------------------------------> */
    /* TRDGRA0��DTC�X�^�[�g
	   �� PWM0=unPMDc_PWMprd_calc �� PWM1=unPMDc_PWMprd_calc �� CMPU=CMPV �� *crr_phase_sw=CMPU �� *nxt_phase_sw=CMPU �� *nnxt_phase_sw=CMPU
        �� addtct_trdgr[0]=addtct_trdgr2[0],addtct_trdgr[1]=addtct_trdgr2[1] �� */
	/* SetDtc(DTC_������, DTC�̃��[�h�I��, �]��Byte��, �J��Ԃ���, (UI16*)(&�]����), (UI16*)(&�]����)); */
	SetDtcChangeDuty(DTC_CHGCRR0,   0x10, 2, (UI16*)(&unPMDc_PWMprd_calc),    (UI16*)(&unCPU_TRDGR_PWM0)); /* PWM0�̎����ύX */	/* �`�F�C���]�� *//* �]��2byte *//* 1��̂� *//* �ύX�p�L�����A�� *//* PWM0�^�C�}���W�X�^�ɏ������� */
	SetDtcChangeDuty(DTC_CHGCRR1,   0x10, 2, (UI16*)(&unPMDc_PWMprd_calc),    (UI16*)(&unCPU_TRDGR_PWM1)); /* PWM1�̎����ύX */	/* �`�F�C���]�� *//* �]��2byte *//* 1��̂� *//* �ύX�p�L�����A�� *//* PWM1�^�C�}���W�X�^�ɏ������� */
	SetDtcChangeDuty(DTC_CHGCMPU,   0x10, 2, (UI16*)(&unPMD_CMPV),            (UI16*)(&unPMD_CMPU));       /* �o�͗pDuty�̎Q�Ɨp�̕ϐ��Ɍv�Z���ʂ��i�[ */
	SetDtcChangeDuty(DTC_CHGCRSW,   0x10, 2, (UI16*)(&unPMD_CMPU),            (UI16*)(crr_phase_sw));      /* ���݂�Duty�Ɋi�[ */
	SetDtcChangeDuty(DTC_CHGNXSW,   0x10, 2, (UI16*)(&unPMD_CMPU),            (UI16*)(nxt_phase_sw));      /* �����Duty�Ɋi�[ */
#ifdef FIRST_HALF_SW
	SetDtcChangeDuty(DTC_CHGNNSW,   0x10, 2, (UI16*)(&unPMD_CMPU),            (UI16*)(nnxt_phase_sw));     /* ���X���Duty�Ɋi�[ */
#else
	SetDtcChangeDuty(DTC_CHGNNSW,   0x10, 2, (UI16*)(&first_half_timer),      (UI16*)(nnxt_phase_sw));
#endif
	SetDtcChangeDuty(DTC_CHGTRDGR,  0x10, 4, (UI16*)(addtct_trdgr2),          (UI16*)(addtct_trdgr));      /* A/D���o�X�^�[�g�^�C�~���O�̌v�Z���ʂ��Q�Ɨp�ϐ��Ɋi�[ */
#if DETECT_PWM_OFF == ACTIVE
	SetDtcChangeDuty(DTC_SPKDTCTTM, 0x10, 1, (UI16*)(&detect_pwm_mode_calc),       (UI16*)(&detect_pwm_mode));
#endif
	SetDtcChangeDuty(DTC_SETPWMFLG, 0x00, 1, (UI16*)(&src_pwm_change_finish), (UI16*)(&call_setpwm_flg));

	/* DTC起動時のDTCテーブル呼び出し設定 */
	RegistDtcChangeDuty();
#endif
}


void setDTC_CHGCRR(void){
#ifdef RENESAS_TO_ST32
//aとりあえず無し　必要ならshadowmodeを利用
	htim1.Instance->ARR = unPMDc_PWMprd_calc;
	*crr_phase_sw = unPMD_CMPV;
	if(ePMD_drvmode > D_forced){
		*nxt_phase_sw = unPMD_CMPV;
	} else {
		*crr_full = first_half_timer;
	}
	htim1.Instance->CCR4 = addtct_trdgr2[0];
	addtct_trdgr[0] = addtct_trdgr2[0];
	addtct_trdgr[1] = addtct_trdgr2[1];
	unPMD_CMPU = unPMD_CMPV;
	call_setpwm_flg = 0;
#else
	DisableDtcChangeCarrier();

	ResetDtcCountChangeDuty();
#ifdef FIRST_HALF_SW
#else
#endif

//    ucCPU_TRDIER0 &= ~0x01;												/* PWM周期割込禁止*/
	vPMD_ChangeCarrierDisable();
	ucCPU_TRDSR0;
    ucCPU_TRDSR0 &= 0xfe;												/* status resister CLR after dummy read before enable interrupt */

	EnableDtcChangeCarrier();

	ucCPU_TRDIER0 |= 0x01;												/* PWM周期割込許可*/
#endif
}
#endif


/**************************************************************************
<Title>			:unsigned8 unPMDc_3kjdg()				:unPMDc_SetPWM
<Function>		:PWM ｷｬﾘｱ3k強制切替設定
<Description>	:	SBS以外の機種で	@回転数30未満切替り～指令回転数到達迄		 3kHZ
									A回転数30未満指令回転到達以降 duty≦25%	 3kHZ
									B回転数30未満指令回転到達以降 duty≧30%	10kHZ
									C回転数30以上								10kHZ
					※　SBS以外の機種で呼出される関数
<Input>			:buf(ucSPD_rpmfgmd.b0-b3), ucSPD_rpmh, unPMDc_dutyn
<Return>		:ucTRNS_3kflag
<Author>		:-Outi	作成
<Date>			:2014-0516
<Update>		:Ver C32WPG       2014-09-10 -Mura  キャリア音クレーム対応  @018awDOD[14/05/16]
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void unPMDc_3kjdg(unsigned8 buf){							/*PWMｷｬﾘｱ周期設定処理(通常) *//* @018awDOD[14/05/16] */		/* @041MzDOD[15/07/23] */

	if(ucSPD_rpmh < 30)										/*駆動回転数30回転未満 */ /* @018awDOD[14/05/16] */
	{
		if(buf < 3)											/*回転数偏差＞1回転 *//* @018awDOD[14/05/16] */
		{
			ucTRNS_3kflag = 1;								/*ｷｬﾘｱ強制flag←SET *//* @018awDOD[14/05/16] */
		}else												/*回転数偏差≦1回転 *//* @018awDOD[14/05/16] */
		{
			if(unPMDc_dutyn <= 250)							/*duty≦25% *//* @018awDOD[14/05/16] */
			{
				ucTRNS_3kflag = 1;							/*ｷｬﾘｱ強制flag←SET *//* @018awDOD[14/05/16] */
			}else if(unPMDc_dutyn >= 300)					/*duty≧30% *//* @018awDOD[14/05/16] */
			{
				ucTRNS_3kflag = 0;							/*ｷｬﾘｱ強制flag←CLR *//* @018awDOD[14/05/16] */
			}else{}											/*25%＜duty＜30% flag維持*//* @018awDOD[14/05/16] */
		}
	}else													/*駆動回転数30回転以上∨SBS *//* @018awDOD[14/05/16] */
	{
		ucTRNS_3kflag = 0;									/*ｷｬﾘｱ強制flag←CLR *//* @018awDOD[14/05/16] */
	}
}

/**************************************************************************
<Title>			:unsigned16 unPMDc_SetPWM_nrm			:unPMDc_SetPWM
<Function>		:PWM ｷｬﾘｱ切替設定(通常)
<Description>	:	起動Seq.&ﾘﾚｰoff中: 3kHzD_PWMPERIODLo／RYon中: 10kHzD_PWMPERIODHi／温度上昇対応仕様: 3kHz
																	※SBSは5.7kHz
<Input>			:buf(ucSPD_rpmfgmd.b0-b3), unsigned16 unPMDc_GetPWMprd()
<Return>		:unPWMprd
<Author>		:-Outi	作成
<Date>			:2014-0516
<Update>		:Ver REF140516-0.01 2014-05-16 -Outi unPMDc_SetPWM(ｷｬﾘｱ設定)を分割	@018awDOD[14/05/16]
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
unsigned16 unPMDc_SetPWM_nrm(unsigned8 buf, unsigned16 unPWMPrd){		/*PWMｷｬﾘｱ周期設定処理(通常) *//* @018awDOD[14/05/16] */	/* @041MzDOD[15/07/23] */
	if(((ucTRNS_sfrg & 0x08) != 0) && (ucTYPE_uart != 0)){	/*【ｷｬﾘｱ切替ﾌﾗｸﾞ≡Ｈ】なら */
		if(unPWMPrd == unPWMPeriodHi){			/*高速時のキャリアの場合                              *//* @250swDOD[23/05/18] */
			if(buf >= 3){						/*回転数偏差が小さければ                              */
				unPWMPrd = D_PWMPERIODLo;		/*低速キャリアをセット*/	/* 午後 05:37 REF045-4.00 */
			}else{								/* 午後 05:36 REF045-4.00 */
				unPWMPrd = unPWMPeriodHi;		/*回転数偏差が大きければ高速キャリアを保持            *//* @250swDOD[23/05/18] */
			}									/* 午後 05:36 REF045-4.00 */
		}else{									/* 午後 05:36 REF045-4.00 */
			unPWMPrd = D_PWMPERIODLo;			/*それ以外（RAM化け)時は低速ｷｬﾘｱセット                */
		}										/* 午後 05:36 REF045-4.00 */
	}else if( ucTYPE_uart == 0 ){				/*default */
		if(unPWMPrd != unPWMPeriodHi){			/*高速時キャリア以外の時 *//* @250swDOD[23/05/18] */
			if(ucPMD_boostmd >= 3){				/*回転数偏差が小さければ */	/* 午後 05:35 REF045-4.00 */	/* @006axOOM[09/05/11] */
					unPWMPrd = unPWMPeriodHi;	/*高速キャリアをセット *//* @250swDOD[23/05/18] */
			}else{								/* 午後 05:35 REF045-4.00 */
					unPWMPrd = D_PWMPERIODLo;	/*回転数偏差が大きければ低速キャリアで保持 */
				}								/* 午後 05:35 REF045-4.00 */
			}
		else{	}								/*それ以外(高速時)は未処理 */ /*＃ 午前 08:55 REF061-0.00 */
	}else{										/*【キャリア周波数選択通常】or【ｺﾝﾌﾟ保護ﾌﾗｸﾞ≡Ｌ】なら ﾘｱｸﾀ対策  午前 09時58分30秒*/
		if(unPWMPrd != unPWMPeriodHi){			/*高速時キャリア以外の時 */																/* @041MzDOD[15/07/23] */
			if(ucPMD_boostmd >= 3){				/*回転数偏差が小さければ */	/* 午後 05:35 REF045-4.00 */	/* @006axOOM[09/05/11] */
				unPWMPrd = unPWMPeriodHi;		/*高速キャリアをセット */					/* @115MzDOD[14/06/09] *//* @250swDOD[23/05/18] */
			}else{																		/* @115MzDOD[14/06/09] */
				unPWMPrd = D_PWMPERIODLo;	/*回転数偏差が大きければ低速キャリアで保持 */		/* @115MzDOD[14/06/09] */
			}																			/* @115MzDOD[14/06/09] */
		}																																/* @041MzDOD[15/07/23] */
		else{	}								/*それ以外(高速時)は未処理 */ /*＃ 午前 08:55 REF061-0.00 */									/* @043MgORM[15/08/19] */
	}/*~03-0806 -Outi ﾌﾞｰｽﾄ制御後常時10kHz仕様(騒音対応) 										  */
	return unPWMPrd;							/*決定したキャリア周波数を返す */
}
/**************************************************************************
<Title>			:unsigned16 unPMDc_GetPWMprd 		;unPMDc_SetPWM
<Function>		:ｷｬﾘｱ周期取得処理
<Description>	:	外部ﾓｼﾞｭｰﾙから本ﾓｼﾞｭｰﾙが呼び出され、ｷｬﾘｱ周期をｵｳﾑ返しに出力
<Input>			:
<Return>		:unPMDc_PWMprd
<Author>		:-hide	作成
<Date>			:2002-1217
<Update>		:Ver REF036-0.01 2002-12-18 -hide ｷｬﾘｱ切替仕様
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
unsigned16 unPMDc_GetPWMprd(void) {			/*PWMｷｬﾘｱ周期取得 */
	return (unPMDc_PWMprd);					/* */
}

void CalcHysteresisDutyDetectPwmMode(UI16 pwm_period_trd){
    hysteresis_duty_trd_detect_pwm_mode = pwm_period_trd / HYSTERESIS_DIV_NUM;
}

/**************************************************************************
<Title>			:SelectBaseCarrier(UI08 ucTYPE_carrier_freq) 
<Function>		:基本キャリア周波数の選択
<Description>	:	一定周期["unPMD_basect"or"20ﾟe(INTTMR1,INTTMR4)"]毎に本関数がCallされDuty更新
<Input>			:ucTYPE_carrier_freq
<Return>		:unPWMPeriodHi
<Author>		:Takida	作成  @250swDOD[23/05/18] 
<Date>			:2023-0519
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
void SelectBaseCarrier(UI08 basecarrier_flg){	/* @258swRRD[23/06/27] */
	if(basecarrier_flg == 0){					/* @258swRRD[23/06/27] */
		unPWMPeriodHi = D_PWMPERIODHi0;	/* 10kHzを選択 */
	}else{
		unPWMPeriodHi = D_PWMPERIODHi1;	/* 6.5kHzを選択 */
	}
}

void SetPwmCarrier(enum eCarrierModeTag mode){
	UI16 unPWMPrd;
	switch(mode){
	case kCarrierInit:
	default:
		unPWMPrd = D_PWMPERIOD;
		break;
	case kCarrierLowSpeed:
		unPWMPrd = D_PWMPERIODLo;
		break;
	case kCarrierHighSpeed:
		unPWMPrd = unPWMPeriodHi;			/* @250swDOD[23/05/18] */
		break;
	case kCarrierMax:
		unPWMPrd = D_PWMPERIODfrc;
	}
	CalcHysteresisDutyDetectPwmMode(unPWMPrd);
	unPMDc_PWMprd = unPWMPrd;
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
				:
<Copyright> 	:
 **************************************************************************/
/* 各運転モードにおけるPWM・ポート設定 */
static inline  void SetPwmCompareStop(void){												/* @007MzDOD[16/12/21] */ /* 差圧起動 */
	crr_phase_sw   = nxt_phase_sw = nxt_phase_sw_tbl_tail[ucPMD_mdidx];
	nxt_full = nnxt_phase_sw_tbl_tail[ucPMD_mdidx];
	crr_full = nxt_phase_sw;
#ifdef RENESAS_TO_ST32
    SelectPwmPort();
#else
    SelectPwmPort(0x00); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    SelectOutPort(0x00);
#endif
}

static inline  void SetPwmCompareStandby(void){											/* @007MzDOD[16/12/21] */ /* 差圧起動 */
	unCPU_TRDGR_X = unCPU_TRDGR_Y = unCPU_TRDGR_Z = unCPU_TRDGR_PWM0 + 1;
	crr_phase_sw   = nxt_phase_sw = nxt_phase_sw_tbl_tail[ucPMD_mdidx];
	nxt_full = nnxt_phase_sw_tbl_tail[ucPMD_mdidx];
	crr_full = nxt_phase_sw;
#ifdef RENESAS_TO_ST32
    SelectPwmPort();
#else /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    SelectPwmPort(0x00);
    SelectOutPort(0xe0);
#endif
}



static inline  void SetPwmComparePosition(void){											/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#ifdef RENESAS_TO_ST32
	//indexクスは2ndを採用 　PWM出力かどうかはMDOUTで制御
		nnxt_phase_sw =                crr_phase_sw_tbl_tail[D_POS2ND_IDX];
		crr_phase_sw  = nxt_phase_sw = nnxt_phase_sw_tbl_tail[D_POS2ND_IDX];
		crr_full      = nxt_full     = nxt_phase_sw_tbl_tail[D_POS2ND_IDX];  //Positionでは次のp痰を採用する必要あり転流パタンを採用のため

	*nxt_phase_sw = *nnxt_phase_sw = unPMD_CMPU; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	*nxt_full = first_half_timer = unCPU_TRDGR_PWM0 + 1 - OFFTIME;

    SelectAllPwmPort();
#else
    SelectPwmPort(0xee);
    SelectOutPort(0x00);
#endif
}

/*強制転流用PWM出力設定*/
static inline  void SetPwmCompareForced(void){					/* @007MzDOD[16/12/21] */ /* 差圧起動 */
	nnxt_phase_sw = nnxt_phase_sw_tbl_tail[ucPMD_mdidx];
	nxt_phase_sw  = nxt_phase_sw_tbl_tail[ucPMD_mdidx];		/* 上相のデューティ幅を設定するレジスタのアドレスを格納する */
	crr_phase_sw  = crr_phase_sw_tbl_tail[ucPMD_mdidx];
	*crr_phase_sw = unPMD_CMPU;

	nxt_full = nnxt_phase_sw;
	crr_full = nxt_phase_sw; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	*crr_full = first_half_timer;
#ifdef RENESAS_TO_ST32
    SelectAllPwmPort();
#else
	*nxt_full = first_half_timer;
    SelectPwmPort(0xee);
    SelectOutPort(0x00);
#endif
}

/* Running用設定PWM出力設定*/
static inline  void SetPwmCompareDefault(void){  // crr_sw と xxx_full他のCompareXXXと違って変更なし
    extern UI08 call_setpwm_flg;

	if( IsMdoutStop() ){return;}
#ifdef FIRST_HALF_SW
    nxt_phase_sw = nxt_phase_sw_tbl_head[ucPMD_mdidx];
	nnxt_phase_sw  = nnxt_phase_sw_tbl_head[ucPMD_mdidx];
 /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    *nnxt_phase_sw	= unPMD_CMPU;
	*nxt_phase_sw   = unPMD_CMPU;

    SelectPwmPort(cls_mask_sw_tbl_head[ucPMD_mdidx]);        /* 上下相PWM出力はModeindexで切替		*/
    SelectOutPort(cls_mask_full_tbl_head[ucPMD_mdidx]);      /* 上下相のﾎﾟｰﾄ出力はModeindexで切替	*/
#else
	nxt_phase_sw = nxt_phase_sw_tbl_tail[ucPMD_mdidx];
	nnxt_phase_sw  = nnxt_phase_sw_tbl_tail[ucPMD_mdidx];

    *nnxt_phase_sw	= unPMD_CMPU;
	*nxt_phase_sw   = unPMD_CMPU;

#ifdef RENESAS_TO_ST32
	SelectPwmPort();
#else
    SelectPwmPort(cls_mask_sw_tbl_tail[ucPMD_mdidx]);        /* 上下相PWM出力はModeindexで切替		*/
    SelectOutPort(cls_mask_full_tbl_tail[ucPMD_mdidx]);      /* 上下相のﾎﾟｰﾄ出力はModeindexで切替	*/
#endif
#endif
}

/**************************************************************************
<Title>			:PWM_wait 		;vPMD_Brake(),vPMD_start(),vPMDc_TC1(),vPMDi_Setcomtptn(),INTTMR3(),INTPDC(),INTTMR1()
<Function>		:信号ptn(駆動信号&検出信号ptn)設定処理
<Description>	:駆動出力portのラッチ(PWM信号ラッチ含む)にptnを転送
<Input>			:unPMD_MDOUT
<Return>		:ucPMD_p2ptn,ucPMD_NowMDOUTpd
<Author>		:-Outi	作成
<Date>			:2008-0411
<Update>		:Ver REF081-0.00 2008-04-11
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void PWM_wait(void){
    /* 関数ポインタは廃止 */
    /* switchのコンパイル結果がテーブル化されるため関数ポインタと同等、更にINLINE化できるためこちらの方が高速 */
    /* コンパイラ変更時はオプション確認 */

    /* 上下相 PWM出力-ﾎﾟｰﾄ出力設定 							*/
    switch(ePMD_drvmode){
    case D_stop:
        SetPwmCompareStop();
        return;

    case D_standby:
        SetPwmCompareStandby();
        return;

    case D_position:
        SetPwmComparePosition();
        return;

    case D_forced:
        SetPwmCompareForced();	/* (Forced時→上下相PWM設定)	*/
        return;

    case D_running:
    case D_vector:
        SetPwmCompareDefault();	/* (Running時→片相 ﾎﾟｰﾄ設定)	*/
        return;

    default:
        return;
    }
}

static inline   void SelectDetectPwmModeCalc(UI16 duty_trd){
//    static UI08 duration_dtect_pwm_mode;
//    if ( duration_dtect_pwm_mode < 20 ) {
//        duration_dtect_pwm_mode++;
//        return;
//    }
//    duration_dtect_pwm_mode = 0;
#if DETECT_PWM_OFF == ACTIVE /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    if( ePMD_drvmode != D_running ){  //電流抑制で上下ともOFFとなると誘起電圧の状態が変わるため検出不可となるため　起動時もPWM OFF検出するためには工夫が必要
        detect_pwm_mode_calc = POS_DETECT_PWM_ON;
    } else if(IsAngleLapOrGap()){
        detect_pwm_mode_calc = POS_DETECT_PWM_ON;
    } else if( duty_trd >= (unPMDc_PWMprd_calc - OFF_DETECT_REQ_OFF_TM_TRD) ){ //pwm_off time <= sampling time   //左記コメントでそのまま計算すると100％duty(unBuf_trd0gr==unPMDc_PWMprd_calc+1)でunderflow発生のため順番入れ替え
        detect_pwm_mode_calc = POS_DETECT_PWM_ON;
    } else if( duty_trd <= (unPMDc_PWMprd_calc - (OFF_DETECT_REQ_OFF_TM_TRD + hysteresis_duty_trd_detect_pwm_mode)) ) { //HYSTERESIS == duty_5percent_trd
        detect_pwm_mode_calc = POS_DETECT_PWM_OFF;
    } else {
        //右の期間は現状維持 (OFF_DETECT_REQ_OFF_TM_TRD) < (unPMDc_PWMprd_calc - unBuf_trd0gr) < (OFF_DETECT_REQ_OFF_TM_TRD + HYSTERESIS_TRD)
    }
#else
    detect_pwm_mode_calc = POS_DETECT_PWM_ON;
#endif
}
static inline   UI16 Duty2TimerCompare(UI16 unDuty){
	UI16 unBuf_trd0gr;
	if(unDuty >= D_DUTYFULL){
	    return unPMDc_PWMprd_calc + 1;				//100%の場合、キャリア周期+1が設定値。周期とPWM Dutyのタイマ設定値が一致すると正常に出力されない仕様のため
	}

	unBuf_trd0gr = (UI16)((UI32)unDuty * unPMDc_PWMprd_calc / D_DUTYFULL);
//		if( ucPMD_boostmd == 0 ){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
//			unBuf_trd0gr += OFFTIME; //既知の不具合　+offtimeしてunBuf_trd0gr == unPMDc_PWMprd_calcとなると不具合発生の可能性あり。unBuf_trd0gr+OFFTIME＜unPMDc_PWMprd_calcが必要だがboostmd=0では実質問題ないためリミッタは設けていない
//		}
	if( unBuf_trd0gr < D_PWMONMIN ){						/* ｵﾝ時間＜ringing除去+10us(下限) */
		unBuf_trd0gr = D_PWMONMIN;
	}
	return unBuf_trd0gr;
}
static inline   UI16 CalcSampling2nd(UI16 duty_trd){
#if DETECT_PWM_OFF == ACTIVE
	if( detect_pwm_mode_calc == POS_DETECT_PWM_OFF ) {
//		unPMD_CMPW = unPMDc_PWMprd_calc - D_POS_DTECT_TM_TRD;			// 位置検出用ADスタートタイミング PWMオフ検出ではON直前に固定
	    return (unPMDc_PWMprd_calc - TIMERD_ADDRV);
		// PN間電圧検出はDuty変更DTCと重なる可能性あるが、処理の遅れは問題なし
		// Duty変更DTCが重なることで、Dutyの値を変更するResetDTC()との重なりがなくなる
	} /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#endif

	//detect_pwm_mode_calc == POS_DETECT_PWM_ON
	//2回目のADスタート時間の設定
	//2回サンプリングは高速低キャリアで運転には必須だが、低速での利用はノイズのある電圧と無い電圧の検出が混在するため速度バラツキの要因となる
	if( duty_trd >= unPMDc_PWMprd_calc - D_POS_DTECT_TM_TRD + TIMERD_ADALL ){ //PWM off time <= PWM ON 以外で必要なAD変換処理時間
		return unPMDc_PWMprd_calc - D_POS_DTECT_TM_TRD;
	}
	return duty_trd - TIMERD_ADALL;
}
static inline   UI16 CalcSampling1st(UI16 sampl_2nd){
#if DETECT_PWM_OFF == ACTIVE
	if( detect_pwm_mode_calc == POS_DETECT_PWM_OFF ){
		return sampl_2nd;                                      // シングルサンプリングなのでCMPWのタイミングをセット
	}
#endif
	if( sampl_2nd < D_2SMPL_TIME_TRD ){
		return sampl_2nd;                                      // シングルサンプリングなのでCMPWのタイミングをセット /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	}
	return (sampl_2nd >> 1);                                      // PWMオフ検出ではダブルサンプリング実施しない
}
/**************************************************************************
<Title>			:vPMDc_Updateduty(unsigned16 unr_duty) 		;vPMDc_TC1,INTTMR1,INTTMR4
<Function>		:Duty更新処理
<Description>	:	一定周期["unPMD_basect"or"20ﾟe(INTTMR1,INTTMR4)"]毎に本関数がCallされDuty更新
<Input>			:unDuty(unPMDc_dutyn), unPMDc_PWMprd
<Return>		:unPMD_CMPU
<Author>		:-Outi	作成
<Date>			:2002-0125
<Update>		:Ver REF02A-0.00 2002-06-03
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.01 2008-06-05 -Outi 信号検出処理修正	@000ahOTM[08/06/05](発覚08/05/21)
				:Ver REF090-0.04 2009-06-18 -Outi 母線電圧に応じた起動duty設定仕様変更 @017axOTT[09/06/18]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vPMDc_UpdateDuty(unsigned16 unDuty) {					/*dutyタイマとPWM周期のタイマ値の計算処理 */
	UI16 unBuf_trd0gr;
	UI16 smpl_1st;
	UI16 smpl_2nd;

	if( call_setpwm_flg != 0 ){return;}

	//キャリア
	unPMDc_PWMprd_calc = unPMDc_PWMprd;						// PWM update
//	if( ucPMD_boostmd == 0 ){								/* 位置検出禁止中 */
//		unCPU_TRDGRA1 = unCPU_TRDGRA0 = unPMDc_PWMprd_calc;	/* carrier frequency */
//	}

	//duty[trd]の計算
	unBuf_trd0gr = Duty2TimerCompare(unDuty);
	// 要検討：モードの切替は3.3msでも良いが、Dutyの更新と同期が必要
    SelectDetectPwmModeCalc(unBuf_trd0gr);


	//AD�X�^�[�g�^�C�~���O�v�Z
    smpl_2nd = CalcSampling2nd(unBuf_trd0gr);
    smpl_1st = CalcSampling1st(smpl_2nd);

	//DTC用変数に反映
#ifdef RENESAS_TO_ST32
#else
	mDisableInterrupt();								// 要検討：call_setpwm_flgがあるので不要？ /*割込禁止 */
#endif
	unPMD_CMPV = unBuf_trd0gr;		// PWM duty タイマ設定値
    addtct_trdgr2[0] = smpl_1st;
	addtct_trdgr2[1] = smpl_2nd;

#if TRQ_CTL == INACTIVE
	mEnableInterrupt();								/*割込許可 */
#endif /* TRQ_CTL */
#ifdef RENESAS_TO_ST32
	if( (htim1.Instance->ARR != unPMDc_PWMprd_calc) || (unPMD_CMPU != unPMD_CMPV) ){		/* @042MzDOD[15/07/27] */
#else
	if( (unCPU_TRDGRA0 != unPMDc_PWMprd_calc) || (unPMD_CMPU != unPMD_CMPV) ){		/* @042MzDOD[15/07/27] */
#endif
		call_setpwm_flg = 1;														/* @042MzDOD[15/07/27] */
    }																				/* @042MzDOD[15/07/27] */
}
void vPMDc_UpdateDutyPdcless(UI16 unDuty){
	unsigned16 unBuf_trd0gr;											/*ﾃﾝﾎﾟﾗﾘ(16bit)の準備 */
	unPMDc_PWMprd_calc = unPMDc_PWMprd;

	unBuf_trd0gr = (UI16) ((UI32) unDuty * unPMDc_PWMprd_calc / 1000);
	mDisableInterrupt(); /* �����֎~ */
	unPMD_CMPU = unPMD_CMPV = unBuf_trd0gr + OFFTIME;
	addtct_trdgr2[1] = addtct_trdgr2[0] = addtct_trdgr[1] =	addtct_trdgr[0] = unPMD_CMPV;
#ifdef RENESAS_TO_ST32
	htim1.Instance->ARR = unPMDc_PWMprd_calc; /* carrier frequency */
#else
	unCPU_TRDGRA1 = unCPU_TRDGRA0 = unPMDc_PWMprd_calc; /* carrier frequency */
#endif
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/09/01
<Update>	:
**************************************************************************/
void AddInitPwmTimer(void) {
	uint32_t ccer_disable;
	uint32_t ccer_tmp;
	uint32_t ccmr2_tmp;
	uint32_t cr_tmp;
//	uint32_t bdtr_tmp;
//	uint32_t af_tmp;

	//PWM出力を一旦OFF
	ccer_disable = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E
			| TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
	ccer_tmp = htim1.Instance->CCER;
	htim1.Instance->CCER = (ccer_tmp & ~ccer_disable);

	//ADスタート用のタイマ設定
	ccmr2_tmp = htim1.Instance->CCMR2; /*  */
	ccmr2_tmp = (ccmr2_tmp & ~TIM_CCMR2_CC4S_Msk)  ; /* ｃｃ設定：出力 */
	ccmr2_tmp = ((ccmr2_tmp & ~TIM_CCMR2_OC4FE_Msk) | TIM_CCMR2_OC4FE); /* compare match のみ レベル判定だと変更後にOFFの状態を拾う可能性があるため */
	ccmr2_tmp = (ccmr2_tmp & ~TIM_CCMR2_OC4PE_Msk)  ; /* プリロード無し */
	ccmr2_tmp = ((ccmr2_tmp & ~TIM_CCMR2_OC4M_Msk) | CC4_PWM)  ; /* PWM1モード */
	htim1.Instance->CCMR2 = ccmr2_tmp  ; /* 転流時のレジスタ設定のプリロード許可 */


	// ADスタート用タイマをADトリガに設定
	cr_tmp = htim1.Instance->CR2;
	cr_tmp =  ((cr_tmp & ~TIM_CR2_MMS2_Msk) | (TIM_CR2_MMS2_2 | TIM_CR2_MMS2_1 | TIM_CR2_MMS2_0));
	htim1.Instance->CR2 = cr_tmp;

	/*---------- TIM1機能設定（外部割込み設定） ---------->*/
	/* MX_TIM1_Init()内で設定済みのためここで設定しない */
	/* InitPwm()内でBRK機能の有効化、vPMD_Brake()内でBRK機能の無効化の処理実施 */
#if 0 /* 過去の検討結果を残す意図でｺﾒﾝﾄとして本モジュールを残す */
	/*---BRK1---*/
	af_tmp = TIM1_AF1_BKINE;		/* BKINは有効 */
	af_tmp &= ~TIM1_AF1_BKCMP1E;	/* 不足電圧保護は無効 */
	af_tmp &= ~TIM1_AF1_BKCMP2E;	/* 過電圧保護は無効 */
	af_tmp &= ~TIM1_AF1_BKINP;		/* BKIN low active */
	af_tmp &= ~TIM1_AF1_ETRSEL_Msk;
	htim1.Instance->AF1 = af_tmp; /* BKIN low active AF1.BKINP=0 <- bdtr.bkp = 0(active low) */
	/*---BRK2---*/
	af_tmp = htim1.Instance->AF2;
	af_tmp &= ~TIM1_AF2_BK2INE;		/* BKINは無効 */
	af_tmp |= TIM1_AF2_BK2CMP1P;	/* COMP1 low acive (=not invert) */
	af_tmp |= TIM1_AF2_BK2CMP1E;	/* 不足電圧保護は有効が初期状態 */
	af_tmp |= TIM1_AF2_BK2CMP2P;	/* COMP2 low acive (=not invert) */
	af_tmp &= ~TIM1_AF2_BK2CMP2E;	/* 過電圧保護は無効が初期状態 *//* ※過電圧入力不定の場合に保護がかかってしまうため */
									/* 過電圧保護はucTYPE_global決定後に有効/無効の設定実施する */
	htim1.Instance->AF2 = af_tmp;	/* COMP1 COMP2 low acive (=not invert) */

	
	bdtr_tmp = htim1.Instance->BDTR;
	bdtr_tmp = (bdtr_tmp | (TIM_BDTR_BK2P | TIM_BDTR_BK2E));  // True で break2 動作  を許可  COMP1とCOMP2のbreak入力の ORが出力
	htim1.Instance->BDTR = bdtr_tmp;
//	htim1.Instance->DIER = TIM_DIER_BIE;
#endif
	/*<---------- TIM1機能設定（外部割込み設定） ----------*/
	
	htim1.Instance->SR &= (~(TIM_SR_BIF | TIM_SR_B2IF));/* BRK割込ﾌﾗｸﾞとBRK2割込ﾌﾗｸﾞをｸﾘｱ */
//	htim1.Instance->SR &= (~TIM_SR_BIF);	/* BRK割込ﾌﾗｸﾞ割込ﾌﾗｸﾞをｸﾘｱ */
	
	htim1.Instance->CR2 |= TIM_CR2_CCPC ; /* 転流時のレジスタ設定のプリロード許可 */

	//indexに見合ったPWMを設定
	PresetPwmOut(D_STOP_INDEX);

}


#ifdef UNIT_TEST
UI16 GetPwmOnMin(void){                 return D_PWMONMIN;                  }
UI16 GetOffDetectReqOffTimeTrd(void){   return OFF_DETECT_REQ_OFF_TM_TRD;   }
UI16 GetHysterisysDuty(void){           return HYSTERESIS_DUTY;             }
UI16 GetAdConversionTime(void){         return D_POS_DTECT_TM_TRD;          }
//UI16 GetAdConversion3Times(void){       return TIMERD_AD3;                   }
UI16 GetAdConversion3Times(void){       return TIMERD_ADDRV;                   } /* AD3回分の時間ではないので注意 */
UI16 GetDoubleSamplingMinTime(void){    return D_2SMPL_TIME_TRD;            }
UI16 GetDefaultNonPwmDutyInBoost(void){ return D_FIRSTHALF;                 }
UI16 GetTrdCompareStop(void){           return DTC_CMPSTOP;                 }
UI16 GetDefaultPwmPeriod(void){         return D_PWMPERIOD;                 }
UI16 GetForcedHighRpsPwmPeriod(void){   return D_PWMPERIODfrc;              }
UI16 GetLowRpsPwmPeriod(void){          return D_PWMPERIODLo;               }
UI16 GetHighRpsPwmPeriod(void){         return D_PWMPERIODHi;               }
UI16 GetTorquCtrlPwmPeriod(void){       return D_PWMPERIODLo;              }
UI16 GetTorquCheckPwmPeriod(void){      return D_PWMPERIODHi;              }
#undef static inline
#endif
