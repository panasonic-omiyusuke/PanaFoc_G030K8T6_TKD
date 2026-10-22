/****************************************************************************/
/*                                                                          */
/*      macro definition                                                    */
/*                                                                          */
/****************************************************************************/
#ifndef COM_DEF_H_
#define COM_DEF_H_

#include "vipdc.h"
#include "stm32g0xx_hal_def.h"

#define    limit(a,b)        a=(a>(b))?(b):a;a=(a<(-(b)))?(-(b)):a
#define    ShtR(equ, bit)    (0 < (bit)) ?  ((equ)>>(bit)) : ((equ)<<(0-(bit)))
#define    ShtL(equ, bit)    (0 < (bit)) ?  ((equ)<<(bit)) : ((equ)>>(0-(bit)))
/* #define    mMDTM2TRC(t)	((t) * 2)    0のときアンダーフロー対策 ⇒ タイマが異常に大きくなる不具合発生 */

#ifdef RENESAS_TO_ST32
#define    mMDTM2TRC(mode_time)	(mode_time)
#define    mTRC2MDTM(trc_time)	(trc_time)
#define    mTRD2MDTM(trd_time)	((trd_time) >> 5)
#define    mTRD2TRC(trd_time)	((trd_time) >> 5)
#define    mDisableInterrupt()   __disable_irq()
#define    mEnableInterrupt()  	 __enable_irq()

#else
#define    mMDTM2TRC(mode_time)	((mode_time) > 0x7fff ? 0xffff : (mode_time) * 2)
/* #define    mMDTM2TRC(t)	(((t) * 2) - 1) */
/* #define    mTRC2MDTM(t)	((t)>> 1) */
#define    mTRC2MDTM(trc_time)	((trc_time) >> 1) /* ((trc_time) / D_fTRC) = (trc_time) * D_f8 / D_fc */
#define    mTRD2MDTM(trd_time)	((trd_time) >> 4) /* ((trd_time) / D_fTRD) = (trd_time) * D_f1 / D_fc */
#define    mTRD2TRC(trd_time)	((trd_time) >> 3) /* ((trd_time) / D_fTRD) = (trd_time) * D_f1 / D_fc */
#define    mEnableLapTimerInt() (ucCPU_TRCIER = 0x8f)
#define    mDisableLapTimerInt() (ucCPU_TRCIER = 0x8d)
#define    mDisableInterrupt()   __disable_irq()
#define    mEnableInterrupt()  	 __enable_irq()
#endif

#endif //COM_DEF_H_
