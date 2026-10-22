#ifndef DTC_H
#define DTC_H
#include "Sys_type.h"
extern void SetDtcChangeDuty(UI08 *dtc_addr, UI08 ctrl_reg, UI08 block_size, UI16 *src_addr, UI16 *dest_addr);
extern void SetDtc(UI08 *dtc_addr, UI08 ctrl_reg, UI08 block_size, UI08 counter, UI16 *src_addr, UI16 *dest_addr);
extern void SetDtcAddrChangeDuty(UI16* crr_sw_dst_addr, UI16* nxt_sw_dst_addr,
								 UI16* nnxt_sw_src_addr, UI16* nnxt_sw_dst_addr,
								 UI16* rest_dst_addr);
extern void RegistDtcChangeDuty(void);
extern void ResetDtcCountChangeDuty(void);
extern void RegistDtcAdc(void);

#ifdef UNIT_TEST
UI08 GetReAdStart(void);
UI08 GetTrdAdStart1(void);
UI08 GetDtcVectorChangeCarrier(void);
UI08 GetRepCtChangeCarrier(void);
#endif

#endif /* #ifndef DTC_H */
