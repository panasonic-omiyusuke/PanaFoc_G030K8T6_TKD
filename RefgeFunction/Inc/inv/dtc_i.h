#ifndef DTC_I_H_
#define DTC_I_H_
#include "Sys_type.h"
#include "vipdc.h"

#define READSTART		2												/* AD変換結果格納 & AD再スタート DTC番号 */
/* #define READSTART_NUM	((UI08)(NUM_DTCVT + NUM_DTCSHUNT + NUM_DTCVPN))		 AD変換数  @007MzDOD[16/12/21]  使用禁止 */

#define TRDADSTART1		7												/* 低速RD時のADスタートDTC番号 */
/* #define TRDADSTART1_NUM	1 */

#define CHANGE_CARRIER		10 /* キャリア変更やDuty変更に伴うRAM変更用のDTCスタート番号 */
/* #define CHANGE_DUTY			13 */
#define CHANGE_CARRIER_NUM	1  /* 繰り返し回数 */
/* #define CHANGE_DUTY_NUM		CHANGE_CARRIER_NUM */

#endif /* #ifndef DTC_I_C */
