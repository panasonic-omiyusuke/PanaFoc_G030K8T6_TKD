#include "stm32g0xx_hal.h"
#include "motor_process.h"
#include "Main_Pros.h"

extern uint8_t u8g_FCT_HTNumber;                       /*FCTヒータ番号*/
extern ST_SW sts_SW_fct;                        /*FCTスイッチ状態*/
extern ST_SW sts_SW_rxfct;                      /*FCT受信スイッチ状態*/    

extern void HEAT_Ctrl_FCT(void);
extern void UART_Ctrl_FCT(void);
extern void HEAT_Ctrl_OFF(void);
extern void MainLoop_PulseOutput(void);
extern int vMAIN_IsOptionByteCheck(void);
