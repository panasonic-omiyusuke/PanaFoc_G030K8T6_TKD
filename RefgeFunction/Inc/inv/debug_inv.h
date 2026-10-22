#ifndef DEBUG_INV_H
#define DEBUG_INV_H

#include "vipdc.h"

#define DA0_BIT           (0x40)      /* p0_6 */
#define DA0_USE           (~DA0_BIT)  /*  */
#define DA0_ENABLE        (0x01)      /*  */

#define DA1_BIT           (0x80)      /* p0_7 */
#define DA1_USE           (~DA1_BIT)  /*  */
#define DA1_ENABLE        (0x02)      /*  */

#define PD0_WRITE_ENABLE  (0x04)      /* 代入して利用 */

/* ここでDA_A or DA_B にDAを割り付け */
#define ucCPU_DA_A     ucCPU_DA0
#define INIT_DA_A() (InitDa0())

#define ucCPU_DA_B     (ucCPU_DA1)
#define INIT_DA_B() (InitDa1())

/* DA出力本体 */
#define OUTPUT_DA(A_or_B, data) (ucCPU_DA_##A_or_B = (data))

/* このブロックを変更 ポートとピン位置を指定 */
#define DEBUG_A_PORT (ucCPU_P6)
#define DEBUG_A_BIT  (0x20)   /* bit5 */
#define DEBUG_B_PORT (ucCPU_P6)
#define DEBUG_B_BIT  (0x01)   /* bit1 */

#define DEBUG_HIGH(A_or_B)   (DEBUG_##A_or_B##_PORT |=  DEBUG_##A_or_B##_BIT)
#define DEBUG_LOW(A_or_B)    (DEBUG_##A_or_B##_PORT &= ~DEBUG_##A_or_B##_BIT)
#define DEBUG_TOGGLE(A_or_B) (DEBUG_##A_or_B##_PORT ^=  DEBUG_##A_or_B##_BIT)

void InitDa0(void);
void InitDa1(void);



/* for button */

void ProcessButtonPushed(void);
void InitDebugButton(void);

#endif /* #ifndef DEBUG_INV_H */
