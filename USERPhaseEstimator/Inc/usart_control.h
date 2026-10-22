/******************************************************************
<File Name> ：usart_control.h
<Function> 	：主控通信头文件
<Program> 	：
<Author> 	：董明鉴
<Date> 		：2024/9/20
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Sys_type.h"
/* Private typedef -----------------------------------------------------------*/

#define MAXCNT_RX                   (45)                /*接收数据最大字节数*/

typedef struct 
{
    uint8_t u8_stat;                /*状态*/
    uint8_t u8_cnt;                 /*计数*/
    uint8_t u8_len;                 /*缓存区总长*/
    volatile uint8_t u8_flg;                 /*标志位*/
    uint8_t u8_buf[MAXCNT_RX];             /*缓存区*/
}ST_COM;

typedef volatile struct 
{
    uint8_t bit0 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    uint8_t bit1 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    uint8_t bit2 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    uint8_t bit3 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    uint8_t bit4 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    uint8_t bit5 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    uint8_t bit6 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    uint8_t bit7 : 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
}ST_BIT;

/* Private define ------------------------------------------------------------*/

#define ACK1                        (0xF1)              /*待机及输出指令返回信号*/
#define ACK2                        (0xF2)              /*从属站发送数据返回信号*/
#define ACK4                        (0xF4)              /*接收指令错误返回信号*/
#define ACK5                        (0xF5)              /*拒收再输出指令返回信号*/
#define NAK                         (0xFF)              /*从属站不能接收数据时返回信号(未使用)*/

#define CMD_WAIT                    (0xA0)              /*待机指令*/
#define CMD_OUT                     (0xA1)              /*输出指令*/
#define CMD_IN                      (0xA2)              /*输入指令*/
#define CMD_AOUT                    (0xA3)              /*再输出指令*/

#define DATA_CTRL                   (0x00)              /*数据指令——变频控制数据*/
#define DATA_STAT                   (0x02)              /*数据指令——运转状态数据*/


#define MAXCNT_TX                   (22)                /*发送数据最大字节数*/
#define MAXCNT_BUF                  (32)                /*缓冲区最大字节数*/

#define DATACNT_RX                  (22)                /*接收数据长度*/
#define DATACNT_TX                  (18)                /*发送数据长度*/

#define TIME_MS_BYTE                (10)                /*字符间隔10ms*/
#define TIME_MS_DATA                (20)                /*数据块间隔20ms*/
#define TIME_MS_KEEP                (50)                /*数据监视时间50ms*/

/*通信数据内容*/
#define ERR_NONE                    (0x0000)
#define ERR1_DR_EN                  (0x0001)            /*保护标志1_硬件过流标志标志*/
#define ERR1_OVER_VOLT              (0x0002)            /*保护标志1_过电压异常标志*/
#define ERR1_UNDER_VOLT             (0x0004)            /*保护标志1_低电压异常标志*/
#define ERR1_SPD_FDBK               (0x0008)            /*保护标志1_脱调异常标志*/
#define ERR1_MSK                    (0x000F)

#define ERR2_OVER_CUR               (0x1000)            /*保护标志2_过流保护*/
#define ERR2_PHASE_LOSS             (0x2000)            /*保护标志2_缺相保护*/
#define ERR2_OFFSET                 (0x4000)            /*保护标志2_相电流零点漂移*/
#define ERR2_WIRE_PROTECT           (0x8000)            /*保护标志2_脱调异常标志*/
#define ERR2_MSK                    (0xF000)

#define STATE_NORMAL                (0x10)              /*变频状态 正常*/
#define STATE_UNREACH               (0x30)              /*变频状态 正常（回转数未达到）*/
#define STATE_ERRSTOP               (0x40)              /*变频状态 异常停止*/
#define STATE_MSK                   (0xF0)

#define CODE_STYLE                  (0x00)              /*变频仕样识别码*/
#define CODE_CTRL1                  (0x0A)              /*变频控制器代码（上位）*/
#define CODE_CTRL2                  (0x0F)              /*变频控制器代码（下位）*/

#define FC_MASK                     (0x40)              /*工程检查标志位掩膜*/

/*通信状态*/
#define COM_IDLE                    (0x00)              /*通信初始化*/
#define COM_RX                      (0x01)              /*接收模式*/
#define COM_TX                      (0x02)              /*发送模式*/
#define COM_ERR                     (0x03)              /*异常模式*/

#define RX_STAT_WAIT                (0x00)              /*接收待机状态*/
#define RX_STAT_CMD                 (0x01)              /*接收指令状态*/
#define RX_STAT_DATA                (0x02)              /*接收数据状态*/
#define RX_STAT_ERR                 (0x03)              /*接收错误状态*/

#define RX_FLG_IDLE                 (0x00)              /*接收标志初始化*/
#define RX_FLG_CMD                  (0x01)              /*允许接收待机以外指令标志*/
#define RX_FLG_DATA                 (0x03)              /*允许接收数据标志*/
#define RX_FLG_GET                  (0x10)              /*已收到数据标志*/

#define TX_STAT_IDLE                (0x00)              /*发送初始化状态*/
#define TX_STAT_ACK1                (0x01)              /*发送ACK1状态*/
#define TX_STAT_ACK2                (0x02)
#define TX_STAT_DATA                (0x03)              /*发送数据状态*/
#define TX_STAT_ACK4                (0x04)
#define TX_STAT_ACK5                (0x05)
#define TX_STAT_SD1                 (0x06)              /*发送中1*/
#define TX_STAT_SD2                 (0x07)              /*发送中2（ACK2后发送数据）*/


#define TX_FLG_BAN                  (0x00)              /*禁止发送*/
#define TX_FLG_BYTE                 (0x01)              /*字节发送间隔允许*/
#define TX_FLG_DATA                 (0x02)              /*数据块发送间隔允许*/

#define TM_CNT_CLR                  (0x00)              /*发送计数清除*/
#define TM_CNT_SET                  (0x80)              /*发送计数标志*/
#define TM_CNT_MSK                  (0x7F)              /*发送计数位段*/

#define COM_ERR_CNT                 60                  /*通信异常判定时间60min*/

/* Private macro -------------------------------------------------------------*/

/* External variables ---------------------------------------------------------*/
extern uint8_t u8g_FCflg;                               /*工程检查模式标志*/
extern uint8_t u8g_FCcmd;                               /*工程检查模式指令*/
extern uint8_t u8g_OLPflg;                              /*OLP保护模式标志*/
/* External function prototypes -----------------------------------------------*/
/*extern void COM_Init(void);*/
extern void COM_main_process(void);
extern void COM_trns_process(void);
extern void COM_IT_rx(void);
extern void COM_IT_tx(void);


