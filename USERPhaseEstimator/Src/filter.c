/******************************************************************
<File Name> ：filter.c
<Function> 	：滤波器
<Program> 	：
<Author> 	：董明鉴
<Date> 		：2025/3/18
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/

#include "filter.h"

/******************************************************************
<Title> 		：Filter_OneOrderLPF(int16_t hInput, int16_t *pOutput)
<Function> 		：一阶低通滤波器
<Description> 	：一阶低通滤波器实现
<Input> 		：hInput 今回计算值
<Return> 		：*pOutput 滤波输出值
<Author> 		：董明鉴
<Date> 			：2025/3/18
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
void Filter_OneOrderLPF(int16_t hInput, int16_t *pOutput)
{
    int16_t hAux;
    int32_t wAux;

    hAux = *pOutput;
    wAux = hInput - hAux;
    wAux *= 3276;               /* 滤波系数α =0.2 * 16384 标幺化; */
    hAux += (int16_t)(wAux >> 14);

    *pOutput = (int16_t)hAux;
}

