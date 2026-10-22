/******************************************************************
<File Name> ：filter.h
<Function> 	：滤波器
<Program> 	：
<Author> 	：董明鉴
<Date> 		：2025/3/18
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/

#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

extern void Filter_OneOrderLPF(int16_t hInput, int16_t *pOutput);

#endif /* FILTER_H */


