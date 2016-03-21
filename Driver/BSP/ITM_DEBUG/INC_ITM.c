/***********************************************************************************
 * 文 件 名   : INC_ITM.c
 * 负 责 人   : Inc
 * 创建日期   : 2015年12月13日
 * 文件描述   : stm32ITM串口debug模块
 * 版权说明   : Copyright (c) 2008-2015   xx xx xx xx 技术有限公司
 * 其    他   :
 				使用说明:
					必须配合STM32DBG.ini文件使用
					MDK设置详情:
					1、打开Option,在右边initialization file中加载STM32DBG.ini文件
					2、打开Setting，使能SW模式，在Trace窗口中，使能enable,同时修改
					   core时钟(当前系统时钟)
					3、ITM Stimulus Port 勾选 bit0，其他取消勾选
 * 修改日志   :
***********************************************************************************/

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
    FILE __stdout;
    FILE __stdin;

int fputc(int ch, FILE *f)
{
    return ITM_SendChar(ch);
}

volatile int32_t ITM_RxBuffer;
int fgetc(FILE *f)
{
  while (ITM_CheckChar() != 1) __NOP();
  return (ITM_ReceiveChar());
}

int ferror(FILE *f)
{
    /* Your implementation of ferror */
    return EOF;
}

void _ttywrch(int c)
{
    FILE *f = 0;
    fputc(c, f);
}

int __backspace()
{
    return 0;
}
void _sys_exit(int return_code)
{
label:
    goto label;  /* endless loop */
}

