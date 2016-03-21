/***********************************************************************************
 * �� �� ��   : INC_ITM.c
 * �� �� ��   : Inc
 * ��������   : 2015��12��13��
 * �ļ�����   : stm32ITM����debugģ��
 * ��Ȩ˵��   : Copyright (c) 2008-2015   xx xx xx xx �������޹�˾
 * ��    ��   :
 				ʹ��˵��:
					�������STM32DBG.ini�ļ�ʹ��
					MDK��������:
					1����Option,���ұ�initialization file�м���STM32DBG.ini�ļ�
					2����Setting��ʹ��SWģʽ����Trace�����У�ʹ��enable,ͬʱ�޸�
					   coreʱ��(��ǰϵͳʱ��)
					3��ITM Stimulus Port ��ѡ bit0������ȡ����ѡ
 * �޸���־   :
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

