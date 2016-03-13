/*
 *****************************************************************************************
 * 文件名	: Queue.h
 * 描述		: 先进先出队列
 * 作者		: Inc
 * 时间		: xxx
 * 修改记录	:
 *****************************************************************************************
 */

#ifndef __QUEUE_H
#define __QUEUE_H

#include "stm32f4xx.h"

//----------------------------------------------------------------------------------------
//define
//----------------------------------------------------------------------------------------
#define QUEUE_BUF_SINGLE_SIZE		8		//缓冲区单位为8BIT

//#if 8 == QUEUE_BUF_SINGLE_SIZE
	typedef uint8_t queue_data_type;
//#elif 16 == QUEUE_BUF_SINGLE_SIZE
//	typedef uint16_t queue_data_type;
//#elif 32 == QUEUE_BUF_SINGLE_SIZE
//	typedef uint32_t queue_data_type;
//#endif


#ifdef _OWN_QUEUE
	#define QUEUE_EXT
#else
	#define QUEUE_EXT	extern
#endif




//----------------------------------------------------------------------------------------
//typedef
//----------------------------------------------------------------------------------------
typedef struct {
	queue_data_type *pQueueBuf;	//队列缓冲区
	uint16_t	hwHead;			//队列头指针
	uint16_t 	hwTail;			//队列尾指针
	uint16_t 	hwCount;		//队列中的有效个数
	uint16_t 	hwSize;			//队列长度
}queue_t;

typedef enum{
    QUEUE_ERR = 0,
    QUEUE_OK = !QUEUE_ERR,
}qstatus_t;

//----------------------------------------------------------------------------------------
//extern
//----------------------------------------------------------------------------------------
QUEUE_EXT void queue_init( queue_t *pQ, queue_data_type *pQBuf, uint16_t hwQSize);
QUEUE_EXT uint16_t queue_get_count( queue_t *pQ);
QUEUE_EXT qstatus_t queue_is_full( queue_t *pQ);
QUEUE_EXT qstatus_t queue_write( queue_t *pQ, queue_data_type t_Dat );
QUEUE_EXT qstatus_t queue_read(queue_t *pQ, queue_data_type *t_Dat);
QUEUE_EXT qstatus_t queue_multi_read( queue_t *pQ, queue_data_type *t_Dat, uint16_t hwLen);
QUEUE_EXT qstatus_t queue_multi_write( queue_t *pQ, queue_data_type *t_Dat, uint16_t hwLen);



#endif
/***************************End Of File(Copyright of Inc)***********************************************/
