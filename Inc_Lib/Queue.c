/*
 *****************************************************************************************
 * 文件名	: Queue.c
 * 描述		: 先进先出队列
 * 作者		: Inc
 * 时间		: 2015/9/11
 * 修改记录	:
 *****************************************************************************************
//2015/9/11
  	修改Count有可能导致的bug
  		因为Count的范围是0~65535,当size为0~65535时，count对应的是65536，
  	处理:
  		加判断，保证 Head 和 Tail 的最大值小于65535
//2016/2/14
    修改get_queue_count函数名称为 queue_get_count
    增加函数：queue_is_full,用于判断队列是否用满，满返回true，否则返回false
    新增枚举结构qstatus_t用于函数判断OK还是ERR
 */
#define _OWN_QUEUE

#include "Queue.h"


//----------------------------------------------------------------------------------------
//Function
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
//队列初始化
void queue_init(queue_t *pQ, queue_data_type *pQBuf, uint16_t hwQSize)
{
	pQ->pQueueBuf = pQBuf;
	pQ->hwHead = 0;
	pQ->hwTail = 0;
	pQ->hwCount = 0;
	pQ->hwSize = hwQSize;
}


//----------------------------------------------------------------------------------------
//获取队列中有效个数
uint16_t queue_get_count( queue_t *pQ)
{
	return pQ->hwCount;
}


//----------------------------------------------------------------------------------------
//判断队列是否满
qstatus_t queue_is_full( queue_t *pQ)
{
    if(pQ->hwCount >= pQ->hwSize){
        return QUEUE_OK;
    }else{
        return QUEUE_ERR;
    }
}

//----------------------------------------------------------------------------------------
//压队
qstatus_t queue_write( queue_t *pQ, queue_data_type t_Dat )
{
	if( (pQ->hwHead == pQ->hwTail) && pQ->hwCount != 0 ) {
		return QUEUE_ERR;  //如果数据处理不过来，执行 Tail Drop机制
	}
	pQ->pQueueBuf[pQ->hwTail] = t_Dat;
	pQ->hwTail = (pQ->hwTail == (pQ->hwSize-2)) ?\
				 0 : pQ->hwTail+1; //如果队列尾指针超过size，返回来从0开始
	pQ->hwCount ++;

	return QUEUE_OK;
}


//----------------------------------------------------------------------------------------
//出队
qstatus_t queue_read( queue_t *pQ, queue_data_type *t_Dat)
{
	//如果队列为空，则读出失败
	if( (pQ->hwHead == pQ->hwTail) && pQ->hwCount == 0) {
		return QUEUE_ERR;
	}
	*t_Dat = pQ->pQueueBuf[pQ->hwHead];
	pQ->hwHead = (pQ->hwHead == (pQ->hwSize-2)) ?\
				  0 : pQ->hwHead +1;
	pQ->hwCount --;

	return QUEUE_OK;
}


//----------------------------------------------------------------------------------------
//数据进队
qstatus_t queue_multi_write( queue_t *pQ, queue_data_type *t_Dat, uint16_t hwLen)
{
	uint16_t i;

	//如果队列剩余空间不够，则退出
	if((pQ->hwSize - pQ->hwCount) < hwLen){
		return QUEUE_ERR;
	}

	for(i = 0; i< hwLen; i++){
		if(QUEUE_ERR == queue_write(pQ,*(t_Dat+i))){
			return QUEUE_ERR;
		}
	}
	return QUEUE_OK;
}


//----------------------------------------------------------------------------------------
//数据出队
qstatus_t queue_multi_read( queue_t *pQ, queue_data_type *t_Dat, uint16_t hwLen)
{
	uint16_t i;

	//如果队列有效个数小于读出数据长度，退出
	if(pQ->hwCount < hwLen){
		return QUEUE_ERR;
	}

	for(i = 0; i< hwLen; i++){
		if(QUEUE_ERR == queue_read(pQ, t_Dat+i)) {
			return QUEUE_ERR;
		}
	}
	return QUEUE_OK;
}

