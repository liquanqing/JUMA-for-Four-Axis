/*
 *****************************************************************************************
 * �ļ���	: Queue.c
 * ����		: �Ƚ��ȳ�����
 * ����		: Inc
 * ʱ��		: 2015/9/11
 * �޸ļ�¼	:
 *****************************************************************************************
//2015/9/11
  	�޸�Count�п��ܵ��µ�bug
  		��ΪCount�ķ�Χ��0~65535,��sizeΪ0~65535ʱ��count��Ӧ����65536��
  	����:
  		���жϣ���֤ Head �� Tail �����ֵС��65535
//2016/2/14
    �޸�get_queue_count��������Ϊ queue_get_count
    ���Ӻ�����queue_is_full,�����ж϶����Ƿ�������������true�����򷵻�false
    ����ö�ٽṹqstatus_t���ں����ж�OK����ERR
 */
#define _OWN_QUEUE

#include "Queue.h"


//----------------------------------------------------------------------------------------
//Function
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
//���г�ʼ��
void queue_init(queue_t *pQ, queue_data_type *pQBuf, uint16_t hwQSize)
{
	pQ->pQueueBuf = pQBuf;
	pQ->hwHead = 0;
	pQ->hwTail = 0;
	pQ->hwCount = 0;
	pQ->hwSize = hwQSize;
}


//----------------------------------------------------------------------------------------
//��ȡ��������Ч����
uint16_t queue_get_count( queue_t *pQ)
{
	return pQ->hwCount;
}


//----------------------------------------------------------------------------------------
//�ж϶����Ƿ���
qstatus_t queue_is_full( queue_t *pQ)
{
    if(pQ->hwCount >= pQ->hwSize){
        return QUEUE_OK;
    }else{
        return QUEUE_ERR;
    }
}

//----------------------------------------------------------------------------------------
//ѹ��
qstatus_t queue_write( queue_t *pQ, queue_data_type t_Dat )
{
	if( (pQ->hwHead == pQ->hwTail) && pQ->hwCount != 0 ) {
		return QUEUE_ERR;  //������ݴ���������ִ�� Tail Drop����
	}
	pQ->pQueueBuf[pQ->hwTail] = t_Dat;
	pQ->hwTail = (pQ->hwTail == (pQ->hwSize-2)) ?\
				 0 : pQ->hwTail+1; //�������βָ�볬��size����������0��ʼ
	pQ->hwCount ++;

	return QUEUE_OK;
}


//----------------------------------------------------------------------------------------
//����
qstatus_t queue_read( queue_t *pQ, queue_data_type *t_Dat)
{
	//�������Ϊ�գ������ʧ��
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
//���ݽ���
qstatus_t queue_multi_write( queue_t *pQ, queue_data_type *t_Dat, uint16_t hwLen)
{
	uint16_t i;

	//�������ʣ��ռ䲻�������˳�
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
//���ݳ���
qstatus_t queue_multi_read( queue_t *pQ, queue_data_type *t_Dat, uint16_t hwLen)
{
	uint16_t i;

	//���������Ч����С�ڶ������ݳ��ȣ��˳�
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

