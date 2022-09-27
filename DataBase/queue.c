#include "queue.h"
 
void Queue_Init(Queue_t *qPtr)
{
	qPtr->front = 0;
	qPtr->rear = 0;
}

bool enQueue(Queue_t *qPtr, uint8_t *Buff,uint8_t length)
{
	//(qu->rear + 1) % maxsize == qu->front
	if ((qPtr->rear + 1) % QUEUE_DATA_BASE_LENGTH == qPtr->front)
	{
		return false;
	}

	qPtr->rear = (qPtr->rear + 1) % QUEUE_DATA_BASE_LENGTH;
    memcpy(qPtr->Buff[qPtr->rear].Buff,Buff,length);
    qPtr->Buff[qPtr->rear].Length = length;
    return true;
}

bool deQueue(Queue_t *qPtr,uint8_t *Buff,uint8_t *length)
{
	if (qPtr->front == qPtr->rear)
		return false;

	qPtr->front = (qPtr->front + 1) % QUEUE_DATA_BASE_LENGTH;
	memcpy(Buff,qPtr->Buff[qPtr->front].Buff,qPtr->Buff[qPtr->front].Length);
    *length = qPtr->Buff[qPtr->front].Length;
	return true;
}

