#ifndef _QUEUE_H
#define _QUEUE_H

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#define QUEUE_DATA_MAXLENGTH         32
#define QUEUE_DATA_BASE_LENGTH       10

typedef struct 
{
    uint8_t Buff[QUEUE_DATA_MAXLENGTH];
    uint8_t Length;
}Buff_t;

typedef struct
{
    uint8_t front;
    uint8_t rear;
    Buff_t Buff[QUEUE_DATA_BASE_LENGTH];
}Queue_t;



void Queue_Init(Queue_t *qPtr);
bool enQueue(Queue_t *qPtr, uint8_t *Buff,uint8_t length);
bool deQueue(Queue_t *qPtr,uint8_t *Buff,uint8_t *length);

#endif
