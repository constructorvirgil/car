#ifndef __CIRQUEUE_H
#define __CIRQUEUE_H

#define CIRQUESIZE 9
typedef struct
{
    int *base;
    int front;
    int rear;
} CirQueue;

CirQueue *Q_Init();
int Q_Put(CirQueue *Q, int e);
int Q_Front(CirQueue *Q);
int Q_Poll(CirQueue *Q);
int Q_Clear(CirQueue *Q);


#endif

