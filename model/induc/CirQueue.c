#include "CirQueue.h"
#include "common.h"


CirQueue *Q_Init()
{
    CirQueue *Q = (CirQueue *)malloc(sizeof(CirQueue));
    // 存储分配失败
    if (!Q)
    {
        return NULL;
    }
    Q->base = (int *)malloc(CIRQUESIZE * sizeof(int));
    // 存储分配失败
    if (!Q->base)
    {
        return NULL;
    }
    Q->front = Q->rear = 0;
    return Q;
}

int Q_Put(CirQueue *Q, int e)
{
    if ((Q->rear + 1) % CIRQUESIZE == Q->front) // 队列满
        return -1;
    Q->base[Q->rear] = e;
    Q->rear = (Q->rear + 1) % CIRQUESIZE;
    return 1;
}

//返回队列头的元素
int Q_Front(CirQueue *Q)
{
    if (Q->front == Q->rear) // 队列空
        return -1;
    return Q->base[Q->front];
}

//返回队列尾的元素
int Q_Rear(CirQueue *Q)
{
    if (Q->front == Q->rear) // 队列空
        return -1;
    return Q->base[Q->rear];
}

// 若队列不空，则删除Q的队头元素然后返回1；否则返回-1
int Q_Poll(CirQueue *Q)
{
    if (Q->front == Q->rear) // 队列空
        return -1;
    Q->front = (Q->front + 1) % CIRQUESIZE;
    return 1;
}

int Q_Clear(CirQueue *Q)
{
    for (int i = 0; i < CIRQUESIZE; i++)
    {
        Q->base[i] = 0;
    }
    Q->front = 0;
    Q->rear = 0;
    return 1;
}