#ifndef __MOTOR_H
#define __MOTOR_H

#include "common.h"
typedef enum{
    enStand,
    enThree
}CarState;

typedef struct
{
    int16 stand;
    int16 three;
}SpeedSet;

void Speed_Measure(void);

#endif