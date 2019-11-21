#ifndef __MOTOR_H
#define __MOTOR_H

#include "common.h"
#include "MK60_ftm.h"
#include "MK60_port.h"


#define	MOTOR_FTM		FTM0
#define	CH_LEFT_FOR		FTM_CH7
#define	CH_RIGHT_FOR	FTM_CH4
#define	CH_LEFT_BAK		FTM_CH6
#define	CH_RIGHT_BAK	FTM_CH5
#define	MOTOR_PORT_EN	PTD2

#define PWM_WIDTH       10*1000

#define MOTOR_AMEND_FACTOR_LEFT     0
#define MOTOR_AMEND_FACTOR_RIGHT    0

void motor_init();
void motor_run(int duty);



#endif