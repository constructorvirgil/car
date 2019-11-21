#include "motor.h"
#include "MK60_gpio.h"


void motor_init()
{
	ftm_pwm_init(MOTOR_FTM, CH_RIGHT_FOR,PWM_WIDTH,0);  //右正
    ftm_pwm_init(MOTOR_FTM, CH_RIGHT_BAK,PWM_WIDTH,0); //右反
    ftm_pwm_init(MOTOR_FTM, CH_LEFT_BAK,PWM_WIDTH,0);   //左反
    ftm_pwm_init(MOTOR_FTM, CH_LEFT_FOR,PWM_WIDTH,0);  //左正
	gpio_init(MOTOR_PORT_EN,GPO,0);//电机使能
	gpio_set(MOTOR_PORT_EN,1);
}


void motor_run(int duty)
{
    if(duty>PWM_WIDTH || duty<-PWM_WIDTH)
    {
        return;
    }
    if(duty > 0)
    {
	    ftm_pwm_duty(MOTOR_FTM, CH_LEFT_FOR,PWM_WIDTH/2 + duty + MOTOR_AMEND_FACTOR_LEFT);
	    ftm_pwm_duty(MOTOR_FTM, CH_RIGHT_FOR,PWM_WIDTH/2 + duty + MOTOR_AMEND_FACTOR_RIGHT);
    }

    if(duty < 0)
    {
        ftm_pwm_duty(MOTOR_FTM, CH_LEFT_BAK,PWM_WIDTH/2 + duty + MOTOR_AMEND_FACTOR_LEFT);
	    ftm_pwm_duty(MOTOR_FTM, CH_RIGHT_BAK,PWM_WIDTH/2 + duty + MOTOR_AMEND_FACTOR_RIGHT);
    }
}
