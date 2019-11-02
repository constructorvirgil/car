#include "Motor.h"
#include "stdio.h"
#include "include.h"
#include "lcd.h"
#include "control.h"
#include "TOF10120.h"

int32 motorPulseLeft = 0; //编码器读值
int32 motorPulseRight = 0;
int32 motorPulseLeftLast = 0; //编码器读值
int32 motorPulseRightLast = 0;


int32 Left_Jump = 0;
int32 Right_Jump = 0;
int16 carSpeed = 0; //计算出来的速度值
int32 Speed_D_L, Speed_D_R;
float Speed_D, Last_Speed_D;

SpeedSet speedSet;


CarState carState = enStand;
CarState lastCarState = enThree;


void Speed_Measure(void)
{
    speedSet.stand = 110;
    speedSet.three = 120;

    int32 Pulses;

    /******* 右电机速度相关控制 ********/
    Pulses = ftm_quad_get(FTM1); // 获取FTM 正交解码 的脉冲数(负数表示反方向)
    ftm_quad_clean(FTM1);        // 正交解码寄存器清零
    motorPulseRight = -Pulses; // 得到右轮转速

    /******* 左电机速度相关控制 ********/
    Pulses = ftm_quad_get(FTM2); // 获取FTM 正交解码 的脉冲数(负数表示反方向)
    ftm_quad_clean(FTM2);        // 正交解码寄存器清零
    motorPulseLeft = Pulses;   // 得到左轮转速

    Left_Jump = abs(motorPulseLeft - motorPulseLeftLast);
    Right_Jump = abs(motorPulseRight - motorPulseRightLast);

    if (Left_Jump < 20)
        motorPulseLeft = 0.9 * motorPulseLeft + 0.1 * motorPulseLeftLast;
    else
        motorPulseLeft = 0.5 * motorPulseLeft + 0.5 * motorPulseLeftLast;

    if (Right_Jump < 20)
        motorPulseRight = 0.9 * motorPulseRight + 0.1 * motorPulseRightLast;
    else
        motorPulseRight = 0.5 * motorPulseRight + 0.5 * motorPulseRightLast;

    if (carState == enStand)
    {
        carSpeed = (motorPulseLeft + motorPulseRight) / 2;
        carSpeed = limit_int(carSpeed, speedSet.stand + 30, -30);
    }
    else if(carState == enThree)
    {
        Speed_D_L = Ab_s(motorPulseLeft - speedSet.three);
        Speed_D_R = Ab_s(motorPulseRight - speedSet.three);

        Speed_D = (Speed_D_L - Speed_D_R) * (Speed_D_L - Speed_D_R) / 1400.0;
        Speed_D = limit_float(Speed_D, 0.3, 0);

        Speed_D = 0.8 * Speed_D + 0.2 * Last_Speed_D;
        Last_Speed_D = Speed_D;

        if (motorPulseLeft > motorPulseRight)
        {
            carSpeed = (0.5 + Speed_D) * motorPulseLeft + (0.5 - Speed_D) * motorPulseRight;
        }
        else
        {
            carSpeed = (0.5 - Speed_D) * motorPulseLeft + (0.5 + Speed_D) * motorPulseRight;
        }
        carSpeed = limit_int(carSpeed, speedSet.three + 30, -30);
    }

    motorPulseLeftLast = motorPulseLeft;
    motorPulseRightLast = motorPulseRight;
}