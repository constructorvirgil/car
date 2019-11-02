#include "include.h"
#include "control.h"

float OutData[4] = { 0 };               //SCI示波器参数
float Gyro_Now,angle_offset_vertical;  //陀螺仪转化后的角速度，转化后的加速度角度
float g_fCarAngle,g_fGyroscopeAngleIntegral; //融合后的角度

volatile float  Speed_L,Speed_R,speed_Start,Speed_L_Last,Speed_R_Last;  //左右轮速度 ，最终速度

volatile int   MMA7361 ,ENC03,real_angle; //加速度计AD ,陀螺仪AD，模块输出的角度

//**************************************************************************
/*
*  功能说明：AD采集
*  参数说明： 无
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//**************************************************************************
void Rd_Ad_Value(void)
{
  	 real_angle = adc_ave(ADC1_SE14, ADC_12bit,40);//角度
	 ENC03 = adc_ave(ADC1_SE15, ADC_12bit,40);//直立角速度

#if 1
    OutData[0] = 1855;
    OutData[1] = real_angle;
    OutData[2] = 2505 ;
    OutData[3] = ENC03;
    OutPut_Data();
#endif
}

//**************************************************************************
/*
*  功能说明：SCI示波器CRC校验
内部调用函数
*  参数说明： 无
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//**************************************************************************
static unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

//************************************************
//
/*
*  功能说明：SCI示波器发送函数

*  参数说明：
OutData[]  需要发送的数值赋予该数组
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//****************************************************
void OutPut_Data(void)
{
    int temp[4] = {0};
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;
    for(i=0;i<4;i++)
    {
		temp[i]  = (int)OutData[i];
		temp1[i] = (unsigned int)temp[i];
    }

    for(i=0;i<4;i++)
    {
		databuf[i*2]   = (unsigned char)(temp1[i]%256);
		databuf[i*2+1] = (unsigned char)(temp1[i]/256);
    }

    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;

    for(i=0;i<10;i++)
    {
        uart_putchar (UART4,(char)databuf[i]);
    }
}
//**************************************************************************
//   Kalman滤波
//**************************************************************************

//float angle_dot;         //外部需要引用的变量
//-------------------------------------------------------
// 0.00015     //0.0001
const float Q_angle=0.001, Q_gyro=0.002, R_angle=0.5, dt=0.001;  //Q_angle和Q_gyro表示角度和角速度的权重，R表示增益（影响延时），dt表示取样时间
                  //0.001       //0.0005        //0.4   //0.01    抗干扰好，跟随性差
     //Q_angle=0.001, Q_gyro=0.001, R_angle=0.1, dt=0.01;  跟随性好，抗干扰差
//Q_gyro越大，跟随越快，超调越多；R_angle越大，跟随越慢，超调越小；
static float P[2][2] = {
    { 1, 0 },
    { 0, 1 }
};

static float Pdot[4] ={0,0,0,0};

static const char C_0 = 1;

static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m,float *angle,float *angle_dot)          //gyro_m:gyro_measure
{
    *angle+=(gyro_m-q_bias) * dt;
    
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;
    
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    
    angle_err = angle_m - *angle;
    
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    
    *angle   += K_0 * angle_err;
    q_bias  += K_1 * angle_err;
    *angle_dot = gyro_m-q_bias;
    
}
//**************************************************************************
//   清华角度滤波方案
//*************************************************************************
/*
*  功能说明：清华角度滤波
*  参数说明：G_angle                       加速度计角度0-90内
*            Gyro                         陀螺仪角速度转花后的数值
*            GRAVITY_ADJUST_TIME_CONSTANT  时间校正系数
DT                             定时器时间 单位s
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考清华源码
*/
//
//*************************************************************************
void QingHua_AngleCalaulate(float G_angle,float Gyro)
{
    float fDeltaValue;
    g_fCarAngle = g_fGyroscopeAngleIntegral;   //最终融合角度
    fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;  //时间系数矫正
    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;                //融合角度
}
//**************************************************************************
/*
*  功能说明：直立角度计算
*  参数说明：
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考清华源码
*/
//**************************************************************************
void AD_Calculate(void)
{
    Rd_Ad_Value();                          //采集 AD
    Gyro_Now = (GYRO_VAL - ENC03 ) * Gyro_ratio;                            //陀螺仪采集到的角速度归一化
    angle_offset_vertical = (MMA7361_vertical - MMA7361) * MMA7361_ratio ;  //将加速度计采集到的角度归一化，乘上0.375是为了归一化到0~90°
    if(angle_offset_vertical > 90)angle_offset_vertical = 90;               //防止加速度角度溢出
    if(angle_offset_vertical < -90)angle_offset_vertical = -90;

    //计算融合后的角度
    QingHua_AngleCalaulate(angle_offset_vertical,Gyro_Now);                 //清华角度滤波方案

    /*****************************串口看波形（选择使用）****************************/
#if 0                           //宏条件编译 选择是否使用 虚拟示波器
    OutData[0] = ENC03;
    OutData[1] = MMA7361;//Gyro_Now;
    OutData[2] = angle_offset_vertical ;
    OutData[3] = g_fCarAngle;
    OutPut_Data();
#elif  0
    OutData[0] = angle_dot;
    OutData[1] = Gyro_Now;
    OutData[2] = angle_offset_vertical ;
    OutData[3] = angle;
    OutPut_Data();
#endif	
}

/***********************************************************************/
/*
*  功能说明：直立速度计算
*  参数说明：angle                融合后最终角度
*            angle_dot            陀螺仪角速度
*
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考清华源码
*/
/******************************************************************************/
void Speed_Calculate(float angle,float angle_dot)
{
    /***********************************速度计算************************************/
    speed_Start = angle * P_ANGLE  + angle_dot * D_ANGLE ;  //直立时所要的速度

    //P_ANGLE  P_GYRO  宏定义 直立所需要的PD参数

    Speed_L = speed_Start;//左轮总速度
    Speed_R = speed_Start;//右轮总速度
    /***********************将最大速度限制在985个PWM内******************************/
    if(Speed_L > 985)  Speed_L = 985;
    if(Speed_L < -985) Speed_L = -985;
    if(Speed_R > 985)  Speed_R = 985;
    if(Speed_R < -985) Speed_R = -985;

    /***************因为驱动部分加了反相器，所以需对速度进行一个最终的处理******************/
    if(Speed_L > 0)     //因为加了反相器，所以PWM要反过来添加
        Speed_L_Last = 1000 - Speed_L;
    else
        Speed_L_Last = -1000 - Speed_L;

    if(Speed_R > 0)     //因为加了反相器，所以PWM要反过来添加
        Speed_R_Last = 1000 - Speed_R;
    else
        Speed_R_Last = -1000 - Speed_R;

    /*************用所得到的对应角度的速度进行PWM控制********************/
    if(Speed_L >= 0)    //angle大于0，向前，小于0，向后
    {
        ftm_pwm_duty(FTM0,FTM_CH3,1000);
        ftm_pwm_duty(FTM0,FTM_CH5,(uint32)(Speed_L_Last - MOTOR_DEAD_VAL_L));    //加入死区电压
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH5,1000);
        ftm_pwm_duty(FTM0,FTM_CH3,(uint32)(-Speed_L_Last - MOTOR_DEAD_VAL_L));    //加入死区电压
    }

    if(Speed_R >= 0)    //angle大于0，向前，小于0，向后
    {
        ftm_pwm_duty(FTM0,FTM_CH6,1000);
        ftm_pwm_duty(FTM0,FTM_CH4,(uint32)(Speed_R_Last - MOTOR_DEAD_VAL_R));    //加入死区电压
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH4,1000);
        ftm_pwm_duty(FTM0,FTM_CH6,(uint32)(-Speed_R_Last - MOTOR_DEAD_VAL_R));   //加入死区电压
    }
}