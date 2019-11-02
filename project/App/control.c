#include "include.h"
#include "control.h"

float OutData[4] = { 0 };               //SCIʾ��������
float Gyro_Now,angle_offset_vertical;  //������ת����Ľ��ٶȣ�ת����ļ��ٶȽǶ�
float g_fCarAngle,g_fGyroscopeAngleIntegral; //�ںϺ�ĽǶ�

volatile float  Speed_L,Speed_R,speed_Start,Speed_L_Last,Speed_R_Last;  //�������ٶ� �������ٶ�

volatile int   MMA7361 ,ENC03,real_angle; //���ٶȼ�AD ,������AD��ģ������ĽǶ�

//**************************************************************************
/*
*  ����˵����AD�ɼ�
*  ����˵���� ��
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
*/
//**************************************************************************
void Rd_Ad_Value(void)
{
  	 real_angle = adc_ave(ADC1_SE14, ADC_12bit,40);//�Ƕ�
	 ENC03 = adc_ave(ADC1_SE15, ADC_12bit,40);//ֱ�����ٶ�

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
*  ����˵����SCIʾ����CRCУ��
�ڲ����ú���
*  ����˵���� ��
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
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
*  ����˵����SCIʾ�������ͺ���

*  ����˵����
OutData[]  ��Ҫ���͵���ֵ���������
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
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
//   Kalman�˲�
//**************************************************************************

//float angle_dot;         //�ⲿ��Ҫ���õı���
//-------------------------------------------------------
// 0.00015     //0.0001
const float Q_angle=0.001, Q_gyro=0.002, R_angle=0.5, dt=0.001;  //Q_angle��Q_gyro��ʾ�ǶȺͽ��ٶȵ�Ȩ�أ�R��ʾ���棨Ӱ����ʱ����dt��ʾȡ��ʱ��
                  //0.001       //0.0005        //0.4   //0.01    �����źã������Բ�
     //Q_angle=0.001, Q_gyro=0.001, R_angle=0.1, dt=0.01;  �����Ժã������Ų�
//Q_gyroԽ�󣬸���Խ�죬����Խ�ࣻR_angleԽ�󣬸���Խ��������ԽС��
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
//   �廪�Ƕ��˲�����
//*************************************************************************
/*
*  ����˵�����廪�Ƕ��˲�
*  ����˵����G_angle                       ���ٶȼƽǶ�0-90��
*            Gyro                         �����ǽ��ٶ�ת�������ֵ
*            GRAVITY_ADJUST_TIME_CONSTANT  ʱ��У��ϵ��
DT                             ��ʱ��ʱ�� ��λs
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע���ο��廪Դ��
*/
//
//*************************************************************************
void QingHua_AngleCalaulate(float G_angle,float Gyro)
{
    float fDeltaValue;
    g_fCarAngle = g_fGyroscopeAngleIntegral;   //�����ںϽǶ�
    fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;  //ʱ��ϵ������
    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;                //�ںϽǶ�
}
//**************************************************************************
/*
*  ����˵����ֱ���Ƕȼ���
*  ����˵����
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע���ο��廪Դ��
*/
//**************************************************************************
void AD_Calculate(void)
{
    Rd_Ad_Value();                          //�ɼ� AD
    Gyro_Now = (GYRO_VAL - ENC03 ) * Gyro_ratio;                            //�����ǲɼ����Ľ��ٶȹ�һ��
    angle_offset_vertical = (MMA7361_vertical - MMA7361) * MMA7361_ratio ;  //�����ٶȼƲɼ����ĽǶȹ�һ��������0.375��Ϊ�˹�һ����0~90��
    if(angle_offset_vertical > 90)angle_offset_vertical = 90;               //��ֹ���ٶȽǶ����
    if(angle_offset_vertical < -90)angle_offset_vertical = -90;

    //�����ںϺ�ĽǶ�
    QingHua_AngleCalaulate(angle_offset_vertical,Gyro_Now);                 //�廪�Ƕ��˲�����

    /*****************************���ڿ����Σ�ѡ��ʹ�ã�****************************/
#if 0                           //���������� ѡ���Ƿ�ʹ�� ����ʾ����
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
*  ����˵����ֱ���ٶȼ���
*  ����˵����angle                �ںϺ����սǶ�
*            angle_dot            �����ǽ��ٶ�
*
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע���ο��廪Դ��
*/
/******************************************************************************/
void Speed_Calculate(float angle,float angle_dot)
{
    /***********************************�ٶȼ���************************************/
    speed_Start = angle * P_ANGLE  + angle_dot * D_ANGLE ;  //ֱ��ʱ��Ҫ���ٶ�

    //P_ANGLE  P_GYRO  �궨�� ֱ������Ҫ��PD����

    Speed_L = speed_Start;//�������ٶ�
    Speed_R = speed_Start;//�������ٶ�
    /***********************������ٶ�������985��PWM��******************************/
    if(Speed_L > 985)  Speed_L = 985;
    if(Speed_L < -985) Speed_L = -985;
    if(Speed_R > 985)  Speed_R = 985;
    if(Speed_R < -985) Speed_R = -985;

    /***************��Ϊ�������ּ��˷���������������ٶȽ���һ�����յĴ���******************/
    if(Speed_L > 0)     //��Ϊ���˷�����������PWMҪ���������
        Speed_L_Last = 1000 - Speed_L;
    else
        Speed_L_Last = -1000 - Speed_L;

    if(Speed_R > 0)     //��Ϊ���˷�����������PWMҪ���������
        Speed_R_Last = 1000 - Speed_R;
    else
        Speed_R_Last = -1000 - Speed_R;

    /*************�����õ��Ķ�Ӧ�Ƕȵ��ٶȽ���PWM����********************/
    if(Speed_L >= 0)    //angle����0����ǰ��С��0�����
    {
        ftm_pwm_duty(FTM0,FTM_CH3,1000);
        ftm_pwm_duty(FTM0,FTM_CH5,(uint32)(Speed_L_Last - MOTOR_DEAD_VAL_L));    //����������ѹ
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH5,1000);
        ftm_pwm_duty(FTM0,FTM_CH3,(uint32)(-Speed_L_Last - MOTOR_DEAD_VAL_L));    //����������ѹ
    }

    if(Speed_R >= 0)    //angle����0����ǰ��С��0�����
    {
        ftm_pwm_duty(FTM0,FTM_CH6,1000);
        ftm_pwm_duty(FTM0,FTM_CH4,(uint32)(Speed_R_Last - MOTOR_DEAD_VAL_R));    //����������ѹ
    }
    else
    {
        ftm_pwm_duty(FTM0,FTM_CH4,1000);
        ftm_pwm_duty(FTM0,FTM_CH6,(uint32)(-Speed_R_Last - MOTOR_DEAD_VAL_R));   //����������ѹ
    }
}