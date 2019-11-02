/*****************************************************************************/
/*                                  Virgil                                  */
/*                              2019/10/19                                */
/*                                  自改                                    */
/*                                                                          */
/*****************************************************************************/

#include "common.h"
#include "stdio.h"
#include "include.h"
#include "lcd.h"
#include "control.h"
#include "TOF10120.h"
#include "Key.h"
#include "LCD_Menu.h"
#include "CirQueue.h"
#include "Induc.h"
#include "Motor.h"
/*********************************** 电磁滤波定义部分 STAR ***********************************/

#define M 10

/*********************************** 电磁滤波定义部分 END  ***********************************/
extern uint8 indVal[IND_NUM];
/*********************************** 电机舵机定义部分 STAR ***********************************/

#define motor_1 gpio_set(PTD2, 1) //电机使能
#define motor_0 gpio_set(PTD2, 0)

#define Servo_Down 1740 //王1740  龚1250
#define Servo_Up 1090   //王1535  龚1390
int16 Servo_Pwm = 0;

extern CarState carState;
extern CarState lastCarState;
extern int16 carSpeed;
extern int32 Speed_D_L;
extern int32 Speed_D_R;
extern float Speed_D, Last_Speed_D;

volatile int32 Motor_L = 0, Motor_R = 0; //左右电机输出值

int32 S_Balance_Max = 2500, S_Balance_Min = -2500;
int32 S_Angle_Max = 2500, S_Angle_Min = -2500;
int32 S_Speed_Max = 3800, S_Speed_Min = 3000; //2900 / 2700
int32 SE_Dir_Max = 2500, SE_Dir_Min = -2500;  //2000
int32 T_Speed_Max = 2500, T_Speed_Min = -2500;
int32 TE_Dir_Max = 4500, TE_Dir_Min = -4500;

int32 Motor_Max = 4500, Motor_Min = -4500;

int16 Lp_dead = 100, Rp_dead = 50, Ln_dead = 100, Rn_dead = 50; //死区电压参数  int16 Lp_dead = 40, Rp_dead = 80, Ln_dead= 50, Rn_dead = 90;
//int16 Lp_dead = 250, Rp_dead = 200, Ln_dead= 200, Rn_dead = 200;

/*********************************** 电机舵机定义部分 END  ***********************************/

/*********************************** PID结构体定义部分 STAR ***********************************/

float ang_get, zang_dot, S_ang_set = 37;
float zang_error = 0, zang_sum = 0, zero_dot = 0, s_zero_dot = 0, t_zero_dot = 0;
int16 zero = 0, zero_count = 0, wait_count = 0;

int16 s_speed_error = 0, s_angle_out = 0;
float s_ang_error = 0, s_ang_error_last = 0, s_dot_out = 0;
float s_balance_error = 0, s_balance_error_last = 0, s_balance_increase = 0, s_balance_pwm = 0;
float nP = 0, nI = 0; //速度控制P、I项
int16 increment_pwm = 0, Actual_pwm = 0;

extern SpeedSet speedSet;

struct DIRECTION
{
    int16 error;
    int16 pwm;
    int16 last_pwm;
} E_Dire;
struct DIRECTION E_Dire = {0, 0, 0};

struct PID
{
    float p;
    float i;
    float d;
} T_Balance, T_Speed, TE_Dir, SE_Dir, S_Balance, S_Angle, S_Speed; //平衡，速度，方向PID参数结构体

struct SpeedError
{
    int16 err;
    int16 err_llast;
    int16 err_last;
} Speed;

struct PID T_Speed = {0, 0, 0};
struct PID S_Balance = {0, 0, 0};
struct PID S_Angle = {0, 0, 0};
struct PID S_Speed = {0, 0, 0};
struct PID TE_Dir = {0, 0, 0};
struct PID SE_Dir = {0, 0, 0};

float S_Balance_p = 0.5, S_Balance_i = 0.03;
float S_Angle_p = 1.2, S_Angle_d = 0.1;
float S_Speed_p = 25;
float T_Speed_p = 5, T_Speed_i = 0.12; //3.5   0.05
float TE1_Dir_p = 10, TE1_Dir_d = 110;
float TE2_Dir_p = 7.0, TE2_Dir_d = 325;
float TE3_Dir_p = 5.5, TE3_Dir_d = 335;

float SE1_Dir_p = 30, SE1_Dir_d = 240;
float SE2_Dir_p = 9.0, SE2_Dir_d = 300;
float SE3_Dir_p = 4.5, SE3_Dir_d = 310;

float S_Block_Dir_p = 120, S_Block_Dir_d = 520; //float S_Block_Dir_p=50,S_Block_Dir_d=520;
float T_Block_Dir_p = 220, T_Block_Dir_d = 700;

float NP, NI;

struct membership_grade
{
    volatile int16 s;
    volatile int16 m;
    volatile int16 b;
} sub;

/*********************************** PID结构体定义部分 END  ***********************************/

/*********************************** 电磁寻迹定义部分 STAR ***********************************/

int16 E_pst0 = 0, E_last_pst0 = 0, E_llast_pst0 = 0;
uint8 maxnum = 0, minnum = 0, last_minnum = 0, last_maxnum = 0, llast_maxnum = 0, llast_minnum = 0;

uint16 last_result[5] = {0};
int16 E_pst0_save[20] = {0};

uint8 L12 = 0, R12 = 0;
uint8 L23 = 0, R23 = 0;

/*********************************** 电磁寻迹定义部分 END  ***********************************/

/*********************************** 各个标志位等定义部分 STAR ***********************************/
uint8 keyvalue = 0; //按键键值
uint8 lastkeyvalue = 0;

int8 start_car = 0; //用于中断，发车标志位

uint8 static_flag = 1; //用于静态动态切换标志，图像获取标志


int8 stand_flag = 1;
int8 three_flag = 0;



int8 s_balance_count = 0;
int8 s_ang_count = 0;
int8 s_speed_count = 0;
int8 t_speed_count = 0;

uint16 distance = 0, s_distance_sure = 60, t_distance_sure = 60, distance_count = 0, last_distance = 0;

uint8 data_save[9] = 0;
uint8 data_now = 0;
uint8 road_type = 0;

uint8 block = 0;  //1
uint8 broken = 0; //2
uint8 stop = 0;   //3
uint8 back = 0;   //4

int32 Pulse_Sum = 0;
int16 Length = 0;

float zang_set = 0, zang_block_error = 0, zang_block_error_last = 0, block_pwm = 0, block_pwm_last = 0;
int16 ramp_flag = 0, ramp_count = 0, delay_count = 2000;
float zang_err = 0, zang_err_save[20] = 0;

float s_block_14 = 1.3, s_block_23 = 1.1, s_block_angle = 60;
float t_block_14 = 1.1, t_block_23 = 1.1, t_block_angle = 60;
uint8 see_block1 = 1, see_block2 = 1, see_block3 = 1, block_over = 1;

int16 ramp_pst_save = 0;

uint8 RD_Flag = 0;
int16 Stand_Round = 1;
int16 Three_Round = 1;
int16 Round_count = 0;

int16 time_count = 0;
extern int32 Left_Jump;
extern int32 Right_Jump;

float pst_x = 0, pst_y = 0;

/*********************************** 各个标志位定义部分 END  ***********************************/

/*********************************** MPU6050定义部分 STAR ***********************************/

#define Angle ang_get * 100   //滤波前的角度
#define FiltAngle angle * 100 //滤波后的角度，可以在control.c里的卡尔曼滤波中修改参数来调整误差，延时

float A_X, A_Y, A_Z, G_X, G_Y, G_Z; //陀螺仪获取加速度，角速度值
float angle, angle_dot;             //角度与角速度

/*********************************** MPU6050定义部分 END  ***********************************/

/*********************************** 函数声明部分 STAR ***********************************/

void Round_Set(void);
void Dir_Set(void);
void Set_T_Dir_p(void);
void Set_TE1_Dir_p(void);
void Set_TE2_Dir_p(void);
void Set_TE3_Dir_p(void);
void Set_T_Dir_d(void);
void Set_TE1_Dir_d(void);
void Set_TE2_Dir_d(void);
void Set_TE3_Dir_d(void);
void Set_S_Dir_p(void);
void Set_SE1_Dir_p(void);
void Set_SE2_Dir_p(void);
void Set_SE3_Dir_p(void);
void Set_S_Dir_d(void);
void Set_SE1_Dir_d(void);
void Set_SE2_Dir_d(void);
void Set_SE3_Dir_d(void);

void Stand_Gear_Set(void);
void Stand_Speed_110(void);
void Stand_Speed_115(void);
void Stand_Speed_120(void);
void Stand_Speed_125(void);
void Three_Gear_Set(void);
void Three_Speed_120(void);
void Three_Speed_125(void);
void Three_Speed_130(void);
void Three_Speed_135(void);
void Block_Set(void);
void Set_S_Block(void);
void Set_S_Block_50(void);
void Set_S_Block_55(void);
void Set_S_Block_60(void);
void Set_S_Block_65(void);
void Set_S_Block_70(void);
void Set_S_Block_75(void);
void Set_S_Block_80(void);
void Set_T_Block(void);
void Set_T_Block_50(void);
void Set_T_Block_55(void);
void Set_T_Block_60(void);
void Set_T_Block_65(void);
void Set_T_Block_70(void);
void Set_T_Block_75(void);
void Block_Count_Set(void);
void Set_Block_One(void);
void Set_Block_Two(void);
void Set_Block_Three(void);
void Set_Block_Num(void);

void Set_B_Dir(void);
void Set_B_Dir_P(void);
void Set_B_Dir_D(void);

void Data_Init(void);

void PIT0_IRQHandler();
void uart0_handler(void);
void uart3_handler(void);
void Speed_Measure(void);
void GetMPUData();
void AngleCalculate(void);
void Angle_Dot();
void filter();
void E_judge(void);
void Trace(void);
void E_Round_Jude(void);
void E_Round_Control(void);

void Stand_Balance_Control(void);
void Stand_Angle_Control(void);
void Stand_Speed_Control(void);
void Three_Speed_Control(void);
void Three_Ramp_Control(void);
void E_Dir_Control(void);
void Motor_Control(void);

void Road_Type_Analyse(void);
void Broken_Control(void);
void Distance_Measure(void);
void Stand_Block_Control(void);
void Three_Block_Control(void);
void Stop_Car(void);
void Serve_Control(void);

void LCD_Printf(void);

float limit_float(float value, float top, float bottom);
int16 limit_int(int16 value, int16 top, int16 bottom);
uint16 Ab_s(int16 value);
float Ab_s_float(float value);
void StartCar(void);

/*********************************** 函数声明部分 END  ***********************************/

void filter(void) //OK
{
    //注意缓冲区的长度问题

    static int first = 1;
    static ADCn_Ch_e adc_ch[5]={ADC1_SE4a,ADC1_SE5a,ADC1_SE6a,ADC0_SE14,ADC0_SE15};
    CirQueue * cq[5];
    static int sum[5] = {0};

    for(int i=0;i<5;i++)
    {
        cq[i] = Q_Init();
    }

    //如果第一次滤波,那么先把缓冲区采满
    if(first ==1)
    {
        first = 0;
        for(int i=0;i<CIRQUESIZE;i++)
        {
            for(int j=0;j<5;j++)
            {
                Q_Put(cq[i],adc_ave(adc_ch[i], ADC_10bit, 5) >> 2);
            }
        }
    }else{
        for(int i=0;i<5;i++)
        {
            Q_Poll(cq[i]);
            Q_Put(cq[i],adc_ave(adc_ch[i], ADC_10bit, 5) >> 2);
        }
    }


    for(int j=0;j<5;j++)
    {
        for (int i=0;i<CIRQUESIZE;i++) 
        {
            sum[j] += (CIRQUESIZE - i) * cq[j]->base[i];
        }
    }
    

    for(int i=0;i<5;i++)
    {
        indVal[i] = 2 * sum[i] / (CIRQUESIZE + 1) / CIRQUESIZE;
    }

}

void Data_Init(void)
{
    start_car = 0;

    data_now = 0;

    carState = carState;
    lastCarState = lastCarState;
    stand_flag = stand_flag;
    three_flag = three_flag;

    block = 0;
    broken = 0;
    stop = 0;
    back = 0;

    s_ang_count = 0;
    s_speed_count = 0;
    t_speed_count = 0;
    distance_count = 0;

    zero = 0;
    zang_sum = 0;
    zero_count = 0;
    wait_count = 0;
    zero_dot = 0;
    t_zero_dot = 0;
    s_zero_dot = 0; //440

    s_speed_error = 0;
    s_angle_out = 0;
    s_ang_error = 0;
    s_dot_out = 0;
    s_balance_error = 0;
    s_balance_increase = 0;
    s_balance_pwm = 0;
    increment_pwm = 0;
    Actual_pwm = 0;
    RD_Flag = 0;


    time_count = 0;
    delay_count = 2000;
}

void S2T_Init(void)
{
    carState = enThree;
    lastCarState = enThree;
    stand_flag = 0;
    three_flag = 1;

    t_speed_count = 0;

    increment_pwm = 0;
    Actual_pwm = -s_balance_pwm / 2;

    ftm_pwm_duty(FTM3, FTM_CH0, Servo_Up);
}

void T2S_Init(void)
{
    carState = enStand;
    lastCarState = enStand;
    stand_flag = 1;
    three_flag = 0;

    s_ang_count = 0;
    s_speed_count = 0;

    s_speed_error = 0;
    s_angle_out = 0;
    s_ang_error = 0;
    s_dot_out = 0;
    s_balance_error = 0;
    s_balance_increase = 0;
    s_balance_pwm = Actual_pwm / 3; //s_balance_pwm = -Actual_pwm/100;

    if (ramp_flag == 2)
        ftm_pwm_duty(FTM3, FTM_CH0, Servo_Up);
    else
        ftm_pwm_duty(FTM3, FTM_CH0, Servo_Down);
}

void main() //OK
{
    DisableInterrupts;
    LCD_Init(); //液晶初始化
    LCD_CLS();
    LCD_Fill(0x00); //亮屏

    led_init(LED1);

    adc_init(ADC1_SE4a); //电感获取
    adc_init(ADC1_SE5a);
    adc_init(ADC1_SE6a);
    adc_init(ADC0_SE14);
    adc_init(ADC0_SE15);

    ftm_pwm_init(FTM3, FTM_CH0, 100, Servo_Down); //舵机     50 一个频率周期为50个机器周期   精确度为10000    Servo_Middle为占空比分子

    ftm_pwm_init(FTM0, FTM_CH4, 10 * 1000, 0); //右正
    ftm_pwm_init(FTM0, FTM_CH5, 10 * 1000, 0); //右反
    ftm_pwm_init(FTM0, FTM_CH6, 10 * 1000, 0); //左反
    ftm_pwm_init(FTM0, FTM_CH7, 10 * 1000, 0); //左正

    gpio_init(PTD2, GPO, 0); //电机使能

    /*********************************** 五向开关初始化部分 STAR ***********************************/

    gpio_init(PTE25, GPI, 1); //左
    gpio_init(PTE28, GPI, 1); //上
    gpio_init(PTE27, GPI, 1); //右
    gpio_init(PTE24, GPI, 1); //下
    gpio_init(PTE26, GPI, 1); //中

    /*********************************** 五向开关初始化部分 END ************************************/

    ftm_quad_init(FTM1);
    ftm_quad_init(FTM2);

    IIC_init();    //I2C初始化
    InitMPU6050(); //陀螺仪初始化

    uart_init(UART0, 115200);
    uart_init(UART3, 115200);

    /*********************************** 中断设置初始化部分 STAR ***********************************/

    set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler); //设置PIT0的中断复位函数为 PIT0_IRQHandle
    set_vector_handler(UART0_RX_TX_VECTORn, uart0_handler);
    set_vector_handler(UART3_RX_TX_VECTORn, uart3_handler);

    NVIC_SetPriorityGrouping(3);           //设置优先级分组,4bit 抢占优先级,没有亚优先级
    NVIC_SetPriority(UART0_RX_TX_IRQn, 0); //配置优先级
    NVIC_SetPriority(UART3_RX_TX_IRQn, 1);
    NVIC_SetPriority(PIT0_IRQn, 2);

    pit_init_ms(PIT0, 2); //这个时间，即1ms，乘以5，就是control.c里卡尔曼滤波的采样时间，也会影响延时，平衡，精度等问题

    /*********************************** 中断设置初始化部分 END ************************************/

    Data_Init();

    EnableInterrupts;

    LCD_CLS();
    LCD_Print(50, 3, "Init OK");
    systick_delay_ms(1000);

    MENU *topMenu;
    topMenu = CreateMenu("Round", "Block", "Stand_V", "Three_V", "Start", NULL, NULL);
    CreateMenu("1000", "2000", "3000", "4000", "back", &topMenu->left_next, topMenu);
    CreateMenu("Three", "Stand", NULL, NULL, "back", &topMenu->right_next, topMenu);
    CreateMenu("30", "60", "90", NULL, "back", &topMenu->up_next, topMenu);
    CreateMenu(NULL, NULL, "about", "author", "back", &topMenu->down_next, topMenu);

    AddHandle("Start", StartCar);
    MenuStart();
}

void StartCar(void)
{
    carState = enThree;
    enable_irq(PIT0_IRQn);
    systick_delay_ms(100);
    motor_1;
    start_car = 1;
    LCD_CLS();
    LCD_Print(0, 0, "Running");
    while (1)
        ;
}

void PIT0_IRQHandler() //OK
{
    PIT_Flag_Clear(PIT0); //清除标志位

    if (zero == 0)
    {
        if (wait_count < 500)
            wait_count++;
        if (wait_count == 500)
        {
            led(LED1, LED_ON);
            GetMPUData();
            AngleCalculate();
            Kalman_Filter(ang_get, G_Y, &angle, &angle_dot);
            Angle_Dot();
            zero_count++;
            zero_dot += zang_dot;
            //      g_sum+=A_Z/cos(angle*PI/180);
            if (zero_count == 500)
            {
                t_zero_dot = zero_dot / 500;
                s_zero_dot = zero_dot / 450; //440
                zero_count = 0;
                wait_count = 0;
                led(LED1, LED_OFF);
                zero = 1;
            }
        }
    }
    else
    {
        if ((delay_count < 2000) && (start_car == 1))
            delay_count++;
        Road_Type_Analyse();
        if (block < 2)
            Serve_Control();

        GetMPUData();
        AngleCalculate();
        Kalman_Filter(ang_get, G_Y, &angle, &angle_dot);
        Angle_Dot();

        if (carState == enStand)
        {
            Stand_Balance_Control();

            s_ang_count++;
            s_speed_count++;

            if (s_ang_count == 5) //20  Stand_Speed_Control();
            {
                s_ang_count = 0;
                Speed_Measure();
                if (block > 1 || RD_Flag == 5 || RD_Flag == 6 || RD_Flag == 9 || RD_Flag == 10)
                    Distance_Measure();
                Stand_Angle_Control();
            }

            if (s_speed_count == 25) //20  Stand_Speed_Control();
            {
                s_speed_count = 0;
                Stand_Speed_Control();
            }
        }
        else if (carState == enThree)
        {
            t_speed_count++;
            distance_count++;
            if (distance_count == 50)
            {
                last_distance = distance;
                distance_count = 0;
            }
            if (t_speed_count == 5) //20
            {
                t_speed_count = 0;
                Speed_Measure();
                if (block > 1 || RD_Flag == 5 || RD_Flag == 6 || RD_Flag == 9 || RD_Flag == 10)
                    Distance_Measure();
                Three_Speed_Control();
            }
        }

        INducUpdate();
        Trace();
        E_Dir_Control();

        if (stop == 0)
            Motor_Control();
    }
}

void uart0_handler(void) //OK
{
    UARTn_e uratn = UART0;

    if (UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK) //接收数据寄存器满
    {
        data_save[data_now] = UART_D_REG(UARTN[uratn]);
        if (data_now < 9)
            data_now++;
        if (data_now == 9)
        {
            data_now = 0;
            distance = data_save[2] + data_save[3] * 256;
        }
    }
}

void uart3_handler(void)
{
    UARTn_e uratn = UART3;

    if (UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK) //接收数据寄存器满
    {
        road_type = UART_D_REG(UARTN[uratn]);
        switch (road_type)
        {
        case 1:
        {
            if (block == 0)
                block = 1;
            break;
        }
        case 2:
        {
            if (broken == 0 && block == 0 && start_car == 1 && ramp_flag == 0)
                broken = 1;
            break;
        }
            //        case 3:
            //            {
            //                if(stop==0)
            //                    stop = 1;
            //                break;
            //            }
        case 4:
        {
            if (back == 0 && broken == 2)
                back = 1;
            break;
        }
        }
    }
}


void GetMPUData() //OK
{
    Get_Gyro();
    Get_AccData();
    A_X = mpu_acc_x;
    A_Y = mpu_acc_y;
    A_Z = mpu_acc_z;

    G_X = mpu_gyro_x;
    G_Y = mpu_gyro_y;
    G_Z = mpu_gyro_z;
}

void AngleCalculate() //OK
{
    ang_get = -atan2(A_X, A_Z) * 180 / PI;
}

void Angle_Dot() //OK
{
    zang_dot = G_Z / cos(angle * PI / 180) / 60.0;

    if (block > 1 || RD_Flag > 2)
    {
        if (carState == enStand)
            zang_error = (zang_dot - s_zero_dot) / 17.2;
        else if (carState < 0)
            zang_error = (zang_dot - t_zero_dot) / 16.6;
        zang_sum += zang_error;
    }
}

void Stand_Balance_Control(void) //坡道有关
{
    if (start_car == 1 && stand_flag == 1)
    {
        S_Balance.p = S_Balance_p;
        S_Balance.i = S_Balance_i;

        s_balance_error = angle_dot * 10 - s_dot_out;
        s_balance_increase = S_Balance.p * (s_balance_error - s_balance_error_last) + S_Balance.i * s_balance_error;
        s_balance_error_last = s_balance_error;
        s_balance_pwm += s_balance_increase;

        if (delay_count < 1500)
            s_balance_pwm = limit_int(s_balance_pwm, 3000, -3000);
        else
            s_balance_pwm = limit_float(s_balance_pwm, S_Balance_Max, S_Balance_Min); //平衡PD环的上下限
    }
}

void Stand_Angle_Control(void) //OK
{
    if (start_car == 1 && stand_flag == 1)
    {
        S_Angle.p = S_Angle_p;
        S_Angle.d = S_Angle_d;

        s_ang_error = s_angle_out - angle * 100;

        s_dot_out = S_Angle.p * s_ang_error + S_Angle.d * (s_ang_error - s_ang_error_last);
        s_ang_error_last = s_ang_error;

        if (delay_count < 1500)
            s_dot_out = limit_int(s_dot_out, 3000, -3000);
        else
            s_dot_out = limit_float(s_dot_out, S_Angle_Max, S_Angle_Min);
    }
}

//void Stand_Speed_Control(void)  //坡道有关
//{
//    if(start_car==1&&stand_flag==1)
//    {
//        S_Speed.p = S_Speed_p;
//
//        s_speed_error = s_speed_set-speed_get;
//        s_angle_out = S_ang_set*100-S_Speed.p*s_speed_error;
//
//        if((speed_get<60)&&(delay_count==2000)&&(ramp_flag==0))
//            S_ang_set = 35;
//        else
//            S_ang_set = 37;
//
//        if(ramp_flag==2||ramp_flag==3)
//            s_angle_out = limit_int(s_angle_out,3000,2000); //Min大，上坡快，易翻车
//        else
//            s_angle_out = limit_int(s_angle_out,S_Speed_Max,S_Speed_Min);
//    }
//}

void Stand_Speed_Control(void) //坡道有关
{
    if (start_car == 1 && stand_flag == 1)
    {
        if (delay_count < 1500)
            S_Speed.p = S_Speed_p / 2;
        else
            S_Speed.p = S_Speed_p;

        if ((carSpeed < 60) && (delay_count == 2000) && (ramp_flag == 0))
            S_ang_set = 35;
        else
            S_ang_set = 37;

        s_speed_error = speedSet.stand - carSpeed;
        s_angle_out = S_ang_set * 100 - S_Speed.p * s_speed_error;

        if (ramp_flag == 2)
            s_angle_out = limit_int(s_angle_out, 4300, 3800);
        else if (ramp_flag == 3)
            s_angle_out = limit_int(s_angle_out, 3900, 3400);

        if (delay_count < 1500)
            s_angle_out = limit_int(s_angle_out, 3800, 3600);
        else
            s_angle_out = limit_int(s_angle_out, S_Speed_Max, S_Speed_Min);
    }
}

void Three_Speed_Control(void) //OK
{
    if (start_car == 1 && three_flag == 1)
    {
        T_Speed.p = T_Speed_p;

        Speed.err = speedSet.three - carSpeed;
        T_Speed.i = T_Speed_i + Speed.err / 1000.0;

        if (Speed.err > 0)
        {
            nP = T_Speed.p * (Speed.err - Speed.err_last);
            nI = T_Speed.i * Speed.err;
        }
        else
        {
            if (Speed.err - Speed.err_last < 0)
                nP = T_Speed.p * (Speed.err - Speed.err_last);
            else
                nP = -T_Speed.p * (Speed.err - Speed.err_last) / 4;

            nI = T_Speed.i * Speed.err * 1.2;
        }

        increment_pwm = nP + nI;
        Actual_pwm += increment_pwm;

        if (ramp_flag == 7)
            Actual_pwm = limit_int(Actual_pwm, 800, 0);

        if (ramp_flag == 8)
        {
            Actual_pwm = 400 - 40 * (angle - 5);
            Actual_pwm = limit_int(Actual_pwm, 400, 0);
        }

        else
            Actual_pwm = limit_int(Actual_pwm, T_Speed_Max, T_Speed_Min);

        Speed.err_last = Speed.err;
    }
}

//void Three_Ramp_Control(void)
//{
//    if(ramp_flag==0)
//    {
//        led(LED1,LED_ON);
//        ramp_flag = 1;
//    }
//    else if(ramp_flag==1)
//    {
//        S_ang_set = 50;
//        Car_State = -Car_State;
//        ramp_flag = 2;
//    }
//    else if(ramp_flag==2)
//    {
//        ramp_count++;
//        if(ramp_count==75)
//        {
//            led(LED1,LED_OFF);
//            ramp_count = 0;
//            ramp_flag = 3;
//        }
//    }
//    else if(ramp_flag==3)
//    {
//        ramp_count++;
//        if(ramp_count==350)
//        {
//            led(LED1,LED_ON);
//            ramp_count = 0;
//            ramp_flag = 4;
//            ramp_pst_save = 0;
//        }
//    }
//    else if(ramp_flag==4)
//    {
//        Car_State = -Car_State;
//        ramp_flag = 5;
//    }
//    else if(ramp_flag==5)
//    {
//        Actual_pwm = 200;
//        ramp_flag = 6;
//    }
//    else if(ramp_flag==6)
//    {
//      if(angle<-5)
//      {
//        ramp_flag = 7;
//        delay_count = 1600;
//      }
//    }
//    else if(ramp_flag==7)
//    {
//        if(delay_count==2000)  //150 200 250
//        {
//            delay_count = 1000;
//            ramp_flag = 0;
//            S_ang_set = 37;
//            led(LED1,LED_OFF);
//        }
//    }
//}

void Three_Ramp_Control(void)
{
    if (ramp_flag == 0)
    {
        led(LED1, LED_ON);
        ramp_flag = 1;
    }
    else if (ramp_flag == 1)
    {
        S_ang_set = 50;
        carState == enThree?enStand:enThree;
        ramp_flag = 2;
    }
    else if (ramp_flag == 2)
    {
        ramp_count++;
        if (ramp_count == 75)
        {
            led(LED1, LED_OFF);
            ramp_count = 0;
            ramp_flag = 3;
            ramp_pst_save = indVal[1];
        }
    }
    else if (ramp_flag == 3)
    {
        if (indVal[1] < ramp_pst_save - 30)
        {
            ramp_flag = 4;
        }
    }
    else if (ramp_flag == 4)
    {
        ramp_count++;
        if (ramp_count == 350)
        {
            led(LED1, LED_ON);
            ramp_count = 0;
            ramp_flag = 5;
            ramp_pst_save = 0;
        }
    }
    else if (ramp_flag == 5)
    {
        carState == enThree?enStand:enThree;
        ramp_flag = 6;
    }
    else if (ramp_flag == 6)
    {
        Actual_pwm = 200;
        ramp_flag = 7;
    }
    else if (ramp_flag == 7)
    {
        if (angle < -5)
        {
            ramp_flag = 8;
            delay_count = 1600;
        }
    }
    else if (ramp_flag == 8)
    {
        if (delay_count == 2000) //150 200 250
        {
            delay_count = 1000;
            ramp_flag = 0;
            S_ang_set = 37;
            led(LED1, LED_OFF);
        }
    }
}

void Trace(void) //OK
{
    E_Round_Jude();
    if (RD_Flag != 0)
    {
        E_Round_Control();
    }
    else
    {
        E_judge();
    }
}

void E_judge(void) //OK
{
    int8 i = 0;
    int8 pst_count = 24;

    if (carState == enStand)
    {
        if (indVal[0] >= 20 || indVal[1] >= 20 || indVal[2] >= 20)
        {
            for (i = 0; i < 3; i++)
            {
                if (indVal[i] > indVal[maxnum])
                    maxnum = i;
                if (indVal[i] < indVal[minnum])
                    minnum = i;
            }
            if ((abs(indVal[0] - indVal[1]) <= 3) && E_pst0_save[8] <= 26)
                maxnum = 1;
            else if ((abs(indVal[1] - indVal[2]) <= 3) && E_pst0_save[8] >= -26) //刚入右弯可能出现
                maxnum = 1;
            if ((indVal[0] >= 130 && indVal[1] >= 130))
            {
                if (((last_maxnum == 1) && (maxnum == 0)) || (abs(indVal[0] - indVal[1]) <= 3))
                    L12 = (indVal[0] + indVal[1]) >> 1;
                if (L12 >= 160)
                    L12 = 160; //170
                if (L12 <= 140)
                    L12 = 140; //142
            }
            if ((indVal[2] >= 130 && indVal[1] >= 130))
            {
                if (((last_maxnum == 1) && (maxnum == 2)) || (abs(indVal[2] - indVal[1]) <= 3))
                    R12 = (indVal[2] + indVal[1]) >> 1;

                if (R12 >= 160)
                    R12 = 160; //170
                if (R12 <= 140)
                    R12 = 140; //142
            }
            if ((maxnum == 0 && (indVal[0] != indVal[2])) && (abs(indVal[1] - 56) <= 2))
            {
                L23 = indVal[0];
                if (L23 >= 170)
                    L23 = 170; //170
                if (L23 <= 135)
                    L23 = 135; //160
            }
            if ((maxnum == 2 && (indVal[0] != indVal[2])) && (abs(indVal[1] - 56) <= 2))
            {
                R23 = indVal[2];
                if (R23 >= 170)
                    R23 = 170;
                if (R23 <= 135)
                    R23 = 135; //160
            }
            if ((abs(indVal[0] - last_result[0]) <= 30) && (abs(indVal[1] - last_result[1]) <= 30) && (abs(indVal[2] - last_result[2]) <= 30)) //非坡道
            {                                                                                                                                  //25                                 //25                                 //25
                //=====================pst0计算=====================//
                if (maxnum == 1) //中间电感1最大 ,并且中间电感和左右两边其中一个不相等
                {
                    E_pst0 = (int8)((indVal[0] - indVal[2]) / 5);

                    if (E_pst0 <= -pst_count)
                        E_pst0 = -pst_count; // pst0= -8~+8
                    if (E_pst0 >= pst_count)
                        E_pst0 = pst_count;
                }
                else if (maxnum == 0) //左边电感0最大        8  13  12
                {
                    if (indVal[1] < 56) //  pst0= 24~38
                    {
                        E_pst0 = (int8)(pst_count + 16 + (L23 - indVal[0]) / 5);
                        if (E_pst0 <= (pst_count + 16))
                            E_pst0 = pst_count + 16;
                    }
                    else
                    {
                        E_pst0 = (int8)(pst_count + (L12 - indVal[1]) * 15 / (L12 - 56)); //pst0= 9~23
                        if (E_pst0 <= pst_count)
                            E_pst0 = pst_count;
                    }
                }
                else //右边电感2最大
                {
                    if (indVal[1] < 56)
                    {
                        E_pst0 = (int8) - (pst_count + 16 + (R23 - indVal[2]) / 5);
                        if (E_pst0 >= -(pst_count + 16))
                            E_pst0 = -(pst_count + 16);
                    }
                    else
                    {
                        E_pst0 = (int8) - (pst_count + (R12 - indVal[1]) * 15 / (R12 - 56));
                        if (E_pst0 >= -pst_count)
                            E_pst0 = -pst_count;
                    }
                }
                if (E_pst0 >= 40)
                {
                    E_pst0 = 40;
                }
                if (E_pst0 <= -40)
                {
                    E_pst0 = -40;
                }
            }
            else
            {
                E_pst0 = E_last_pst0;
                minnum = last_minnum;
                maxnum = last_maxnum;
                for (i = 0; i < 5; i++)
                    indVal[i] = (indVal[i] + last_result[i]) / 2;
            }
        }
        else
        {
            E_pst0 = E_last_pst0;
            maxnum = last_maxnum;
            minnum = last_minnum;
            if (block == 0)
            {
                motor_0;
            }
        }
        if ((Stand_Round == 0) && (indVal[1] > 195) && ((indVal[0] > 110) || (indVal[2] > 110))) //(indVal[1]>210)&&((indVal[0]>120)||(indVal[2]>120))
        {
            Round_count = 50;
        }
        else
        {
            Round_count--;
            if (Round_count > 0)
                E_pst0 = limit_int(E_pst0, 3, -3);
        }
    }
    else if (carState == enThree) //三轮
    {
        //if(indVal[0]>=20||indVal[1]>=20||indVal[2]>=20)
        if (1) //即使超出赛道也继续跑,这里我把转向的控制给屏蔽掉了
        {      /*
      for(i=0;i<3;i++)
      {
        if(indVal[i]>indVal[maxnum]) maxnum=i;
        if(indVal[i]<indVal[minnum]) minnum=i;
      }
      if((abs(indVal[0]-indVal[1])<=3)&&E_pst0_save[8]<=26)
        maxnum=1;
      else if((abs(indVal[1]-indVal[2])<=3)&&E_pst0_save[8]>=-26)//刚入右弯可能出现
        maxnum=1;
      if((indVal[0]>=120&&indVal[1]>=120))
      {
        if(((last_maxnum==1)&&(maxnum==0))||(abs(indVal[0]-indVal[1])<=3))
          L12=(indVal[0]+indVal[1])>>1;
        if(L12>=160) L12=160;   //170
        if(L12<=130) L12=130;   //142
      }
      if((indVal[2]>=115&&indVal[1]>=115))
      {
        if(((last_maxnum==1)&&(maxnum==2))||(abs(indVal[2]-indVal[1])<=3))
          R12=(indVal[2]+indVal[1])>>1;
        
        if(R12>=160) R12=160;  //170
        if(R12<=130) R12=130;  //142
      }
      if((maxnum==0&&(indVal[0]!=indVal[2]))&&(abs(indVal[1]-56)<=2))
      {
        L23=indVal[0];
        if(L23>=170) L23=170;  //170
        if(L23<=126) L23=126;  //160  
      }
      if((maxnum==2&&(indVal[0]!=indVal[2]))&&(abs(indVal[1]-56)<=2))
      {
        R23=indVal[2];
        if(R23>=170) R23=170;
        if(R23<=126) R23=126;  //160
      }
      if((abs(indVal[0]-last_result[0])<=30)&&(abs(indVal[1]-last_result[1])<=30)&&(abs(indVal[2]-last_result[2])<=30))//非坡道
      {                                //25                                 //25                                 //25
        //=====================pst0计算=====================//
        if(maxnum==1)//中间电感1最大 ,并且中间电感和左右两边其中一个不相等
        {
          E_pst0=(int8)((indVal[0]-indVal[2])/4.3);
          if(E_pst0<=-pst_count)    E_pst0=-pst_count;            // pst0= -8~+8
          if(E_pst0>=pst_count)     E_pst0=pst_count;
        }
        else if(maxnum==0)//左边电感0最大        8  13  12
        {
          if(indVal[1]<56)                                           //  pst0= 24~38
          {
            E_pst0=(int8)(pst_count+16+(L23-indVal[0])/3.7);
            if(E_pst0<=(pst_count+16))    E_pst0=pst_count+16;
          }
          else
          {
            E_pst0=(int8)(pst_count+(L12-indVal[1])*15/(L12-58));   //pst0= 9~23
            if(E_pst0<=pst_count)   E_pst0=pst_count;
          }
        }
        else //右边电感2最大
        {
          if(indVal[1]<56)
          {
            E_pst0=(int8)-(pst_count+16+(R23-indVal[2])/3.7);
            if(E_pst0>=-(pst_count+16)) E_pst0=-(pst_count+16);
          }
          else
          {
            E_pst0=(int8)-(pst_count+(R12-indVal[1])*15/(R12-58));
            if(E_pst0>=-pst_count) E_pst0=-pst_count;  
          }
        }
        if(E_pst0>=40)                              {E_pst0=40;}                          
        if(E_pst0<=-40)                             {E_pst0=-40;} 
      }
      else
      {
        E_pst0=E_last_pst0;
        minnum=last_minnum;
        maxnum=last_maxnum;
        for(i=0;i<5;i++)
          indVal[i]=(indVal[i]+last_result[i])/2;
      }
      */
        }
        else
        {
            E_pst0 = E_last_pst0;
            maxnum = last_maxnum;
            minnum = last_minnum;
            if (block == 0)
            {
                motor_0;
            }
        }
        if ((Three_Round == 0) && (indVal[1] > 190 + 5.0 * (37.0 - angle)) && ((indVal[0] > 120) || (indVal[2] > 120))) //(indVal[1]>210)&&((indVal[0]>120)||(indVal[2]>120))
        {
            Round_count = 50;
        }
        else
        {
            Round_count--;
            if (Round_count > 0)
                E_pst0 = limit_int(E_pst0, 3, -3);
        }
    }

    llast_maxnum = last_maxnum;
    last_maxnum = maxnum;
    llast_minnum = last_minnum;
    last_minnum = minnum;

    E_llast_pst0 = E_last_pst0;
    E_last_pst0 = E_pst0;

    for (i = 0; i < 5; i++)
        last_result[i] = indVal[i];
    for (i = 19; i > 0; i--)
    {
        E_pst0_save[i] = E_pst0_save[i - 1];
    }
    E_pst0_save[0] = E_pst0;
}

void E_Dir_Control(void) //OK
{
    int16 D_Control, P_Control, pst_error;
    pst_error = E_pst0 - (E_pst0_save[15] + 8 * E_pst0_save[16] + E_pst0_save[17]) / 10;

    if (carState == enStand)
    {
        if (abs(E_pst0) < 6)
        {
            P_Control = (int16)(SE1_Dir_p * E_pst0 * abs(E_pst0));
            D_Control = (int16)(SE1_Dir_d * pst_error);
        }
        else if (abs(E_pst0) < 15)
        {
            P_Control = (int16)(SE2_Dir_p * E_pst0 * abs(E_pst0));
            D_Control = (int16)(SE2_Dir_d * pst_error);
        }
        else
        {
            P_Control = (int16)(SE3_Dir_p * E_pst0 * abs(E_pst0));
            D_Control = (int16)(SE3_Dir_d * pst_error);
        }

        NP = P_Control;
        NI = D_Control;

        E_Dire.pwm = NP + NI;
        E_Dire.last_pwm = E_Dire.pwm;

        E_Dire.pwm = limit_int(E_Dire.pwm, SE_Dir_Max, SE_Dir_Min);
    }
    else
    {
        if (abs(E_pst0) < 7)
        {
            P_Control = (int16)(TE1_Dir_p * E_pst0 * abs(E_pst0));
            D_Control = (int16)(TE1_Dir_d * pst_error);
        }
        else if (abs(E_pst0) < 17)
        {
            P_Control = (int16)(TE2_Dir_p * E_pst0 * abs(E_pst0));
            D_Control = (int16)(TE2_Dir_d * pst_error);
        }
        else
        {
            P_Control = (int16)(TE3_Dir_p * E_pst0 * abs(E_pst0));
            D_Control = (int16)(TE3_Dir_d * pst_error);
        }

        NP = P_Control;
        NI = D_Control;

        E_Dire.pwm = NP + NI;
        E_Dire.last_pwm = E_Dire.pwm;

        E_Dire.pwm = limit_int(E_Dire.pwm, TE_Dir_Max, TE_Dir_Min);
    }
}

void E_Round_Jude(void) //OK
{
    int i = 0;
    if (carState == enStand) //OK
    {
        if ((RD_Flag == 0) && (indVal[0] > 60) && (indVal[1] > 190 + 5.0 * (37.0 - angle)) && (indVal[2] > 60))
        {
            if ((indVal[3] < 160) && ((indVal[3] - indVal[4]) > 5) && (indVal[0] > 110))
            {
                RD_Flag = 1;

                led(LED1, LED_ON);
            }
            else if ((indVal[4] < 160) && ((indVal[4] - indVal[3]) > 5) && (indVal[2] > 110))
            {
                RD_Flag = 2;

                led(LED1, LED_ON);
            }
        }

        if (RD_Flag == 1)
        {
            if ((abs(indVal[3] - indVal[4]) < 7) && (indVal[3] < 16) && (indVal[4] < 16))
            {
                RD_Flag = 3;
                zang_sum = 0;
                led(LED1, LED_OFF);
            }
        }
        if (RD_Flag == 2)
        {
            if ((abs(indVal[3] - indVal[4]) < 7) && (indVal[3] < 16) && (indVal[4] < 16))
            {
                RD_Flag = 4;
                zang_sum = 0;
                led(LED1, LED_OFF);
            }
        }

        if (RD_Flag == 3)
        {
            if (zang_sum > 40)
            {
                RD_Flag = 5;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }
        else if (RD_Flag == 4)
        {
            if (zang_sum < -40)
            {
                RD_Flag = 6;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }

        if (RD_Flag == 5)
        {
            if (zang_sum > 280) //if((zang_sum>260)&&((indVal[0]-indVal[2])>10))
            {
                RD_Flag = 7;
                pst_x = Length / 6;
                led(LED1, LED_OFF);
            }
        }
        else if (RD_Flag == 6)
        {
            if (zang_sum < -280)
            {
                RD_Flag = 8;
                pst_x = Length / 6;
                led(LED1, LED_OFF);
            }
        }

        if (RD_Flag == 7)
        {
            if (((indVal[3] - indVal[4]) > 5) && (zang_sum > 358) && (indVal[1] > 220)) //(zang_sum>352)&&(indVal[1]>220)&&(Car_State<0))||((zang_sum>352)&&(indVal[1]>220)&&(Car_State>0)
            {                                                                           //(zang_sum>334)
                RD_Flag = 9;
                for (i = 0; i < 20; i++)
                    E_pst0_save[i] = 0;
                Length = 0;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }
        else if (RD_Flag == 8)
        {
            if (((indVal[4] - indVal[3]) > 5) && (zang_sum < -358) && (indVal[1] > 220)) //((zang_sum<-352)&&(indVal[1]>220)&&(Car_State<0))||((zang_sum<-352)&&(indVal[1]>220)&&(Car_State>0))
            {                                                                            //&&(abs(result0_save-indVal[0])<20)&&(abs(result2_save-indVal[2])<20)
                RD_Flag = 10;
                for (i = 0; i < 20; i++)
                    E_pst0_save[i] = 0;
                Length = 0;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }

        if ((RD_Flag == 9) && (Length > 105) && (indVal[1] < 235)) //&&(indVal[3]<10)&&(indVal[4]<10)  +pst_x
        {
            RD_Flag = 0;
            Pulse_Sum = 0;
            Length = 0;
            pst_x = 0;
            zang_error = 0;
            zang_sum = 0;
            led(LED1, LED_OFF);
        }
        else if ((RD_Flag == 10) && (Length > 105) && (indVal[1] < 235))
        {
            RD_Flag = 0;
            Pulse_Sum = 0;
            Length = 0;
            pst_x = 0;
            zang_error = 0;
            zang_sum = 0;
            led(LED1, LED_OFF);
        }
    }
    else if (carState == enThree)
    {
        if ((RD_Flag == 0) && (indVal[0] > 60) && (indVal[1] > 200) && (indVal[2] > 60)) //if((RD_Flag==0)&&(indVal[0]>60)&&(indVal[1]>195)&&(indVal[2]>60))
        {
            if ((indVal[3] < 160) && ((indVal[3] - indVal[4]) > 25) && (indVal[0] > 120)) //if((indVal[3]<160)&&((indVal[3]-indVal[4])>25)&&(indVal[0]>110))
            {
                RD_Flag = 1;

                led(LED1, LED_ON);
            }
            else if ((indVal[4] < 160) && ((indVal[4] - indVal[3]) > 25) && (indVal[2] > 120))
            {
                RD_Flag = 2;

                led(LED1, LED_ON);
            }
        }

        if (RD_Flag == 1)
        {
            if ((abs(indVal[3] - indVal[4]) < 7) && indVal[3] < 13 && indVal[4] < 13) //||(indVal[4]<10&&indVal[3]>40)
            {
                RD_Flag = 3;
                zang_sum = 0;
                led(LED1, LED_OFF);
            }
        }
        if (RD_Flag == 2)
        {
            if ((abs(indVal[3] - indVal[4]) < 7) && indVal[3] < 13 && indVal[4] < 13) //||(indVal[3]<10&&indVal[4]>40)
            {
                RD_Flag = 4;
                zang_sum = 0;
                led(LED1, LED_OFF);
            }
        }

        if (RD_Flag == 3)
        {
            if (zang_sum > 40)
            {
                RD_Flag = 5;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }
        else if (RD_Flag == 4)
        {
            if (zang_sum < -40)
            {
                RD_Flag = 6;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }

        if (RD_Flag == 5)
        {
            if (zang_sum > 280)
            {
                RD_Flag = 7;
                pst_x = Length / 6;
                led(LED1, LED_OFF);
            }
        }
        else if (RD_Flag == 6)
        {
            if (zang_sum < -280)
            {
                RD_Flag = 8;
                pst_x = Length / 6;
                led(LED1, LED_OFF);
            }
        }

        if (RD_Flag == 7)
        {
            if (((indVal[3] - indVal[4]) > 5) && (zang_sum > 358) && (indVal[1] > 230)) //((indVal[0]-indVal[2])>30)&&
            {
                RD_Flag = 9;
                for (i = 0; i < 20; i++)
                    E_pst0_save[i] = 0;
                Length = 0;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }
        else if (RD_Flag == 8)
        {
            if (((indVal[4] - indVal[3]) > 5) && (zang_sum < -358) && (indVal[1] > 230)) //((indVal[2]-indVal[0])>30)&&
            {
                RD_Flag = 10;
                for (i = 0; i < 20; i++)
                    E_pst0_save[i] = 0;
                Length = 0;
                Pulse_Sum = 0;
                led(LED1, LED_ON);
            }
        }

        if ((RD_Flag == 9) && (Length > 100) && (indVal[1] < 200)) //&&(indVal[3]<10)&&(indVal[4]<10)
        {
            RD_Flag = 0;
            Pulse_Sum = 0;
            Length = 0;
            pst_x = 0;
            zang_error = 0;
            zang_sum = 0;
            led(LED1, LED_OFF);
        }
        else if ((RD_Flag == 10) && (Length > 100) && (indVal[1] < 200))
        {
            RD_Flag = 0;
            Pulse_Sum = 0;
            Length = 0;
            pst_x = 0;
            zang_error = 0;
            zang_sum = 0;
            led(LED1, LED_OFF);
        }
    }
}

void E_Round_Control(void) //OK
{
    static int16 i = 0;

    if (carState == enStand) //OK
    {
        if (RD_Flag == 5 || RD_Flag == 6)
        {
            E_judge();
            if (RD_Flag == 5)
            {
                E_pst0 = limit_int(E_pst0, 20, -20);
            }
            else if (RD_Flag == 6)
            {
                E_pst0 = limit_int(E_pst0, 20, -20);
            }
        }
        else if (RD_Flag != 5 && RD_Flag != 6)
        {
            if (RD_Flag == 1)
            {
                E_pst0 = (0.9 * (indVal[0] - 1.5 * indVal[3]) - (indVal[2] - indVal[4])) / 10; //10

                E_pst0 = limit_int(E_pst0, 3, -4); //2,-1
            }
            else if (RD_Flag == 2)
            {
                E_pst0 = -((0.9 * (indVal[2] - 1.5 * indVal[4]) - (indVal[0] - indVal[3])) / 10); //10

                E_pst0 = limit_int(E_pst0, 4, -3); //2,-1
            }
            else if (RD_Flag == 3)
            {
                if ((indVal[3] - 2 * (angle - 30) < 60) && (zang_sum < 7)) //if((indVal[3]-3*(angle-30)<30)&&(zang_sum<6))
                    E_pst0 = (indVal[3] - 2 * (angle - 30)) / 13;          //10
                else
                    E_pst0 = indVal[3] / 4; //E_pst0=indVal[3]/2;

                E_pst0 = limit_int(E_pst0, 20, -1);
            }
            else if (RD_Flag == 4)
            {
                if ((indVal[4] - 2 * (angle - 30) < 60) && (zang_sum > -7)) //if((indVal[4]-3*(angle-30)<30)&&(zang_sum>-6))
                    E_pst0 = -((indVal[4] - 2 * (angle - 30)) / 13);        //10
                else
                    E_pst0 = -(indVal[4] / 4); //E_pst0=-(indVal[4]/2);

                E_pst0 = limit_int(E_pst0, 1, -20);
            }
            else if (RD_Flag == 7)
            {
                if (indVal[4] > 60)
                    E_pst0 = (indVal[0] - indVal[2] + (indVal[4] - 60)) / 5; //40
                else
                    E_pst0 = (indVal[0] - indVal[2]) / 5;

                E_pst0 = limit_int(E_pst0, 20, -2); //sign/6
            }
            else if (RD_Flag == 8)
            {
                if (indVal[3] > 60)
                    E_pst0 = -((indVal[2] - indVal[0] + (indVal[3] - 60)) / 5); //40
                else
                    E_pst0 = -(indVal[2] - indVal[0]) / 5;

                E_pst0 = limit_int(E_pst0, 2, -20); //sign/6
            }
            else if (RD_Flag == 9)
            {
                if (Length > pst_x)
                {
                    E_pst0 = (55 - (indVal[2] - (angle - 30))) / 3;
                    E_pst0 = limit_int(E_pst0, 9, -9);
                }
                else
                {
                    E_pst0 = ((indVal[0] - 0.5 * indVal[3]) - (indVal[2] - indVal[4])) / 8; //0.8
                    E_pst0 = limit_int(E_pst0, 4, -5);
                }
            }
            else if (RD_Flag == 10)
            {
                if (Length > pst_x)
                {
                    E_pst0 = -((55 - (indVal[0] - (angle - 30))) / 3);
                    E_pst0 = limit_int(E_pst0, 9, -9);
                }
                else
                {
                    E_pst0 = -(((indVal[2] - 0.5 * indVal[4]) - (indVal[0] - indVal[3])) / 8); //0.8
                    E_pst0 = limit_int(E_pst0, 5, -4);
                }
            }

            for (i = 0; i < 5; i++)
                last_result[i] = indVal[i];
            for (i = 19; i > 0; i--)
                E_pst0_save[i] = E_pst0_save[i - 1];
        }

        E_llast_pst0 = E_last_pst0;
        E_last_pst0 = E_pst0;

        E_pst0_save[0] = E_pst0;
    }
    else if (carState == enThree)
    {
        if (RD_Flag == 5 || RD_Flag == 6)
        {
            E_judge();
            if (RD_Flag == 5)
            {
                E_pst0 = limit_int(E_pst0, 24, -24);
            }
            else if (RD_Flag == 6)
            {
                E_pst0 = limit_int(E_pst0, 24, -24);
            }
        }
        else if (RD_Flag != 5 && RD_Flag != 6)
        {
            if (RD_Flag == 1)
            {
                E_pst0 = (0.8 * (indVal[0] - 1.5 * indVal[3]) - (indVal[2] - indVal[4])) / 8; //E_pst0 = (0.8*(indVal[0]-1.5*indVal[3])-(indVal[2]-indVal[4]))/8;

                E_pst0 = limit_int(E_pst0, 3, -5);
            }
            else if (RD_Flag == 2)
            {
                E_pst0 = -((0.8 * (indVal[2] - 1.5 * indVal[4]) - (indVal[0] - indVal[3])) / 8); //E_pst0 = -((0.8*(indVal[2]-1.5*indVal[4])-(indVal[0]-indVal[3]))/8);

                E_pst0 = limit_int(E_pst0, 5, -3);
            }
            else if (RD_Flag == 3)
            {
                if ((indVal[3] < 40) && (zang_sum < 6))     //if((indVal[3]<20)&&(zang_sum<5))
                    E_pst0 = (indVal[3] - indVal[4]) / 6.5; //E_pst0=(indVal[3]-indVal[4])/4;
                else
                    E_pst0 = indVal[3] / 3; //E_pst0=indVal[3]/2.5;  3

                E_pst0 = limit_int(E_pst0, 24, -2);
            }
            else if (RD_Flag == 4)
            {
                if ((indVal[4] < 40) && (zang_sum > -6))
                    E_pst0 = -(indVal[4] - indVal[3]) / 6.5;
                else
                    E_pst0 = -indVal[4] / 3; //3

                E_pst0 = limit_int(E_pst0, 2, -24);
            }
            else if (RD_Flag == 7)
            {
                if (indVal[4] > 60)
                    E_pst0 = (indVal[0] - indVal[2] + indVal[4] / 5) / 5; //40
                else
                    E_pst0 = (indVal[0] - indVal[2]) / 5;

                E_pst0 = limit_int(E_pst0, 24, -2); //sign/6
            }
            else if (RD_Flag == 8)
            {
                if (indVal[3] > 60)
                    E_pst0 = -((indVal[2] - indVal[0] + indVal[3] / 5) / 5); //40
                else
                    E_pst0 = -(indVal[2] - indVal[0]) / 5;

                E_pst0 = limit_int(E_pst0, 2, -24); //sign/6
            }
            else if (RD_Flag == 9)
            {
                if (Length > pst_x)
                {
                    E_pst0 = (55 - indVal[2]) / 3;
                    E_pst0 = limit_int(E_pst0, 10, -10);
                }
                else
                {
                    E_pst0 = ((indVal[0] - 0.8 * indVal[3]) - (indVal[2] - indVal[4])) / 8; //E_pst0 = (0.8*(indVal[0]-0.8*indVal[3])-(indVal[2]-indVal[4]))/8;

                    E_pst0 = limit_int(E_pst0, 5, -5); //E_pst0=limit_int(E_pst0,3,-5);
                }
            }
            else if (RD_Flag == 10)
            {
                if (Length > pst_x)
                {
                    E_pst0 = -((55 - indVal[0]) / 3);
                    E_pst0 = limit_int(E_pst0, 10, -10);
                }
                else
                {
                    E_pst0 = -(((indVal[2] - 0.8 * indVal[4]) - (indVal[0] - indVal[3])) / 8); //E_pst0 = -(((indVal[2]-0.8*indVal[4])-(indVal[0]-indVal[3]))/8);

                    E_pst0 = limit_int(E_pst0, 5, -5); //E_pst0=limit_int(E_pst0,5,-3);
                }
            }

            for (i = 0; i < 5; i++)
                last_result[i] = indVal[i];
            for (i = 19; i > 0; i--)
                E_pst0_save[i] = E_pst0_save[i - 1];
        }

        E_llast_pst0 = E_last_pst0;
        E_last_pst0 = E_pst0;

        E_pst0_save[0] = E_pst0;
    }
}

void Road_Type_Analyse(void) //OK
{
    if (block == 1) //路障
    {
        if (((carState == enStand) && (distance < s_distance_sure)) || ((carState == enThree) && (distance < t_distance_sure)))
            block = 2;
        else if (((carState == enStand) && (distance > s_distance_sure + 20)) || ((carState == enThree) && (distance < t_distance_sure + 20)))
            block = 0;
    }

    if ((last_distance < 70 && last_distance > 0 && angle > 15 && angle_dot > 120 && carState == enThree && ramp_flag == 0 && E_pst0_save[4] < 6 && delay_count == 2000) || (ramp_flag != 0))
    {
        Three_Ramp_Control();
    }

    if (broken != 0 && block == 0 && delay_count == 2000) //断路
    {
        Broken_Control();
    }

    //    if(stop==1)
    //    {
    //        if(broken==0&&block==0&&ramp_flag==0)
    //            Stop_Car();
    //        else
    //            stop = 0;
    //    }
}

void Broken_Control(void) //OK
{
    if (broken == 1)
    {
        carState == enThree?enStand:enThree;
        delay_count = 1000;
        broken = 2;
    }
    if (broken == 2)
    {
        if (back == 1)
        {
            broken = 0;
            back = 0;
        }
    }
}

void Distance_Measure(void) //OK
{
    Pulse_Sum = Pulse_Sum + carSpeed;

    Length = Pulse_Sum / 50;
}

void Stand_Block_Control(void) //OK
{
    int8 i = 0;

    if (block == 2)
    {
        led(LED1, LED_ON);
        zang_set = s_block_14 * Length; //zang_set = Length*(1.2+zang_sum/650);
        if (zang_sum > s_block_angle)
        {
            Pulse_Sum = 0;
            Length = 0;
            block = 3;
        }
    }
    else if (block == 3)
    {
        zang_set = s_block_angle - s_block_23 * Length; //zang_set = 65-Length*(1.3-zang_sum/650);
        if (zang_sum < 0)
        {
            Pulse_Sum = 0;
            Length = 0;
            block = 4;
        }
    }
    else if (block == 4)
    {
        zang_set = -s_block_23 * Length; //zang_set = -Length*(1.2-zang_sum/650);
        if (zang_sum < -s_block_angle + 5)
        {
            Pulse_Sum = 0;
            Length = 0;
            block = 5;
        }
    }
    else if (block == 5)
    {
        zang_set = -s_block_angle + 5 + s_block_14 * Length; //zang_set = -65+Length*(1.3+zang_sum/650);
        if (zang_sum > 0)
        {
            zang_set = 0;
            zang_error = 0;
            zang_sum = 0;
            Pulse_Sum = 0;
            Length = 0;
            block = 0;
            for (i = 0; i < 20; i++)
            {
                zang_err_save[i] = 0;
            }
            led(LED1, LED_OFF);
        }
    }

    if (block > 1)
    {
        zang_block_error = zang_set - zang_sum;

        for (i = 19; i > 0; i--)
        {
            zang_err_save[i] = zang_err_save[i - 1];
        }
        zang_err_save[0] = zang_block_error;

        zang_err = zang_block_error - (zang_err_save[13] + 8 * zang_err_save[14] + zang_err_save[15]) / 10; //7 8 9    10 11 12
        block_pwm = S_Block_Dir_p * zang_block_error + S_Block_Dir_d * zang_err;

        //        block_pwm = 0.7*block_pwm + 0.3*block_pwm_last;
        block_pwm_last = block_pwm;

        Motor_L = s_balance_pwm + block_pwm;
        Motor_R = s_balance_pwm - block_pwm;
    }
}

void Three_Block_Control(void) //OK
{
    int8 i = 0;

    if (block == 2)
    {
        led(LED1, LED_ON);
        zang_set = t_block_14 * Length; //zang_set = Length*(1.2+zang_sum/650);
        if (zang_sum > t_block_angle)
        {
            Pulse_Sum = 0;
            Length = 0;
            block = 3;
        }
    }
    else if (block == 3)
    {
        zang_set = t_block_angle - t_block_23 * Length; //zang_set = 65-Length*(1.3-zang_sum/650);
        if (zang_sum < 0)
        {
            Pulse_Sum = 0;
            Length = 0;
            block = 4;
        }
    }
    else if (block == 4)
    {
        zang_set = -t_block_23 * Length; //zang_set = -Length*(1.2-zang_sum/650);
        if (zang_sum < -t_block_angle)
        {
            Pulse_Sum = 0;
            Length = 0;
            block = 5;
        }
    }
    else if (block == 5)
    {
        zang_set = -t_block_angle + t_block_14 * Length; //zang_set = -65+Length*(1.3+zang_sum/650);
        if (zang_sum > 0)
        {
            zang_set = 0;
            zang_error = 0;
            zang_sum = 0;
            Pulse_Sum = 0;
            Length = 0;
            block = 0;
            for (i = 0; i < 20; i++)
            {
                zang_err_save[i] = 0;
            }
            led(LED1, LED_OFF);
        }
    }

    if (block > 1)
    {
        zang_block_error = zang_set - zang_sum;

        for (i = 19; i > 0; i--)
        {
            zang_err_save[i] = zang_err_save[i - 1];
        }
        zang_err_save[0] = zang_block_error;

        zang_err = zang_block_error - (zang_err_save[11] + 8 * zang_err_save[12] + zang_err_save[13]) / 10; //7 8 9
        block_pwm = T_Block_Dir_p * zang_block_error + T_Block_Dir_d * zang_err;

        //        block_pwm = 0.7*block_pwm + 0.3*block_pwm_last;
        block_pwm_last = block_pwm;

        Motor_L = -Actual_pwm + block_pwm;
        Motor_R = -Actual_pwm - block_pwm;
    }
}

void Stop_Car(void) //OK
{
    ftm_pwm_duty(FTM0, FTM_CH7, 5000 - 400); //-600
    ftm_pwm_duty(FTM0, FTM_CH5, 5000 + 400); //+600

    ftm_pwm_duty(FTM0, FTM_CH6, 5000 - 400);
    ftm_pwm_duty(FTM0, FTM_CH4, 5000 + 400);
    if (carSpeed < 40) //50
        motor_0;
}

void Serve_Control(void) //OK
{
    if (carState != lastCarState)
    {
        lastCarState = carState;
        if (carState == enStand)
        {
            T2S_Init();
        }
        else
        {
            S2T_Init();
        }
    }
}

void Motor_Control(void) //OK
{
    if (carState == enStand)
    {
        if (block < 2)
        {
            Motor_L = s_balance_pwm + E_Dire.pwm;
            Motor_R = s_balance_pwm - E_Dire.pwm;
        }
        else
            Stand_Block_Control();
    }
    else if (carState == enThree)
    {
        if (block < 2)
        {
            Motor_L = -Actual_pwm + E_Dire.pwm;
            Motor_R = -Actual_pwm - E_Dire.pwm;
        }
        else
            Three_Block_Control();
    }

    Motor_L = limit_int(Motor_L, Motor_Max, Motor_Min); //电机输出的上下限
    Motor_R = limit_int(Motor_R, Motor_Max, Motor_Min);

    if (Motor_L < 0)
    {
        Motor_L = -Motor_L;
        ftm_pwm_duty(FTM0, FTM_CH7, 5000 + Motor_L + Lp_dead); //左正  +Motor_L+Lp_dead
        ftm_pwm_duty(FTM0, FTM_CH5, 5000 - Motor_L - Lp_dead); //-Motor_L-Lp_dead
    }
    else
    {
        Motor_L = Motor_L;
        ftm_pwm_duty(FTM0, FTM_CH7, 5000 - Motor_L - Ln_dead); //左反  -Motor_L-Lp_dead
        ftm_pwm_duty(FTM0, FTM_CH5, 5000 + Motor_L + Ln_dead); //+Motor_L+Lp_dead
    }
    if (Motor_R < 0)
    {
        Motor_R = -Motor_R;
        ftm_pwm_duty(FTM0, FTM_CH6, 5000 + Motor_R + Rp_dead); //右正  +Motor_R+Rp_dead
        ftm_pwm_duty(FTM0, FTM_CH4, 5000 - Motor_R - Rp_dead); //-Motor_R-Rp_dead
    }
    else
    {
        Motor_R = Motor_R;
        ftm_pwm_duty(FTM0, FTM_CH6, 5000 - Motor_R - Rn_dead); //右反  -Motor_R-Rn_dead
        ftm_pwm_duty(FTM0, FTM_CH4, 5000 + Motor_R + Rn_dead); //+Motor_R+Rn_dead
    }
}

//**************************************限幅函数*********************************************//

float limit_float(float value, float top, float bottom) //浮点型限值整形
{
    if (value >= top)
    {
        value = top;
    }
    else if (value < bottom)
    {
        value = bottom;
    }
    return value;
}

int16 limit_int(int16 value, int16 top, int16 bottom) //整形限制整形
{
    if (value >= top)
    {
        value = top;
    }
    else if (value < bottom)
    {
        value = bottom;
    }
    return value;
}

uint16 Ab_s(int16 value)
{
    if (value < 0)
        value = -value;
    return value;
}

float Ab_s_float(float value)
{
    if (value < 0)
        value = -value;
    return value;
}

//**************************************参数修改函数*********************************************//

/*环岛屏蔽参数*/
void Round_Set(void)
{
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(20, 1, "Stand_Round");
        LCD_P6x8Str(20, 7, "Three_Round");
        LCD_P6x8Str(50, 3, "Round");
        LCD_P6x8Str(55, 4, "Set");

        if (Stand_Round == 1)
            LCD_P6x8Str(100, 1, "ON");
        else
            LCD_P6x8Str(100, 1, "OFF");

        if (Three_Round == 1)
            LCD_P6x8Str(100, 7, "ON");
        else
            LCD_P6x8Str(100, 7, "OFF");

        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 3)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Stand_Round = !Stand_Round;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 3)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Three_Round = !Three_Round;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*直立挡位设置*/
void Stand_Gear_Set(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 1, "V=110");
        LCD_P6x8Str(48, 6, "V=125");
        LCD_P6x8Str(4, 3, "V=115");
        LCD_P6x8Str(90, 3, "V=120");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Stand_Speed_110();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Stand_Speed_125();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 1)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Stand_Speed_115();
            keyvalue = 0;
            break;
        }
        case 3:
        {
            while (keyvalue == 3)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Stand_Speed_120();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Stand_Speed_110(void)
{
    uint8 Exit_Flag = 1;

    speedSet.stand = 110;

    S_Speed_p = 25;

    S_Angle_p = 1.2;
    S_Angle_d = 0.1;

    S_Balance_p = 0.5;
    S_Balance_i = 0.03;

    SE1_Dir_p = 25;  //30
    SE1_Dir_d = 200; //240
    SE2_Dir_p = 6.8; //6.2
    SE2_Dir_d = 235; //255
    SE3_Dir_p = 4.3; //5.0
    SE3_Dir_d = 220; //260

    s_distance_sure = 60;

    s_block_14 = 1.3;
    s_block_23 = 1.2;
    s_block_angle = 60;
    S_Block_Dir_p = 120;
    S_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=110!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Stand_Speed_115(void)
{
    uint8 Exit_Flag = 1;

    speedSet.stand = 115;

    S_Speed_p = 25;

    S_Angle_p = 1.2;
    S_Angle_d = 0.1;

    S_Balance_p = 0.5;
    S_Balance_i = 0.03;

    SE1_Dir_p = 30;  //30
    SE1_Dir_d = 240; //240
    SE2_Dir_p = 6.8; //9.0
    SE2_Dir_d = 240; //300
    SE3_Dir_p = 4.3; //4.5
    SE3_Dir_d = 240; //310

    s_distance_sure = 60;

    s_block_14 = 1.3;
    s_block_23 = 1.2;
    s_block_angle = 60;
    S_Block_Dir_p = 120;
    S_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=115!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Stand_Speed_120(void)
{
    uint8 Exit_Flag = 1;

    speedSet.stand = 120;

    S_Speed_p = 25;

    S_Angle_p = 1.2;
    S_Angle_d = 0.1;

    S_Balance_p = 0.5;
    S_Balance_i = 0.03;

    SE1_Dir_p = 35;
    SE1_Dir_d = 255;
    SE2_Dir_p = 9.8;
    SE2_Dir_d = 315;
    SE3_Dir_p = 4.5;
    SE3_Dir_d = 320;

    s_distance_sure = 60;

    s_block_14 = 1.3;
    s_block_23 = 1.2;
    s_block_angle = 60;
    S_Block_Dir_p = 120;
    S_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=120!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Stand_Speed_125(void)
{
    uint8 Exit_Flag = 1;

    speedSet.stand = 125;

    S_Speed_p = 25;

    S_Angle_p = 1.2;
    S_Angle_d = 0.1;

    S_Balance_p = 0.5;
    S_Balance_i = 0.03;

    SE1_Dir_p = 35;
    SE1_Dir_d = 255;
    SE2_Dir_p = 9.8;
    SE2_Dir_d = 315;
    SE3_Dir_p = 4.5;
    SE3_Dir_d = 320;

    s_distance_sure = 60;

    s_block_14 = 1.3;
    s_block_23 = 1.2;
    s_block_angle = 60;
    S_Block_Dir_p = 120;
    S_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=125!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*三轮挡位设置*/
void Three_Gear_Set(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 1, "V=120");
        LCD_P6x8Str(48, 6, "V=135");
        LCD_P6x8Str(4, 3, "V=125");
        LCD_P6x8Str(90, 3, "V=130");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Three_Speed_120();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Three_Speed_135();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 1)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Three_Speed_125();
            keyvalue = 0;
            break;
        }
        case 3:
        {
            while (keyvalue == 3)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Three_Speed_130();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Three_Speed_120(void)
{
    uint8 Exit_Flag = 1;

    speedSet.three = 120;

    T_Speed_p = 5;
    T_Speed_i = 0.12;

    TE1_Dir_p = 10;
    TE1_Dir_d = 110;
    TE2_Dir_p = 6.2;
    TE2_Dir_d = 260;
    TE3_Dir_p = 5.2;
    TE3_Dir_d = 260;

    t_distance_sure = 60;

    t_block_14 = 1.3;
    t_block_23 = 1.2;
    t_block_angle = 60;
    T_Block_Dir_p = 120;
    T_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=120!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Three_Speed_125(void)
{
    uint8 Exit_Flag = 1;

    speedSet.three = 125;

    T_Speed_p = 5;
    T_Speed_i = 0.12;

    TE1_Dir_p = 9;
    TE1_Dir_d = 100;
    TE2_Dir_p = 7.0;
    TE2_Dir_d = 320;
    TE3_Dir_p = 5.6;
    TE3_Dir_d = 325;

    t_distance_sure = 60;

    t_block_14 = 1.3;
    t_block_23 = 1.2;
    t_block_angle = 60;
    T_Block_Dir_p = 120;
    T_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=125!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Three_Speed_130(void)
{
    uint8 Exit_Flag = 1;

    speedSet.three = 130;

    T_Speed_p = 5;
    T_Speed_i = 0.12;

    TE1_Dir_p = 10;
    TE1_Dir_d = 105;
    TE2_Dir_p = 6.7;
    TE2_Dir_d = 335;
    TE3_Dir_p = 5.7;
    TE3_Dir_d = 335;

    t_distance_sure = 60;

    t_block_14 = 1.3;
    t_block_23 = 1.2;
    t_block_angle = 60;
    T_Block_Dir_p = 120;
    T_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=130!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Three_Speed_135(void)
{
    uint8 Exit_Flag = 1;

    speedSet.three = 135;

    T_Speed_p = 5;
    T_Speed_i = 0.12;

    TE1_Dir_p = 10;
    TE1_Dir_d = 105;
    TE2_Dir_p = 6.7;
    TE2_Dir_d = 335;
    TE3_Dir_p = 5.8;
    TE3_Dir_d = 340;

    t_distance_sure = 60;

    t_block_14 = 1.3;
    t_block_23 = 1.2;
    t_block_angle = 60;
    T_Block_Dir_p = 120;
    T_Block_Dir_d = 480;

    while (Exit_Flag)
    {
        LCD_P6x8Str(48, 3, "V=135!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*避障参数设置*/
void Block_Set(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 1, "Stand");
        LCD_P6x8Str(45, 6, "Three");
        LCD_P6x8Str(4, 3, "Count");
        LCD_P6x8Str(86, 3, "Color");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_S_Block();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_T_Block();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Block_Count_Set();
            keyvalue = 0;
            break;
        }
        case 3:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_T_Block();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*直立避障识别距离*/
void Set_S_Block(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 1, "Dis=55"); //ang and exspeed
        LCD_P6x8Str(4, 3, "Dis=60");
        LCD_P6x8Str(88, 3, "Dis=65");
        LCD_P6x8Str(45, 6, "Dis=70");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_S_Block_55();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_S_Block_70();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            //Set_S_Block_70();
            Set_S_Block_60();
            keyvalue = 0;
            break;
        }
        case 3:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            //Set_S_Block_75();
            Set_S_Block_65();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_S_Block_50(void)
{
    uint8 Exit_Flag = 1;

    s_distance_sure = 50;

    s_block_14 = 1.5;
    s_block_23 = 1.3;
    s_block_angle = 64;
    S_Block_Dir_p = 200; //70
    S_Block_Dir_d = 430; //580

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "S_Block=50!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_S_Block_55(void)
{
    uint8 Exit_Flag = 1;

    s_distance_sure = 55;

    s_block_14 = 1.4;
    s_block_23 = 1.2;
    s_block_angle = 65;
    S_Block_Dir_p = 260; //70
    S_Block_Dir_d = 700; //580

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "S_Block=55!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_S_Block_60(void)
{
    uint8 Exit_Flag = 1;

    s_distance_sure = 60;

    s_block_14 = 1.3;
    s_block_23 = 1.1;
    s_block_angle = 60;
    S_Block_Dir_p = 270; //270
    S_Block_Dir_d = 540; //500

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "S_Block=60!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_S_Block_65(void)
{
    uint8 Exit_Flag = 1;

    s_distance_sure = 65;

    s_block_14 = 1.1;
    s_block_23 = 1.1;
    s_block_angle = 60;
    S_Block_Dir_p = 290; //70
    S_Block_Dir_d = 430; //580

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "S_Block=65!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_S_Block_70(void)
{
    uint8 Exit_Flag = 1;

    s_distance_sure = 70;

    s_block_14 = 1.1;
    s_block_23 = 1.0;
    s_block_angle = 62;
    S_Block_Dir_p = 280; //290  280
    S_Block_Dir_d = 390; //360  390

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "S_Block=70!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_S_Block_75(void)
{
    uint8 Exit_Flag = 1;

    s_distance_sure = 75;

    s_block_14 = 1.0;
    s_block_23 = 1.0;
    s_block_angle = 60;
    S_Block_Dir_p = 280;
    S_Block_Dir_d = 360;

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "S_Block=75!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*三轮避障识别距离*/
void Set_T_Block(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 1, "Dis=60"); //ang and exspeed
        LCD_P6x8Str(4, 3, "Dis=65");
        LCD_P6x8Str(88, 3, "Dis=70");
        LCD_P6x8Str(45, 6, "Dis=75");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            //Set_T_Block_60();
            Set_T_Block_55();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_T_Block_75();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            //Set_T_Block_65();
            Set_T_Block_60();
            keyvalue = 0;
            break;
        }
        case 3:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_T_Block_70();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_T_Block_50(void)
{
    uint8 Exit_Flag = 1;

    t_distance_sure = 50;

    t_block_14 = 1.5;
    t_block_23 = 1.2;
    t_block_angle = 62;
    T_Block_Dir_p = 360; //280  380
    T_Block_Dir_d = 650; //520  550

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "T_Block=50!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_T_Block_55(void)
{
    uint8 Exit_Flag = 1;

    t_distance_sure = 55;

    t_block_14 = 1.4;
    t_block_23 = 1.2;
    t_block_angle = 65;
    T_Block_Dir_p = 330; //280  380  330
    T_Block_Dir_d = 750; //520  550  750

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "T_Block=55!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_T_Block_60(void)
{
    uint8 Exit_Flag = 1;

    t_distance_sure = 60;

    t_block_14 = 1.1;
    t_block_23 = 1.0;
    t_block_angle = 63;
    T_Block_Dir_p = 360; //350  360
    T_Block_Dir_d = 300; //330  350

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "T_Block=60!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_T_Block_65(void)
{
    uint8 Exit_Flag = 1;

    t_distance_sure = 65;

    t_block_14 = 1.1;
    t_block_23 = 1.0;
    t_block_angle = 59;
    T_Block_Dir_p = 360; //120
    T_Block_Dir_d = 540; //480

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "T_Block=65!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_T_Block_70(void)
{
    uint8 Exit_Flag = 1;

    t_distance_sure = 70;

    t_block_14 = 1;
    t_block_23 = 0.9;
    t_block_angle = 56;
    T_Block_Dir_p = 410; //120
    T_Block_Dir_d = 500; //480

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "T_Block=70!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_T_Block_75(void)
{
    uint8 Exit_Flag = 1;

    t_distance_sure = 75;

    t_block_14 = 1;
    t_block_23 = 0.9;
    t_block_angle = 54;
    T_Block_Dir_p = 450; //120   320
    T_Block_Dir_d = 480; //480   460

    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "T_Block=75!");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*避障数目设置*/
void Block_Count_Set(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(42, 0, "block1:");
        LCD_P6x8Str(82, 3, "block3:");
        LCD_P6x8Str(5, 3, "block2:");
        LCD_P6x8Str(30, 6, "block_count:");
        LCD_P6x8Str_1(60, 1, see_block1);
        LCD_P6x8Str_1(100, 4, see_block3);
        LCD_P6x8Str_1(23, 4, see_block2);
        LCD_P6x8Str_1(60, 7, block_over);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_Block_One();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_Block_Num();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 1)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_Block_Two();
            keyvalue = 0;
            break;
        }
        case 3:
        {
            while (keyvalue == 3)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_Block_Three();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_Block_One(void)
{
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(35, 3, "see_block1");
        LCD_P6x8Str_1(62, 5, see_block1);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            see_block1 = see_block1 + 1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            see_block1 = see_block1 - 1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_Block_Two(void)
{
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(35, 3, "see_block2");
        LCD_P6x8Str_1(62, 5, see_block2);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            see_block2 = see_block2 + 1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            see_block2 = see_block2 - 1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_Block_Three(void)
{
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(35, 3, "see_block3");
        LCD_P6x8Str_1(62, 5, see_block3);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            see_block3 = see_block3 + 1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            see_block3 = see_block3 - 1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_Block_Num(void)
{
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(35, 3, "block_over");
        LCD_P6x8Str_1(62, 5, block_over);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            block_over = block_over + 1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            block_over = block_over - 1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*转向参数*/
void Dir_Set(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 1, "Stand_P");
        LCD_P6x8Str(45, 6, "Stand_D");
        LCD_P6x8Str(4, 3, "Three_P");
        LCD_P6x8Str(85, 3, "Three_D");
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_S_Dir_p();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_S_Dir_d();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_T_Dir_p();
            keyvalue = 0;
            break;
        }
        case 3:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_T_Dir_d();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*三轮转向P*/
void Set_T_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 0, "TE1_p");
        LCD_P6x8Str_4(53, 1, TE1_Dir_p * 10);
        LCD_P6x8Str(45, 6, "TE3_p");
        LCD_P6x8Str_4(53, 7, TE3_Dir_p * 10);
        LCD_P6x8Str(4, 2, "TE2_p");
        LCD_P6x8Str_4(10, 3, TE2_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_TE1_Dir_p();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_TE3_Dir_p();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 1)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_TE2_Dir_p();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_TE1_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "TE1_p");
        LCD_P6x8Str_4(46, 4, TE1_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE1_Dir_p = TE1_Dir_p + 1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE1_Dir_p = TE1_Dir_p - 1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_TE2_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "TE2_p");
        LCD_P6x8Str_4(46, 4, TE2_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE2_Dir_p = TE2_Dir_p + 0.1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE2_Dir_p = TE2_Dir_p - 0.1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_TE3_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "TE3_p");
        LCD_P6x8Str_4(46, 4, TE3_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE3_Dir_p = TE3_Dir_p + 0.1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE3_Dir_p = TE3_Dir_p - 0.1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*三轮转向D*/
void Set_T_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 0, "TE1_d");
        LCD_P6x8Str_3(53, 1, TE1_Dir_d);
        LCD_P6x8Str(45, 6, "TE3_d");
        LCD_P6x8Str_3(53, 7, TE3_Dir_d);
        LCD_P6x8Str(4, 2, "TE2_d");
        LCD_P6x8Str_3(10, 3, TE2_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_TE1_Dir_d();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_TE3_Dir_d();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 1)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_TE2_Dir_d();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_TE1_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "TE1_d");
        LCD_P6x8Str_3(46, 4, TE1_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE1_Dir_d = TE1_Dir_d + 5;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE1_Dir_d = TE1_Dir_d - 5;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_TE2_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "TE2_d");
        LCD_P6x8Str_3(46, 4, TE2_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE2_Dir_d = TE2_Dir_d + 5;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE2_Dir_d = TE2_Dir_d - 5;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_TE3_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "TE3_d");
        LCD_P6x8Str_3(46, 4, TE3_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE3_Dir_d = TE3_Dir_d + 5;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            TE3_Dir_d = TE3_Dir_d - 5;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*直立转向P*/
void Set_S_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 0, "SE1_p");
        LCD_P6x8Str_3(53, 1, SE1_Dir_p * 10);
        LCD_P6x8Str(45, 6, "SE3_p");
        LCD_P6x8Str_3(53, 7, SE3_Dir_p * 10);
        LCD_P6x8Str(4, 2, "SE2_p");
        LCD_P6x8Str_3(10, 3, SE2_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_SE1_Dir_p();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_SE3_Dir_p();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 1)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_SE2_Dir_p();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_SE1_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "SE1_p");
        LCD_P6x8Str_3(46, 4, SE1_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE1_Dir_p = SE1_Dir_p + 1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE1_Dir_p = SE1_Dir_p - 1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_SE2_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "SE2_p");
        LCD_P6x8Str_3(46, 4, SE2_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE2_Dir_p = SE2_Dir_p + 0.1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE2_Dir_p = SE2_Dir_p - 0.1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_SE3_Dir_p(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "SE3_p");
        LCD_P6x8Str_3(46, 4, SE3_Dir_p * 10);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE3_Dir_p = SE3_Dir_p + 0.1;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE3_Dir_p = SE3_Dir_p - 0.1;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

/*直立转向D*/
void Set_S_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 0, "SE1_d");
        LCD_P6x8Str_3(53, 1, SE1_Dir_d);
        LCD_P6x8Str(45, 6, "SE3_d");
        LCD_P6x8Str_3(53, 7, SE3_Dir_d);
        LCD_P6x8Str(4, 2, "SE2_d");
        LCD_P6x8Str_3(10, 3, SE2_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_SE1_Dir_d();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_SE3_Dir_d();
            keyvalue = 0;
            break;
        }
        case 1:
        {
            while (keyvalue == 1)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_SE2_Dir_d();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_SE1_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "SE1_d");
        LCD_P6x8Str_3(46, 4, SE1_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE1_Dir_d = SE1_Dir_d + 5;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE1_Dir_d = SE1_Dir_d - 5;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_SE2_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "SE2_d");
        LCD_P6x8Str_3(46, 4, SE2_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE2_Dir_d = SE2_Dir_d + 5;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE2_Dir_d = SE2_Dir_d - 5;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_SE3_Dir_d(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(45, 3, "SE3_d");
        LCD_P6x8Str_3(46, 4, SE3_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE3_Dir_d = SE3_Dir_d + 5;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            SE3_Dir_d = SE3_Dir_d - 5;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_B_Dir(void)
{
    motor_0;
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(41, 0, "B_P_Set"); //ang and exspeed
        LCD_P6x8Str(41, 6, "B_D_Set");
        LCD_P6x8Str_4(55, 1, T_Block_Dir_p); //ang and exspeed
        LCD_P6x8Str_4(55, 7, T_Block_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_B_Dir_P();
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Set_B_Dir_D();
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_B_Dir_P(void)
{
    //T_Block_Dir_p
    //S_Block_Dir_p
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "Block_Dir_p");
        LCD_P6x8Str_4(50, 5, T_Block_Dir_p);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            S_Block_Dir_p = S_Block_Dir_p + 10;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            S_Block_Dir_p = S_Block_Dir_p - 10;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void Set_B_Dir_D(void)
{
    //T_Block_Dir_d
    //S_Block_Dir_d
    uint8 Exit_Flag = 1;
    while (Exit_Flag)
    {
        LCD_P6x8Str(30, 3, "Block_Dir_d");
        LCD_P6x8Str_4(50, 5, S_Block_Dir_d);
        keyvalue = 0;
        Keyboardscanf();
        switch (keyvalue)
        {
        case 2:
        {
            while (keyvalue == 2)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            S_Block_Dir_d = S_Block_Dir_d + 10;
            keyvalue = 0;
            break;
        }
        case 4:
        {
            while (keyvalue == 4)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            S_Block_Dir_d = S_Block_Dir_d - 10;
            keyvalue = 0;
            break;
        }
        case 5:
        {
            while (keyvalue == 5)
            {
                keyvalue = 30;
                Keyboardscanf();
            }
            LCD_CLS();
            Exit_Flag = 0;
            keyvalue = 0;
            break;
        }
        }
    }
}

void LCD_Printf(void) //液晶显示  zang_dot
{
    LCD_P6x8Str(10, 0, "L");
    LCD_P6x8Str3(4, 1, indVal[0]);
    LCD_P6x8Str(40, 0, "M");
    LCD_P6x8Str3(34, 1, indVal[1]);
    LCD_P6x8Str(70, 0, "R");
    LCD_P6x8Str3(64, 1, indVal[2]);
    LCD_P6x8Str(7, 2, "LT");
    LCD_P6x8Str3(3, 3, indVal[3]);
    LCD_P6x8Str(36, 2, "RT");
    LCD_P6x8Str3(34, 3, indVal[4]);
    LCD_P6x8Str(60, 2, "e_pst0");
    LCD_P6x8Str_3(66, 3, E_pst0);

    LCD_P6x8Str(4, 4, "Angle:");
    LCD_P6x8Str_4(10, 5, Angle);
    LCD_P6x8Str(4, 6, "FAngle:");
    LCD_P6x8Str_4(10, 7, FiltAngle);

    LCD_P6x8Str_4(60, 4, RD_Flag);
    LCD_P6x8Str_4(60, 5, zang_sum);
    LCD_P6x8Str_4(60, 6, Length);
    LCD_P6x8Str_4(60, 7, distance);

    LCD_P6x8Str_4(95, 4, delay_count);
    //  LCD_P6x8Str_4(95,5,pst2);
    //  LCD_P6x8Str_4(95,6,pst3);
    //  LCD_P6x8Str_4(95,7,pst4);
}
