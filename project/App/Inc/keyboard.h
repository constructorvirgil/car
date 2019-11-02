#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#define CH451_RESET     0x0201             //复位
#define CH451_LEFTMOV   0x0300             //设置移动方式-左移
#define CH451_LEFTCYC   0x0301             //设置移动方式-左循
#define CH451_RIGHTMOV  0x0302             //设置移动方式-右移
#define CH451_RIGHTCYC  0x0303             //设置移动方式-右循 
#define CH451_SYSOFF    0x0400             //关显示、键盘、看门狗
#define CH451_SYSON1    0x0401             //开显示
#define CH451_KEYEN     0x0402             //开键盘
#define CH451_SYSON2    0x0403             //开显示、键盘
#define CH451_SYSON3    0x0407             //开显示、键盘、看门狗功能
#define CH451_DSP       0x0500             //设置默认显示方式
#define CH451_BCD       0x0580             //设置BCD译码方式
#define CH451_TWINKLE   0x0600             //设置闪烁控制
#define CH451_KEY       0x0700             //读键盘代码
#define CH451_DIG0      0x0800             //数码管位0显示
#define CH451_DIG1      0x0900             //数码管位1显示 
#define CH451_DIG2      0x0A00             //数码管位2显示
#define CH451_DIG3      0x0B00             //数码管位3显示
#define CH451_DIG4      0x0C00             //数码管位4显示
#define CH451_DIG5      0x0D00             //数码管位5显示 
#define CH451_DIG6      0x0E00             //数码管位6显示 
#define CH451_DIG7      0x0F00             //数码管位7显示
 
#define LOAD   PTA14          // 3    
#define DIN    PTA12          // 2    
#define DCLK   PTA15        // 1    
#define DOUT   PTA13         // 4    
/*修改端口时，需改*/

uint8 KeyCode[9]={0x41,0x42,0x43,//1,2,3
                  0x49,0x4A,0x4B,//4,5,6
                  0x51,0x52,0x53,};//7,8,9

uint8 KeyValue;
uint8 GetKeyValue=0;

void CH451_WriteCommand(uint32 Command)
{                                                                  //向CH451写命令
  uint8 k=12;
  gpio_set(LOAD,0);                                                //gpio_ctrl(key_port,Load,0); 
  for(k=0;k<12;k++){
	 gpio_set(DIN,Command&1);                                       //gpio_ctrl(key_port,IN,Command&1);         
	 gpio_set(DCLK,0);                                               //gpio_ctrl(key_port,CLK,0);                
	 Command>>=1;                                                     //数据右移一位
	 gpio_set(DCLK,1);                                               //gpio_ctrl(key_port,CLK,1);
  }
  gpio_set(LOAD,1);                                               //gpio_ctrl(key_port,Load,1);
}

void CH451_GetKeyValue(void)
{
    uint8 k;
    uint8 Command=0x07;                                            //读取键值命令
    GetKeyValue=0x00;                                              //键盘代码
                                                                   //CH451_Load=0;
    gpio_set(LOAD,0);                                             //gpio_ctrl(key_port,Load,0);
                                                                   //命令开始       
    for(k=0;k<4;k++)
    {                                                              //送入4位数据，低位在前
                                                                   //CH451_IN=Command&0x0001; 
    gpio_set(DIN,Command&0X0001);                                //gpio_ctrl(key_port,IN,Command&0X0001);
                                                                   //CH451_CLK=0;
    gpio_set(DCLK,0);                                             //gpio_ctrl(key_port,CLK,0);
      Command>>=1;
                                                                   //CH451_CLK=1;                   //上升沿有效
    gpio_set(DCLK,1);                                             // gpio_ctrl(key_port,CLK,1);
    }
                                                                   // CH451_Load=1;                     //上升沿加载数据
   gpio_set(LOAD,1);                                              // gpio_ctrl(key_port,Load,1);    
    for(k=0;k<7;k++)
    {
        GetKeyValue<<=1;
        GetKeyValue|=gpio_get (DOUT);                             //gpio_Get(key_port,Out);//CH451_Out;
                                                                   //CH451_CLK=1;               //下降沿有效
        gpio_set(DCLK,1);                                          //gpio_ctrl(key_port,CLK,1);
                                                                   //CH451_CLK=0;
        gpio_set(DCLK,0);
    }
                                                                   //查找键盘代码对应的键值
    for(k=0;k<12;k++)
    {                
      if(GetKeyValue==KeyCode[k])
      {  
        KeyValue=k+1;  
        break;
      }
    }
}
void CH451_ini(void)
{
  gpio_set(DIN,1);                                            // gpio_ctrl(key_port,IN,1);
  gpio_set(DIN,0); 
  gpio_set(DIN,1);                                              // gpio_ctrl(key_port,IN,1);
  CH451_WriteCommand(CH451_RESET);                                   //复位CH451
  CH451_WriteCommand(CH451_SYSON2); 
}

void JIANPAN_ini(void)         //键盘控制端口初始化
{
  gpio_init(DIN,GPO,1);
  gpio_init(DCLK,GPO,1);
  gpio_init(LOAD,GPO,1);
  gpio_init(DOUT,GPI,1);
  
}
#endif