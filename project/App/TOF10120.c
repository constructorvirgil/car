#include "TOF10120.h"

#define CHECK_SDA             gpio_get (TOF_SDA)
#define SDA_H()          gpio_set (TOF_SDA, 1)		//IO口输出高电平
#define SDA_L()          gpio_set (TOF_SDA, 0)		//IO口输出低电平
#define SCL_H()          gpio_set (TOF_SCL, 1)		//IO口输出高电平
#define SCL_L()          gpio_set (TOF_SCL, 0)		//IO口输出低电平
#define TOF_DIR_OUT()       gpio_ddr (TOF_SDA, GPO)    //输出方向
#define TOF_DIR_IN()        gpio_ddr (TOF_SDA, GPI)    //输入方向

#define TOF_ack 1      //主应答
#define TOF_no_ack 0   //从应答

//**************************************内部调用函数************************************//

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
void TOF_delay(void)
{
  //64为100K的速率(bus频率为100M)
  //特别提示OV7725的通信速率不能太高，最好50K左右，j设置为120通信速率为60K，60K的时候可以正常通信
  //其他IIC器件一般可以400K的通信速率
  uint16 j=400;   
  while(j--);
}

void TOF_start(void)
{
  SDA_H();
  SCL_H();
  TOF_delay();
  SDA_L();
  TOF_delay();
  SCL_L();
}

void TOF_stop(void)
{
  SDA_L();
  SCL_L();
  TOF_delay();
  SCL_H();
  TOF_delay();
  SDA_H();
  TOF_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
void TOF_SendACK(unsigned char ack_dat)
{
  SCL_L();
  TOF_delay();
  if(ack_dat)
    SDA_L();
  else
    SDA_H();
  SCL_H();
  TOF_delay();
  SCL_L();
  TOF_delay();
}

static int WaitAck(void)
{
  SCL_L();
  TOF_DIR_IN();
  TOF_delay();
  
  SCL_H();
  TOF_delay();
  
  if(CHECK_SDA)           //应答为高电平，异常，通信失败
  {
    TOF_DIR_OUT();
    SCL_L();
    return 0;
  }
  TOF_DIR_OUT();
  SCL_L();
  TOF_delay();
  return 1;
}


//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void TOF_send_ch(uint8 c)
{
  uint8 i = 8;
  while(i--)
  {
    if(c & 0x80)
      SDA_H();//SDA 输出数据
    else
      SDA_L();
    c <<= 1;
    TOF_delay();
    SCL_H();                //SCL 拉高，采集信号
    TOF_delay();
    SCL_L();                //SCL 时钟线拉低
  }
  WaitAck();
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//内部使用，用户无需调用
uint8 TOF_read_ch(void)
{
  uint8 i;
  uint8 c;
  c=0;
  SCL_L();
  TOF_delay();
  SDA_H();             //置数据线为输入方式
  TOF_DIR_IN();
  for(i=0;i<8;i++)
  {
    TOF_delay();
    SCL_L();         //置时钟线为低，准备接收数据位
    TOF_delay();
    SCL_H();         //置时钟线为高，使数据线上数据有效
    TOF_delay();
    c<<=1;
    if(CHECK_SDA) c+=1;   //读数据位，将接收的数据存c
  }
  TOF_DIR_OUT();
  SCL_L();
  TOF_delay();
  TOF_SendACK(TOF_no_ack);
  
  return c;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 read_reg(uint8 dev_add, uint8 reg, type type)
{
  uint8 dat;
  TOF_start();
  TOF_send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
  TOF_send_ch( reg );   				//发送从机寄存器地址
  if(type == TOF_SCCB)
    TOF_stop();
  TOF_start();
  TOF_send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
  dat = TOF_read_ch();   				//发送需要写入的数据
  TOF_stop();
  
  return dat;
}


void TOF_init(void)
{
  gpio_init (TOF_SCL, GPO,1);
  gpio_init (TOF_SDA, GPO,1);
  port_init_NoALT (TOF_SCL, ODO | PULLUP);//ODO
  port_init_NoALT (TOF_SDA, ODO | PULLUP);
}

void TOF_Measurement(uint8 *data)
{
    uint8 TOF_L;
    uint16 TOF_H;
    TOF_H = read_reg(TOF10120_DEV_ADDR, 0x00, TOF_IIC);
    TOF_L = read_reg(TOF10120_DEV_ADDR, 0x00+1, TOF_IIC);
    *data=((TOF_H*256)+TOF_L)/10;//合成数据
}

