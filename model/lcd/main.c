/**
 * @file main.c
 * @brief LCD 测试
 * @author virgil
 * @email 275380431@qq.com
 * @version v1.0
 * @date 2019-11-05
 */

#include "common.h"
#include "include.h"
#include "a_lcd.h"

//test inc
#include <stdio.h>
#include "lcd.h"

int angle_value;
int speed_value;

/**
* @note 用来测试的handler函数,显示进入菜单时设置的参数值
*/
void run_car(void)
{
    LCD_P6x8Str(12,3,"CAR RUNNING");

	//angle
	char tmp[48]={0};
	sprintf(tmp,"angle:%d",angle_value);
    LCD_P6x8Str(12,4,(byte *)tmp);

	//speed
	memset(tmp,0,48);
	sprintf(tmp,"speed:%d",speed_value);
    LCD_P6x8Str(12,5,(byte *)tmp);
	while(1);

}


int main()
{
    DisableInterrupts;
    
    LCD_Init();
    LCD_CLS();
    LCD_P6x8Str(24,3,"LCD unit test");
	systick_delay_ms(1000);

	//第一层创建了三个单元,ready,angle,speed
	UNIT* unit_top = create_top("ready");
	UNIT* unit1 = add_unit_down(unit_top, "angle");
	UNIT* unit2 = add_unit_down(unit1, "speed");

	//添加对应ready的动作unit
	add_unit_handler(unit_top, "run_car", run_car);

	//添加对应angle的三个参数
	unit1 = add_unit_param(unit1, "30", 30, &angle_value);
	unit1 = add_unit_param(unit1, "60", 60, &angle_value);
	add_unit_param(unit1, "90", 90, &angle_value);

	//添加对应speed的三个参数
	unit2 = add_unit_param(unit2, "120", 120, &speed_value);
	unit2 = add_unit_param(unit2, "130", 130, &speed_value);
	add_unit_param(unit2, "140", 140, &speed_value);

	//在LCD上显示,并循环判断按键,阻塞
	menu_start();

	//实际并不会执行到这一句
	//程序会阻塞在menu_start死循环获取键值之中
	//需要在自定义的handler函数中恰当地调用menu_close回收堆内存
	//否则,会导致之后的malloc失败
	menu_close();

	return 0;
}

