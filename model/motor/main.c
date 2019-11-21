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
#include "motor.h"

//test inc
#include "lcd.h"

void run_car_for()
{
	motor_run(5000);
	while(1);
}
void run_car_bak()
{
	motor_run(-5000);
	while(1);
}

int main()
{
    DisableInterrupts;

    LCD_Init();
    LCD_CLS();
    LCD_P6x8Str(24,3,"Motor unit test");
	systick_delay_ms(1000);

	//第一层创建了三个单元,ready,angle,speed
	UNIT* unit_top = create_top("mode:");
	UNIT* unit1 = add_unit_down(unit_top, "positive");
	UNIT* unit2 = add_unit_down(unit1, "nagative");

	add_unit_handler(unit1, "go", run_car_for);
	add_unit_handler(unit2, "go", run_car_bak);

	motor_init();
	
	//在LCD上显示,并循环判断按键,阻塞
	menu_start();

	//menu_close();

	return 0;
}

