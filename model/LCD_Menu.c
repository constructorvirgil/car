#include "LCD_Menu.h"
#include "common.h"
#include "stdio.h"
#include "include.h"
#include "lcd.h"
#include "control.h"
#include "TOF10120.h"

MENU *topMenu;
MENU *nowMenu;
uint8 first_create = 1;
ITEM items[48];
int pos = 0;

/***************************************************************  
*  @brief     	创建一个菜单以及创建链接下一个菜单
*  @param     
				p_xxxx:前四个参数是要显示的字符串,顺序,左右上下,左右控制5个字符以内,中间4个字符,上下8个字符
				pp_menu:指向下一级菜单的二级指针,以便拓展菜单
				p_bak:指向上一级菜单的指针,五向开关按下中间键后会返回上一级菜单
*  @note      	
*  @return 		返回指向菜单的指针
**************************************************************/
MENU *CreateMenu(const char *p_left, const char *p_right, const char *p_up, const char *p_down, const char *p_centre, MENU **pp_menu, MENU *p_bak)
{
	MENU *p_menu = (MENU *)malloc(sizeof(MENU));
	if (first_create == 1)
	{
		first_create = 0;
		topMenu = p_menu;
		nowMenu = topMenu;
	}
	else
	{
		*pp_menu = p_menu; //改写传进来的指针
	}
	p_menu->left = p_left;
	p_menu->left_next = NULL;

	p_menu->right = p_right;
	p_menu->right_next = NULL;

	p_menu->up = p_up;
	p_menu->up_next = NULL;

	p_menu->down = p_down;
	p_menu->down_next = NULL;

	p_menu->centre = p_centre;
	p_menu->centre_next = p_bak;

	return p_menu;
}


static void PrintMenu()
{
	LCD_CLS();
	if (nowMenu->left != NULL)
	{
		LCD_P6x8Str(0, 4, (uint8_t *)nowMenu->left);
	}
	if (nowMenu->right != NULL)
	{
		LCD_P6x8Str(90, 4, (uint8_t *)nowMenu->right);
	}
	if (nowMenu->up != NULL)
	{
		LCD_P6x8Str(48, 1, (uint8_t *)nowMenu->up);
	}
	if (nowMenu->down != NULL)
	{
		LCD_P6x8Str(48, 7, (uint8_t *)nowMenu->down);
	}
	if (nowMenu->centre != NULL)
	{
		LCD_P6x8Str(53, 4, (uint8_t *)nowMenu->centre);
	}
}

static void Goto(_DIR dir)
{
	switch (dir)
	{
	case LEFT:
		if (nowMenu->left_next != NULL)
		{
			nowMenu = nowMenu->left_next;
		}
		else
		{
			ExcuteCmd(nowMenu->left);
		}
		break;
	case RIGHT:
		if (nowMenu->right_next != NULL)
		{
			nowMenu = nowMenu->right_next;
		}
		else
		{
			ExcuteCmd(nowMenu->right);
		}
		break;

	case UP:
		if (nowMenu->up_next != NULL)
		{
			nowMenu = nowMenu->up_next;
		}
		else
		{
			ExcuteCmd(nowMenu->up);
		}
		break;

	case DOWN:
		if (nowMenu->down_next != NULL)
		{
			nowMenu = nowMenu->down_next;
		}
		else
		{
			ExcuteCmd(nowMenu->down);
		}
		break;
	case CENTRE:
		if (nowMenu->centre_next != NULL)
		{
			nowMenu = nowMenu->centre_next;
		}
		else
		{
			ExcuteCmd(nowMenu->centre);
		}
		break;
	}
	PrintMenu();
}

/***************************************************************  
*  @brief     	添加一个字符串对回调函数的映射
*  @param     
				str:回调函数标识,字符串
				hand_func:回调函数指针
*  @note      	当选中对应的字符串的时候,会自动调用对应这个字符串的回调函数
				因为是通过字符串的形式来匹配回调函数的,所以字符串标识应该唯一
**************************************************************/
void AddHandle(const char *str, HD hand_func)
{
	ITEM item;
	strcpy(item.str, str);
	item.p_func = hand_func;
	items[pos] = item;
	pos++;
}

static void ExcuteCmd(const char *str)
{
	if (str == NULL)
	{
		return;
	}

	for (int i = 0; i < 48; i++)
	{
		if (strcmp(items[i].str, str) == 0)
		{
			(*items[i].p_func)();
			break;
		}
	}

	nowMenu = topMenu;
}

/***************************************************************  
*  @brief     	显示菜单
*  @param     	
*  @note      	
**************************************************************/
void MenuStart(void)
{
	_KEY_CODE key;
	PrintMenu();
	while (1)
	{
		systick_delay_ms(500);
		key = Keyboardscanf();
		switch (key)
		{
		case _KEY_LEFT:
			Goto(LEFT);
			break;
		case _KEY_RIGHT:
			Goto(RIGHT);
			break;
		case _KEY_UP:
			Goto(UP);
			break;
		case _KEY_DOWN:
			Goto(DOWN);
			break;
		case _KEY_CENTRE:
			Goto(CENTRE);
			break;
		default:
			break;
		}
	}
}