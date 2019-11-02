#ifndef __LCD_MENU_h
#define __LCD_MENU_h
#include "Key.h"
typedef void(*HD)(void);

typedef enum 
{
	LEFT,
	RIGHT,
	UP,
	DOWN,
	CENTRE
}_DIR;

typedef struct __MENU
{
	const char *left;
	struct __MENU *left_next;
	const char *right;
	struct __MENU *right_next;
	const char *up;
	struct __MENU *up_next;
	const char *down;
	struct __MENU *down_next;
	const char *centre;
	struct __MENU *centre_next;
}MENU;


typedef struct 
{
	char str[16];
	HD p_func;
}ITEM;

MENU *CreateMenu(const char *p_left, const char *p_right, const char *p_up, const char *p_down, const char *p_centre, MENU **pp_menu, MENU *p_bak);
static void Goto(_DIR dir);
static void ExcuteCmd(const char *str);
static void PrintMenu();
void MenuStart(void);
void AddHandle(const char *str, HD hand_func);
#endif