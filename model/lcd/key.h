/**
 * @file key.h
 * @brief 按键相关操作
 * @author virgil
 * @email 275380431@qq.com
 * @version v1.0
 * @date 2019-11-05
 */

#ifndef __KEY_H
#define __KEY_H

#include "common.h"
#include "MK60_gpio.h"

//** 按键对应的PORT口 */
#define KEY_PORT_UP         PTE28
#define KEY_PORT_DOWN       PTE24
#define KEY_PORT_LEFT       PTE25
#define KEY_PORT_RIGHT      PTE27
#define KEY_PORT_CENTER     PTE26


/** 键值枚举 */
typedef enum 
{
    KEY_CODE_NULL,
    KEY_CODE_LEFT,
    KEY_CODE_RIGHT,
    KEY_CODE_UP,
    KEY_CODE_DOWN,
    KEY_CODE_CENTER
}KEY_CODE_EN;

void five_key_init(void);
//** 获取按下的键值 */
KEY_CODE_EN key_scanf(void);


#endif