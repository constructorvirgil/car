/**
 * @file key.c
 * @brief 按键相关操作
 * @author virgil
 * @email 275380431@qq.com
 * @version v1.0
 * @date 2019-11-05
 */

#include "key.h"

void five_key_init(void)
{
    gpio_init(KEY_PORT_LEFT, GPI, 1); //左
    gpio_init(KEY_PORT_RIGHT, GPI, 1); //上
    gpio_init(KEY_PORT_UP, GPI, 1); //右
    gpio_init(KEY_PORT_DOWN, GPI, 1); //下
    gpio_init(KEY_PORT_CENTER, GPI, 1); //中
}


/**
 * @brief 阻塞式等待一个按键,并获取键值
 * @param 无
 * @return 返回按键按下的键值
 *   @retval 键值已经通过 @see KEY_CODE_EN 定义
 */
KEY_CODE_EN key_scanf(void)
{
    while (1)
    {
        if (gpio_get(KEY_PORT_RIGHT) == 0)
        {
            while (gpio_get(KEY_PORT_RIGHT) == 0)
                ;
            return KEY_CODE_RIGHT;
        }
        else if (gpio_get(KEY_PORT_UP) == 0)
        {
            while (gpio_get(KEY_PORT_UP) == 0)
                ;
            return KEY_CODE_UP;
        }
        else if (gpio_get(KEY_PORT_LEFT) == 0)
        {
            while (gpio_get(KEY_PORT_LEFT) == 0)
                ;
            return KEY_CODE_LEFT;
        }
        else if (gpio_get(KEY_PORT_DOWN) == 0)
        {
            while (gpio_get(KEY_PORT_DOWN) == 0)
                ;
            return KEY_CODE_DOWN;
        }
        else if (gpio_get(KEY_PORT_CENTER) == 0)
        {

            while (gpio_get(KEY_PORT_CENTER) == 0)
                ;
            return KEY_CODE_CENTER;
        }
    }
}