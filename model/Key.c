#include "Key.h"

_KEY_CODE Keyboardscanf(void)
{
    while (1)
    {
        if (gpio_get(PTE25) == 0)
        {
            while (gpio_get(PTE25) == 0)
                ;
            return _KEY_LEFT;
        }
        else if (gpio_get(PTE28) == 0)
        {
            while (gpio_get(PTE28) == 0)
                ;
            return _KEY_UP;
        }
        else if (gpio_get(PTE27) == 0)
        {
            while (gpio_get(PTE27) == 0)
                ;
            return _KEY_RIGHT;
        }
        else if (gpio_get(PTE24) == 0)
        {
            while (gpio_get(PTE24) == 0)
                ;
            return _KEY_DOWN;
        }
        else if (gpio_get(PTE26) == 0)
        {

            while (gpio_get(PTE26) == 0)
                ;
            return _KEY_CENTRE;
        }
    }
}