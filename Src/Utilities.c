#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "string.h"
#include "usb_device.h"

//Tramsmit string via USB
void USB_write(char *msg)
{
	while (CDC_Transmit_FS((uint8_t *)msg, strlen(msg)) == USBD_BUSY);
}


// searches for the string sfind in the string str
// returns position if string found
// returns 0 if string not found
int StrContains(char *str, char *sfind)
{
    #define LEN strlen(str)
	uint8_t found = 0;
	uint8_t index = 0;

    if (strlen(sfind) > LEN) return 0;
    while (index < LEN)
    {
        if (str[index] == sfind[found])
        {
            found++;
            if (strlen(sfind) == found) return index;
        }
        else found = 0;
        index++;
    }
    return 0;
}



