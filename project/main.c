#include "CH58x_common.h"
#include "msh_log.h"  
#include "drv_gpio.h"
#define LED0_PIN    drv_gpio_get(B,17)
#define SOFT_VERSION    "V1.1"

int main()
{
    uint8_t len;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_78MHz);

    Msh_Init(); 
    LOG_I("System Power On...");
    while(1)
    {
        Msh_Process();
    }
}