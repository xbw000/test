#include "CH58x_common.h"
#include "msh_log.h"  
#include "drv_gpio.h"
#define LED0_PIN    drv_gpio_get(B,17)

int main()
{
    uint8_t len;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_78MHz);

    Msh_Init(); 
    LOG_I("System Power On...");
    drv_gpio_mode_cfg(LED0_PIN,GPIO_ModeOut_PP_20mA);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
        drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);

        drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
        drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
        drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
        drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    drv_gpio_write(LED0_PIN,PIN_HIGH);
    while(1)
    {
        Msh_Process();
    }
}