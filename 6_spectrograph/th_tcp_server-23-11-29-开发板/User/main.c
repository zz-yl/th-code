
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/MPU/mpu.h"
#include "./BSP/PCF8574/pcf8574.h"
#include "./BSP/DAC/dac.h"
#include "lwip_comm.h"
#include "lwipopts.h"
#include "freertos_demo.h"

#include "th_cfg.h"

int main(void)
{
    Write_Through();
    mpu_memory_protection();                 /* 保护相关存储区域 */
    sys_cache_enable();                      /* 打开L1-Cache */
    HAL_Init();                              /* 初始化HAL库 */
    sys_stm32_clock_init(192, 5, 2, 4);      /* 设置时钟, 480Mhz */
    delay_init(480);                         /* 延时初始化 */
    usart_init(115200);                      /* 串口初始化 */
    led_init();                              /* 初始化LED */
    key_init();                              /* 初始化KEY */
    dac_init(1);
    dac_init(2);
    
    th_init();
    
    while (pcf8574_init())                   /* 检测不到PCF8574 */
    {
        delay_ms(500);
        LED0_TOGGLE();                       /* 红灯闪烁 */
    }
    
    freertos_demo();
}
