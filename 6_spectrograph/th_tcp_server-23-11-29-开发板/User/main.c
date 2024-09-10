
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
    mpu_memory_protection();                 /* ������ش洢���� */
    sys_cache_enable();                      /* ��L1-Cache */
    HAL_Init();                              /* ��ʼ��HAL�� */
    sys_stm32_clock_init(192, 5, 2, 4);      /* ����ʱ��, 480Mhz */
    delay_init(480);                         /* ��ʱ��ʼ�� */
    usart_init(115200);                      /* ���ڳ�ʼ�� */
    led_init();                              /* ��ʼ��LED */
    key_init();                              /* ��ʼ��KEY */
    dac_init(1);
    dac_init(2);
    
    th_init();
    
    while (pcf8574_init())                   /* ��ⲻ��PCF8574 */
    {
        delay_ms(500);
        LED0_TOGGLE();                       /* �����˸ */
    }
    
    freertos_demo();
}
