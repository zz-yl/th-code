/**@file   bsp_mpu.c
* @brief   mpu����
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_mpu.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/


/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/
 
 
/**
 * @brief       ����ĳ�������MPU����
 * @param       baseaddr:MPU��������Ļ�ַ(�׵�ַ)
 * @param       size:MPU��������Ĵ�С(������32�ı���,��λΪ�ֽ�),�����õ�ֵ�ο�:CORTEX_MPU_Region_Size
 * @param       rnum:MPU���������,��Χ:0~7,���֧��8����������,�����õ�ֵ�ο���CORTEX_MPU_Region_Number
 * @param       de:��ָֹ�����;0,����ָ�����;1,��ָֹ�����
 * @param       ap:����Ȩ��,���ʹ�ϵ����:�����õ�ֵ�ο���CORTEX_MPU_Region_Permission_Attributes
 *   @arg       MPU_REGION_NO_ACCESS,�޷��ʣ���Ȩ&�û������ɷ��ʣ�
 *   @arg       MPU_REGION_PRIV_RW,��֧����Ȩ��д����
 *   @arg       MPU_REGION_PRIV_RW_URO,��ֹ�û�д���ʣ���Ȩ�ɶ�д���ʣ�
 *   @arg       MPU_REGION_FULL_ACCESS,ȫ���ʣ���Ȩ&�û����ɷ��ʣ�
 *   @arg       MPU_REGION_PRIV_RO,��֧����Ȩ������
 *   @arg       MPU_REGION_PRIV_RO_URO,ֻ������Ȩ&�û���������д��
 *   @arg       ���:STM32F7����ֲ�.pdf,4.6��,Table 89.
 * @param       sen  : �Ƿ�������;MPU_ACCESS_NOT_SHAREABLE,������;MPU_ACCESS_SHAREABLE,����
 * @param       cen  : �Ƿ�����cache;MPU_ACCESS_NOT_CACHEABLE,������;MPU_ACCESS_CACHEABLE,����
 * @param       ben  : �Ƿ�������;MPU_ACCESS_NOT_BUFFERABLE,������;MPU_ACCESS_BUFFERABLE,����
 * @retval      0,�ɹ�.
 *              ����,����.
 */
uint8_t mpu_set_protection(uint32_t baseaddr, uint32_t size, uint32_t rnum, uint8_t de, uint8_t ap, uint8_t sen, uint8_t cen, uint8_t ben)
{
    MPU_Region_InitTypeDef mpu_region_init_handle;

    HAL_MPU_Disable();                                                   /* ����MPU֮ǰ�ȹر�MPU,��������Ժ���ʹ��MPU */

    mpu_region_init_handle.Enable = MPU_REGION_ENABLE;                   /* ʹ�ܸñ������� */
    mpu_region_init_handle.Number = rnum;                                /* ���ñ������� */
    mpu_region_init_handle.BaseAddress = baseaddr;                       /* ���û�ַ */
    mpu_region_init_handle.DisableExec = de;                             /* �Ƿ�����ָ����� */
    mpu_region_init_handle.Size = size;                                  /* ���ñ��������С */
    mpu_region_init_handle.SubRegionDisable = 0X00;                      /* ��ֹ������ */
    mpu_region_init_handle.TypeExtField = MPU_TEX_LEVEL0;                /* ����������չ��Ϊlevel0 */
    mpu_region_init_handle.AccessPermission = (uint8_t)ap;               /* ���÷���Ȩ��, */
    mpu_region_init_handle.IsShareable = sen;                            /* �Ƿ���? */
    mpu_region_init_handle.IsCacheable = cen;                            /* �Ƿ�cache? */
    mpu_region_init_handle.IsBufferable = ben;                           /* �Ƿ񻺳�? */
    HAL_MPU_ConfigRegion(&mpu_region_init_handle);                       /* ����MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);                              /* ����MPU */
    return 0;
}

/**
 * @brief       ������Ҫ�����Ĵ洢��
 * @param       ��
 * @note        ����Բ��ִ洢�������MPU����,������ܵ��³��������쳣
 *              ����MCU������ʾ,����ͷ�ɼ����ݳ���ȵ�����...
 */
void mpu_memory_protection(void)
{
    /* ��������DTCM,��128K�ֽ� */
    mpu_set_protection( 0x20000000,                 /* ����ַ */
                        MPU_REGION_SIZE_128KB,      /* ���� */
                        MPU_REGION_NUMBER1, 0,      /* NUMER1,����ָ����� */
                        MPU_REGION_FULL_ACCESS,     /* ȫ���� */
                        MPU_ACCESS_NOT_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_BUFFERABLE);     /* ������ */
    
    /* ��������AXI SRAM,��512K�ֽ� */
    mpu_set_protection( 0x24000000,                 /* ����ַ */
                        MPU_REGION_SIZE_512KB,      /* ���� */
                        MPU_REGION_NUMBER2, 0,      /* NUMER2,����ָ����� */
                        MPU_REGION_FULL_ACCESS,     /* ȫ���� */
                        MPU_ACCESS_NOT_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
//                        MPU_ACCESS_NOT_CACHEABLE,   /* ��ֹcache */
                        MPU_ACCESS_NOT_BUFFERABLE); /* ��ֹ���� */
    
    /* ��������SRAM1~SRAM3,��512K�ֽ� */
    mpu_set_protection( 0x30000000,                 /* ����ַ */
                        MPU_REGION_SIZE_512KB,      /* ���� */
                        MPU_REGION_NUMBER3, 0,      /* NUMER3,����ָ����� */
                        MPU_REGION_FULL_ACCESS,     /* ȫ���� */
                        MPU_ACCESS_NOT_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_BUFFERABLE);     /* ������ */
                        
    /* ��������SRAM4,��64K�ֽ� */
    mpu_set_protection( 0x38000000,                 /* ����ַ */
                        MPU_REGION_SIZE_64KB,       /* ���� */
                        MPU_REGION_NUMBER4, 0,      /* NUMER4,����ָ����� */
                        MPU_REGION_FULL_ACCESS,     /* ȫ���� */
                        MPU_ACCESS_NOT_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_NOT_CACHEABLE,   /* ��ֹcache */
                        MPU_ACCESS_NOT_BUFFERABLE); /* ��ֹ���� */
                        
    /* ����MCU LCD�����ڵ�FMC����,��64M�ֽ� */
    mpu_set_protection( 0x60000000,                 /* ����ַ */
                        MPU_REGION_SIZE_64MB,       /* ���� */
                        MPU_REGION_NUMBER5, 0,      /* NUMER5,����ָ����� */
                        MPU_REGION_FULL_ACCESS,     /* ȫ���� */
                        MPU_ACCESS_NOT_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_NOT_CACHEABLE,   /* ��ֹcache */
                        MPU_ACCESS_NOT_BUFFERABLE); /* ��ֹ���� */
                        
    /* ����SDRAM����,��32M�ֽ� */
    mpu_set_protection( 0xC0000000,                 /* ����ַ */
                        MPU_REGION_SIZE_32MB,       /* ���� */
                        MPU_REGION_NUMBER6, 0,      /* NUMER6,����ָ����� */
                        MPU_REGION_FULL_ACCESS,     /* ȫ���� */
                        MPU_ACCESS_NOT_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_CACHEABLE,       /* ����cache */
                        MPU_ACCESS_BUFFERABLE);     /* ������ */
                        
    /* ��������NAND FLASH����,��256M�ֽ� */
    mpu_set_protection( 0x80000000,                 /* ����ַ */
                        MPU_REGION_SIZE_256MB,      /* ���� */
                        MPU_REGION_NUMBER7, 1,      /* NUMER7,��ָֹ����� */
                        MPU_REGION_FULL_ACCESS,     /* ȫ���� */
                        MPU_ACCESS_NOT_SHAREABLE,   /* ��ֹ���� */
                        MPU_ACCESS_NOT_CACHEABLE,   /* ��ֹcache */
                        MPU_ACCESS_NOT_BUFFERABLE); /* ��ֹ���� */
}

