// Ethercat_Master.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include <stdio.h>
#include <string.h>
#include "sys.h"
#include "usart.h"
#include "DM9162.h"
#include "sram.h"
#include "rtc.h"
#include "bsp_timer.h"
#include "sram.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatconfig.h"

#include "ethercatcoe.h"
#include "stm32f4xx.h"
#include "DM9000.h"


#define SYNC0TIME 1000   //8000
/*
DM9161 ������  DM9000������
0    ������ ���� ������ ����   ��������
1    ������ ���� ������ ����		�����������߸���������û��
2    ������ ���� ������ ����   ����������������������û��
3    ������ ���� ������ ����   ������ ���� ����������  �м�ģ�����������
ff   ������ ������ ����
*/

/**
* @struct test_ctrl_t 
* @brief  ����������
*/
typedef struct
{
    float ctrl;
    uint8_t id;
    uint8_t state;
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
    uint32_t data4;
    uint32_t cnt;
    uint8_t arr[50];
    int arr_len;
    char cpu_state1[400];
    char cpu_state2[400];
}test_ctrl_t;

test_ctrl_t test_ctrl = 
{
    .data1 = 20,
    .arr = {0},
    .arr_len = 4,
};

struct
{
    uint32_t t_arr[5];
    int t_len[5];
    uint32_t t_addr[5];
    uint32_t r_arr[25];
    int r_len[25];
    uint32_t r_addr[25];
}sdoo = 
{
    .t_len = {4,4,4,4,4},
    .r_len = {4,4,4,4,4,4,4,4,4,4,
            4,4,4,4,4,4,4,4,4,4,4,4,4,4,4},
    .r_addr = 
{
0x607A,
0x607d,
0x607f,
0x6081,
0x6083,
0x6084,
0x6085,
0x6077,
0x606c,
0x6067,
0x6068,
0x6041,
0x6064,
0x60f4,
0x6065,
0x6062,
0x60b1,
0x60b2,
0x60e0,
0x60e1,
}
};

char IOmap[512];
uint32_t vid=0;
uint16 cur_status;
int32 cur_pos = 0;
uint8 cur_mode;
uint8 flag_time = 0;
__IO uint32_t pdoTimeFlag = 0;
int dorun=0;
PDO_Output *outputs1;
PDO_Input *inputs1;

uint8_t socket_mode;

static void TIM3_Config(uint16_t period,uint16_t prescaler);

int sahdasdj;
uint8  sadkakd[128];
int Servosetup(uint16 slave)
{	
    int retval;
    uint16 u16val;
    uint8  u8val;
    uint32 u32val;
    uint16_t index = 0;

    retval = 0;

//    u8val = 0;
//    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
//    u16val = 0x1600;	
//    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
//    u8val = 1;
//    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

//    u8val = 0;
//    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
//    u32val = 0x607A0020;
//    retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//    u32val = 0x60FE0020;
//    retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//    u32val = 0x60400010;
//    retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//    u8val = 3;
//    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

//    u8val = 0;
//    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
//    u16val = 0x1A00;
//    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
//    u8val = 1;
//    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

//    u8val = 0;
//    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
//    u32val = 0x60640020;	
//    retval += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//    u32val = 0x60FD0020;	
//    retval += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//    u32val = 0x60410010;	
//    retval += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//    u8val = 3;
//    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1606;	
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1606, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u32val = 0x607A0020;
    retval += ec_SDOwrite(slave, 0x1606, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60FE0020;
    retval += ec_SDOwrite(slave, 0x1606, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60FF0020;
    retval += ec_SDOwrite(slave, 0x1606, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60B10020;
    retval += ec_SDOwrite(slave, 0x1606, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60B20010;
    retval += ec_SDOwrite(slave, 0x1606, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60400010;
    retval += ec_SDOwrite(slave, 0x1606, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u8val = 6;
    retval += ec_SDOwrite(slave, 0x1606, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1A06;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1A06, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u32val = 0x603F0010;	
    retval += ec_SDOwrite(slave, 0x1A06, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60410010;	
    retval += ec_SDOwrite(slave, 0x1A06, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60640020;	
    retval += ec_SDOwrite(slave, 0x1A06, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x6066C020;	
    retval += ec_SDOwrite(slave, 0x1A06, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60770010;	
    retval += ec_SDOwrite(slave, 0x1A06, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60610008;	
    retval += ec_SDOwrite(slave, 0x1A06, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x00000008;	
    retval += ec_SDOwrite(slave, 0x1A06, 0x07, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u8val = 7;
    retval += ec_SDOwrite(slave, 0x1A06, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    u8val = 1;
    retval += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u32val = 0x20000;
    retval += ec_SDOwrite(slave, 0x6081, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 15566;
    retval += ec_SDOwrite(slave, 0x6083, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 15566;
    retval += ec_SDOwrite(slave, 0x6084, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x40000;
    retval += ec_SDOwrite(slave, 0x6080, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x40000;
    retval += ec_SDOwrite(slave, 0x607F, 0x00, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    while(index < 20)
    {
        ec_SDOread(slave, sdoo.r_addr[index], 0x00, FALSE, &sdoo.r_len[index], &sdoo.r_arr[index], EC_TIMEOUTRXM);
        index+=1;
    }
    return 1;
}


/**
* @brief  ���Թ�������
* @attention 
*/
void test_run(void)
{
    static uint16_t tim = 0;

    switch(test_ctrl.id)
    {
        case 1: ec_SDOread(1, sdoo.r_addr[0], 0x00, FALSE, &sdoo.r_len[0], &sdoo.r_arr[0], EC_TIMEOUTRXM); break;
        case 2: ec_SDOwrite(1, sdoo.t_addr[0], 0x00, FALSE, sdoo.t_len[0], &sdoo.t_arr[0], EC_TIMEOUTRXM); break;
        case 3:  break;
        default: break;
    }
    if(test_ctrl.state == 0)
    {
        test_ctrl.id = 0;
    }
}
RCC_ClocksTypeDef ClockInfo;
/** Read PDO assign structure */
/****************************************************
������
*****************************************************/
uint32_t timeoftest;
int main(void)
{	
    uint32_t i,j;
    uint16_t slc,cnt;
    uint8 u8val;
    uint16 u16val;

    dorun=0;
    bsp_Init();  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
    uart_init(115200);   	//���ڲ���������
    TIM2_Int_Init(0x7fffffff-1,84-1);		//us��ʱ����ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms 	  
    TIM3_Config(999,899);//10ms��ʱ��
    //	FSMC_SRAM_Init();		//��ʼ�������ⲿSRAM ����ϵͳ��ʼ����
    printf("\r\nWelcome to the ECAT World \n\r");

    vid = dm9k_ReadID();
    DM9000_Init(); //DM9000���ڳ�ʼ��

    DM9162_Init(); //DM9162���ڳ�ʼ��
    ////˫����������ģʽ
    if((ETH_ReadPHYRegister(DM9162_PHY_ADDRESS, PHY_BSR) & 0x0004)&&(dm9k_ReadReg(DM9000_REG_NSR) & DM9000_PHY))
    {
        socket_mode=2;
    }
    ////��������ģʽ
    else if((ETH_ReadPHYRegister(DM9162_PHY_ADDRESS, PHY_BSR) & 0x0004)&&(!(dm9k_ReadReg(DM9000_REG_NSR) & DM9000_PHY)))
    {
        socket_mode=1;
    }
    ////�������
    else
    {
        socket_mode=0;
    }	
    do
    {
        printf("Please plug in the Ethernet cable first \n\r");
        Delay_ms(100);
        bsp_LedToggle(2);    // �������		
    }while(socket_mode==0); //��⵽������������

    bsp_LedOff(2);  // �������

    MAC_Init(); //DM9162������ʼ��
    ec_init();//���õ�������or˫����������

    printf("\r\nSocket Mode: %d\n\r", socket_mode);	
    bsp_LedOn(1);   //  Run on	
    if(ec_config_init(TRUE)>0)		
    {
        printf("%d slaves found and configured.\r\n",ec_slavecount);         
        if((ec_slavecount >= 1))
        {
            for(slc = 1; slc <= ec_slavecount; slc++)
            {
            // beckhoff EL7031, using ec_slave[].name is not very reliable
            //							 if((ec_slave[slc].eep_man == 0x00850104) && (ec_slave[slc].eep_id == 0x01030507))
            //							 {
                printf("Found %s at position %d\n", ec_slave[slc].name, slc);
            //									 u8val = 8;
            //									 ec_SDOwrite(slc, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
            //							 }
                ec_slave[slc].PO2SOconfig = &Servosetup;
            }
        }
        /* Run IO mapping */

        ec_configdc();
        ec_dcsync0(1, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1
        ec_config_map(&IOmap);		
        //				 ec_dcsync0(2, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1 
        //				 ec_dcsync0(3, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1
        //				 ec_dcsync0(4, TRUE, SYNC0TIME, 250000); // SYNC0 on slave 1 						
        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE*4);
        ec_readstate();
        printf("Slave 0 State=0x%04x\r\n",ec_slave[0].state);
        printf("Slave 1 State=0x%04x\r\n",ec_slave[1].state);
        //				 do
        //				   {
        //							ec_statecheck(0, EC_STATE_SAFE_OP, 50000);
        //			   		 ec_statecheck(1, EC_STATE_SAFE_OP, 50000);
        //				    }
        //				    while ((ec_slave[0].state != EC_STATE_SAFE_OP) || (ec_slave[1].state != EC_STATE_SAFE_OP));
        //						printf("Slave 0 State=0x%04x\r\n",ec_slave[0].state);
        //						printf("Slave 1 State=0x%04x\r\n",ec_slave[1].state);
        /* Print som information on the mapped network */
        for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
        {
            printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
            cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
            ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            printf(" Configured address: %x\n", ec_slave[cnt].configadr);
            printf(" Outputs address: %x\n", ec_slave[cnt].outputs);
            printf(" Inputs address: %x\n", ec_slave[cnt].inputs);

            for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
            {
                printf(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n", j,
                (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
                ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
                ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
            }
            printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
            ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
        }
        printf("Request operational state for all slaves\n");
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        /* send one valid process data to make outputs in slaves happy*/
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        /* request OP state for all slaves */
        ec_writestate(0);

        /* wait for all slaves to reach OP state */
        ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);								
        do
        {
            //				   		ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            ec_statecheck(1, EC_STATE_OPERATIONAL, 50000);
        }
        while ((ec_slave[0].state != EC_STATE_OPERATIONAL)&&(ec_slave[1].state != EC_STATE_OPERATIONAL));
        printf("Slave 0 State=0x%04x\r\n",ec_slave[0].state);

        if (ec_slave[0].state == EC_STATE_OPERATIONAL)
        {	
            printf("Operational state reached for all slaves.\n");
            flag_time = 1;	
            dorun=1;							
            outputs1 = (PDO_Output *)ec_slave[1].outputs;
            inputs1  = (PDO_Input *)ec_slave[1].inputs;
        }
        else
        {
            printf("Not all slaves reached operational state.\n");
            //               ec_readstate();
            for(i = 1; i<=ec_slavecount ; i++)
            {
                if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                {
                    printf("Slave %d State=0x%04x StatusCode=0x%04x\n",
                    i, ec_slave[i].state, ec_slave[i].ALstatuscode);
                }
            }
        }
    }
    else
    {
        printf("No slaves found!\r\n");
    }			
    while(1)
    {
        if(dorun==1)
        {
            if(pdoTimeFlag == 1)	
            {
                pdoTimeFlag=0;									
                ec_receive_processdata(EC_TIMEOUTRET);
                //		  ecat_loop();

                test_run();
            }
        }
    }
}
/**
  * @brief  ͨ�ö�ʱ��3�жϳ�ʼ��
  * @param  period : �Զ���װֵ��
  * @param  prescaler : ʱ��Ԥ��Ƶ��            
  * @retval ��                             
  * @note   ��ʱ�����ʱ����㷽��:Tout=((period+1)*(prescaler+1))/Ft us.
  *          Ft=��ʱ������Ƶ��,ΪSystemCoreClock/2=90,��λ:Mhz
  */
static void TIM3_Config(uint16_t period,uint16_t prescaler)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��

    TIM_TimeBaseInitStructure.TIM_Prescaler=84 - 1;//��ʱ����Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseInitStructure.TIM_Period=SYNC0TIME - 1 ;//�Զ���װ��ֵ   84*10000/168000000
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

    //NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x0; //��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
    TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
}

/**
  * @brief  ��ʱ��3�жϷ�����
  * @param  ��
  * @retval ��
  */
void TIM3_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{  
		if(dorun==1)
		{
			 pdoTimeFlag = 1;
		   ecat_loop();
		 } 
		else
		{
		  ec_send_processdata();	
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}