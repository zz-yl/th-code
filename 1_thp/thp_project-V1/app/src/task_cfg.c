/**@file   task_cfg.c
* @brief
* UCOSIII���������ȼ��û�������ʹ��:
* ����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
* ���ȼ�0���жϷ������������� OS_IntQTask()
* ���ȼ�1��ʱ�ӽ������� OS_TickTask()
* ���ȼ�2����ʱ���� OS_TmrTask()
* ���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
* ���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()
*
* �ٽ���(�����ж�)����:
* 1,CPU_SR_ALLOC(); -- ��ʼ��
* 2,OS_CRITICAL_ENTER(); -- �����ٽ���
* 3,OS_CRITICAL_EXIT(); -- �˳��ٽ���
*
* OSʱ�ӽ���,��ÿ�������ȴ���(Ĭ��1000):
* OS_CFG_TICK_RATE_HZ
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "task_cfg.h"
#include "control.h"
#include "device.h"
#include "comm.h"
#include "test.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/* �����ջ��С */
#define TASK_STK_SIZE_START  128  ///< ��ʼ����
#define TASK_STK_SIZE        1024
#define TASK_STK_SIZE_LED    128

/**
* @enum    TASK_PRIO
* @brief   �������ȼ�
*/
typedef enum
{
    TASK_PRIO_START = 3,
    TASK_PRIO_4,
    TASK_PRIO_5,
    TASK_PRIO_6,
    TASK_PRIO_7,
    TASK_PRIO_8,
    TASK_PRIO_9,
    TASK_PRIO_10,
    TASK_PRIO_11,
}TASK_PRIO;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

/* ������ƿ� */
OS_TCB TaskTcbStart;
OS_TCB TaskTcbDev;
OS_TCB TaskTcbCtrl;
OS_TCB TaskTcbComm;
OS_TCB TaskTcbTest;
OS_TCB TaskTcbLed;

/* �����ջ */	
CPU_STK TaskStkStart[TASK_STK_SIZE_START];
CPU_STK TaskStkDev[TASK_STK_SIZE];
CPU_STK TaskStkCtrl[TASK_STK_SIZE];
CPU_STK TaskStkComm[TASK_STK_SIZE];
CPU_STK TaskStkTest[TASK_STK_SIZE];
CPU_STK TaskStkLed[TASK_STK_SIZE_LED];

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief �豸����
* @attention 
*/
void TaskDev(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        DevRun();
        OSTimeDly(1, OS_OPT_TIME_PERIODIC, &err);  //��ʱ1ms
    }
}
/**
* @brief ��������
* @attention 
*/
void TaskCtrl(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    OSTimeDly(2000, OS_OPT_TIME_PERIODIC, &err);  //��ʱ
    
    
    CtrlSysInit();
    ControlRun();
    
//    while(1)
//    {
//        OSTimeDly(1, OS_OPT_TIME_PERIODIC, &err);  //��ʱ1ms
//    }
}
/**
* @brief ͨ������
* @attention 
*/
void TaskComm(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        CommRun();
        OSTimeDly(1, OS_OPT_TIME_PERIODIC, &err);  //��ʱ1ms
    }
}
/**
* @brief ��������
* @attention 
*/
void TaskTest(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        TestRun();
        OSTimeDly(2, OS_OPT_TIME_PERIODIC, &err);  //��ʱ1ms
    }
}
/**
* @brief ϵͳ������
* @attention 
*/
void TaskLed(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        LED_RUN_TOGGLE;
        OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ
        LED_RUN_TOGGLE;
        OSTimeDlyHMSM(0,0,0,950,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ
    }
}

/**
* @brief  ��ʼ������
* @attention 
*/
void TaskStart(void *p_arg)
{
    OS_ERR err;
    CPU_SR_ALLOC();

    (void)p_arg;

    CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err); //ͳ������
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN    //���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
    //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*1=1ms
    OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
#endif

    OS_CRITICAL_ENTER();	//�����ٽ���
    
    //��������
    OSTaskCreate((OS_TCB 	* )&TaskTcbDev,
                 (CPU_CHAR	* )"dev task",
                 (OS_TASK_PTR )TaskDev,
                 (void		* )0,
                 (OS_PRIO	  )TASK_PRIO_5,
                 (CPU_STK   * )&TaskStkDev[0],
                 (CPU_STK_SIZE)TASK_STK_SIZE/10,
                 (CPU_STK_SIZE)TASK_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK	  )0,
                 (void   	* )0,
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
    //��������
    OSTaskCreate((OS_TCB 	* )&TaskTcbCtrl,
                 (CPU_CHAR	* )"ctrl task",
                 (OS_TASK_PTR )TaskCtrl,
                 (void		* )0,
                 (OS_PRIO	  )TASK_PRIO_4,
                 (CPU_STK   * )&TaskStkCtrl[0],
                 (CPU_STK_SIZE)TASK_STK_SIZE/10,
                 (CPU_STK_SIZE)TASK_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK	  )0,
                 (void   	* )0,
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
    //��������
    OSTaskCreate((OS_TCB 	* )&TaskTcbComm,
                 (CPU_CHAR	* )"comm task",
                 (OS_TASK_PTR )TaskComm,
                 (void		* )0,
                 (OS_PRIO	  )TASK_PRIO_5,
                 (CPU_STK   * )&TaskStkComm[0],
                 (CPU_STK_SIZE)TASK_STK_SIZE/10,
                 (CPU_STK_SIZE)TASK_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK	  )0,
                 (void   	* )0,
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
    //��������
    OSTaskCreate((OS_TCB 	* )&TaskTcbTest,
                 (CPU_CHAR	* )"test task",
                 (OS_TASK_PTR )TaskTest,
                 (void		* )0,
                 (OS_PRIO	  )TASK_PRIO_10,
                 (CPU_STK   * )&TaskStkTest[0],
                 (CPU_STK_SIZE)TASK_STK_SIZE/10,
                 (CPU_STK_SIZE)TASK_STK_SIZE,
                 (OS_MSG_QTY  )0,
                 (OS_TICK	  )0,
                 (void   	* )0,
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
    //��������
    OSTaskCreate((OS_TCB 	* )&TaskTcbLed,
                 (CPU_CHAR	* )"led task",
                 (OS_TASK_PTR )TaskLed,
                 (void		* )0,
                 (OS_PRIO	  )TASK_PRIO_11,
                 (CPU_STK   * )&TaskStkLed[0],
                 (CPU_STK_SIZE)TASK_STK_SIZE_LED/10,
                 (CPU_STK_SIZE)TASK_STK_SIZE_LED,
                 (OS_MSG_QTY  )0,
                 (OS_TICK	  )0,
                 (void   	* )0,
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);

    OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}
/**
* @brief  ucos3��ʼ��,��ʼ���񴴽�
* @attention 
*/
void TaskCreate(void)
{
    OS_ERR err;
    CPU_SR_ALLOC();

    OSInit(&err);    //��ʼ��UCOSIII
    OS_CRITICAL_ENTER();//�����ٽ���
    //������ʼ����
    OSTaskCreate((OS_TCB 	* )&TaskTcbStart,           //������ƿ�
                 (CPU_CHAR	* )"start task",            //��������
                 (OS_TASK_PTR )TaskStart,               //������
                 (void		* )0,                       //���ݸ��������Ĳ���
                 (OS_PRIO	  )TASK_PRIO_START,         //�������ȼ�
                 (CPU_STK   * )&TaskStkStart[0],        //�����ջ����ַ
                 (CPU_STK_SIZE)TASK_STK_SIZE_START/10,  //�����ջ�����λ
                 (CPU_STK_SIZE)TASK_STK_SIZE_START,     //�����ջ��С
                 (OS_MSG_QTY  )0,                       //�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,                       //��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ���
                 (void   	* )0,                       //�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);                   //��Ÿú�������ʱ�ķ���ֵ
    OS_CRITICAL_EXIT(); //�˳��ٽ���
    OSStart(&err);      //����UCOSIII
}
