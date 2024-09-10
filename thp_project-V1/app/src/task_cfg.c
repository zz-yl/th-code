/**@file   task_cfg.c
* @brief
* UCOSIII中以下优先级用户程序不能使用:
* 将这些优先级分配给了UCOSIII的5个系统内部任务
* 优先级0：中断服务服务管理任务 OS_IntQTask()
* 优先级1：时钟节拍任务 OS_TickTask()
* 优先级2：定时任务 OS_TmrTask()
* 优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
* 优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()
*
* 临界区(屏蔽中断)方法:
* 1,CPU_SR_ALLOC(); -- 初始化
* 2,OS_CRITICAL_ENTER(); -- 进入临界区
* 3,OS_CRITICAL_EXIT(); -- 退出临界区
*
* OS时钟节拍,即每秒最快调度次数(默认1000):
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

/* 任务堆栈大小 */
#define TASK_STK_SIZE_START  128  ///< 起始任务
#define TASK_STK_SIZE        1024
#define TASK_STK_SIZE_LED    128

/**
* @enum    TASK_PRIO
* @brief   任务优先级
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

/* 任务控制块 */
OS_TCB TaskTcbStart;
OS_TCB TaskTcbDev;
OS_TCB TaskTcbCtrl;
OS_TCB TaskTcbComm;
OS_TCB TaskTcbTest;
OS_TCB TaskTcbLed;

/* 任务堆栈 */	
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
* @brief 设备任务
* @attention 
*/
void TaskDev(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        DevRun();
        OSTimeDly(1, OS_OPT_TIME_PERIODIC, &err);  //延时1ms
    }
}
/**
* @brief 控制任务
* @attention 
*/
void TaskCtrl(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    OSTimeDly(2000, OS_OPT_TIME_PERIODIC, &err);  //延时
    
    
    CtrlSysInit();
    ControlRun();
    
//    while(1)
//    {
//        OSTimeDly(1, OS_OPT_TIME_PERIODIC, &err);  //延时1ms
//    }
}
/**
* @brief 通信任务
* @attention 
*/
void TaskComm(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        CommRun();
        OSTimeDly(1, OS_OPT_TIME_PERIODIC, &err);  //延时1ms
    }
}
/**
* @brief 测试任务
* @attention 
*/
void TaskTest(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        TestRun();
        OSTimeDly(2, OS_OPT_TIME_PERIODIC, &err);  //延时1ms
    }
}
/**
* @brief 系统灯任务
* @attention 
*/
void TaskLed(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;
    
    while(1)
    {
        LED_RUN_TOGGLE;
        OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); //延时
        LED_RUN_TOGGLE;
        OSTimeDlyHMSM(0,0,0,950,OS_OPT_TIME_HMSM_STRICT,&err); //延时
    }
}

/**
* @brief  起始任务函数
* @attention 
*/
void TaskStart(void *p_arg)
{
    OS_ERR err;
    CPU_SR_ALLOC();

    (void)p_arg;

    CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err); //统计任务
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN    //如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
    //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*1=1ms
    OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
#endif

    OS_CRITICAL_ENTER();	//进入临界区
    
    //创建任务
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
    //创建任务
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
    //创建任务
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
    //创建任务
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
    //创建任务
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

    OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}
/**
* @brief  ucos3初始化,起始任务创建
* @attention 
*/
void TaskCreate(void)
{
    OS_ERR err;
    CPU_SR_ALLOC();

    OSInit(&err);    //初始化UCOSIII
    OS_CRITICAL_ENTER();//进入临界区
    //创建开始任务
    OSTaskCreate((OS_TCB 	* )&TaskTcbStart,           //任务控制块
                 (CPU_CHAR	* )"start task",            //任务名字
                 (OS_TASK_PTR )TaskStart,               //任务函数
                 (void		* )0,                       //传递给任务函数的参数
                 (OS_PRIO	  )TASK_PRIO_START,         //任务优先级
                 (CPU_STK   * )&TaskStkStart[0],        //任务堆栈基地址
                 (CPU_STK_SIZE)TASK_STK_SIZE_START/10,  //任务堆栈深度限位
                 (CPU_STK_SIZE)TASK_STK_SIZE_START,     //任务堆栈大小
                 (OS_MSG_QTY  )0,                       //任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,                       //当使能时间片轮转时的时间片长度，为0时为默认长度
                 (void   	* )0,                       //用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);                   //存放该函数错误时的返回值
    OS_CRITICAL_EXIT(); //退出临界区
    OSStart(&err);      //开启UCOSIII
}
