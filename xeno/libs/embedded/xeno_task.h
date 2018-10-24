#ifndef _XENO_TASK_H_
#define _XENO_TASK_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
/*****************************************************************************/
/* Xenomai */
/*****************************************************************************/
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h> //rt_printf
#include <embdCOMMON.h>
/*****************************************************************************/
/* Real-time Task */
/*****************************************************************************/
int _create_rt_task(RT_TASK *task, const char *name, int stksize, int prio, int mode);
int _set_rt_task_period(RT_TASK *task, SRTIME period);
int create_rt_task(RT_TASK *task, const char *name, int prio);
int set_rt_task_period(RT_TASK *task, RTIME period);
int start_rt_task(int enable, RT_TASK *task, void (*fun)(void *cookie));

#endif // _XENO_TASK_H_