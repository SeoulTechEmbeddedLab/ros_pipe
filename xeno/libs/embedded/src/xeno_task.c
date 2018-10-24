#include "xeno_task.h"
/*****************************************************************************/
#define DEFAULT_TASK_STKSIZE 0
#define DEFAULT_TASK_MODE T_FPU|T_CPU(0)
/*****************************************************************************/
int _create_rt_task(RT_TASK *task, const char *name, int stksize, int prio, int mode) {
	int ret = -1;
	char str[1024]={0,};
	fprintf(stderr, "Create RTtask %s\n", name);
	ret = rt_task_create(task, name, stksize, prio, mode);
	if (ret != 0) {
		snprintf(str, sizeof(str), "[ERROR] Failed to create RTtask \"%s\",%d", name, ret);
		perror(str);
	}
	return ret;
}
/*****************************************************************************/
int _set_rt_task_period(RT_TASK *task, SRTIME period) {
	int ret = -1;
	RT_TASK_INFO info;
	char str[1024]={0,};

	ret = rt_task_inquire(task, &info);
	if (ret != 0) {
		return ret;
	}

	ret = rt_task_set_periodic(task, TM_NOW, period);
	if ( ret != 0 ) {
		snprintf(str, sizeof(str), "[ERROR] Failed to make periodic task \"%s\",%d", info.name, ret);
		perror(str);
	}

	return ret;
}
/*****************************************************************************/
int create_rt_task(RT_TASK *task, const char *name, int prio) {
	return _create_rt_task(task, name, DEFAULT_TASK_STKSIZE, prio, DEFAULT_TASK_MODE);
}
/*****************************************************************************/
int set_rt_task_period(RT_TASK *task, RTIME period) {
	return _set_rt_task_period(task, rt_timer_ns2ticks(period));
}
/*****************************************************************************/
int start_rt_task(int enable, RT_TASK *task, void (*fun)(void *cookie)) {
	int ret = -1;
	RT_TASK_INFO info;
	char str[1024]={0,};

	ret = rt_task_inquire(task, &info);
	if (ret != 0) {
		return ret;
	}
	if (enable) {
		ret = rt_task_start(task, fun, NULL);
		if (ret != 0) {
			snprintf(str, sizeof(str), "[ERROR] Failed to start RT task \"%s\",%d", info.name, ret);
			perror(str);
		}
		/* code */
	} else {
		rt_fprintf(stderr, "RTtask(%s) is not enabled.\n", info.name);
	}

	return ret;
}
/****************************************************************************/