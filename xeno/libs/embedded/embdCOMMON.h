/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  
 *  2016 Raimarius Delgado
*/
/****************************************************************************/
#ifndef EMBD_COMMON_H
#define EMBD_COMMON_H
/****************************************************************************/
#include <rtdk.h>
//#include <embdMATH.h>
/* Scaling Macros */
#define SCALE_1K		(1000)
#define SCALE_10K		(10000)
#define SCALE_100K		(100000)
#define SCALE_1M		(1000000)

/* Error Handling */
#define _EMBD_RET_ERR_		(-1)
#define _EMBD_RET_SCC_		(0)

#define printf rt_printf
#define PI (float)3.14159265359
#define TWO_PI (float)(2*PI)


typedef enum{
	OFF = 0,
	ON,
}FLAG;

typedef enum{
	false = 0,
	true = 1,
} bool;

#define NSEC_PER_USEC (1000L)
#define NSEC_PER_MSEC (1000000L)
#define NSEC_PER_SEC  (1000000000L)

#define FREQ_PER_SEC(x)		(NSEC_PER_SEC/(x))

/****************************************************************************/
#endif // EMBD_COMMON_H
