/*
 * jog_v2.h
 *
 *  Created on: 2017. 11. 1.
 *      Author: netbugger
 */

#ifndef INC_JOG_V2_H_
#define INC_JOG_V2_H_

#if 0
#define LOG_ERR	printf
#define LOG_NOR printf
#define LOG_DBG printf
#else
#define LOG_ERR(...)
#define LOG_NOR(...)
#define LOG_DBG(...)
#endif

#define N_KEYCODE_QUEUE		20

#define GEST_UART_DATA_LEN	52
#define MAX_GEST_DATA_LEN	80

#endif /* INC_JOG_V2_H_ */
