/*
 * task_light.h
 *
 *  Created on: 2020. 1. 30.
 *      Author: netbugger
 */

#ifndef INC_TASK_LIGHT_H_
#define INC_TASK_LIGHT_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "em_i2c.h"
#include "em_usart.h"
#include "lp55231.h"

#define LIGHT_COLOR_DATA_LEN	4
typedef struct {
	int		idx;
	uint8_t data[LIGHT_COLOR_DATA_LEN];
}rx_buf_t;
#define LIGHT_COLOR_HEADER	'A'

//Function Prototype
void do_light_task(void *pvParameters);
int16_t i2c_write_lightning(uint8_t reg, uint8_t *data, int16_t len);
int16_t i2c_read_lightning(uint8_t reg, uint8_t *data, int16_t len);
void cb_rx_timeout(TimerHandle_t xTimer);
void USART1_RX_IRQHandler(void);
#endif /* INC_TASK_LIGHT_H_ */
