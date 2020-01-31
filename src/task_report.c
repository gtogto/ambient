/*
 * task_report.c
 *
 *  Created on: 2017. 11. 1.
 *      Author: netbugger
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "task_report.h"
#include "jog_v2.h"
#include <stdint.h>
#include <stdio.h>
#include "em_usart.h"

extern QueueHandle_t gKeyCodeQueueHandle;
void do_report_task(void *pvParameters)
{
	/* Task Initialize */
	uint8_t data[GEST_UART_DATA_LEN];
	BaseType_t ret;
	int i;

	vTaskDelay(100);
	LOG_NOR("[REPORT]Start\n");

	/* Service Routine */
	while(1) {
		/* Check Queue */
		 ret = xQueueReceive( gKeyCodeQueueHandle, (void*)data, portMAX_DELAY );
		 if(ret != pdTRUE ) {
			 LOG_ERR("xQueueReceive fail\n");
			 continue;
		 }

		for(i=0; i<GEST_UART_DATA_LEN;i++) {
			USART_Tx(USART1, data[i]);
		}
	}
}
