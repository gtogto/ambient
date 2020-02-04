#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "InitDevice.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "jog_v2.h"
#include "task_mgc.h"
#include "task_report.h"
#include "task_light.h"
#include <stdio.h>


TaskHandle_t gMgcTaskHandle, gReportTaskHandle, gLightTaskHandle;
QueueHandle_t gKeyCodeQueueHandle, gLightColorQueue;

int _write(int file, char *ptr, int len);

int main(void)
{
	/* Chip errata */
	CHIP_Init();

	/* Peripherals Init */
	enter_DefaultMode_from_RESET();
	// Enable USART Interrupt
	USART_IntClear(USART1, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART1_RX_IRQn);
	USART_IntEnable(USART1, USART_IEN_RXDATAV);
	NVIC_EnableIRQ(USART1_RX_IRQn);

	/* Init variables */
	gMgcTaskHandle = NULL;
	gReportTaskHandle = NULL;

	/* Creating key code queue */
	gKeyCodeQueueHandle = xQueueCreate(20, GEST_UART_DATA_LEN);
	if (gKeyCodeQueueHandle == NULL) {
		LOG_ERR("Queue Create fail 0\n");
		return 0;
	}
	gLightColorQueue = xQueueCreate(10, LIGHT_COLOR_DATA_LEN);
	if (gLightColorQueue == NULL) {
		LOG_ERR("Queue Create fail 1\n");
		return 0;
	}

	/* Start MGC Task */
	xTaskCreate(do_mgc_task, (const char* )"gesture", configMINIMAL_STACK_SIZE*3,
			NULL, configMAX_PRIORITIES -2, gMgcTaskHandle);

	/* Start Report Task */
	xTaskCreate(do_report_task, (const char*) "report", configMINIMAL_STACK_SIZE,
			NULL, configMAX_PRIORITIES - 2, gReportTaskHandle);

	/* Start Light Task */
	xTaskCreate(do_light_task, (const char*) "light", configMINIMAL_STACK_SIZE,
				NULL, configMAX_PRIORITIES - 2, gLightTaskHandle);

	vTaskStartScheduler();
	/* Infinite loop */
}


int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i=0;
	for(i=0 ; i<len ; i++) {
		if(*ptr == '\n') {
			USART_Tx(USART1, '\r');
		}
		USART_Tx(USART1, (*ptr++));
	}
	return len;
}
