/*
 * task_light.c
 *
 *  Created on: 2020. 1. 30.
 *      Author: netbugger
 */

#include "task_light.h"
#include "em_gpio.h"

TimerHandle_t rxTimeoutTimer;
rx_buf_t rxBuf;
extern QueueHandle_t gLightColorQueue;

void do_light_task(void *pvParameters)
{
	uint8_t lightColor[LIGHT_COLOR_DATA_LEN];
	BaseType_t ret;

	// Waiting for i2c & lightning module(lp55231) init
	vTaskDelay(200);

	lp_init(i2c_write_lightning, i2c_read_lightning);
	lp_enable_chip();

	// Rx Timer Create
	rxTimeoutTimer = xTimerCreate("rxTimeout", pdMS_TO_TICKS(1000), pdTRUE, NULL, cb_rx_timeout);

	rxBuf.idx = 0;
	vTaskDelay(2000);
	probe_i2c_addr(I2C0);

	while(1)
	{
		//wait queue color value
		ret = xQueueReceive( gLightColorQueue, (void*)lightColor, portMAX_DELAY );
		if (ret != pdTRUE) {
			//LOG_ERR("xQueueReceive fail\n");
			continue;
		}
		lp_set_pwm_by_rgb(lightColor[1], lightColor[2], lightColor[3]);
	}

}

int16_t i2c_write_lightning(uint8_t reg, uint8_t *data, int16_t len)
{
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef	i2cSeq;

	i2cSeq.addr = DEF_LP_ADDR;
	i2cSeq.flags = I2C_FLAG_WRITE_WRITE;
	i2cSeq.buf[0].data = &reg;
	i2cSeq.buf[0].len = 1;
	i2cSeq.buf[1].data = (uint8_t *)(data);
	i2cSeq.buf[1].len = len;
	ret = I2C_TransferInit(I2C0, &i2cSeq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}

	return (int16_t)ret;
}

int16_t i2c_read_lightning(uint8_t reg, uint8_t *data, int16_t len)
{
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef	i2cSeq;

	i2cSeq.addr = DEF_LP_ADDR;
	i2cSeq.flags = I2C_FLAG_WRITE_READ;
	i2cSeq.buf[0].data = &reg;
	i2cSeq.buf[0].len = 1;
	i2cSeq.buf[1].data = (uint8_t *)data;
	i2cSeq.buf[1].len = len;
	ret = I2C_TransferInit(I2C0, &i2cSeq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}

	return (int16_t)ret;
}

void USART1_RX_IRQHandler(void)
{
	uint8_t ch;

	/* Store Data */
	ch = USART_Rx(USART1);

	if(rxBuf.idx == 0) {
		// Check Header
		if(ch == LIGHT_COLOR_HEADER) {
			rxBuf.data[rxBuf.idx] = ch;
			rxBuf.idx++;
			xTimerStart(rxTimeoutTimer, 0);
		}
	}
	else {
		xTimerStart(rxTimeoutTimer, 0);
		rxBuf.data[rxBuf.idx] = ch;
		rxBuf.idx++;
		// Data Completed
		if(rxBuf.idx == LIGHT_COLOR_DATA_LEN) {
			xTimerStop(rxTimeoutTimer, 0);
			rxBuf.idx = 0;
			xQueueSend(gLightColorQueue, rxBuf.data, 0);
		}
	}
}

void cb_rx_timeout(TimerHandle_t xTimer)
{
	xTimerStop(rxTimeoutTimer, 0);
	rxBuf.idx = 0;
}

void probe_i2c_addr(I2C_TypeDef *i2c)
{
	int i;
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef	i2cSeq;

	for(i=0; i<0xff; i++) {
		i2cSeq.addr = i;
		i2cSeq.buf[0].data = 0;
		i2cSeq.buf[0].len = 0;
		i2cSeq.buf[1].data = 0;
		i2cSeq.buf[1].len = 0;
		ret = I2C_TransferInit(i2c, &i2cSeq);

		while (ret == i2cTransferInProgress)
		{
			ret = I2C_Transfer(i2c);
		}

		if( ret == i2cTransferDone) {
			printf("I2C Addr [0x%02x] Found\n", i);
		}
	}
	printf("i2c Find done\n");

}
