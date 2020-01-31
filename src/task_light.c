/*
 * task_light.c
 *
 *  Created on: 2020. 1. 30.
 *      Author: netbugger
 */

#include "task_light.h"

int16_t i2c_write_lightning(uint8_t reg, uint8_t *data, int len);
int16_t i2c_read_lightning(uint8_t reg, uint8_t *data, int len);

void do_light_task(void *pvParameters)
{
	// Waiting for i2c & lightning module(lp55231) init
	vTaskDelay(200);

	lp_init(i2c_write_lightning, i2c_read_lightning);
	void lp_enable_chip();
	while(1)
	{
		//wait queue color value
		ret = xQueueReceive( gKeyCodeQueueHandle, (void*)data, portMAX_DELAY );
				 if(ret != pdTRUE ) {
					 LOG_ERR("xQueueReceive fail\n");
					 continue;
				 }


	}

}

int16_t i2c_write_lightning(uint8_t reg, uint8_t *data, int len)
{
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef	i2cSeq;

	i2cSeq.addr = DEF_LP_ADDR;
	i2cSeq.flags = I2C_FLAG_WRITE_WRITE;
	i2cSeq.buf[0].data = &reg;
	i2cSeq.buf[0].len = 1;
	i2cSeq.buf[1].data = (uint8_t *)(data);
	i2cSeq.buf[1].len = len;
	ret = I2C_TransferInit(I2C1, &i2cSeq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C1);
	}

	return (int16_t)ret;
}

int16_t i2c_read_lightning(uint8_t reg, uint8_t *data, int len)
{
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef	i2cSeq;

	i2cSeq.addr = DEF_LP_ADDR;
	i2cSeq.flags = I2C_FLAG_WRITE_READ;
	i2cSeq.buf[0].data = &reg;
	i2cSeq.buf[0].len = 1;
	i2cSeq.buf[1].data = (uint8_t *)data;
	i2cSeq.buf[1].len = len;
	ret = I2C_TransferInit(I2C1, &i2cSeq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C1);
	}

	return (int16_t)ret;
}


