/*
 * lp55231.c
 *
 *  Created on: 2020. 1. 28.
 *      Author: netbugger
 */
#include "lp55231.h"

void lp_init(int16_t (*wf)(uint8_t, uint8_t*, int16_t), int16_t (*rf)(uint8_t, uint8_t*, int16_t))
{
	i2c_write_reg_fptr = wf;
	i2c_read_reg_fptr = rf;
}

void lp_enable_chip(void)
{
	uint8_t val;

	// Set enable bit
	val = (1 << CNTRL1_OFFSET_CHIP_EN);
	i2c_write_reg_fptr(REG_CNTRL1, &val, 1);

	// enable internal clock & charge pump & write auto increment
	val = (1 << MISC_OFFSET_INT_CLK_EN) | (1 << MISC_OFFSET_CLK_DET_EN) | (3 << MISC_OFFSET_CP_MODE) | (1 << MISC_OFFSET_AUTO_INCR_EN);
	i2c_write_reg_fptr(REG_MISC, &val, 1);
}

void lp_disable_chip(void)
{
	uint8_t val;
	i2c_read_reg_fptr(REG_CNTRL1, &val, 1);
	val &= ~(1<<CNTRL1_OFFSET_CHIP_EN);
	i2c_write_reg_fptr(REG_CNTRL1, &val, 1);
}

void lp_reset_chip()
{
	uint8_t val;
	val = 0xff;
	i2c_write_reg_fptr(REG_RESET, &val, 1);
}

lp_err_t lp_set_pwm_by_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t vals[9];
	vals[0] = g;		// CH0 G
	vals[1] = b;		// CH0 B
	vals[2] = g;		// CH1 G
	vals[3] = b;		// CH1 B
	vals[4] = g;		// CH2 G
	vals[5] = b;		// CH2 B
	vals[6] = r;		// CH0 R
	vals[7] = r;		// CH1 R
	vals[8] = r;		// CH2 R

	i2c_write_reg_fptr(REG_D1_PWM, vals, 9);
	return LP_ERR_NONE;
}
