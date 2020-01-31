/*
 * task_mgc.c
 *
 *  Created on: 2017. 11. 2.
 *      Author: netbugger
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include "InitDevice.h"
#include "em_gpio.h"
#include "task_mgc.h"
#include "jog_v2.h"

uint8_t gControlBuffer[GEST_CONTROL_BUFFER_SIZE] = {0,};
uint8_t gDataBuffer[GEST_RECEIVED_BUFFER_SIZE] = {0,};

gesture_buffer_t gest_buf;
extern QueueHandle_t gKeyCodeQueueHandle;

void do_mgc_task(void *pvParameters)
{
	uint32_t arg0, arg1;
	uint8_t buffer[256], param[4] = {0,};
	int i;

	/* MCLR to High */
	gest_release_reset();

	/* Wait for Init */
	vTaskDelay(100);

	LOG_NOR("[MGC]start\n");

	/* touch timer create */
	gest_buf.cnt = 0;
	for(i=0; i<GEST_BUF_SIZE; i++) {
		gest_buf.gesture[i] = GESTURE_NONE;
	}

	/* Check Firmware Virsion */
	send_request_message(GEST_FW_VERSION_INFO, param);
	if(gest_is_ts_asserted()) {
		read_gest_data(buffer, GEST_MSG_SIZE_FW_VERSION_INFO);
	}

	/* Wait for Ready to receive Parameter Setting data */
	vTaskDelay(1000);

	/* Touch Detection Enable */
#if DOUT_TOUCH_EN
	arg0 = GEST_RPID_TOUCH_DETECTION_ARG0_ENABLE;
#else
	arg0 = GEST_RPID_TOUCH_DETECTION_ARG0_DISABL;
#endif
	arg1 = GEST_RPID_TOUCH_DETECTION_ARG1;
	send_rp_with_response(GEST_RPID_TOUCH_AND_APPROACH_DETECTION, arg0, arg1, buffer, 0x20);

	/* Approach Detection Disable */
	arg0 = GEST_RPID_APPROACH_DETECTION_ARG0_DISALBLE;
	arg1 = GEST_RPID_APPROACH_DETECTION_ARG1;
	send_rp_with_response(GEST_RPID_TOUCH_AND_APPROACH_DETECTION, arg0, arg1, buffer, 0x20);

	/* Airwheel On/Off */
#if DOUT_AIRWHEEL_EN
	arg0 = GEST_RPID_AIRWHEEL_ARG0_ENABLE;
#else
	arg0 = GEST_RPID_AIRWHEEL_ARG0_DISABLE;
#endif
	arg1 = GEST_RPID_AIRWHEEL_ARG1;
	send_rp_with_response(GEST_RPID_AIRWHEEL, arg0, arg1, buffer, 0x20);

	/* Gesture Setting */
#if DOUT_GESTURE_EN
	arg0 = GEST_RPID_GESTURE_ARG0_GARBAGE_MODEL | GEST_RPID_GESTURE_ARG0_FLICK_W_TO_E |
			GEST_RPID_GESTURE_ARG0_FLICK_E_TO_W | GEST_RPID_GESTURE_ARG0_FLICK_S_TO_N |
			GEST_RPID_GESTURE_ARG0_FLICK_N_TO_S | GEST_RPID_GESTURE_ARG0_CLOCK |
			GEST_RPID_GESTURE_ARG0_COUNTER_CLOCK;
#else
	arg0 = 0;
#endif
	arg1 = GEST_RPID_GESTURE_ARG1;
	send_rp_with_response(GEST_RPID_GESTURE, arg0, arg1, buffer, 0x20);

	arg0 = 0;
#if DOUT_DSP_STATUS_EN
	arg0 |= GEST_PRID_DOUT_EN_DSP_STATUS;
#endif
#if DOUT_GESTURE_EN
	arg0 |= GEST_PRID_DOUT_EN_GESTURE_DATA;
#endif
#if DOUT_TOUCH_EN
	arg0 |= GEST_PRID_DOUT_EN_TOUCHINFO;
#endif
#if DOUT_AIRWHEEL_EN
	arg0 |= GEST_PRID_DOUT_EN_AIRWHEELINFO;
#endif
#if DOUT_XYZ_EN
	arg0 |= GEST_PRID_DOUT_EN_XYZPOSITION;
#endif
	arg0 |= GEST_PRID_DOUT_EN_CICDATA;
	arg0 |= GEST_PRID_DOUT_EN_SDDATA;
	arg1 = GEST_RPID_DOUT_ENABLE_MASK_ARG1;
	send_rp_with_response(GEST_RPID_DOUT_ENABLE_MASK, arg0, arg1, buffer, 0x20);

/* DOUT LOCK */
	arg0 = 0;
#if DOUT_DSP_STATUS_EN
	arg0 |= GEST_PRID_DOUT_LOCK_DSP_STATUS;
#endif
#if DOUT_GESTURE_EN
	arg0 |= GEST_PRID_DOUT_LOCK_GESTURE_DATA;
#endif
#if DOUT_TOUCH_EN
	arg0 |= GEST_PRID_DOUT_LOCK_TOUCHINFO;
#endif
#if DOUT_AIRWHEEL_EN
	arg0 |= GEST_PRID_DOUT_LOCK_AIRWHEELINFO;
#endif
#if DOUT_XYZ_EN
	arg0 |= GEST_PRID_DOUT_LOCK_XYZPOSITION;
#endif
	arg1 = GEST_RPID_DOUT_LOCK_MASK_ARG1;
	send_rp_with_response(GEST_RPID_DOUT_LOCK_MASK, arg0, arg1, buffer, 0x20);

	while(1) {
		//read_gest_data(buffer, 0x20);
		read_gest_data(buffer, MAX_GEST_DATA_LEN);
		vTaskDelay(MGC_DEFAULT_DELAY);
	}
}

void gest_release_reset()
{
	GPIO_PinOutSet(MGC_MCLR_PORT, MGC_MCLR_PIN);
}

void gest_hold_reset()
{
	GPIO_PinOutClear(MGC_MCLR_PORT, MGC_MCLR_PIN);
}

uint8_t gest_is_ts_asserted()
{
	return (uint8_t)!(GPIO_PinInGet(MGC_TS_PORT, MGC_TS_PIN));
}

void gest_ts_assert()
{
	GPIO_PinModeSet(MGC_TS_PORT, MGC_TS_PIN, gpioModePushPull, 0);
}

void gest_ts_release()
{
	GPIO_PinOutSet(MGC_TS_PORT, MGC_TS_PIN);
	vTaskDelay(1); // minimum 200us
	GPIO_PinModeSet(MGC_TS_PORT, MGC_TS_PIN, gpioModeInput, 1);
}

I2C_TransferReturn_TypeDef i2c_write_gest_ic(const uint8_t *data, int len)
{
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef	i2cSeq;

	i2cSeq.addr = GEST_IC_ADDR;
	i2cSeq.flags = I2C_FLAG_WRITE_WRITE;
	i2cSeq.buf[0].data = 0;
	i2cSeq.buf[0].len = 0;
	i2cSeq.buf[1].data = (uint8_t *)(data);
	i2cSeq.buf[1].len = len;
	ret = I2C_TransferInit(I2C0, &i2cSeq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}

	return ret;
}

I2C_TransferReturn_TypeDef i2c_read_gest_ic(uint8_t *data, int len)
{
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef	i2cSeq;

	gest_ts_assert();

	i2cSeq.addr = GEST_IC_ADDR;
	i2cSeq.flags = I2C_FLAG_WRITE_READ;
	i2cSeq.buf[0].data = 0;
	i2cSeq.buf[0].len = 0;
	i2cSeq.buf[1].data = (uint8_t *)(data);
	i2cSeq.buf[1].len = len;
	ret = I2C_TransferInit(I2C0, &i2cSeq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}

	gest_ts_release();
	return ret;
}

int send_request_message(uint8_t msgId, uint8_t param[])
{
	gest_header_t* pHeader = (gest_header_t*)gControlBuffer;
	gest_request_message_t *pPayload = (gest_request_message_t*)(gControlBuffer+sizeof(gest_header_t));

	pHeader->msgSize = GEST_MSG_SIZE_REQUEST_MSG;
	pHeader->id = GEST_REQUEST_MESSAGE;
	pPayload->messageId = msgId;
	memset(pPayload->reserved, 0x00, 3);
	memcpy(pPayload->param, param, 4);

	return (i2c_write_gest_ic(gControlBuffer, sizeof(gest_header_t)+ sizeof(gest_request_message_t)));

	return 0;
}

int send_set_runtime_parameter(uint16_t id, uint32_t arg0, uint32_t arg1)
{
	gest_header_t* pHeader = (gest_header_t*)gControlBuffer;
	gest_runtime_parameter_t *pPayload = (gest_runtime_parameter_t*)(gControlBuffer+sizeof(gest_header_t));

	pHeader->msgSize = GEST_MSG_SIZE_SET_RUNTIME;
	pHeader->id = GEST_SET_RUNTIME_PARAMETER;
	pPayload->id[0] = (uint8_t)((id & 0x00FF) >> 0);
	pPayload->id[1] = (uint8_t)((id & 0xFF00) >> 8);
	memset(pPayload->reserved, 0x00, 2);
	pPayload->arg0[0] = (uint8_t)((arg0 & 0x000000FF) >> 0);
	pPayload->arg0[1] = (uint8_t)((arg0 & 0x0000FF00) >> 8);
	pPayload->arg0[2] = (uint8_t)((arg0 & 0x00FF0000) >> 16);
	pPayload->arg0[3] = (uint8_t)((arg0 & 0xFF000000) >> 24);

	pPayload->arg1[0] = (uint8_t)((arg1 & 0x000000FF) >> 0);
	pPayload->arg1[1] = (uint8_t)((arg1 & 0x0000FF00) >> 8);
	pPayload->arg1[2] = (uint8_t)((arg1 & 0x00FF0000) >> 16);
	pPayload->arg1[3] = (uint8_t)((arg1 & 0xFF000000) >> 24);

	return (i2c_write_gest_ic(gControlBuffer, sizeof(gest_header_t)+ sizeof(gest_runtime_parameter_t)));

	return 0;
}

int parse_gest_data(uint8_t *data)
{
	gest_header_t* pHeader = (gest_header_t*)data;
	switch(pHeader->id) {
	case GEST_SYSTEM_STATUS :
		hadle_system_status(data);
		break;
	case GEST_FW_VERSION_INFO :
		hadle_fw_version_info(data);
		break;

	case GEST_SENSOR_DATA_OUTPUT :
		handle_sensor_data_output(data);
		break;

	default :
		LOG_ERR("UNKNOWN ID[%02x]\n", pHeader->id);
		return -1;
	}
	return 0;
}

void hadle_system_status(uint8_t *data)
{
	gest_system_status_t *pPayload = (gest_system_status_t*)(data + sizeof(gest_header_t));

	LOG_DBG("[System Status]\n");
	LOG_DBG("msgID      : 0x%02x\n", pPayload->messageId);
	LOG_DBG("maxCmdSize : 0x%02x\n", pPayload->maxCmdSize);
	LOG_DBG("errorCode  : 0x%02x%02x\n\n", pPayload->errorCode[1], pPayload->errorCode[0]);
}

void hadle_fw_version_info(uint8_t *data)
{
	gest_fw_version_info_t *pPayload = (gest_fw_version_info_t*)(data + sizeof(gest_header_t));

	data[sizeof(gest_fw_version_info_t) + sizeof(gest_header_t)] = '\0';
	LOG_DBG("[Fw Version Info]\n");
	LOG_DBG("fwValid               : 0x%02x\n", pPayload->fwValid);
	LOG_DBG("hwRev                 : 0x%02x%02x\n", pPayload->hwRev[0], pPayload->hwRev[1]);
	LOG_DBG("parameterStartAddr    : 0x%02x\n", pPayload->parameterStartAddr);
	LOG_DBG("libraryLoaderVersion  : 0x%02x%02x\n",
			pPayload->libraryLoaderVersion[0], pPayload->libraryLoaderVersion[1]);
	LOG_DBG("libraryLoaderPlatform : 0x%02x\n", pPayload->libraryLoaderPlatform);
	LOG_DBG("fwStartAddr           : 0x%02x\n", pPayload->fwStartAddr);
	LOG_DBG("fwVersion             : %s\n\n", pPayload->fwVersion);
}

void handle_sensor_data_output(uint8_t *data)
{
	gest_sensor_data_output_t *pPayload = (gest_sensor_data_output_t*)(data + sizeof(gest_header_t));
	sensor_data_t sensorData;
	uint16_t touchEvent;

	/* Airwheel and XYZ position must be compared wtih last data */
#if DOUT_AIRWHEEL_EN
	static airwheel_info_t	lastAirWheel;
#endif
#if DOUT_XYZ_EN
	static position_t lastPosition;
#endif

#if DOUT_GESTURE_EN
	parse_gesture_info(&(sensorData.gestureInfo), pPayload->gestureInfo);
#endif
#if DOUT_TOUCH_EN
	parse_touch_info(&(sensorData.touchInfo), pPayload->touchInfo);
#endif
#if DOUT_AIRWHEEL_EN
	parse_airwheel_info(&(sensorData.airwheelInfo), pPayload->airWheelInfo);
#endif
#if DOUT_XYZ_EN
	parse_xyz_position_info(&(sensorData.position), pPayload->xyzPosition);
#endif
	sensorData.pCicData = pPayload->cicData;
	sensorData.pSdData = pPayload->sdData;

#if DOUT_GESTURE_EN
	if(sensorData.gestureInfo.gesture != GESTURE_NONE) {
#if 0
		if(gest_buf.cnt == 0) {
			xTimerStart(mgcGestureTimerHandle, 0);
		}
		add_gesture(sensorData.gestureInfo.gesture);
#else
		//send_gesture_info(sensorData.gestureInfo.gesture);
#endif
	}
	//print_gesture_info(&(sensorData.gestureInfo));

#endif
#if DOUT_TOUCH_EN
	//touchEvent = get_touch_event(&(sensorData.touchInfo));
	//print_touch_info(touchEvent);
	//send_touch_info(touchEvent);
#endif
#if DOUT_AIRWHEEL_EN
	if(memcmp(&lastAirWheel, &(sensorData.airwheelInfo), sizeof(airwheel_info_t)) != 0) {
		print_airwheel_info(&(sensorData.airwheelInfo));
		memcpy(&lastAirWheel, &(sensorData.airwheelInfo), sizeof(airwheel_info_t));
	}
#endif
#if DOUT_XYZ_EN
	//if(memcmp(&lastPosition, &(sensorData.position), sizeof(position_t)) != 0) {
	//	print_xyz_position(&(sensorData.position));
	//	memcpy(&lastPosition, &(sensorData.position), sizeof(position_t));
	//}
#endif

	send_gest_info(&sensorData);
}

void parse_gesture_info(gesture_info_t *pGestureInfo, uint8_t *data)
{
	uint32_t val;
	val = conv_array_to_32bit(data);

	pGestureInfo->gesture = (uint8_t)((val & GESTURE_INFO_GESTURE_MASK) >> GESTURE_INFO_GESTURE_OFFSET);
	pGestureInfo->gestureClass = (uint8_t)((val & GESTURE_INFO_GESTURE_CLASS_MASK) >> GESTURE_INFO_GESTURE_CLASS_OFFSET);
	pGestureInfo->edgeFlickFlag = (uint8_t)((val & GUESTURE_INFO_EDGE_FLICK_FLAG_MASK) >> GUESTURE_INFO_EDGE_FLICK_FLAG_OFFSET);
	pGestureInfo->handPresenceFlag = (uint8_t)((val & GESTURE_INFO_HAND_PRESENCE_FLAG_MASK) >> GESTURE_INFO_HAND_PRESENCE_FLAG_OFFSET);
	pGestureInfo->handHoldFlag = (uint8_t)((val & GESTURE_INFO_HAND_HOLD_FLAG_MASK) >> GESTURE_INFO_HAND_HOLD_FLAG_OFFSET);
	pGestureInfo->handInsideFlag = (uint8_t)((val & GESTURE_INFO_HAND_INSIDE_FLAG_MASK) >> GESTURE_INFO_HAND_INSIDE_FLAG_OFFSET);
	pGestureInfo->gestureRecognizingFlag = (uint8_t)((val & GESTURE_INFO_RECOGNIZING_FLAG_MASK) >> GESTURE_INFO_RECOGNIZING_FLAG_OFFSET);
}

void parse_touch_info(touch_info_t *pTouchInfo, uint8_t *data)
{
	uint32_t val;
	val = conv_array_to_32bit(data);

	pTouchInfo->event = (uint16_t)((val & TOUCH_INFO_TOUCH_EVENT_MASK) >> TOUCH_INFO_TOUCH_EVENT_OFFSET);
	pTouchInfo->counter = (uint8_t)((val & TOUCH_INFO_TOUCH_COUNTER_MASK) >> TOUCH_INFO_TOUCH_COUNTER_OFFSET);
}

void parse_airwheel_info(airwheel_info_t *pAirwheelInfo, uint8_t *data)
{
	pAirwheelInfo->currentPosition = (uint8_t)((data[0] & AIRWHEEL_INFO_CURRENT_POSITION_MASK) >> AIRWHEEL_INFO_CURRENT_POSITION_OFFSET);
	pAirwheelInfo->fullLotationCount = (uint8_t)((data[0] & AIRWHEEL_INFO_ROTATION_COUNT_MASK) >> AIRWHEEL_INFO_ROTATION_COUNT_OFFSET);
}

void parse_xyz_position_info(position_t *pPosition, uint8_t *data)
{
	pPosition->x = ((uint16_t)(data[1])) << 8 | (uint16_t)data[0];
	pPosition->y = ((uint16_t)(data[3])) << 8 | (uint16_t)data[2];
	pPosition->z = ((uint16_t)(data[5])) << 8 | (uint16_t)data[4];
}

uint32_t conv_array_to_32bit(uint8_t *src)
{
	return (uint32_t)((uint32_t)src[3] << 24 | (uint32_t)src[2] << 16 | (uint32_t)src[1] << 8 | (uint32_t)src[0]);
}

void print_gesture_info(gesture_info_t *pGestureInfo)
{
#if 0
	LOG_DBG("[Gesture Info : Gesture(edge|presence|hold|inside|recognizing)]\n");
	LOG_DBG("Gesture : %s(%s|%s|%s|%s|%s)\n", str_gesture(pGestureInfo->gesture),
			pGestureInfo->edgeFlickFlag?"ON":"OFF",
			pGestureInfo->handPresenceFlag?"ON":"OFF",
			pGestureInfo->handHoldFlag?"ON":"OFF",
			pGestureInfo->handInsideFlag?"ON":"OFF",
			pGestureInfo->gestureRecognizingFlag?"ON":"OFF");
#else
	if(pGestureInfo->gesture != GESTURE_NONE) {
		LOG_DBG("Gesture : %s\n", str_gesture(pGestureInfo->gesture));
	}
#endif
}

uint16_t get_touch_event(touch_info_t *pTouchInfo)
{
	uint16_t event = TOUCH_NONE;


	if (pTouchInfo->event & (0x0001 << TOUCH_SOUTH)) {
		event = TOUCH_SOUTH;
	}
	else if (pTouchInfo->event & (0x0001 << TOUCH_WEST)) {
		event = TOUCH_WEST;
	}
	else if (pTouchInfo->event & (0x0001 << TOUCH_NORTH)) {
		event = TOUCH_NORTH;
	}
	else if (pTouchInfo->event & (0x0001 << TOUCH_EAST)) {
		event = TOUCH_EAST;
	}
	else if (pTouchInfo->event & (0x0001 << TOUCH_CENTER)) {
		event = TOUCH_CENTER;
	}
	if (pTouchInfo->event & (0x0001 << TAP_SOUTH)) {
		event = TAP_SOUTH;
	}
	else if (pTouchInfo->event & (0x0001 << TAP_WEST)) {
		event = TAP_WEST;
	}
	else if (pTouchInfo->event & (0x0001 << TAP_NORTH)) {
		event = TAP_NORTH;
	}
	else if (pTouchInfo->event & (0x0001 << TAP_EAST)) {
		event = TAP_EAST;
	}
	else if (pTouchInfo->event & (0x0001 << TAP_CENTER)) {
		event = TAP_CENTER;
	}
	else if (pTouchInfo->event & (0x0001 << DOUBLE_TAP_SOUTH)) {
		event = DOUBLE_TAP_SOUTH;
	}
	else if (pTouchInfo->event & (0x0001 << DOUBLE_TAP_WEST)) {
		event = DOUBLE_TAP_WEST;
	}
	else if (pTouchInfo->event & (0x0001 << DOUBLE_TAP_NORTH)) {
		event = DOUBLE_TAP_NORTH;
	}
	else if (pTouchInfo->event & (0x0001 << DOUBLE_TAP_EAST)) {
		event = DOUBLE_TAP_EAST;
	}
	else if (pTouchInfo->event & (0x0001 << DOUBLE_TAP_CENTER)) {
		event = DOUBLE_TAP_CENTER;
	}
	return event;
}

void print_touch_info(uint16_t event)
{
	if(event == TOUCH_NONE) {
	//	LOG_DBG("TOUCH_NONE\n");
		return;
	}
//	LOG_DBG("[Touch Info]\n");
	switch(event) {
#if 0
	case TOUCH_SOUTH :
		LOG_DBG("SOUTH\n");
		break;

	case TOUCH_WEST :
		LOG_DBG("WEST\n");
		break;

	case TOUCH_NORTH :
		LOG_DBG("NORTH\n");
		break;

	case TOUCH_EAST:
		LOG_DBG("EAST\n");
		break;

	case TOUCH_CENTER:
		LOG_DBG("CENTER\n");
		break;
#endif

	case TAP_SOUTH:
		LOG_DBG("TAP_SOUTH\n");
		break;

	case TAP_WEST:
		LOG_DBG("TAP_WEST\n");
		break;

	case TAP_EAST:
		LOG_DBG("TAP_EAST\n");
		break;

	case TAP_CENTER:
		LOG_DBG("TAP_CENTER\n");
		break;

	case DOUBLE_TAP_SOUTH:
		LOG_DBG("DOUBLE_TAP_SOUTH\n");
		break;

	case DOUBLE_TAP_WEST:
		LOG_DBG("DOUBLE_TAP_WEST\n");
		break;

	case DOUBLE_TAP_NORTH:
		LOG_DBG("DOUBLE_TAP_NORTH\n");
		break;

	case DOUBLE_TAP_EAST:
		LOG_DBG("DOUBLE_TAP_EAST\n");
		break;

	case DOUBLE_TAP_CENTER:
		LOG_DBG("DOUBLE_TAP_CENTER\n");
		break;

	default :
		LOG_DBG("UNKNOWN(0x%04x)\n", event);
		break;
	}

}

void send_gest_info(sensor_data_t *pData)
{
	BaseType_t ret;
	uint8_t idx = 0;
	uint8_t gestData[GEST_UART_DATA_LEN] = {0,};

	gestData[idx++] = 0xFE;
	gestData[idx++] = 0xFD;
	gestData[idx++] = (uint8_t)(pData->position.x & 0xff);
	gestData[idx++] = (uint8_t)(pData->position.x >> 8);

	gestData[idx++] = (uint8_t)(pData->position.y & 0xff);
	gestData[idx++] = (uint8_t)(pData->position.y >> 8);

	gestData[idx++] = (uint8_t)(pData->position.z & 0xff);
	gestData[idx++] = (uint8_t)(pData->position.z >> 8);

	switch (get_touch_event(&(pData->touchInfo))) {
	case TAP_NORTH:
		gestData[idx++] = 'N';
		break;

	case TAP_SOUTH:
		gestData[idx++] = 'S';
		break;

	case TAP_WEST:
		gestData[idx++] = 'W';
		break;

	case TAP_EAST:
		gestData[idx++] = 'E';
		break;

	case TAP_CENTER:
		gestData[idx++] = 'C';
		break;
	default:
		gestData[idx++] = ' ';
		break;
	}
	switch (pData->gestureInfo.gesture) {
	case GESTURE_FLICK_W_TO_E:
		gestData[idx++] = 'R';
		break;
	case GESTURE_FLICK_E_TO_W:
		gestData[idx++] = 'L';
		break;
	case GESTURE_FLICK_S_TO_N:
		gestData[idx++] = 'U';
		break;
	case GESTURE_FLICK_N_TO_S:
		gestData[idx++] = 'D';
		break;
	case GESTURE_CIRCLE_C:
		gestData[idx++] = 'C';
		break;
	case GESTURE_CIRCLE_CC:
		gestData[idx++] = 'c';
		break;
	case GESTURE_WAVE_X:
		gestData[idx++] = 'X';
		break;
	case GESTURE_WAVE_Y:
		gestData[idx++] = 'Y';
		break;
	case GESTURE_HOLD:
		gestData[idx++] = 'H';
		break;
	case GESTURE_EDGE_FLICK_W_TO_E:
		gestData[idx++] = '1';
		break;
	case GESTURE_EDGE_FLICK_E_TO_W:
		gestData[idx++] = '2';
		break;
	case GESTURE_EDGE_FLICK_S_TO_N:
		gestData[idx++] = '3';
		break;
	case GESTURE_EDGE_FLICK_N_TO_S:
		gestData[idx++] = '4';
		break;
	case GESTURE_DOUBLE_FLICK_W_TO_E:
		gestData[idx++] = 'r';
		break;
	case GESTURE_DOUBLE_FLICK_E_TO_W:
		gestData[idx++] = 'l';
		break;
	case GESTURE_DOUBLE_FLICK_S_TO_N:
		gestData[idx++] = 'u';
		break;
	case GESTURE_DOUBLE_FLICK_N_TO_S:
		gestData[idx++] = 'd';
		break;
	case GESTURE_PRESENCE:
		gestData[idx++] = 'P';
		break;
	default:
		gestData[idx++] = ' ';
		break;
	}
	memcpy(gestData + idx, pData->pCicData, 20);
	idx+=20;
	memcpy(gestData + idx, pData->pSdData, 20);
	idx+=20;
	gestData[idx++] = '\r';
	gestData[idx++] = '\n';

	ret = xQueueSend( gKeyCodeQueueHandle, (void *)gestData, 0);
}

void print_airwheel_info(airwheel_info_t *pAirwheelInfo) {
LOG_DBG("[Ariwheel Info]\n");
LOG_DBG("Position, Count : %d, %d\n", pAirwheelInfo->currentPosition,
		pAirwheelInfo->fullLotationCount);
}

void print_xyz_position(position_t *pPosition) {
LOG_DBG("[Position x,y,z]\n");
LOG_DBG("%u, %u, %u\n", pPosition->x, pPosition->y, pPosition->z);
}

char *str_gesture(uint8_t gesture)
{
	static char strGesture[32] = { 0, };

	switch (gesture) {
	case GESTURE_NONE:
		sprintf(strGesture, "NONE");
		break;
case GESTURE_GARBAGE:
	sprintf(strGesture, "GARBAGE");
	break;
case GESTURE_FLICK_W_TO_E:
	sprintf(strGesture, "RIGHT");
	break;
case GESTURE_FLICK_E_TO_W:
	sprintf(strGesture, "LEFT");
	break;
case GESTURE_FLICK_S_TO_N:
	sprintf(strGesture, "UP");
	break;
case GESTURE_FLICK_N_TO_S:
	sprintf(strGesture, "DOWN");
	break;
case GESTURE_CIRCLE_C:
	sprintf(strGesture, "CLOCK");
	break;
case GESTURE_CIRCLE_CC:
	sprintf(strGesture, "COUNTER CLOCK");
	break;
case GESTURE_WAVE_X:
	sprintf(strGesture, "WAVE X");
	break;
case GESTURE_WAVE_Y:
	sprintf(strGesture, "WAVE Y");
	break;
case GESTURE_HOLD:
	sprintf(strGesture, "HOLD");
	break;
case GESTURE_EDGE_FLICK_W_TO_E:
	sprintf(strGesture, "EDGE RIGHT");
	break;
case GESTURE_EDGE_FLICK_E_TO_W:
	sprintf(strGesture, "EDGE LEFT");
	break;
case GESTURE_EDGE_FLICK_S_TO_N:
	sprintf(strGesture, "EDGE UP");
	break;
case GESTURE_EDGE_FLICK_N_TO_S:
	sprintf(strGesture, "EDGE DOWN");
	break;
case GESTURE_DOUBLE_FLICK_W_TO_E:
	sprintf(strGesture, "DOUBLE RIGHT");
	break;
case GESTURE_DOUBLE_FLICK_E_TO_W:
	sprintf(strGesture, "DOUBLE LEFT");
	break;
case GESTURE_DOUBLE_FLICK_S_TO_N:
	sprintf(strGesture, "DOUBLE UP");
	break;
case GESTURE_DOUBLE_FLICK_N_TO_S:
	sprintf(strGesture, "DOUBLE DOWN");
	break;
case GESTURE_PRESENCE:
	sprintf(strGesture, "PRESENCE");
	break;
default:
	sprintf(strGesture, "UNKNOWN(0x%02x)", gesture);
	break;
}
return strGesture;
}

int read_gest_data(uint8_t *buffer, int len)
{
	I2C_TransferReturn_TypeDef	ret;
	/* Wait Ts Assserted */
	while(!gest_is_ts_asserted());
	ret = i2c_read_gest_ic(buffer, len);
	if(ret != i2cTransferDone) {
		LOG_ERR("i2c_read_gest_ic fail, ret = 0x%02x\n", ret);
		return ret;
	}
	return (parse_gest_data(buffer));
}

int wait_system_status(uint8_t *buffer, int len)
{
	I2C_TransferReturn_TypeDef	ret;
	gest_header_t* pHeader = (gest_header_t*)buffer;
	gest_system_status_t *pPayload = (gest_system_status_t*)(buffer + sizeof(gest_header_t));
	uint16_t errorcode;
	while (1) {
		while (!gest_is_ts_asserted());
		ret = i2c_read_gest_ic(buffer, len);
		if (ret != i2cTransferDone) {
			LOG_ERR("i2c_read_gest_ic fail, ret = 0x%02x\n", ret);
			return ret;
		}
		if (pHeader->id == GEST_SYSTEM_STATUS) {
			break;
		}
	}
	hadle_system_status(buffer);
	errorcode = (pPayload->errorCode[1] << 8) | pPayload->errorCode[0];
	return errorcode;
}

void send_rp_with_response(uint16_t id, uint32_t arg0, uint32_t arg1, uint8_t *buffer, int buflen)
{
	while(1) {
		send_set_runtime_parameter(id, arg0, arg1);
		LOG_DBG("SET RP : 0x%04x\n", id);
		vTaskDelay(MGC_SETUP_TIME_MS);
		if (wait_system_status(buffer, buflen) == 0) {
			break;
		}
		vTaskDelay(1000);
	}
}

void add_gesture(uint8_t gesture)
{
	gest_buf.gesture[gest_buf.cnt] = gesture;
	gest_buf.cnt++;

}

void conv_u16_str(uint16_t val, char *str)
{
	uint8_t idx = 0;
	uint16_t dividend;

	dividend = val;
	str[idx++] = (char)(dividend/10000) + '0';
	dividend = (uint16_t)(dividend % 10000);

	str[idx++] = (char)(dividend / 1000) + '0';
	dividend = (uint16_t)(dividend % 1000);

	str[idx++] = (char)(dividend / 100) + '0';
	dividend = (uint16_t)(dividend % 100);

	str[idx++] = (char)(dividend / 10) + '0';
	dividend = (uint16_t)(dividend % 10);

	str[idx++] = (char)(dividend) + '0';
}
