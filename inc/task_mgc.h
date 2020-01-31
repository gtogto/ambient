/*
 * task_mgc.h
 *
 *  Created on: 2017. 11. 2.
 *      Author: netbugger
 */

#ifndef INC_TASK_MGC_H_
#define INC_TASK_MGC_H_

#include "FreeRTOS.h"
#include "timers.h"
#include "em_i2c.h"

#define MGC_SETUP_TIME_MS					100

#define GEST_SET_RUNTIME_PARAM_ID	0xA2
#define GEST_SET_RUNTIME_PARAM_LEN	0x10
#define GEST_IC_ADDR	0x84

#define GEST_CONTROL_BUFFER_SIZE			140
#define GEST_RECEIVED_BUFFER_SIZE			256
#define GEST_MSG_SIZE_REQUEST_MSG			0x0C
#define GEST_MSG_SIZE_SET_RUNTIME			0x10
#define GEST_MSG_SIZE_FW_VERSION_INFO		0x84
#define GEST_MSG_SIZE_FW_UPDATE_START		0x1C
#define GEST_MSG_SIZE_FW_UPDATE_COMPLETED	0x88

#define GEST_MSG_SIZE_SYSTEM_STATUS			0x10


#define DIR_RIGHT	2
#define DIR_LEFT	3
#define DIR_UP		4
#define DIR_DOWN	5
#define DIR_CLOCK	6
#define DIR_CCLOCK	7

/* DOUT ENABLE */
#define DOUT_DSP_STATUS_EN	1
#define DOUT_GESTURE_EN		1
#define DOUT_TOUCH_EN		1
#define DOUT_AIRWHEEL_EN 	0
#define DOUT_XYZ_EN			1

#define GEST_TS_MODE_INPUT	0
#define GEST_TS_MODE_OUTPUT	1

typedef struct {
	uint8_t msgSize;
	uint8_t flags;
	uint8_t seq;
	uint8_t id;
}__attribute__((packed))gest_header_t;

typedef struct {
	uint8_t messageId;
	uint8_t reserved[3];
	uint8_t param[4];
}__attribute__((packed)) gest_request_message_t;

typedef struct {
	uint8_t id[2] ;
	uint8_t reserved[2];
	uint8_t arg0[4];
	uint8_t arg1[4];
}__attribute__((packed)) gest_runtime_parameter_t;



typedef struct {
	uint8_t messageId;
	uint8_t maxCmdSize;
	uint8_t errorCode[2];
	uint8_t reserved[2];
}__attribute__((packed)) gest_system_status_t;

typedef struct {
	uint8_t fwValid;
	uint8_t hwRev[2];
	uint8_t parameterStartAddr;
	uint8_t libraryLoaderVersion[2];
	uint8_t libraryLoaderPlatform;
	uint8_t fwStartAddr;
	uint8_t fwVersion[120];
}__attribute__((packed)) gest_fw_version_info_t;

typedef struct {
	uint8_t dataOutputConfigMask[2];
	uint8_t timeStamp;
	uint8_t systemInfo;
	uint8_t dspStatus[2];
	uint8_t gestureInfo[4];
	uint8_t touchInfo[4];
#if DOUT_AIRWHEEL_EN
	uint8_t airWheelInfo[2];
#endif
	uint8_t xyzPosition[6];
	//uint8_t noisePower[4];
	uint8_t cicData[20];
	uint8_t sdData[20];
}__attribute__((packed)) gest_sensor_data_output_t;

typedef enum {
	GEST_REQUEST_MESSAGE		= 0x06,
	GEST_SYSTEM_STATUS			= 0x15,
	GEST_FW_VERSION_INFO		= 0x83,
	GEST_SENSOR_DATA_OUTPUT		= 0x91,
	GEST_SET_RUNTIME_PARAMETER	= 0xA2,
}gest_msgid_t;


typedef enum {
	GESTURE_NONE 				= 0,
	GESTURE_GARBAGE				= 1,
	GESTURE_FLICK_W_TO_E		= 2,
	GESTURE_FLICK_E_TO_W		= 3,
	GESTURE_FLICK_S_TO_N		= 4,
	GESTURE_FLICK_N_TO_S		= 5,
	GESTURE_CIRCLE_C			= 6,
	GESTURE_CIRCLE_CC			= 7,
	GESTURE_WAVE_X				= 8,
	GESTURE_WAVE_Y				= 9,
	GESTURE_HOLD				= 64,
	GESTURE_EDGE_FLICK_W_TO_E	= 65,
	GESTURE_EDGE_FLICK_E_TO_W	= 66,
	GESTURE_EDGE_FLICK_S_TO_N	= 67,
	GESTURE_EDGE_FLICK_N_TO_S	= 68,
	GESTURE_DOUBLE_FLICK_W_TO_E	= 69,
	GESTURE_DOUBLE_FLICK_E_TO_W	= 70,
	GESTURE_DOUBLE_FLICK_S_TO_N	= 71,
	GESTURE_DOUBLE_FLICK_N_TO_S	= 72,
	GESTURE_PRESENCE			= 73,
}gesture_t;
#define GESTURE_INFO_GESTURE_OFFSET	0
#define GESTURE_INFO_GESTURE_MASK	0x000000FF

#define GESTURE_INFO_GESTURE_CLASS_OFFSET	12
#define GESTURE_INFO_GESTURE_CLASS_MASK		0x0000F000

typedef enum {
	GESTURE_CLASS_GARBAGE	= 0,
	GESTURE_CLASS_FLICK		= 1,
	GESTURE_CLASS_CIRULAR	= 2,
} gesture_class_t;

#define GUESTURE_INFO_EDGE_FLICK_FLAG_OFFSET	16
#define GUESTURE_INFO_EDGE_FLICK_FLAG_MASK		0x00010000

#define GESTURE_INFO_HAND_PRESENCE_FLAG_OFFSET	27
#define GESTURE_INFO_HAND_PRESENCE_FLAG_MASK	0x08000000

#define GESTURE_INFO_HAND_HOLD_FLAG_OFFSET	28
#define GESTURE_INFO_HAND_HOLD_FLAG_MASK	0x10000000

#define GESTURE_INFO_HAND_INSIDE_FLAG_OFFSET	29
#define GESTURE_INFO_HAND_INSIDE_FLAG_MASK		0x20000000

#define GESTURE_INFO_RECOGNIZING_FLAG_OFFSET	31
#define GESTURE_INFO_RECOGNIZING_FLAG_MASK		0x80000000

/* RPID : RunTime Parameter ID */
#define GEST_RPID_TRIGGER	0x1000
/* ..... ARG0 ARG1 .......... */
#define GEST_RPID_MAKE_PERSISTENT	0xFF00
/* ..... ARG0 ARG1 .......... */
/* .......................... */

#define GEST_RPID_TOUCH_AND_APPROACH_DETECTION		0x0097
#define GEST_RPID_TOUCH_DETECTION_ARG0_ENABLE		0x08
#define GEST_RPID_TOUCH_DETECTION_ARG0_DISABL		0x00
#define GEST_RPID_TOUCH_DETECTION_ARG1				0x08
#define GEST_RPID_APPROACH_DETECTION_ARG0_ENABLE	0x01
#define GEST_RPID_APPROACH_DETECTION_ARG0_DISALBLE	0x00
#define GEST_RPID_APPROACH_DETECTION_ARG1			0x01

#define GEST_RPID_AIRWHEEL							0x0090
#define GEST_RPID_AIRWHEEL_ARG0_ENABLE				0x20
#define GEST_RPID_AIRWHEEL_ARG0_DISABLE				0x00
#define GEST_RPID_AIRWHEEL_ARG1						0x20

#define GEST_RPID_GESTURE							0x0085
#define GEST_RPID_GESTURE_ARG0_GARBAGE_MODEL		(0x00000001 << 0)
#define GEST_RPID_GESTURE_ARG0_FLICK_W_TO_E			(0x00000001 << 1)
#define GEST_RPID_GESTURE_ARG0_FLICK_E_TO_W			(0x00000001 << 2)
#define GEST_RPID_GESTURE_ARG0_FLICK_S_TO_N			(0x00000001 << 3)
#define GEST_RPID_GESTURE_ARG0_FLICK_N_TO_S			(0x00000001 << 4)
#define GEST_RPID_GESTURE_ARG0_CLOCK				(0x00000001 << 5)
#define GEST_RPID_GESTURE_ARG0_COUNTER_CLOCK		(0x00000001 << 6)
#define GEST_RPID_GESTURE_ARG0_WAVE_X				(0x00000001 << 7)
#define GEST_RPID_GESTURE_ARG0_WAVE_Y				(0x00000001 << 8)
#define GEST_RPID_GESTURE_ARG0_HOLD					(0x00000001 << 22)
#define GEST_RPID_GESTURE_ARG0_PRESENCE				(0x00000001 << 23)
#define GEST_RPID_GESTURE_ARG0_EDGE_W_TO_E			(0x00000001 << 24)
#define GEST_RPID_GESTURE_ARG0_EDGE_E_TO_W			(0x00000001 << 25)
#define GEST_RPID_GESTURE_ARG0_EDGE_S_TO_N			(0x00000001 << 26)
#define GEST_RPID_GESTURE_ARG0_EDGE_N_TO_S			(0x00000001 << 27)
#define GEST_RPID_GESTURE_ARG0_DOUBLE_W_TO_E		(0x00000001 << 28)
#define GEST_RPID_GESTURE_ARG0_DOUBLE_E_TO_W		(0x00000001 << 29)
#define GEST_RPID_GESTURE_ARG0_DOUBLE_S_TO_N		(0x00000001 << 30)
#define GEST_RPID_GESTURE_ARG0_DOUBLE_N_TO_S		(0x00000001 << 31)
#define GEST_RPID_GESTURE_ARG1						0xFFFFFFFF

#define GEST_RPID_DOUT_ENABLE_MASK					0x00A0
#define GEST_PRID_DOUT_EN_DSP_STATUS				(0x00000001 << 0)
#define GEST_PRID_DOUT_EN_GESTURE_DATA				(0x00000001 << 1)
#define GEST_PRID_DOUT_EN_TOUCHINFO					(0x00000001 << 2)
#define GEST_PRID_DOUT_EN_AIRWHEELINFO				(0x00000001 << 3)
#define GEST_PRID_DOUT_EN_XYZPOSITION				(0x00000001 << 4)
#define GEST_PRID_DOUT_EN_NOISE_POWER				(0x00000001 << 5)
#define GEST_PRID_DOUT_EN_CICDATA					(0x00000001 << 11)
#define GEST_PRID_DOUT_EN_SDDATA					(0x00000001 << 12)
#define GEST_PRID_DOUT_EN_NOISE_INDICATION			(0x00000001 << 16)
#define GEST_PRID_DOUT_EN_CLIPPING_INDICATION		(0x00000001 << 17)
#define GEST_PRID_DOUT_EN_DSP_RUNNING				(0x00000001 << 18)
#define GEST_PRID_DOUT_EN_AIRWHEEL_DECIMATION		(0x00000001 << 19)
#define GEST_PRID_DOUT_EN_TIMESTAMP_OVERFLOW		(0x00000001 << 20)
#define GEST_PRID_DOUT_EN_HAND_PRESENCE_FLAG		(0x00000001 << 27)
#define GEST_PRID_DOUT_EN_HAND_HOLD_FLAG			(0x00000001 << 28)
#define GEST_PRID_DOUT_EN_HAND_INSIDE_FLAG			(0x00000001 << 29)
#define GEST_PRID_DOUT_EN_INPROGRESS_FLAG			(0x00000001 << 31)
#define GEST_RPID_DOUT_ENABLE_MASK_ARG1				0xFFFFFFFF

#define GEST_RPID_DOUT_LOCK_MASK					0x00A1
#define GEST_PRID_DOUT_LOCK_DSP_STATUS				(0x00000001 << 0)
#define GEST_PRID_DOUT_LOCK_GESTURE_DATA			(0x00000001 << 1)
#define GEST_PRID_DOUT_LOCK_TOUCHINFO				(0x00000001 << 2)
#define GEST_PRID_DOUT_LOCK_AIRWHEELINFO			(0x00000001 << 3)
#define GEST_PRID_DOUT_LOCK_XYZPOSITION				(0x00000001 << 4)
#define GEST_PRID_DOUT_LOCK_NOISE_POWER				(0x00000001 << 5)
#define GEST_PRID_DOUT_LOCK_CICDATA					(0x00000001 << 11)
#define GEST_PRID_DOUT_LOCK_SDDATA					(0x00000001 << 12)
#define GEST_RPID_DOUT_LOCK_MASK_ARG1				0xFFFFFFFF




typedef struct {
	uint8_t gesture;
	uint8_t gestureClass;
	uint8_t edgeFlickFlag;
	uint8_t handPresenceFlag;
	uint8_t handHoldFlag;
	uint8_t handInsideFlag;
	uint8_t gestureRecognizingFlag;
}gesture_info_t;

typedef enum {
	TOUCH_SOUTH			= 0,
	TOUCH_WEST			= 1,
	TOUCH_NORTH			= 2,
	TOUCH_EAST			= 3,
	TOUCH_CENTER		= 4,
	TAP_SOUTH			= 5,
	TAP_WEST			= 6,
	TAP_NORTH			= 7,
	TAP_EAST			= 8,
	TAP_CENTER			= 9,
	DOUBLE_TAP_SOUTH	= 10,
	DOUBLE_TAP_WEST		= 11,
	DOUBLE_TAP_NORTH	= 12,
	DOUBLE_TAP_EAST		= 13,
	DOUBLE_TAP_CENTER	= 14,
	TOUCH_NONE			= 0xFF,
}touch_event_bit_offset_t;
#define TOUCH_INFO_TOUCH_EVENT_OFFSET	0
#define TOUCH_INFO_TOUCH_EVENT_MASK		0x0000FFFF
#define TOUCH_INFO_TOUCH_COUNTER_OFFSET	16
#define TOUCH_INFO_TOUCH_COUNTER_MASK	0x00FF0000

typedef struct {
	uint16_t event;
	uint8_t counter;
	uint8_t reserved;
}touch_info_t;

typedef struct {
	uint8_t currentPosition;
	uint8_t fullLotationCount;
}airwheel_info_t;
#define AIRWHEEL_INFO_CURRENT_POSITION_MASK		0x01F
#define AIRWHEEL_INFO_CURRENT_POSITION_OFFSET	0
#define AIRWHEEL_INFO_ROTATION_COUNT_MASK		0x0E0
#define AIRWHEEL_INFO_ROTATION_COUNT_OFFSET		5

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
}position_t;

typedef struct {
	gesture_info_t gestureInfo;
	touch_info_t touchInfo;
	airwheel_info_t airwheelInfo;
	position_t	position;
	uint8_t *pCicData;
	uint8_t *pSdData;
}sensor_data_t;


#define MGC_DEFAULT_DELAY		100
#define DEF_GESTURE_TIMER_PERIOD	(1000)
#define GEST_BUF_SIZE	(1000/MGC_DEFAULT_DELAY)
typedef struct {
	uint8_t  cnt;
	uint8_t gesture[GEST_BUF_SIZE];
}gesture_buffer_t;

void do_mgc_task(void *pvParameters);

void gest_release_reset();
void gest_hold_reset();
uint8_t gest_is_ts_asserted();

I2C_TransferReturn_TypeDef i2c_write_gest_ic(const uint8_t *data, int len);
I2C_TransferReturn_TypeDef i2c_read_gest_ic(uint8_t *data, int len);
int read_gest_data(uint8_t *buffer, int len);
int wait_system_status(uint8_t *buffer, int len);
void send_rp_with_response(uint16_t id, uint32_t arg0, uint32_t arg1, uint8_t *buffer, int buflen);

int send_request_message(uint8_t msgId, uint8_t param[]);
int send_set_runtime_parameter(uint16_t id, uint32_t arg0, uint32_t arg1);

void hadle_system_status(uint8_t *data);
void hadle_fw_version_info(uint8_t *data);
void handle_sensor_data_output(uint8_t *data);


int parse_gest_data(uint8_t *data);
void parse_gesture_info(gesture_info_t *pGestureInfo, uint8_t *data);
void parse_touch_info(touch_info_t *pTouchInfo, uint8_t *data);
void parse_airwheel_info(airwheel_info_t *pAirwheelInfo, uint8_t *data);
void parse_xyz_position_info(position_t *pPosition, uint8_t *data);

void print_gesture_info(gesture_info_t *pGestureInfo);

uint16_t get_touch_event(touch_info_t *pTouchInfo);
void print_touch_info(uint16_t event);
void print_airwheel_info(airwheel_info_t *pAirwheelInfo);
void print_xyz_position(position_t *pPosition);


char *str_gesture(gesture_t gesture);
uint32_t conv_array_to_32bit(uint8_t *src);

void add_gesture(uint8_t gesture);
void send_gest_info(sensor_data_t *pData);

#endif /* INC_TASK_MGC_H_ */
