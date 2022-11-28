#ifndef REMTCAM_INM_H
#define REMTCAM_INM_H

#include <stdint.h>

#include "std_tlv.h"

enum {
	REMTCAM_MSG_T_GET_VAR       = 0x01 + TTLV_MSG_T_APPLICATION,
	REMTCAM_MSG_T_GET_VAR_RES   = 0x02 + TTLV_MSG_T_APPLICATION,
	REMTCAM_MSG_T_SET_VAR       = 0x03 + TTLV_MSG_T_APPLICATION,
	REMTCAM_MSG_T_HEARTBEAT     = 0x04 + TTLV_MSG_T_APPLICATION,
	REMTCAM_MSG_T_SENSOR_EVENT  = 0x05 + TTLV_MSG_T_APPLICATION,
	REMTCAM_MSG_T_SENSOR_SAMPLE = 0x06 + TTLV_MSG_T_APPLICATION,
	REMTCAM_MSG_T_ALERT_EVENT   = 0x07 + TTLV_MSG_T_APPLICATION
};

enum {
	REMTCAM_RES_INDEX = 0x01 + TTLV_RES_APPLICATION,  // Invalid variable index.
	REMTCAM_RES_VALUE = 0x02 + TTLV_RES_APPLICATION   // Invalid variable value.
};

enum {
	REMTCAM_REG_CTRL_FLAGS = 0x01 + TTLV_REG_APPLICATION
};

typedef uint16_t remtcam_var_id;

typedef uint8_t remtcam_var_index;

enum {
	REMTCAM_VAR_DEBUG0        = 0x0100,
	REMTCAM_VAR_DEBUG1        = 0x0101,
	REMTCAM_VAR_DEBUG2        = 0x0102,
	REMTCAM_VAR_DEBUG3        = 0x0103,
	REMTCAM_VAR_DEBUG4        = 0x0104,
	REMTCAM_VAR_DEBUG5        = 0x0105,
	REMTCAM_VAR_DEBUG6        = 0x0106,
	REMTCAM_VAR_DEBUG7        = 0x0107,
	REMTCAM_VAR_DEBUG8        = 0x0108,
	REMTCAM_VAR_DEBUG9        = 0x0109,
	// NOTE: By combining this prefix with a sensor type ID in the low byte
	//       of the variable ID, and placing the sensor ID in the variable
	//       index field, the raw value (i.e. not scaled to 8 bits) of any
	//       sensor can be accessed as a node variable.
	REMTCAM_VAR_SENSOR_PREFIX = 0x0200
};

typedef struct __attribute__ ((__packed__)) remtcam_var_header {
	remtcam_var_id id;
	remtcam_var_index index;
} remtcam_var_header;

typedef struct __attribute__ ((__packed__)) remtcam_var_res_header {
	remtcam_var_id id;
	remtcam_var_index index;
	uint16_t request_id;
} remtcam_var_res_header;

typedef uint8_t remtcam_sensor_t;

typedef uint8_t remtcam_sensor_id;

typedef uint8_t remtcam_sensor_event_t;

typedef uint8_t remtcam_sensor_value;

enum {
	REMTCAM_SENSOR_T_ANY         = 0x00,
	REMTCAM_SENSOR_T_LIGHT       = 0x01,
	REMTCAM_SENSOR_T_SOUND       = 0x02,
	REMTCAM_SENSOR_T_MOTION      = 0x03,
	REMTCAM_SENSOR_T_PRESSURE    = 0x04,
	REMTCAM_SENSOR_T_PROXIMITY   = 0x05,
	REMTCAM_SENSOR_T_TEMPERATURE = 0x06
};

enum {
	REMTCAM_SENSOR_VAR_T_ANY         = REMTCAM_SENSOR_T_ANY | REMTCAM_VAR_SENSOR_PREFIX,
	REMTCAM_SENSOR_VAR_T_LIGHT       = REMTCAM_SENSOR_T_LIGHT | REMTCAM_VAR_SENSOR_PREFIX,
	REMTCAM_SENSOR_VAR_T_SOUND       = REMTCAM_SENSOR_T_SOUND | REMTCAM_VAR_SENSOR_PREFIX,
	REMTCAM_SENSOR_VAR_T_MOTION      = REMTCAM_SENSOR_T_MOTION | REMTCAM_VAR_SENSOR_PREFIX,
	REMTCAM_SENSOR_VAR_T_PRESSURE    = REMTCAM_SENSOR_T_PRESSURE | REMTCAM_VAR_SENSOR_PREFIX,
	REMTCAM_SENSOR_VAR_T_PROXIMITY   = REMTCAM_SENSOR_T_PROXIMITY | REMTCAM_VAR_SENSOR_PREFIX,
	REMTCAM_SENSOR_VAR_T_TEMPERATURE = REMTCAM_SENSOR_T_TEMPERATURE | REMTCAM_VAR_SENSOR_PREFIX
};

enum {
	REMTCAM_SENSOR_EVENT_T_ANY     = 0x00,
	REMTCAM_SENSOR_EVENT_T_HIGH    = 0x01,  // Stimuli (e.g. light) over high trigger level.
	REMTCAM_SENSOR_EVENT_T_LOW     = 0x02,  // Stimuli (e.g. light) under low trigger level.
	REMTCAM_SENSOR_EVENT_T_RISE    = 0x03,  // Stimuli rising significantly.
	REMTCAM_SENSOR_EVENT_T_FALL    = 0x04,  // Stimuli falling significantly.
	REMTCAM_SENSOR_EVENT_T_SILENT  = 0x05,  // Sensor sample not received within expected time.
	REMTCAM_SENSOR_EVENT_T_CONTACT = 0x06   // Sensor sample received after silence.
};

typedef struct __attribute__ ((__packed__)) remtcam_sensor_event {
	remtcam_sensor_t sensor_type;
	remtcam_sensor_id sensor_id;
	remtcam_sensor_event_t event_type;
} remtcam_sensor_event;

typedef struct __attribute__ ((__packed__)) remtcam_sensor_sample {
	remtcam_sensor_t sensor_type;
	remtcam_sensor_id sensor_id;
	remtcam_sensor_value value;
} remtcam_sensor_sample;

enum {
	REMTCAM_MSG_L_GET_VAR       = sizeof(remtcam_var_header),
	// NOTE: The actual message length is this plus the size of the variable value.
	REMTCAM_MSG_L_GET_VAR_RES   = sizeof(remtcam_var_res_header),
	// NOTE: The actual message length is this plus the size of the variable value.
	REMTCAM_MSG_L_SET_VAR       = sizeof(remtcam_var_header),
	REMTCAM_MSG_L_SENSOR_EVENT  = sizeof(remtcam_sensor_event),
	REMTCAM_MSG_L_SENSOR_SAMPLE = sizeof(remtcam_sensor_sample)
};

#define REMTCAM_CHECK_GET_VAR TTLV_CHECK_TL(REMTCAM_MSG_T_GET_VAR, REMTCAM_MSG_L_GET_VAR)
#define REMTCAM_CHECK_GET_VAR_RES TTLV_CHECK_TL(REMTCAM_MSG_T_GET_VAR_RES, REMTCAM_MSG_L_GET_VAR_RES)
#define REMTCAM_CHECK_SET_VAR TTLV_CHECK_TL(REMTCAM_MSG_T_SET_VAR, REMTCAM_MSG_L_SET_VAR)
#define REMTCAM_CHECK_SENSOR_EVENT TTLV_CHECK_TL(REMTCAM_MSG_T_SENSOR_EVENT, REMTCAM_MSG_L_SENSOR_EVENT)
#define REMTCAM_CHECK_SENSOR_SAMPLE TTLV_CHECK_TL(REMTCAM_MSG_T_SENSOR_SAMPLE, REMTCAM_MSG_L_SENSOR_SAMPLE)

#endif
