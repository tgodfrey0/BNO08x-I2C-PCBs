/******************************************************************************
 * @file   defs.h
 * @author Toby Godfrey
 * @brief  Preprocessor definitions
 *****************************************************************************/

#pragma once

#include <stdint.h>
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#define BNO085_ADDR					0b1001010U

#define I2C_INST						i2c0
#define SDA_PIN							16
#define SCL_PIN							17
#define BAUD_RATE_HZ				400000 // Fast mode 400 kb/s
#define TIMEOUT_MS					10000

#define MAX_PAYLOAD_SIZE		512
#define MAX_ATTEMPTS				5
#define SAMPLE_DELAY_MS			500

typedef struct sensor_reports_s {
	uint8_t input_sensor_reports[MAX_PAYLOAD_SIZE];
	uint8_t wake_input_sensor_reports[MAX_PAYLOAD_SIZE];
  uint8_t gyro_rotation_vector_reports[MAX_PAYLOAD_SIZE];
} sensor_reports_t;
