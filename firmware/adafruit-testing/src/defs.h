/******************************************************************************
 * @file   defs.h
 * @author Toby Godfrey
 * @brief  Preprocessor definitions
 *****************************************************************************/

#pragma once

#include <stdint.h>
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#define BNO085_ADDR	    0b1001010U

#define I2C_INST	    i2c0
#define SDA_PIN		    16
#define SCL_PIN		    17
#define BAUD_RATE_HZ	    400000 // Fast mode 400 kb/s
#define TIMEOUT_MS	    10000

#define MAX_PAYLOAD_SIZE    512
#define MAX_ATTEMPTS	    5 // Max tries to open an I2C connection to the sensor
#define SAMPLE_DELAY_MS	    100 // Delay between reading the sensors

struct single_sensor_reports {
  uint8_t reports[MAX_PAYLOAD_SIZE];
  uint16_t size;
};

struct full_sensor_reports {
  struct single_sensor_reports *input_sensor_reports;
  struct single_sensor_reports *wake_input_sensor_reports;
  struct single_sensor_reports *gyro_rotation_vector_reports;
};

