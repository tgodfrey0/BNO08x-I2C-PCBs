/******************************************************************************
 * @file   defs.h
 * @author Toby Godfrey
 * @brief  Preprocessor definitions
 *****************************************************************************/

#pragma once

#include <stdbool.h>
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
#define SAMPLE_DELAY_MS	    200 // Delay between reading the sensors
#define SAMPLE_RATE_MS	    60 // The rate at which sensors push data to the bus


// These are not necessarily all the same so the union allows forward compatibility
struct accelerometer_input_report {
  uint8_t status;
  uint8_t delay;
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

struct magnetic_field_input_report {
  uint8_t status;
  uint8_t delay;
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

struct gyroscope_calibrated_input_report {
  uint8_t status;
  uint8_t delay;
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

union input_report {
  struct accelerometer_input_report* accelerometer_input_report;
  struct magnetic_field_input_report* magnetic_field_input_report;
  struct gyroscope_calibrated_input_report* gyroscope_input_report;
};

struct single_sensor_reports {
  const uint8_t chan;
  const uint8_t sensor_id;
  union input_report* input_report;
  uint16_t size;
  bool enabled;
};

struct full_sensor_reports {
  struct single_sensor_reports *accelerometer;
  struct single_sensor_reports *gyroscope;
  struct single_sensor_reports *magnetic_field;
};

