/***********************************************************************************
 * @file   main.c
 * @author Toby Godfrey
 * @brief  I2C communication implementation for a Pico and Adafruit BNO085 board
 **********************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "boards/pico.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "defs.h"
#include "pico/time.h"
#include "pico/types.h"

uint8_t payload[MAX_PAYLOAD_SIZE] = {0};
uint16_t inner_cargo_size = 0;

/*************************
 * Byte | Value
 *   0  | LSB Length
 *   1  | MSB Length
 *   2  | Channel
 *   3  | Sequence Number
 *   4+ | Data
 ************************/

uint8_t get_seq_num(){
  static uint8_t seq_num;
  uint8_t sn = seq_num;
  seq_num++;
  return sn;
}

/**
 * Endless loop with flashing the on-board LED
 *
 * @param[in] t_on     The time the LED is on for
 * @param[in] t_off    The time the LED is off for
 */
void flash_led_inf(uint64_t t_on, uint64_t t_off){
  for(;;){
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(t_on);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(t_off);
  }
}


/**
 * Bounded loop with flashing the on-board LED
 *
 * @param[in] t_on     The time the LED is on for
 * @param[in] t_off    The time the LED is off for
 * @param[in] n        The number of iterations
 */
void flash_led_n(uint64_t t_on, uint64_t t_off, uint16_t n){
  for(uint16_t i = 0; i < n; i++){
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(t_on);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(t_off);
  }
}

/**
 * Create the absolute time struct for the specified timeout
 */
absolute_time_t calc_timeout(){
  absolute_time_t abs_time_limit = {(time_us_64() + (TIMEOUT_MS * 1000))};
  return abs_time_limit;
}

unsigned int write_sensor(uint8_t* msg, uint16_t len){
  return i2c_write_blocking_until(I2C_INST, BNO085_ADDR, msg, len, false, calc_timeout());
}

/**
 * Initialise the I2C functionality and LED
 */
void init_pins(){

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_pull_down(PICO_DEFAULT_LED_PIN);

  printf("Status LED successfully configured\n");

  flash_led_n(500, 500, 3);

#if !defined(I2C_INST) || !defined(SDA_PIN) || !defined(SCL_PIN)
  #warning No I2C pins are set
  puts("I2C pins were not defined")
#else

  i2c_init(I2C_INST, BAUD_RATE_HZ);

  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

  gpio_pull_up(SCL_PIN);
  gpio_pull_up(SDA_PIN);

  printf("I2C interface successfully configured\n");

#endif
}

/**
 * Open the I2C channel
 */
void open_channel(){
  /*
   *  Byte  | Value
   *   0,1  | Length = 5
   *   2    | Channel = 1 (Executable)
   *   3    | Sequence Number = 0
   *   4    | Payload = CMD 2 (On)
   */
  uint8_t pkt[] = {5, 0, 1, get_seq_num(), 2};

  bool succ = false;
  for(uint8_t attempts; attempts < MAX_ATTEMPTS; attempts++){
    if(write_sensor(pkt, sizeof(pkt)) != PICO_ERROR_GENERIC){
      succ = true;
      break;
    }
    sleep_ms(100);
  }

  if(!succ){
    printf("Failed to open channel to sensor\n");
    flash_led_inf(1000, 1000);
  }

  sleep_ms(300);
}

/**
 * Format the received payload into the accelerometer struct
 *
 * @param[in] sensor_reports    The full sensor struct
 */
void format_accelerometer_data(struct full_sensor_reports* sensor_reports){
  printf("Accelerometer data:\n");
  printf("Status: %d\n", sensor_reports->accelerometer->input_report->accelerometer_input_report->status);
  printf("Delay: %d\n", sensor_reports->accelerometer->input_report->accelerometer_input_report->delay);
  printf("(x, y, z) m/s^2: (%f, %f, %f) m/s^2\n", sensor_reports->accelerometer->input_report->accelerometer_input_report->x, sensor_reports->accelerometer->input_report->accelerometer_input_report->y, sensor_reports->accelerometer->input_report->accelerometer_input_report->z);
}

/**
 * Format the received payload into the gyroscope struct
 *
 * @param[in] sensor_reports    The full sensor struct
 */
void format_gyroscope_data(struct full_sensor_reports* sensor_reports){
  printf("Gyroscope data:\n");
  printf("Status: %d\n", sensor_reports->gyroscope->input_report->gyroscope_input_report->status);
  printf("Delay: %d\n", sensor_reports->gyroscope->input_report->gyroscope_input_report->delay);
  printf("(x, y, z) rad/s: (%f, %f, %f) rad/s\n", sensor_reports->gyroscope->input_report->gyroscope_input_report->x, sensor_reports->gyroscope->input_report->gyroscope_input_report->y, sensor_reports->gyroscope->input_report->gyroscope_input_report->z);
}

/**
 * Format the received payload into the magnetic field struct
 *
 * @param[in] sensor_reports    The full sensor struct
 */
void format_magnetic_field_data(struct full_sensor_reports* sensor_reports){
  printf("Magnetic field sensor data:\n");
  printf("Status: %d\n", sensor_reports->magnetic_field->input_report->magnetic_field_input_report->status);
  printf("Delay: %d\n", sensor_reports->magnetic_field->input_report->magnetic_field_input_report->delay);
  printf("(x, y, z) uT: (%f, %f, %f) uT\n", sensor_reports->magnetic_field->input_report->magnetic_field_input_report->x, sensor_reports->magnetic_field->input_report->magnetic_field_input_report->y, sensor_reports->magnetic_field->input_report->magnetic_field_input_report->z);
}

/**
 * Print the reports received
 *
 * @param[in] sensor_reports    The full sensor struct
 */
void output_reports(struct full_sensor_reports* sensor_reports){
  static uint16_t report_count = 0;

  if(sensor_reports->accelerometer->enabled) format_accelerometer_data(sensor_reports);
  if(sensor_reports->magnetic_field->enabled) format_magnetic_field_data(sensor_reports);
  if(sensor_reports->gyroscope->enabled) format_gyroscope_data(sensor_reports);

  if(sensor_reports->accelerometer->enabled || sensor_reports->magnetic_field->enabled || sensor_reports->gyroscope->enabled){
    printf("\n");
    report_count++;
  }
}

/**
 * Parse the received payload into the accelerometer struct
 *
 * @param[out] sensor_report    The full sensor struct
 * @param[in] cargo_ptr A pointer to the start of the cargo in the payload buffer
 */
void parse_accelerometer_data(struct full_sensor_reports* sensor_reports, uint8_t* cargo_ptr){
  if(inner_cargo_size != 10){
    printf("Invalid report from accelerometer\n");
    return;
  }

  int16_t x = (*(cargo_ptr + 4) | (*(cargo_ptr + 5) << 8)) * scale_q(8);
  int16_t y = (*(cargo_ptr + 6) | (*(cargo_ptr + 7) << 8)) * scale_q(8);
  int16_t z = (*(cargo_ptr + 8) | (*(cargo_ptr + 9) << 8)) * scale_q(8);

  sensor_reports->accelerometer->input_report->accelerometer_input_report->status = *(cargo_ptr + 2);
  sensor_reports->accelerometer->input_report->accelerometer_input_report->delay = *(cargo_ptr + 3);
  sensor_reports->accelerometer->input_report->accelerometer_input_report->x = x;
  sensor_reports->accelerometer->input_report->accelerometer_input_report->y = y;
  sensor_reports->accelerometer->input_report->accelerometer_input_report->z = z;
}

/**
 * Parse the received payload into the gyroscope struct
 *
 * @param[out] sensor_report    The full sensor struct
 * @param[in] cargo_ptr A pointer to the start of the cargo in the payload buffer
 */
void parse_gyroscope_data(struct full_sensor_reports* sensor_reports, uint8_t* cargo_ptr){
  if(inner_cargo_size != 10){
    printf("Invalid report from gyroscope\n");
    return;
  }

  int16_t x = (*(cargo_ptr + 4) | (*(cargo_ptr + 5) << 8)) * scale_q(9);
  int16_t y = (*(cargo_ptr + 6) | (*(cargo_ptr + 7) << 8)) * scale_q(9);
  int16_t z = (*(cargo_ptr + 8) | (*(cargo_ptr + 9) << 8)) * scale_q(9);

  sensor_reports->gyroscope->input_report->gyroscope_input_report->status = *(cargo_ptr + 2);
  sensor_reports->gyroscope->input_report->gyroscope_input_report->delay = *(cargo_ptr + 3);
  sensor_reports->gyroscope->input_report->gyroscope_input_report->x = x;
  sensor_reports->gyroscope->input_report->gyroscope_input_report->y = y;
  sensor_reports->gyroscope->input_report->gyroscope_input_report->z = z;
}

/**
 * Parse the received payload into the magnetic field struct
 *
 * @param[out] sensor_report    The full sensor struct
 * @param[in] cargo_ptr         A pointer to the start of the cargo in the payload buffer
 */
void parse_magnetic_field_data(struct full_sensor_reports* sensor_reports, uint8_t* cargo_ptr){
  if(inner_cargo_size != 10){
    printf("Invalid report from magnetic field sensor\n");
    return;
  }

  int16_t x = (*(cargo_ptr + 4) | (*(cargo_ptr + 5) << 8)) * scale_q(4);
  int16_t y = (*(cargo_ptr + 6) | (*(cargo_ptr + 7) << 8)) * scale_q(4);
  int16_t z = (*(cargo_ptr + 8) | (*(cargo_ptr + 9) << 8)) * scale_q(4);

  sensor_reports->magnetic_field->input_report->magnetic_field_input_report->status = *(cargo_ptr + 2);
  sensor_reports->magnetic_field->input_report->magnetic_field_input_report->delay = *(cargo_ptr + 3);
  sensor_reports->magnetic_field->input_report->magnetic_field_input_report->x = x;
  sensor_reports->magnetic_field->input_report->magnetic_field_input_report->y = y;
  sensor_reports->magnetic_field->input_report->magnetic_field_input_report->z = z;
}

/**
 * Convert the payload buffer into useful data
 *
 * @param[out] sensor_reports   The full sensor struct
 */
void parse_payload(struct full_sensor_reports* sensor_reports){
  uint8_t* cargo_ptr = &(payload[9]); // First 9 bytes are the header and timebase references
  switch (payload[9]) {     
    case ACCELEROMETER_ID:
      parse_accelerometer_data(sensor_reports, cargo_ptr);
      break;
    case GYROSCOPE_CALIBRATED_ID:
      parse_gyroscope_data(sensor_reports, cargo_ptr);
      break;
    case MAGNET_FIELD_CALIBRATED_ID:
      parse_magnetic_field_data(sensor_reports, cargo_ptr);
      break;
  }
}

/**
 * Read the header and then the payload data for a given channel
 *
 * @param[out] sensor_reports   The full sensor struct
 * @param[out] buf              A struct pointer to store the output
 */
void read_sensor(struct full_sensor_reports* sensor_reports, struct single_sensor_reports* buf){
  buf->size = 0;

  uint8_t header[4];
  uint8_t* payload_ptr = payload;

  uint8_t res = i2c_read_blocking_until(I2C_INST, BNO085_ADDR, header, 4, false, calc_timeout());

  if(res == PICO_ERROR_GENERIC){
    printf("Failed to read header\n");
    flash_led_inf(2000, 2000);
  } else if(res == PICO_ERROR_TIMEOUT){
    printf("Timeout reached whilst reading header\n");
    flash_led_inf(5000, 5000);
  }

  uint16_t payload_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Remove continuation bit
  payload_size &= ~0x8000;

  if(payload_size > MAX_PAYLOAD_SIZE){
    printf("Payload too large for buffer\n");
    flash_led_inf(3000, 3000);
  }

  uint16_t bytes_remaining = payload_size;
  uint16_t read_size;
  uint8_t i2c_buf[MAX_PAYLOAD_SIZE];
  uint16_t payload_read_amount = 0;
  bool first_read = true;

  while(bytes_remaining > 0){
    if(first_read) read_size = min(MAX_PAYLOAD_SIZE, (size_t)bytes_remaining);
    else read_size = min(MAX_PAYLOAD_SIZE, (size_t)(bytes_remaining + 4));

    res = i2c_read_blocking(I2C_INST, BNO085_ADDR, i2c_buf, read_size, false);

    if(first_read){
      payload_read_amount = read_size;
      memcpy(payload_ptr, i2c_buf, payload_read_amount);
      first_read = false;
    } else {
      payload_read_amount = read_size - 4;
      memcpy(payload_ptr, i2c_buf, payload_read_amount);
    }

    payload_ptr += payload_read_amount;
    bytes_remaining -= payload_read_amount;
  }

  if(payload_size == 65535) { // 65535 indicates an error
    printf("Error returned from sensor\n");
    flash_led_inf(4000, 4000);
  }


  buf->size = payload_size;

  if(payload_size > 0) {
    inner_cargo_size = payload_size - HEADER_TIMEBASE_OFFSET;
    parse_payload(sensor_reports);
  }

  memset(payload, 0, payload_size);
}

/**
 * Read all three sensor channels
 *
 * @param[in] sensor_reports    The struct with all sensor data
 */
void read_all_sensors(struct full_sensor_reports* sensor_reports){
  read_sensor(sensor_reports, sensor_reports->accelerometer);
  read_sensor(sensor_reports, sensor_reports->magnetic_field);
  read_sensor(sensor_reports, sensor_reports->gyroscope);
}

/**
 * Polls the sensor over I2C and process any output
 *
 * @param[in] sensor_reports    The struct with all sensor data
 */
void poll_sensor(struct full_sensor_reports* sensor_reports){
  read_all_sensors(sensor_reports);

  output_reports(sensor_reports);

  sleep_ms(SAMPLE_DELAY_MS);
}

/**
 * Send a Set Feature packet to enable the given sensor
 *
 * @param sensor    The struct for the sensor to enable
 */
void enable_sensor(struct single_sensor_reports* sensor){
  uint8_t period[4];

  uint64_t period_us = SAMPLE_RATE_MS * 1000;

  period[0] = period_us & 0xFF;
  period[1] = (period_us >> 8) & 0xFF;
  period[2] = (period_us >> 16) & 0xFF;
  period[3] = (period_us >> 24) & 0xFF;

  uint8_t pkt[] = {
    0x15, // Length LSB
    0x00, // Length MSB
    0x02, // Channel
    get_seq_num(),
    0xFD, // Report ID
    sensor->sensor_id,
    0, // Feature flags
    0, // Change sensitivity LSB
    0, // Change sensitivity MSB
    period[0], // Report interval LSB
    period[1],
    period[2],
    period[3], // Report interval MSB
    0, // Batch interval LSB
    0,
    0,
    0, // Batch interval MSB
    0, // Sensor-specific configuration word LSB
    0,
    0,
    0  // Sensor-specific configuration word MSB
  };

  uint8_t res = write_sensor(pkt, sizeof(pkt));

  if(res == PICO_ERROR_GENERIC){
    printf("Failed to enable feature %x\n", sensor->sensor_id);
  } else if(res == PICO_ERROR_TIMEOUT){
    printf("Timeout reached when enabling feature %x\n", sensor->sensor_id);
  } else {
    printf("Successfully enabled feature 0x%x with a reporting period of %d ms\n", sensor->sensor_id, SAMPLE_RATE_MS);
    sensor->enabled = true;
  }
}

/**
 * Configures which sensors should run
 *
 * @param[in] sensor_reports    Full sensor struct
 */
void configure_sensors(struct full_sensor_reports* sensor_reports){
  printf("Enabling sensors\n");

  enable_sensor(sensor_reports->accelerometer);
  enable_sensor(sensor_reports->magnetic_field);
  enable_sensor(sensor_reports->gyroscope); 

  printf("Sensors enabled\n");
}

int main()
{
  stdio_init_all();

  struct accelerometer_input_report* acc_data = &(struct accelerometer_input_report){
    .status = 0,
    .delay = 0,
    .x = 0,
    .y = 0,
    .z = 0
  };

  struct magnetic_field_input_report* mag_data = &(struct magnetic_field_input_report){
    .status = 0,
    .delay = 0,
    .x = 0,
    .y = 0,
    .z = 0
  };

  struct gyroscope_calibrated_input_report* gyro_data = &(struct gyroscope_calibrated_input_report){
    .status = 0,
    .delay = 0,
    .x = 0,
    .y = 0,
    .z = 0
  };

  union input_report* acc_rep = &(union input_report){
    .accelerometer_input_report = acc_data
  };

  union input_report* mag_rep = &(union input_report){
    .magnetic_field_input_report = mag_data
  };

  union input_report* gyro_rep = &(union input_report){
    .gyroscope_input_report = gyro_data
  };

  struct single_sensor_reports* accelerometer = &(struct single_sensor_reports) {
    .chan = 3,
    .sensor_id = ACCELEROMETER_ID,
    .size = 0,
    .input_report = acc_rep,
    .enabled = false
  };

  struct single_sensor_reports* gyroscope_calibrated = &(struct single_sensor_reports) {
    .chan = 3,
    .sensor_id = GYROSCOPE_CALIBRATED_ID,
    .size = 0,
    .input_report = gyro_rep,
    .enabled = false
  };

  struct single_sensor_reports* magnetic_field = &(struct single_sensor_reports) {
    .chan = 3,
    .sensor_id = MAGNET_FIELD_CALIBRATED_ID,
    .size = 0,
    .input_report = mag_rep,
    .enabled = false
  };

  struct full_sensor_reports* sensor_reports = &(struct full_sensor_reports) {
    .accelerometer = accelerometer,
    .gyroscope = gyroscope_calibrated,
    .magnetic_field = magnetic_field
  };

  init_pins();

  open_channel();

  configure_sensors(sensor_reports);

  flash_led_n(100, 100, 5);

  for (;;) poll_sensor(sensor_reports);

  return 0;
}
