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

struct single_sensor_reports* accelerometer = &(struct single_sensor_reports) {
  .chan = 3,
  .sensor_id = 0x01,
  .size = 0,
  .reports = {0}
};

struct single_sensor_reports* rotation_vector = &(struct single_sensor_reports) {
  .chan = 5,
  .sensor_id = 0x05,
  .size = 0,
  .reports = {0}
};

struct single_sensor_reports* magnetic_field = &(struct single_sensor_reports) {
  .chan = 3,
  .sensor_id = 0x03,
  .size = 0,
  .reports = {0}
};

struct full_sensor_reports* sensor_reports = &(struct full_sensor_reports) {
  .accelerometer = NULL,
  .gyroscope = NULL,
  .magnetic_field = NULL
};

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
void init(){

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

  sensor_reports->accelerometer = accelerometer;
  sensor_reports->magnetic_field = magnetic_field;
  sensor_reports->gyroscope = rotation_vector;

  if(sensor_reports == NULL || accelerometer == NULL || magnetic_field == NULL || rotation_vector == NULL){
    printf("Failed to allocated memory for sensor struct\n");
    flash_led_inf(500, 500);
  }

  printf("Sensor reports struct allocated\n");

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
 * Format a report to stdout
 *
 * @param[in] name      The display name for the reports
 * @param[in] size      The size of the data
 * @param[in] payload   The report data
 */
void format_sensor_reports(const char* name, uint16_t size, uint8_t payload[]){
  if(size == 0){
    return;
  }

  printf("%s Report:\n", name);
  printf("%d bytes received\n", size);

  printf("[");
  for(uint16_t i = 0; i < (size-1); i++){
    printf("%d, ", payload[i]);
  }
  printf("%d]\n", payload[size-1]);
}

/**
 * Print the reports received
 */
void output_report(){
  static uint16_t report_count = 0;

  //printf("Read %d completed successfully\n", report_count);

  format_sensor_reports("Accelerometer", sensor_reports->accelerometer->size, sensor_reports->accelerometer->reports);
  //format_sensor_reports("Magnetic Field", sensor_reports->magnetic_field->size, sensor_reports->magnetic_field->reports);
  //format_sensor_reports("Gyro Rotation Vector", sensor_reports->gyroscope->size, sensor_reports->gyroscope->reports);

  printf("\n");
  report_count++;
}

/**
 * Read the header and then the payload data for a given channel
 *
 * @param[in] channel   The channel to read data from
 * @param[out] buf      A struct pointer to store the output
 */
void read_sensor(struct single_sensor_reports* buf){
  buf->size = 0;
  memset(buf->reports, 0, MAX_PAYLOAD_SIZE);

  unsigned int res;
  uint8_t cmd[] = {6, 0, buf->chan, get_seq_num(), 0xF0, buf->sensor_id};

  res = write_sensor(cmd, sizeof(cmd));

  if(res == PICO_ERROR_GENERIC){
    flash_led_inf(6000, 6000);
  } else if(res == PICO_ERROR_TIMEOUT){
    flash_led_inf(7000, 7000);
  }

  uint8_t header[4];
  uint8_t* payload_ptr = payload;

  res = i2c_read_blocking_until(I2C_INST, BNO085_ADDR, header, 4, false, calc_timeout());

  if(res == PICO_ERROR_GENERIC){
    printf("Failed to read header\n");
    flash_led_inf(2000, 2000);
  } else if(res == PICO_ERROR_TIMEOUT){
    printf("Timeout reached whilst reading header\n");
    flash_led_inf(5000, 5000);
  }

  uint16_t payload_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Remove continutation bit
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


  memcpy(buf->reports, payload, payload_size);
  buf->size = payload_size;
  memset(payload, 0, payload_size);
}

/**
 * Read the accelerometer
 */
void read_accelerometer(){
  read_sensor(sensor_reports->accelerometer);
}

/**
 * Read the magnetic field
 */
void read_magnetic_field(){
  read_sensor(sensor_reports->magnetic_field);
}

/**
 * Read the rotation vector
 */
void read_rotation_vector(){
  read_sensor(sensor_reports->gyroscope);
}

/**
 * Read all three sensor channels
 */
void read_all_sensors(){
  read_accelerometer();
  read_magnetic_field();
  read_rotation_vector();
}

/**
 * Polls the sensor over I2C and process any output
 */
void poll_sensor(){
  read_all_sensors();

  output_report();

  sleep_ms(SAMPLE_DELAY_MS);
}

void enable_feature(uint8_t feature_report_id, uint64_t period_ms){
  uint8_t period[4];

  uint64_t period_us = period_ms * 1000;

  period[0] = period_us & 0xFF;
  period[1] = (period_us >> 8) & 0xFF;
  period[2] = (period_us >> 16) & 0xFF;
  period[3] = (period_us >> 24) & 0xFF;

  //printf("Period of %" PRIu64 " ms is now MSB[%x, %x, %x, %x]LSB", period_ms, period[3], period[2], period[1], period[0]);

  uint8_t pkt[] = {
    0x15, // Length LSB
    0x00, // Length MSB
    0x02, // Channel
    get_seq_num(),
    0xFD, // Report ID
    feature_report_id,
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
    printf("Failed to enable feature %x\n", feature_report_id);
  } else if(res == PICO_ERROR_TIMEOUT){
    printf("Timeout reached when enabling feature %x\n", feature_report_id);
  } else {
    printf("Successfully enabled feature %x with a reporting period of %" PRIu64 " ms", feature_report_id, period_ms);
  }
}

/**
 * Configures which sensors should run
 */
void configure_sensors(){
  unsigned int res;

  uint8_t pkt[] = {12, 0, 2, get_seq_num(), 0xF9};

  res = i2c_write_blocking(I2C_INST, BNO085_ADDR, pkt, sizeof(pkt), false);

  if(res == PICO_ERROR_GENERIC){
    printf("Failed to send message\n");
    flash_led_inf(3000, 3000);
  }
  
  uint8_t header[4];
  uint8_t* payload_ptr = payload;

  res = i2c_read_blocking_until(I2C_INST, BNO085_ADDR, header, 4, false, calc_timeout());

  if(res == PICO_ERROR_GENERIC){
    printf("Failed to read header\n");
    flash_led_inf(2000, 2000);
  } else if(res == PICO_ERROR_TIMEOUT){
    printf("Timeout reached whilst reading header\n");
    flash_led_inf(5000, 5000);
  }

  uint16_t payload_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Remove continutation bit
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

  uint32_t part_num = payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24);
  uint32_t build_num = payload[8] | (payload[9] << 8) | (payload[10] << 16) | (payload[11] << 24);
  uint16_t patch_num = payload[12] | (payload[13] << 8);

  printf("Report ID: %d\n", payload[0]);
  printf("Reset Cause: %d\n", payload[1]);
  printf("SW Version Major: %d\n", payload[2]);
  printf("SW Version Minor: %d\n", payload[3]);
  printf("SW Part Number: %d [%d, %d, %d, %d]\n", part_num, payload[4], payload[5], payload[6], payload[7]);
  printf("SW Build Number: %d [%d, %d, %d, %d]\n", build_num, payload[8], payload[9], payload[10], payload[11]);
  printf("SW Version Patch: %d [%d, %d]\n", patch_num, payload[12], payload[13]);

  enable_feature(0x01, 60);
}

int main()
{
  stdio_init_all();

  init();

  open_channel();

  configure_sensors();

  flash_led_n(100, 100, 5);

  for (;;) poll_sensor();

  return 0;
}
