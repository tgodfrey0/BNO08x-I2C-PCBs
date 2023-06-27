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

single_sensor_reports_t* input_sensors = NULL;
single_sensor_reports_t* wake_input_sensors = NULL;
single_sensor_reports_t* gyro_rotation_vector = NULL;
full_sensor_reports_t* sensor_reports = NULL;

/*************************
 * Byte | Value
 *   0  | LSB Length
 *   1  | MSB Length
 *   2  | Channel
 *   3  | Sequence Number
 *   4+ | Data
 ************************/

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

absolute_time_t parse_timeout(){
  absolute_time_t abs_time_limit = {(time_us_64() + (TIMEOUT_MS * 1000))};
  return abs_time_limit;
}

unsigned int write_sensor(uint8_t* msg, uint16_t len){
  return i2c_write_blocking_until(I2C_INST, BNO085_ADDR, msg, len, false, parse_timeout());
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

  input_sensors = malloc(sizeof(single_sensor_reports_t));
  wake_input_sensors = malloc(sizeof(single_sensor_reports_t));
  gyro_rotation_vector = malloc(sizeof(single_sensor_reports_t));

  sensor_reports = &(full_sensor_reports_t) {
    .input_sensor_reports = input_sensors,
    .wake_input_sensor_reports = wake_input_sensors,
    .gyro_rotation_vector_reports = gyro_rotation_vector
  };

  if(sensor_reports == NULL || input_sensors == NULL || wake_input_sensors == NULL || gyro_rotation_vector == NULL){
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
  uint8_t pkt[] = {5, 0, 1, 0, 2};

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

  printf("Read %d completed successfully\n", report_count);
  
  format_sensor_reports("Input Sensors", sensor_reports->input_sensor_reports->size, sensor_reports->input_sensor_reports->reports);
  format_sensor_reports("Wake Input Sensors", sensor_reports->wake_input_sensor_reports->size, sensor_reports->wake_input_sensor_reports->reports);
  format_sensor_reports("Gyro Rotation Vector", sensor_reports->gyro_rotation_vector_reports->size, sensor_reports->gyro_rotation_vector_reports->reports);

  printf("\n");
  report_count++;
}

/**
 * Read the header and then the payload data for a given channel
 *
 * @param[in] channel   The channel to read data from
 * @param[out] buf      A struct pointer to store the output
 */
void read_sensor(uint8_t channel, single_sensor_reports_t* buf){
  unsigned int res;
  uint8_t cmd[] = {4, 0, channel, 0};

  res = write_sensor(cmd, sizeof(cmd));

  if(res == PICO_ERROR_GENERIC){
    flash_led_inf(6000, 6000);
  } else if(res == PICO_ERROR_TIMEOUT){
    flash_led_inf(7000, 7000);
  }

  uint8_t header[4];
  uint8_t payload[MAX_PAYLOAD_SIZE] = {0};
  uint8_t* payload_ptr = payload;
  
  res = i2c_read_blocking_until(I2C_INST, BNO085_ADDR, header, 4, false, parse_timeout());

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

  memset(buf->reports, 0, buf->size);
  memcpy(buf->reports, payload, payload_size);
  buf->size = payload_size; 
}

/**
 * Read Input sensors
 */
void read_input_sensors(){
  read_sensor(3, input_sensors);
}

/**
 * Read Wake Input sensors
 */
void read_wake_input_sensors(){
  read_sensor(4, wake_input_sensors);
}

/**
 * Read the Gyro Rotation Vector sensors
 */
void read_gyro_rotation_vector_sensors(){
  read_sensor(5, gyro_rotation_vector);
}

/**
 * Read all three sensor channels
 */
void read_all_sensors(){
  read_input_sensors();
  read_wake_input_sensors();
  read_gyro_rotation_vector_sensors();
}

/**
 * Polls the sensor over I2C and process any output
 */
void poll_sensor(){

  unsigned int res;

  read_all_sensors();

  output_report();

  sleep_ms(SAMPLE_DELAY_MS);
}

int main()
{

  stdio_init_all();

  init();
  
  open_channel();

  for (;;) poll_sensor();

  return 0;
}
