/***********************************************************************************
 * @file   main.c
 * @author Toby Godfrey
 * @brief  I2C communication implementation for a Pico and Adafruit BNO085 board
 **********************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "defs.h"
#include "pico/time.h"

uint8_t payload[MAX_PAYLOAD_SIZE];

/**
 * Endless loop with flashing the on-board LED
 */
void flash_led(uint16_t t_on, uint16_t t_off){
  for(;;){
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(t_on);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(t_off);
  }
}

/**
 * Initialise the I2C functionality and LED
 */
void init(){

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

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_pull_down(PICO_DEFAULT_LED_PIN);

#endif
}

/**
 * Format and print the report stored in the payload
 *
 * @param[in] size The number of bytes to output
 */
void output_report(unsigned int size){
  printf("Report received successfully\n");

  printf("%d bytes received\n", size);

  for(uint16_t i = 0; i < size; i++){
    printf("Index %d: %c", i, payload[i]);
  }

  for(uint8_t i = 0; i < 5; i++){
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(200);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(200);
  }
}

/**
 * Open the I2C channel
 */
void open_channel(){
  uint8_t pkt[] = {5, 0, 1, 0, 2};

  bool succ = false;
  for(uint8_t attempts; attempts < MAX_ATTEMPTS; attempts++){
    if(i2c_write_blocking(I2C_INST, SENSOR_ADDR, pkt, sizeof(pkt), false) != PICO_ERROR_GENERIC){
      succ = true;
      break;
    }
    sleep_ms(100);
  }

  if(!succ){
    printf("Failed to open channel to sensor\n");
    flash_led(1000, 1000);
  }

  sleep_ms(300);
}

/**
 * Read the header and then the payload data
 *
 * @returns the number of bytes of the payload returned
 */
uint16_t read_sensor(){
  uint8_t header[4];
  unsigned int res;
  uint8_t* payload_ptr = payload;
  
  res = i2c_read_blocking(I2C_INST, SENSOR_ADDR, header, 4, false);

  if(res == PICO_ERROR_GENERIC){
    printf("Failed to read header\n");
    flash_led(2000, 2000);
  }

  uint16_t payload_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  payload_size &= ~0x8000;

  if(payload_size > MAX_PAYLOAD_SIZE){
    printf("Payload too large for buffer\n");
    flash_led(3000, 3000);
  }

  uint16_t bytes_remaining = payload_size;
  uint16_t read_size;
  uint8_t i2c_buf[MAX_PAYLOAD_SIZE];
  uint16_t payload_read_amount = 0;
  bool first_read = true;

  while(bytes_remaining > 0){
    if(first_read) read_size = min(MAX_PAYLOAD_SIZE, (size_t)bytes_remaining);
    else read_size = min(MAX_PAYLOAD_SIZE, (size_t)(bytes_remaining + 4));

    res = i2c_read_blocking(I2C_INST, SENSOR_ADDR, i2c_buf, read_size, false);

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

  return payload_size;
}

/**
 * Polls the sensor over I2C and prints any output
 */
void poll_sensor(){
  unsigned int res;

  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  res = read_sensor();

  if(res == 0) {
    printf("No bytes from sensor\n");
    flash_led(4000, 4000);
  } else output_report(res);

  memset(&(payload[0]), 0, res);
}

int main()
{

  stdio_init_all();

  init();

  open_channel();

  for (;;) poll_sensor();

  return 0;
}
