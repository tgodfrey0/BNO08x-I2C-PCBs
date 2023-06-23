#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "defs.h"

uint8_t payload[REPORT_SIZE];

/**
 * Initialise the I2C functionality
 */
void init(){

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning No I2C pins are set
  puts("I2C pins were not defined")
#else

  i2c_init(i2c_default, BAUD_RATE_HZ);
  
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);

  printf("I2C interface successfully configured\n");

#endif
}

/**
 * Format and print the report stored in the payload
 */
void output_report(){
  // TODO: Implement
  printf("Report received successfully\n");
}

/**
 * Polls the sensor over I2C and prints any output
 */
void poll_sensor(){
  int res;

  res = i2c_read_blocking(i2c_default, SENSOR_ADDR, &(payload[0]), REPORT_SIZE, false);

  if(res == PICO_ERROR_GENERIC) printf("Failed to read from sensor\n");
  else output_report();

  memset(&payload, 0, REPORT_SIZE);
}

int main()
{

  stdio_init_all();

  init();

  for (;;) poll_sensor();

  return 0;
}
