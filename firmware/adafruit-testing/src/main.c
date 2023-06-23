#include <stdio.h>

#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "defs.h"

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

#endif
}

int main()
{

  stdio_init_all();

  init();

  for (;;) tight_loop_contents();

  return 0;
}
