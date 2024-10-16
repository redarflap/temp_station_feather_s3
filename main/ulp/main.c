/* ULP-RISC-V example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This code runs on ULP-RISC-V  coprocessor
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "../config.h"

volatile int32_t ds18b20_temperature[MAX_DS18B20] = {INT32_MIN};
volatile uint64_t ds18b20_roms[MAX_DS18B20] = {0};

static int32_t temperatures[MAX_DS18B20] = {INT32_MIN};

static int16_t timeSinceLastWakeup = 0;
static bool firstWakeup = true;

bool isTemperatureValid(int32_t temp)
{
  // Only temperatures between -50 and +200° are valid
  if (temp == INT32_MIN || temp < -800 || temp > 3200)
  {
    return false;
  }
  return true;
}

uint64_t getRom(uint8_t idx)
{
  if (idx > MAX_DS18B20)
    return 0;

  return ds18b20_roms[idx];
}

void setTemperature(uint8_t idx, int32_t value)
{

  if (idx > MAX_DS18B20)
    return;

  ds18b20_temperature[idx] = value;
}

int32_t getTemperature(uint8_t idx)
{
  if (idx >= MAX_DS18B20)
  {
    return INT32_MIN;
  }
  return ds18b20_temperature[idx];
}

static void ds18b20_write_bit(gpio_num_t gpio, bool bit)
{
  ulp_riscv_gpio_output_level(gpio, 0);
  if (bit)
  {
    /* Must pull high within 15 us, without delay this takes 5 us */
    ulp_riscv_gpio_output_level(gpio, 1);
  }

  /* Write slot duration at least 60 us */
  ulp_riscv_delay_cycles(75 * ULP_RISCV_CYCLES_PER_US);
  ulp_riscv_gpio_output_level(gpio, 1);
}

static bool ds18b20_read_bit(gpio_num_t gpio)
{
  bool bit;

  /* Pull low minimum 1 us */
  ulp_riscv_gpio_output_level(gpio, 0);
  ulp_riscv_delay_cycles(3 * ULP_RISCV_CYCLES_PER_US);
  ulp_riscv_gpio_output_level(gpio, 1);

  /* Must sample within 15 us of the failing edge */
  ulp_riscv_delay_cycles(5 * ULP_RISCV_CYCLES_PER_US);
  bit = ulp_riscv_gpio_get_level(gpio);

  /* Read slot duration at least 60 us */
  ulp_riscv_delay_cycles(70 * ULP_RISCV_CYCLES_PER_US);

  return bit;
}

static void ds18b20_write_byte(gpio_num_t gpio, uint8_t data)
{
  for (int i = 0; i < 8; i++)
  {
    ds18b20_write_bit(gpio, (data >> i) & 0x1);
  }
}

static uint8_t ds18b20_read_byte(gpio_num_t gpio)
{
  uint8_t data = 0;
  for (int i = 0; i < 8; i++)
  {
    data |= ds18b20_read_bit(gpio) << i;
  }
  return data;
}

bool ds18b20_reset_pulse(gpio_num_t gpio)
{
  bool presence_pulse;
  /* min 480 us reset pulse + 480 us reply time is specified by datasheet */
  ulp_riscv_gpio_output_level(gpio, 0);
  ulp_riscv_delay_cycles(700 * ULP_RISCV_CYCLES_PER_US);

  ulp_riscv_gpio_output_level(gpio, 1);

  /* Wait for ds18b20 to pull low before sampling */
  ulp_riscv_delay_cycles(75 * ULP_RISCV_CYCLES_PER_US);
  presence_pulse = ulp_riscv_gpio_get_level(gpio) == 0;

  ulp_riscv_delay_cycles(500 * ULP_RISCV_CYCLES_PER_US);

  return presence_pulse;
}

uint8_t getByte64(int idx, uint64_t value)
{
  return (value >> idx * 8) & 0xFF;
}

int main(void)
{
  uint8_t temp_high_byte;
  uint8_t temp_low_byte;

  for (int i = 0; i < MAX_DS18B20; i++)
  {
    setTemperature(i, INT32_MIN);
    temperatures[i] = INT32_MIN;
  }

  // Init GPIO and start conversion
  for (int gpioIdx = 0; gpioIdx < NUM_DS18B20_GPIO; gpioIdx++)
  {
    gpio_num_t gpio;
    switch (gpioIdx)
    {
    case 0:
      gpio = SENSOR_1WIRE_GPIO1;
      break;
    case 1:
      gpio = SENSOR_1WIRE_GPIO2;
      break;
    case 2:
      gpio = SENSOR_1WIRE_GPIO3;
      break;
    case 3:
      gpio = SENSOR_1WIRE_GPIO4;
      break;
    default:
      gpio = SENSOR_1WIRE_GPIO5;
      break;
    }

    /* Setup GPIO used for 1wire */
    ulp_riscv_gpio_init(gpio);
    ulp_riscv_gpio_input_enable(gpio);
    ulp_riscv_gpio_output_enable(gpio);
    ulp_riscv_gpio_set_output_mode(gpio, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup(gpio);
    ulp_riscv_gpio_pulldown_disable(gpio);

    if (!ds18b20_reset_pulse(gpio))
    {
      continue;
    }

    /* Start conversion on all connected sensors */
    ds18b20_write_byte(gpio, 0xCC);
    ds18b20_write_byte(gpio, 0x44);
  }

  // Wait for conversion to finish
  ulp_riscv_delay_cycles(750 * 1000 * ULP_RISCV_CYCLES_PER_US);

  // Read results
  for (int gpioIdx = 0; gpioIdx < NUM_DS18B20_GPIO; gpioIdx++)
  {
    gpio_num_t gpio;
    switch (gpioIdx)
    {
    case 0:
      gpio = SENSOR_1WIRE_GPIO1;
      break;
    case 1:
      gpio = SENSOR_1WIRE_GPIO2;
      break;
    case 2:
      gpio = SENSOR_1WIRE_GPIO3;
      break;
    case 3:
      gpio = SENSOR_1WIRE_GPIO4;
      break;
    default:
      gpio = SENSOR_1WIRE_GPIO5;
      break;
    }

    for (int sensorIdx = 0; sensorIdx < MAX_DS18B20_PER_GPIO; sensorIdx++)
    {

      uint8_t globalIndex = MAX_DS18B20_PER_GPIO * gpioIdx + sensorIdx;
      uint64_t rom = getRom(globalIndex);
      if (rom == 0)
        continue;

      if (!ds18b20_reset_pulse(gpio))
      {
        continue;
      }

      /* Read scratchpad */
      ds18b20_write_byte(gpio, 0x55);
      for (int j = 0; j < 8; j++)
      {
        ds18b20_write_byte(gpio, getByte64(j, rom));
      }
      ds18b20_write_byte(gpio, 0xBE);

      temp_low_byte = 0;
      temp_high_byte = 0;

      temp_low_byte = ds18b20_read_byte(gpio);
      temp_high_byte = ds18b20_read_byte(gpio);

      int16_t temp = temp_low_byte;
      temp |= temp_high_byte << 8;

      temperatures[globalIndex] = temp;

      // Delay here necessary before reading next sensor?
      // ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_US);
    }
  }

  // Decide wether to wakeup
  bool forceWakeup = false;
  timeSinceLastWakeup += POLL_RATE_SECONDS;
  if (timeSinceLastWakeup >= MINIMUM_REPORT_PERIOD)
  {
    forceWakeup = true;
  }

  bool changed = false;
  for (int i = 0; i < MAX_DS18B20; i++)
  {
    if (isTemperatureValid(temperatures[i]) && abs(getTemperature(i) - temperatures[i]) >= MIN_CHANGE_FOR_WAKEUP)
    {
      changed = true;
    }
  }
  if (changed || forceWakeup || firstWakeup)
  {
    for (int i = 0; i < MAX_DS18B20; i++)
    {
      if (isTemperatureValid(temperatures[i]))
      {
        setTemperature(i, temperatures[i]);
      }
      else
      {
        setTemperature(i, INT32_MIN);
      }
    }

    firstWakeup = false;
    timeSinceLastWakeup = 0;
    ulp_riscv_wakeup_main_processor();
  }

  return 0;
}
