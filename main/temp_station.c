/* ULP riscv DS18B20 1wire temperature sensor example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ulp_riscv.h"
#include "ulp_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ds18b20.h"
#include "ds18b20_types.h"
#include "onewire_bus.h"
#include "config.h"
#include "../components/lora/include/lora.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "TEMP_STATION"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);

int32_t getTemperature(uint8_t idx)
{
  if (idx >= MAX_DS18B20)
  {
    return INT32_MIN;
  }

  int32_t *arrayPtr = (int32_t *)&ulp_ds18b20_temperature;

  return arrayPtr[idx];
}

void setRom(uint8_t idx, uint64_t value)
{
  if (idx > MAX_DS18B20)
    return;

  uint64_t *arrayPtr = (uint64_t *)&ulp_ds18b20_roms;

  arrayPtr[idx] = value;
}

uint64_t getRom(uint8_t idx)
{
  if (idx > MAX_DS18B20)
    return 0;

  return ((uint64_t *)&ulp_ds18b20_roms)[idx];
}

void initLora()
{
  // Init LoRa
  lora_init();
  lora_set_frequency(868e6);
  lora_enable_crc();
}

void configLora()
{
  lora_set_coding_rate(5);
  lora_set_bandwidth(7);
  lora_set_spreading_factor(7);
}

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated)
  {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK)
    {
      calibrated = true;
    }
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated)
  {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK)
    {
      calibrated = true;
    }
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "Calibration Success");
  }
  else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
  {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  }
  else
  {
    ESP_LOGE(TAG, "Invalid arg or no memory");
  }

  return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static uint16_t readBatteryVoltage()
{
  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = ADC_CHANNEL_1;
  adc_atten_t atten = ADC_ATTEN_DB_11;
  adc_bitwidth_t bitwith = ADC_BITWIDTH_DEFAULT;

  // Read battery voltage
  adc_oneshot_unit_handle_t adc_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = unit,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = bitwith,
      .atten = atten,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &config));
  adc_cali_handle_t adc1_cali_chan1_handle = NULL;
  example_adc_calibration_init(unit, channel, atten, &adc1_cali_chan1_handle);

  int rawValues = 0;
  int rawValue;
  ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &rawValue));
  for (int i = 0; i < 5; i++)
  {

    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &rawValue));
    rawValues += rawValue;
    // printf("Raw value read: %d\n", rawValue);

    // ESP_LOGI(TAG, "ADC%d Channel[%d] raw value: %d  Cali Voltage: %d mV", unit + 1, channel, rawValue, voltage);
    // printf("ADC%d Channel[%d] raw value: %d  Cali Voltage: %d mV", unit + 1, channel, rawValue, voltage);
  }
  rawValue = rawValues / 5;
  int voltage;

  ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, rawValue, &voltage));

  ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
  example_adc_calibration_deinit(adc1_cali_chan1_handle);

  return (uint16_t)(voltage * 4.014);
}

void app_main(void)
{

  initLora();
  configLora();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  /* not a wakeup from ULP, load the firmware */
  if (cause != ESP_SLEEP_WAKEUP_ULP)
  {
    uint32_t device_count = 0;
    uint64_t rom_ids[MAX_DS18B20] = {0};

    for (int i = 0; i < NUM_DS18B20_GPIO; i++)
    {
      uint8_t local_device_count = 0;
      gpio_num_t gpio;
      switch (i)
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

      // Searching for DS18B10 sensors
      // install 1-wire bus
      onewire_bus_handle_t bus = NULL;
      onewire_bus_config_t bus_config = {
          .bus_gpio_num = gpio,
      };
      onewire_bus_rmt_config_t rmt_config = {
          .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
      };
      ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

      ds18b20_device_handle_t ds18b20s[MAX_DS18B20_PER_GPIO];
      onewire_device_iter_handle_t iter = NULL;
      onewire_device_t next_onewire_device;
      esp_err_t search_result = ESP_OK;

      // create 1-wire device iterator, which is used for device search
      ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
      do
      {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK && local_device_count < MAX_DS18B20_PER_GPIO)
        { // found a new device, let's check if we can upgrade it to a DS18B20
          ds18b20_config_t ds_cfg = {};
          // check if the device is a DS18B20, if so, return the ds18b20 handle
          if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[local_device_count]) == ESP_OK)
          {
            uint8_t globalIndex = i * MAX_DS18B20_PER_GPIO + local_device_count;
            // ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", globalIndex, next_onewire_device.address);
            rom_ids[globalIndex] = next_onewire_device.address;
            printf("%x %x\n", (uint8_t)((rom_ids[globalIndex] >> 56) & 0xFF), (uint8_t)((rom_ids[globalIndex] >> 48) & 0xFF));
            local_device_count++;
          }
        }
        else
        {
          break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

      } while (search_result != ESP_ERR_NOT_FOUND);

      // Set resolution
      for (int k = 0; k < local_device_count; k++)
      {
        ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20s[k], DS18B20_RESOLUTION));
      }
      device_count += local_device_count;

      ESP_ERROR_CHECK(onewire_del_device_iter(iter));
      onewire_bus_del(bus);
    }

    // printf("%ld device(s) found", device_count);

    // if (device_count == 0)
    // {

    //   printf("Restarting ESP...");
    //   vTaskDelay(pdMS_TO_TICKS(5000));

    //   esp_restart();
    // }

    // vTaskDelay(pdMS_TO_TICKS(5000));

    init_ulp_program();
    vTaskDelay(pdMS_TO_TICKS(5));

    for (int k = 0; k < MAX_DS18B20; k++)
    {
      setRom(k, rom_ids[k]);
    }
  }
  /* ULP Risc-V detected a change in temperatures */
  else if (cause == ESP_SLEEP_WAKEUP_ULP)
  {

    // for (int i = 0; i < MAX_DS18B20; i++)
    // {

    //   if (getRom(i) != 0)
    //   {
    //     printf("%x %x: %.1f", (uint8_t)((getRom(i) >> 56) & 0xFF), (uint8_t)((getRom(i) >> 48) & 0xFF), getTemperature(i) * 0.0625);
    //   }
    // }

    // Create buffer for node id + sensor ids + measurements
    uint8_t buffer[2 * sizeof(uint16_t) + MAX_DS18B20 * 2 * sizeof(int16_t)];
    uint8_t bufferSize = 4;

    uint8_t mac[8] = {0};
    esp_efuse_mac_get_default(mac);
    buffer[0] = mac[6];
    buffer[1] = mac[5];

    uint16_t voltage = readBatteryVoltage();
    buffer[2] = voltage & 0xFF;
    buffer[3] = voltage >> 8 & 0xFF;

    uint8_t *offset = buffer + 2 * sizeof(uint16_t);

    // Copy last 2 bytes of sensor ids and measured values into buffer
    for (int i = 0; i < MAX_DS18B20; i++)
    {
      if (getRom(i) == 0 || getTemperature(i) == INT32_MIN)
      {
        continue;
      }

      // Copy last 2 byte of sensors rom Id to buffer
      *(int16_t *)offset = (getRom(i) >> 48) & 0xFFFF;
      offset += sizeof(uint16_t);
      // Copy last measured value (in 10th °) to buffer
      *(int16_t *)offset = (int16_t)((int16_t)(getTemperature(i))) * 0.625;
      offset += sizeof(uint16_t);
      bufferSize += 4;
    }

    lora_send_packet(buffer, bufferSize);

    int lost = lora_packet_lost();
    if (lost != 0)
    {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
    }
  }

  lora_sleep();
  //  Enable wakeup by ULP (normal way)
  //  Also enable a timer wakeup to reset the esp in case nothing happened
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(((uint64_t)(1000000)) * BACKUP_WAKEUP_TIMER));
  vTaskDelay(pdMS_TO_TICKS(1));
  esp_deep_sleep_start();
}

static void init_ulp_program(void)
{
  esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
  ESP_ERROR_CHECK(err);

  // Polling period
  ulp_set_wakeup_period(0, POLL_RATE_SECONDS * 1000 * 1000);

  /* Start the program */
  err = ulp_riscv_run();
  ESP_ERROR_CHECK(err);
}
