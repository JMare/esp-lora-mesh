#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora_driver.h"
#include "lora_tdm.h"

//#define SENDER

static const char* TAG = "MAIN";


extern "C" {
  void app_main(void);
};

void spi_init()
{
  spi_bus_config_t bus = {};
  bus.miso_io_num = 19;
  bus.mosi_io_num = 27;
  bus.sclk_io_num = 5;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 0;
  esp_err_t ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
  assert(ret == ESP_OK);
  ESP_LOGI(TAG,"VSPI Initialized");
}
void app_main(void)
{
  ESP_LOGI(TAG,"MAIN ENTRY");
  spi_init();

  // Configure the driver with appropriate pins
  loraSetPins(VSPI_HOST,(gpio_num_t)14,(gpio_num_t)18,(gpio_num_t)26);
  loraTDMStart();
}
