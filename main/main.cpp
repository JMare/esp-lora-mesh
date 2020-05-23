#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "loral1.h"

//#define SENDER

static const char* TAG = "MAIN";

LoraL1 _loraL1((gpio_num_t)14,(gpio_num_t)18,915e6);

extern "C" {
  void app_main(void);
};

void app_main(void)
{
  ESP_LOGI(TAG,"MAIN ENTRY");

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

  _loraL1.begin(VSPI_HOST);
  _loraL1.setFrequency(915e6);
  _loraL1.enableCrc();

#ifdef SENDER
  uint8_t buf[255];
  int counter = 1;
  while(1)
  {
    int len = sprintf((char*)buf,"long msg over lora %i",counter++);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG,"Trying to send packet...");
    _loraL1.sendPacket(buf,len);
    ESP_LOGI(TAG,"Tried to send packet...");
  }
#else
  uint8_t buf[255];

  while(1)
  {
    _loraL1.receive();
    while(_loraL1.available())
    {
      int x = _loraL1.readPacket(buf,sizeof(buf));
      int rssi = _loraL1.packetRssi();
      float snr = _loraL1.packetSnr();
      buf[x] = 0;
      ESP_LOGI(TAG,"Received %i bytes: \"%s\", SNR: %f, RSSI: %i",x,buf,snr,rssi);
      _loraL1.receive();
    }
    vTaskDelay(1);
  }

#endif
}
