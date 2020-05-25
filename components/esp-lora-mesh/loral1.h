#ifndef LORA_H
#define LORA_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

class LoraL1 {
public:
  LoraL1(gpio_num_t rst_pin, gpio_num_t cs_pin, gpio_num_t dio0_pin, long freq);

  int begin(spi_host_device_t spi_id);

  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);

  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();
  long packetFrequencyError();


  int available();
  void sendPacket(uint8_t *buf, int size);
  int readPacket(uint8_t *buf, int size);

  void receive();

  void idle();
  void sleep();

  void reset();

  void setTxPower(int level);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();
  void enableInvertIQ();
  void disableInvertIQ();

  void setOCP(uint8_t mA); // Over Current Protection control

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  uint8_t random();

private:
  void explicitHeaderMode();

  static void IRAM_ATTR handleDio0Rise(void *args);
  static void loraTask(void *args);

  bool isTransmitting();

  int getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

private:
  spi_device_handle_t _spi;
  long _frequency;
  gpio_num_t _rstPin;
  gpio_num_t _csPin;
  gpio_num_t _dio0Pin;

};

#endif
