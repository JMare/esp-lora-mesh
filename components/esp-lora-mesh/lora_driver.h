#ifndef LORA_H
#define LORA_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

extern SemaphoreHandle_t _dio0Semaphore;

void loraSetPins(gpio_num_t rst_pin, gpio_num_t cs_pin, gpio_num_t dio0_pin, long freq);

int loraBegin(spi_host_device_t spi_id);

int loraPacketRssi();
float loraPacketSnr();
long loraPacketFrequencyError();

int loraAvailable();
void loraSendPacket(uint8_t *buf, int size);
int loraReadPacket(uint8_t *buf, int size);

void loraReceive();

void loraIdle();
void loraSleep();

void loraReset();

void loraSetTxPower(int level);
void loraSetFrequency(long frequency);
void loraSetSpreadingFactor(int sf);
void loraSetSignalBandwidth(long sbw);
void loraSetCodingRate4(int denominator);
void loraSetPreambleLength(long length);
void loraSetSyncWord(int sw);
void loraEnableCrc();
void loraDisableCrc();
void loraEnableInvertIQ();
void loraDisableInvertIQ();
int loraGetSpreadingFactor();
long loraGetSignalBandwidth();
void loraSetLdoFlag();
void loraSetOCP(uint8_t mA); // Over Current Protection control
uint8_t loraRandom();
void loraExplicitHeaderMode();

uint8_t loraReadRegister(uint8_t reg);
void loraWriteRegister(uint8_t reg, uint8_t val);

#endif
