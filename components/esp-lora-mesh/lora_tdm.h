#ifndef LORA_TDM_H
#define LORA_TDM_H

#include <stdbool.h>
#include <stdint.h>

// What are we
#define MASTER
#define TDM_THIS_SLOT_ID 0

// Timing defines
#define TDM_SLOT_WIDTH_MICROS 1000000 //1s
#define TDM_SLOT_GUARD_MICROS 100000 //100ms
#define TDM_NUM_SLOTS 3

#define TIMER_DIVIDER 80 // Divide 80mhz to 1mhz = 1us clicks

// Radio Config
#define LORA_FREQ 915e6
#define LORA_BW 500e3
#define LORA_SF 8
#define LORA_TX_PWR 14
#define LORA_CR_DEN 5


enum TDMEventType {
  TDM_EVENT_DIO_IRQ,
  TDM_EVENT_SLOT_END
};

void loraTDMTask(void *args);
void loraTDMStart();
void loraTDMConfigureRadio();

#endif
