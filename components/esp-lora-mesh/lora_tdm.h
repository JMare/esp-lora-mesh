#ifndef LORA_TDM_H
#define LORA_TDM_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// What are we
#define MASTER
//#define TDM_THIS_SLOT_ID 1

// Timing defines
#define TDM_SLOT_WIDTH_MICROS 100000 //100ms
#define TDM_SLOT_GUARD_MICROS 5000 //5ms
#define TDM_USABLE_SLOT TDM_SLOT_WIDTH_MICROS - TDM_SLOT_GUARD_MICROS
#define TDM_LISTEN_MS 3000 // how long to wait for a sync message
#define TDM_NUM_SLOTS 2

#define TIMER_DIVIDER 80 // Divide 80mhz to 1mhz = 1us clicks

// Radio Config
#define LORA_FREQ 915e6
#define LORA_BW 500e3
#define LORA_SF 8
#define LORA_TX_PWR 12
#define LORA_CR_DEN 5
#define LORA_PREAMBLE_LEN 8

enum TDMEventType {
  TDM_EVENT_DIO_IRQ,
  TDM_EVENT_SLOT_END
};

struct TDMEvent {
  TDMEventType type;
  uint64_t timestamp;
};

enum TDMState {
  TDM_STATE_LISTEN, // waiting for a tdm to lock on to
  TDM_STATE_TRANSMIT,
  TDM_STATE_RECEIVE
};

void loraTDMStart();

#endif
