#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define LORA_MAX_PACKET_SIZE 256
#define PACKET_HEADER_SIZE 2
#define LORA_MAX_MESSAGE_LEN LORA_MAX_PACKET_SIZE-PACKET_HEADER_SIZE

#define LORA_FREQ 915e6
#define LORA_BW 500e3
#define LORA_SF 8
#define LORA_TX_PWR 14
#define LORA_CR_DEN 5

// Virtual class for message
class Message {
 public:
  virtual int pack(uint8_t *buf);
  virtual void unpack(uint8_t *buf);
};

class SyncMessage : Message {
 public:
  int pack(uint8_t *buf)
  {
    uint8_t byte0 = ((micros_to_slot_end >> 24) & 0xFF) ;
    uint8_t byte1 = ((micros_to_slot_end >> 16) & 0xFF) ;
    uint8_t byte2 = ((micros_to_slot_end >> 8 ) & 0XFF);
    uint8_t byte3 = (micros_to_slot_end & 0XFF);

    buf[0] = this->slot_number;
    buf[1] = byte0;
    buf[2] = byte1;
    buf[3] = byte2;
    buf[4] = byte3;

    return 5;
  };
  void unpack(uint8_t *buf)
  {
    this->slot_number = buf[0];
    micros_to_slot_end = buf[4];
    micros_to_slot_end = micros_to_slot_end | (buf[3] << 8);
    micros_to_slot_end = micros_to_slot_end | (buf[2] << 16);
    micros_to_slot_end = micros_to_slot_end | (buf[1] << 24);
  }

  const uint8_t msg_id = 0;

  uint8_t slot_number;
  uint32_t micros_to_slot_end;
};

struct Packet {
  uint8_t source_id;
  uint8_t msg_id;
  Message msg;
};

#endif
