#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define LORA_MAX_PACKET_SIZE 256
#define PACKET_HEADER_SIZE 4
#define LORA_MAX_MESSAGE_LEN LORA_MAX_PACKET_SIZE-PACKET_HEADER_SIZE

#define MSG_ID_SYNC 0x00
#define MSG_ID_HBT 0x01


// Virtual class for message
class Message {
 public:
  virtual int pack(uint8_t *buf, int index){return 0;};
  virtual void unpack(uint8_t *buf, int index){};
};

// contains source and dest for entire trip
class Packet {
public:
  uint8_t source_id;
  uint8_t dest_id;
  uint8_t ttl;
  uint8_t msg_id;
  Message msg;

  int pack(uint8_t *buf);
  bool unpack(uint8_t *buf);
};

class SyncMessage : public Message {
 public:
  int pack(uint8_t *buf, int index)
  {
    uint8_t byte0 = ((micros_to_slot_end >> 24) & 0xFF) ;
    uint8_t byte1 = ((micros_to_slot_end >> 16) & 0xFF) ;
    uint8_t byte2 = ((micros_to_slot_end >> 8 ) & 0XFF);
    uint8_t byte3 = (micros_to_slot_end & 0XFF);

    buf[index] = this->slot_number;
    buf[index + 1] = byte0;
    buf[index + 2] = byte1;
    buf[index + 3] = byte2;
    buf[index + 4] = byte3;

    return 5;
  };

  void unpack(uint8_t *buf, int index)
  {
    this->slot_number = buf[index];
    micros_to_slot_end = buf[index + 4];
    micros_to_slot_end = micros_to_slot_end | (buf[index + 3] << 8);
    micros_to_slot_end = micros_to_slot_end | (buf[index + 2] << 16);
    micros_to_slot_end = micros_to_slot_end | (buf[index + 1] << 24);
  }

  uint8_t slot_number;
  uint32_t micros_to_slot_end;
};

#endif
