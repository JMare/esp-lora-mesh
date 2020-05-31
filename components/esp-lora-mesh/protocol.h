#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define LORA_MAX_PACKET_SIZE 256
#define PACKET_HEADER_SIZE 5
#define LORA_MAX_MESSAGE_LEN LORA_MAX_PACKET_SIZE-PACKET_HEADER_SIZE

#define MSG_ID_SYNC 0x00
#define MSG_LEN_SYNC 5
#define MSG_ID_HBT 0x01

class Packet {
public:
  uint8_t source_id; // sending node id
  uint8_t dest_id; // target node id
  uint8_t ttl; // time to live for future routing implementations
  uint8_t seq; // sequence to track packet loss from each node
  uint8_t msg_id; //to idenfity what msg the payload is
  uint8_t msg_buf[LORA_MAX_MESSAGE_LEN];

  int pack(uint8_t *buf);
  void unpack(uint8_t *buf,uint8_t len);

  // this is not encoded in the air frame since the length per msg is fixed
  // but is here so we can pack a message without needing to know what type it is
  uint8_t msg_len = 0;
};

class SyncMessage {
 public:
  SyncMessage(){}; // Default constructor
  SyncMessage(Packet *pkt) // decode from packet
  {
    //TODO: check msg length is correct
    this->slot_number  = pkt->msg_buf[0];
    micros_to_slot_end = pkt->msg_buf[4];
    micros_to_slot_end = micros_to_slot_end | (pkt->msg_buf[3] << 8);
    micros_to_slot_end = micros_to_slot_end | (pkt->msg_buf[2] << 16);
    micros_to_slot_end = micros_to_slot_end | (pkt->msg_buf[1] << 24);
  };

  void pack(Packet *pkt)
  {
    uint8_t byte0 = ((micros_to_slot_end >> 24) & 0xFF) ;
    uint8_t byte1 = ((micros_to_slot_end >> 16) & 0xFF) ;
    uint8_t byte2 = ((micros_to_slot_end >> 8 ) & 0XFF);
    uint8_t byte3 = (micros_to_slot_end & 0XFF);

    pkt->msg_buf[0] = this->slot_number;
    pkt->msg_buf[1] = byte0;
    pkt->msg_buf[2] = byte1;
    pkt->msg_buf[3] = byte2;
    pkt->msg_buf[4] = byte3;

    pkt->msg_len = MSG_LEN_SYNC;
    pkt->msg_id = MSG_ID_SYNC;
  };

  void unpack(Packet *pkt)
  {
    this->slot_number  = pkt->msg_buf[0];
    micros_to_slot_end = pkt->msg_buf[4];
    micros_to_slot_end = micros_to_slot_end | (pkt->msg_buf[3] << 8);
    micros_to_slot_end = micros_to_slot_end | (pkt->msg_buf[2] << 16);
    micros_to_slot_end = micros_to_slot_end | (pkt->msg_buf[1] << 24);
  }

  uint8_t slot_number;
  uint32_t micros_to_slot_end;
};

#endif
