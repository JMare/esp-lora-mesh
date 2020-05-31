#include <protocol.h>

int Packet::pack(uint8_t *buf)
{
  // Pack the header
  buf[0] = this->source_id;
  buf[1] = this->dest_id;
  buf[2] = this->ttl;
  buf[3] = this->seq;
  buf[4] = this->msg_id;

  // Fill the rest of the buffer from the msg buffer
  for(int i = 0; i < this->msg_len; i++)
  {
    buf[PACKET_HEADER_SIZE + i] = this->msg_buf[i];
  }

  // Return total length of header + message
  return this->msg_len + PACKET_HEADER_SIZE;
}

void Packet::unpack(uint8_t *buf, uint8_t len)
{
  this->source_id = buf[0];
  this->dest_id = buf[1];
  this->ttl = buf[2];
  this->seq = buf[3];
  this->msg_id = buf[4];
  this->msg_len = len - PACKET_HEADER_SIZE;

  // copy the message into the message buffer
  for(int i = 0; i < msg_len; i++)
  {
    this->msg_buf[i] = buf[PACKET_HEADER_SIZE + i];
  }
}
