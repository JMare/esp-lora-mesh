#include <protocol.h>

int Packet::pack(uint8_t *buf)
{
  // Pack the header
  buf[0] = source_id;
  buf[1] = dest_id;
  buf[2] = ttl;
  buf[3] = msg_id;

  // call msg.pack to fill the rest of the buffer
  int msg_len = this->msg.pack(buf,PACKET_HEADER_SIZE);

  // Return total length of header + message
  return msg_len + PACKET_HEADER_SIZE;
}

bool Packet::unpack(uint8_t *buf)
{
  source_id = buf[0];
  dest_id = buf[1];
  ttl = buf[2];
  msg_id = buf[3];

  switch(msg_id)
  {
  case MSG_ID_SYNC:
    SyncMessage tmp_msg;
    tmp_msg.unpack(buf,PACKET_HEADER_SIZE);
    this->msg = tmp_msg;
    return true;
    break;
  }

  return false;
}
