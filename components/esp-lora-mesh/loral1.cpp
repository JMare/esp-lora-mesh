#include <loral1.h>
#include <lora_constants.h>

#define BITSET(x,y) x |= (1 << y)
#define BITCLEAR(x,y) x &= ~(1<< y)
#define BITREAD(x,y) ((0u == (x & (1<<y)))?0u:1u)
#define BITTOGGLE(x,y) (x ^= (1<<y))

static const char* TAG = "LORA L1";

LoraL1::LoraL1(gpio_num_t rst_pin, gpio_num_t cs_pin, long freq) :
  _frequency(freq),
  _rstPin(rst_pin),
  _csPin(cs_pin)
{
}

// we do nothing until the begin function is called to give user time to setup spi bus.
int LoraL1::begin(spi_host_device_t  spi_id)
{
  ESP_LOGI(TAG,"Lora Begin");
  // Store a ref to the spi device

  esp_err_t ret;

  gpio_pad_select_gpio(this->_rstPin);
  gpio_set_direction(this->_rstPin, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(this->_csPin);
  gpio_set_direction(this->_csPin, GPIO_MODE_OUTPUT);

  //the bus should already be initialized by the user, so try to attach this device to it
  spi_device_interface_config_t dev = {};
  dev.clock_speed_hz = 9000000;
  dev.mode = 0;
  dev.spics_io_num = -1;
  dev.queue_size = 1;
  dev.flags = 0;
  dev.pre_cb = NULL;

  ret = spi_bus_add_device(spi_id, &dev, &_spi);
  assert(ret == ESP_OK);
  ESP_LOGI(TAG,"Device added to bus");

  this->reset();

  uint8_t version;
  uint8_t i = 0;
  while(i++ < TIMEOUT_RESET) {
    version = this->readRegister(REG_VERSION);
    if(version == 0x12) break;
    vTaskDelay(2);
  }
  assert(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1
  ESP_LOGI(TAG,"Device Version: 0x%x",version);

  // put in sleep mode
  this->sleep();

  // set frequency
  this->setFrequency(this->_frequency);

  // set base addresses
  this->writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  this->writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  this->writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  this->writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  this->setTxPower(17);

  // put in standby mode
  this->idle();

  return 1;
}

void LoraL1::reset()
{
  ESP_LOGI(TAG,"Resetting Lora Module");
  gpio_set_level(this->_rstPin, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(this->_rstPin, 1);
  vTaskDelay(pdMS_TO_TICKS(10)); 
}

void LoraL1::sendPacket(uint8_t *buf, int size)
{
  this->idle();
  this->writeRegister(REG_FIFO_ADDR_PTR, 0);

  for(int i=0; i<size; i++)
    this->writeRegister(REG_FIFO, *buf++);

  this->writeRegister(REG_PAYLOAD_LENGTH, size);
  /*
   * Start transmission and wait for conclusion.
   */

  this->writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  while((this->readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
    vTaskDelay(2);

  this->writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

int LoraL1::readPacket(uint8_t *buf, int size)
{
  int len = 0;

  /*
   * Check interrupts.
   */
  int irq = this->readRegister(REG_IRQ_FLAGS);
  this->writeRegister(REG_IRQ_FLAGS, irq);
  if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
  if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

  /*
   * Find packet size.
   */
  len = this->readRegister(REG_RX_NB_BYTES);

  /*
   * Transfer data from radio.
   */
  this->idle();
  this->writeRegister(REG_FIFO_ADDR_PTR, this->readRegister(REG_FIFO_RX_CURRENT_ADDR));
  if(len > size) len = size;
  for(int i=0; i<len; i++) 
    *buf++ = this->readRegister(REG_FIFO);

  return len;
}

void LoraL1::writeRegister(uint8_t reg,uint8_t val)
{
  uint8_t out[2] = {(uint8_t)(0x80 | reg), val };
  uint8_t in[2];

  spi_transaction_t t = {};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level(_csPin,0);
  spi_device_transmit(_spi, &t);
  gpio_set_level(_csPin,1);
}

uint8_t LoraL1::readRegister(uint8_t reg)
{
  uint8_t out[2] = { reg, 0xff };
  uint8_t in[2];

  spi_transaction_t t = {};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level(_csPin,0);
  spi_device_transmit(_spi, &t);
  gpio_set_level(_csPin,1);
  return in[1]; 
}

int LoraL1::packetRssi()
{
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float LoraL1::packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoraL1::packetFrequencyError()
{
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return static_cast<long>(fError);
}

int LoraL1::available()
{
  if(this->readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
  return 0;
}

void LoraL1::receive()
{
  this->writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoraL1::idle()
{
  this->writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoraL1::sleep()
{
  this->writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoraL1::setTxPower(int level)
{
  ESP_LOGI(TAG,"Setting TX Power to %i",level);
  // RF9x module uses PA_BOOST pin
  if (level < 2) level = 2;
  else if (level > 17) level = 17;
  this->writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void LoraL1::setFrequency(long frequency)
{
  ESP_LOGI(TAG,"Setting Frequency to %li",frequency);
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  this->writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  this->writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  this->writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoraL1::getSpreadingFactor()
{
  return this->readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void LoraL1::setSpreadingFactor(int sf)
{
  ESP_LOGI(TAG,"Setting Spreading Factor to %i",sf);
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    this->writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    this->writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    this->writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    this->writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  this->writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long LoraL1::getSignalBandwidth()
{
  uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

void LoraL1::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  setLdoFlag();
}

void LoraL1::setLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);

  if(ldoOn)
    BITSET(config3,3);
  else
    BITCLEAR(config3,3);

  writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoraL1::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoraL1::setPreambleLength(long length)
{
  ESP_LOGI(TAG,"Setting Preamble Length to %li",length);
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoraL1::setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void LoraL1::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoraL1::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoraL1::enableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x66);
  writeRegister(REG_INVERTIQ2, 0x19);
}

void LoraL1::disableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x27);
  writeRegister(REG_INVERTIQ2, 0x1d);
}

void LoraL1::setOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

uint8_t LoraL1::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

void LoraL1::explicitHeaderMode()
{
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}
