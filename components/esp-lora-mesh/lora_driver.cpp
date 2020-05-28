#include <lora_driver.h>
#include <lora_constants.h>
#include <lora_tdm.h>
#include <math.h>

#define BITSET(x,y) x |= (1 << y)
#define BITCLEAR(x,y) x &= ~(1<< y)
#define BITREAD(x,y) ((0u == (x & (1<<y)))?0u:1u)
#define BITTOGGLE(x,y) (x ^= (1<<y))

#define ESP_INTR_FLAG_DEFAULT 0

static const char* TAG = "LORA DRIVER";

SemaphoreHandle_t _dio0Semaphore = NULL;

void (*dio0_isr)(void*) = NULL;

spi_host_device_t _spi_id;
spi_device_handle_t _spi;
long _frequency;
gpio_num_t _rstPin;
gpio_num_t _csPin;
gpio_num_t _dio0Pin;

void loraSetPins(spi_host_device_t spi_id, gpio_num_t rst_pin, gpio_num_t cs_pin, gpio_num_t dio0_pin)
{
  _spi_id = spi_id;
  _rstPin = rst_pin;
  _csPin = cs_pin;
  _dio0Pin = dio0_pin;
}

void loraRegisterISR(void (*isr_fun)(void*))
{
  dio0_isr = isr_fun;
}
// we do nothing until the begin function is called to give user time to setup spi bus.
int loraBegin()
{
  ESP_LOGI(TAG,"Lora Begin");
  // Store a ref to the spi device

  esp_err_t ret;

  gpio_pad_select_gpio(_rstPin);
  gpio_set_direction(_rstPin, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(_csPin);
  gpio_set_direction(_csPin, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(_dio0Pin);
  gpio_set_direction(_dio0Pin, GPIO_MODE_INPUT);

  if(dio0_isr != NULL)
  {
    ret = gpio_set_intr_type(_dio0Pin,GPIO_INTR_POSEDGE);
    ESP_ERROR_CHECK(ret);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(_dio0Pin, dio0_isr,NULL);
  }

  _dio0Semaphore = xSemaphoreCreateBinary();


  //the bus should already be initialized by the user, so try to attach this device to it
  spi_device_interface_config_t dev = {};
  dev.clock_speed_hz = 9000000;
  dev.mode = 0;
  dev.spics_io_num = _csPin;
  dev.queue_size = 1;
  dev.flags = 0;
  dev.pre_cb = NULL;

  ret = spi_bus_add_device(_spi_id, &dev, &_spi);
  assert(ret == ESP_OK);
  ESP_LOGI(TAG,"Device added to bus");

  loraReset();

  uint8_t version;
  uint8_t i = 0;
  while(i++ < TIMEOUT_RESET) {
    version = loraReadRegister(REG_VERSION);
    if(version == 0x12) break;
    vTaskDelay(2);
  }
  assert(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1
  ESP_LOGI(TAG,"Device Version: 0x%x",version);

  // put in sleep mode
  loraSleep();

  // set base addresses
  loraWriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
  loraWriteRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  loraWriteRegister(REG_LNA, loraReadRegister(REG_LNA) | 0x03);

  // set auto AGC
  loraWriteRegister(REG_MODEM_CONFIG_3, 0x04);

  // put in standby mode
  loraIdle();

  return 1;
}

void loraReset()
{
  ESP_LOGI(TAG,"Resetting Lora Module");
  gpio_set_level(_rstPin, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(_rstPin, 1);
  vTaskDelay(pdMS_TO_TICKS(10)); 
}

void loraSendPacket(uint8_t *buf, int size)
{
  loraIdle();
  loraWriteRegister(REG_FIFO_ADDR_PTR, 0);

  for(int i=0; i<size; i++)
    loraWriteRegister(REG_FIFO, *buf++);

  loraWriteRegister(REG_PAYLOAD_LENGTH, size);
  /*
   * Start transmission and wait for conclusion.
   */

  loraWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  while((loraReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
    vTaskDelay(2);

  loraWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

int loraReadPacket(uint8_t *buf, int size)
{
  int len = 0;

  /*
   * Check interrupts.
   */
  int irq = loraReadRegister(REG_IRQ_FLAGS);
  loraWriteRegister(REG_IRQ_FLAGS, irq);
  if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
  if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

  /*
   * Find packet size.
   */
  len = loraReadRegister(REG_RX_NB_BYTES);

  /*
   * Transfer data from radio.
   */
  loraIdle();
  loraWriteRegister(REG_FIFO_ADDR_PTR, loraReadRegister(REG_FIFO_RX_CURRENT_ADDR));
  if(len > size) len = size;
  for(int i=0; i<len; i++) 
    *buf++ = loraReadRegister(REG_FIFO);

  return len;
}

void loraWriteRegister(uint8_t reg,uint8_t val)
{
  uint8_t out[2] = {(uint8_t)(0x80 | reg), val };
  uint8_t in[2];

  spi_transaction_t t = {};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  spi_device_transmit(_spi, &t);
}

uint8_t loraReadRegister(uint8_t reg)
{
  uint8_t out[2] = { reg, 0xff };
  uint8_t in[2];

  spi_transaction_t t = {};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  spi_device_transmit(_spi, &t);
  return in[1]; 
}

int loraPacketRssi()
{
  return (loraReadRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float loraPacketSnr()
{
  return ((int8_t)loraReadRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long loraPacketFrequencyError()
{
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(loraReadRegister(REG_FREQ_ERROR_MSB) & 0b111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(loraReadRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(loraReadRegister(REG_FREQ_ERROR_LSB));

  if (loraReadRegister(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (loraGetSignalBandwidth() / 500000.0f); // p. 37

  return static_cast<long>(fError);
}

int loraAvailable()
{
  if(loraReadRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
  return 0;
}

void loraReceive()
{
  loraWriteRegister(REG_DIO_MAPPING_1,0x00); // RXDONE
  loraWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void loraIdle()
{
  loraWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void loraSleep()
{
  loraWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void loraSetTxPower(int level)
{
  ESP_LOGI(TAG,"Setting TX Power to %i",level);
  // RF9x module uses PA_BOOST pin
  if (level < 2) level = 2;
  else if (level > 17) level = 17;
  loraWriteRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void loraSetFrequency(long frequency)
{
  ESP_LOGI(TAG,"Setting Frequency to %li",frequency);
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  loraWriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  loraWriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  loraWriteRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int loraGetSpreadingFactor()
{
  return loraReadRegister(REG_MODEM_CONFIG_2) >> 4;
}

void loraSetSpreadingFactor(int sf)
{
  ESP_LOGI(TAG,"Setting Spreading Factor to %i",sf);
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    loraWriteRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    loraWriteRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    loraWriteRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    loraWriteRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  loraWriteRegister(REG_MODEM_CONFIG_2, (loraReadRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  loraSetLdoFlag();
}

long loraGetSignalBandwidth()
{
  uint8_t bw = (loraReadRegister(REG_MODEM_CONFIG_1) >> 4);

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

void loraSetSignalBandwidth(long sbw)
{
  ESP_LOGI(TAG,"Setting signal bandwidth to %ld",sbw);

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

  loraWriteRegister(REG_MODEM_CONFIG_1, (loraReadRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  loraSetLdoFlag();
}

void loraSetLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( loraGetSignalBandwidth() / (1L << loraGetSpreadingFactor()) ) ;

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t config3 = loraReadRegister(REG_MODEM_CONFIG_3);

  if(ldoOn)
    BITSET(config3,3);
  else
    BITCLEAR(config3,3);

  loraWriteRegister(REG_MODEM_CONFIG_3, config3);
}

void loraSetCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  loraWriteRegister(REG_MODEM_CONFIG_1, (loraReadRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void loraSetPreambleLength(long length)
{
  ESP_LOGI(TAG,"Setting Preamble Length to %li",length);
  loraWriteRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  loraWriteRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void loraSetSyncWord(int sw)
{
  loraWriteRegister(REG_SYNC_WORD, sw);
}

void loraEnableCrc()
{
  loraWriteRegister(REG_MODEM_CONFIG_2, loraReadRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void loraDisableCrc()
{
  loraWriteRegister(REG_MODEM_CONFIG_2, loraReadRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void loraEnableInvertIQ()
{
  loraWriteRegister(REG_INVERTIQ,  0x66);
  loraWriteRegister(REG_INVERTIQ2, 0x19);
}

void loraDisableInvertIQ()
{
  loraWriteRegister(REG_INVERTIQ,  0x27);
  loraWriteRegister(REG_INVERTIQ2, 0x1d);
}

void loraSetOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  loraWriteRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

uint8_t loraRandom()
{
  return loraReadRegister(REG_RSSI_WIDEBAND);
}

void loraExplicitHeaderMode()
{
  loraWriteRegister(REG_MODEM_CONFIG_1, loraReadRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

long loraCalculateAirtime(int length, int spreadingFactor, bool explicitHeader, int lowDR, int codingRate, long bandwidth)
{
  // TODO: Verify this with logic analyzer, could be wrong, preamble length??
  double _length = (double)length;
  double _spreadingFactor = (double)spreadingFactor;
  double _explicitHeader = (double)explicitHeader;
  double _lowDR = (double)lowDR;
  double _codingRate = (double)codingRate;
  double _bandwidth = (double)bandwidth;

  double timePerSymbol = pow(2, _spreadingFactor)/(_bandwidth);
  double arg = ceil(((8*_length)-(4*_spreadingFactor)+28+16-(20*(1-_explicitHeader)))/(4*(_spreadingFactor-2*_lowDR)))*(_codingRate);
  double symbolsPerPayload=8+(fmax(arg, 0.0));
  double timePerPayload = timePerSymbol*symbolsPerPayload;
  return 1000000*timePerPayload; // Convert to micros
}
