#include <lora_tdm.h>
#include <lora_driver.h>
#include <lora_constants.h>
#include <protocol.h>

#include "driver/timer.h"


static const char* TAG = "LORA TDM";

uint32_t slot_ends[TDM_NUM_SLOTS];

QueueHandle_t qTDMEvent=NULL;

TDMState tdm_state;
bool tdm_lock;
uint8_t current_slot_number;

const gpio_num_t slotPin = (gpio_num_t)17;
volatile bool slotOutput = false;

void IRAM_ATTR dio0_isr(void *para)
{
  TDMEventType tdm_event = TDM_EVENT_DIO_IRQ;
  xQueueSend(qTDMEvent,&tdm_event,(TickType_t)0);
}

void IRAM_ATTR slot_timer_isr(void *para)
{
  gpio_set_level(slotPin,slotOutput);
  slotOutput = !slotOutput;

  timer_spinlock_take(TIMER_GROUP_0);

  TDMEventType tdm_event = TDM_EVENT_SLOT_END;
  xQueueSend(qTDMEvent,&tdm_event,(TickType_t)0);

  uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_counter_value += (uint64_t) TDM_SLOT_WIDTH_MICROS;

  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0,TIMER_0);
  timer_group_set_alarm_value_in_isr(TIMER_GROUP_0,TIMER_0,timer_counter_value);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0,TIMER_0);

  timer_spinlock_give(TIMER_GROUP_0);
}

void slot_timer_init()
{
  gpio_pad_select_gpio(slotPin);
  gpio_set_direction(slotPin, GPIO_MODE_OUTPUT);

  timer_config_t config = {};
  config.divider = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_EN;
  config.auto_reload = TIMER_AUTORELOAD_DIS;

  // Initialize the counter and load it with the slot width
  timer_init(TIMER_GROUP_0,TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0,TIMER_0,TDM_SLOT_WIDTH_MICROS);
  timer_set_alarm_value(TIMER_GROUP_0,TIMER_0,0);
  timer_enable_intr(TIMER_GROUP_0,TIMER_0);
  timer_isr_register(TIMER_GROUP_0,TIMER_0,slot_timer_isr,(void *) TIMER_0,ESP_INTR_FLAG_IRAM,NULL);
}

void loraTDMStart()
{
  qTDMEvent = xQueueCreate(1,sizeof(TDMEventType));
  slot_timer_init();
  tdm_lock = true; //not really but lets pretend
  current_slot_number = 0;

  // we must register the ISR before we call begin or it will not get called
  loraRegisterISR(&dio0_isr);
  loraBegin();
  loraTDMConfigureRadio();

  TaskHandle_t loraTDMTask_handle = NULL;
  xTaskCreate(&loraTDMTask,"LoRa TDM Task",3096,NULL,5,&loraTDMTask_handle);
}

void loraTDMTask(void *args)
{

  while(true)
  {
    loraTDMStateMachine();
  }
}

void loraTDMStateMachine()
{
  // each state function must return before the end of the slot
  // this means we will be blocking here when the next slot starts
  // the lora radio should be put in idle so we wont get any stray interrupts

  // If we dont have lock then we need to go direct to loraTDMListen() since the timer isnt running
  if(tdm_state != TDM_STATE_LISTEN)
  {
    loraTDMWaitForNextSlot();
    loraTDMNextSlot();
  }


  switch(tdm_state)
  {
  case TDM_STATE_LISTEN:
    loraTDMListen();
    break;

  case TDM_STATE_TRANSMIT:
    loraTDMTransmit();
    break;

  case TDM_STATE_RECEIVE:
    loraTDMReceive();
    break;

  default:
    break;
  }
}

void loraTDMWaitForNextSlot()
{
  TDMEventType tdm_event;
  if(xQueueReceive(qTDMEvent,&tdm_event,10000/portTICK_PERIOD_MS))
    {
      if(tdm_event != TDM_EVENT_SLOT_END)
        ESP_LOGE(TAG,"We were pending for a TDM_EVENT_SLOT_END but got something else, this is bad");
    }
}

void loraTDMListen()
{
  uint64_t start_listen = esp_timer_get_time()/1000;
  loraReceive();
  while(1)
  {
    uint64_t nowtime = esp_timer_get_time()/1000;
    if(nowtime-start_listen >= TDM_LISTEN_MS)
    {
      // time to do our own thing
      ESP_LOGI(TAG,"No Sync Packets Received, setting our own timing");
      current_slot_number = 0;
      tdm_state = TDM_STATE_RECEIVE; // doesnt matter what we set this to just need it to not be LISTEN
      loraTDMStartTimer(TDM_SLOT_WIDTH_MICROS);
      break;
    }
    else // We are still waiting for a sync message
    {
      TDMEventType tdm_event;
      if(xQueueReceive(qTDMEvent,&tdm_event,100 / portTICK_PERIOD_MS))
      {
        if(tdm_event == TDM_EVENT_DIO_IRQ)
        {
          uint8_t buf[LORA_MAX_MESSAGE_LEN];
          uint8_t len = loraReadPacket(buf,LORA_MAX_MESSAGE_LEN);
          Packet pkt;
          pkt.unpack(buf,len);
          if(pkt.msg_id == MSG_ID_SYNC)
          {
            SyncMessage msg(&pkt);
            loraTDMHandleSyncMsg(&msg);
            break;
          }
        }
      }
    }
  }
  loraIdle();
}

void loraTDMStartTimer(uint32_t time_to_slot_end)
{
  uint64_t now_timer;
  timer_get_counter_value(TIMER_GROUP_0,TIMER_0,&now_timer);
  timer_set_alarm_value(TIMER_GROUP_0,TIMER_0,now_timer + time_to_slot_end);
  timer_start(TIMER_GROUP_0,TIMER_0);
}

void loraTDMHandleSyncMsg(SyncMessage *msg)
{
  ESP_LOGI(TAG,"Rx SyncMessage: Slot %u",msg->slot_number);

  if(tdm_state == TDM_STATE_LISTEN)
  {
    loraTDMStartTimer(msg->micros_to_slot_end);
    // any sync message we rx means we must be in someone elses slot, go to rx
    current_slot_number = msg->slot_number;
    tdm_state = TDM_STATE_RECEIVE;
  }
  else
  {
    // Calculate the error and do something with it
    uint64_t next_alarm;
    uint64_t now_counter;
    timer_get_counter_value(TIMER_GROUP_0,TIMER_0,&now_counter);
    timer_get_alarm_value(TIMER_GROUP_0,TIMER_0,&next_alarm);
    uint32_t our_micros_to_slot_end = next_alarm - now_counter;
    int slot_error = (int)our_micros_to_slot_end - (int)msg->micros_to_slot_end;
    ESP_LOGI(TAG,"TDM Error %i",slot_error);

    static unsigned int sync_counter = 0;
    sync_counter++;
    //if(sync_counter%5==0)
      loraTDMAdjustLock(slot_error);
  }
}

void loraTDMAdjustLock(int error)
{
  if(error == 0)
    return;

  uint64_t alarm;
  timer_get_alarm_value(TIMER_GROUP_0,TIMER_0,&alarm);
  int shift = constrain(0.5*error,-50,50);
  alarm -= shift;
  ESP_LOGI(TAG,"Adjusting TDM Lock by %i micros",shift);
  timer_set_alarm_value(TIMER_GROUP_0,TIMER_0,alarm);
}

void loraTDMReceive()
{
  loraReceive();

  TDMEventType tdm_event;
  if(xQueueReceive(qTDMEvent,&tdm_event,(TDM_USABLE_SLOT/1000) / portTICK_PERIOD_MS))
    {
      if(tdm_event == TDM_EVENT_DIO_IRQ)
        {
          uint8_t buf[LORA_MAX_MESSAGE_LEN];
          uint8_t len = loraReadPacket(buf,LORA_MAX_MESSAGE_LEN);
          Packet pkt;
          pkt.unpack(buf,len);
          if(pkt.msg_id == MSG_ID_SYNC)
            {
              SyncMessage msg(&pkt);
              loraTDMHandleSyncMsg(&msg);
            }
        }
    }
  loraIdle();
}

void loraTDMTransmit()
{
  vTaskDelay((TDM_SLOT_GUARD_MICROS/1000)/portTICK_PERIOD_MS);
  Packet pkt = {};
  SyncMessage msg = {};
  msg.slot_number = TDM_THIS_SLOT_ID;

  long airtime = loraCalculateAirtime(MSG_LEN_SYNC + PACKET_HEADER_SIZE,LORA_SF,true, 0, LORA_CR_DEN, LORA_BW);
  uint64_t next_alarm;
  uint64_t now_counter;
  timer_get_alarm_value(TIMER_GROUP_0,TIMER_0,&next_alarm);
  timer_get_counter_value(TIMER_GROUP_0,TIMER_0,&now_counter);
  msg.micros_to_slot_end = next_alarm - now_counter - airtime;

  msg.pack(&pkt);

  uint8_t buf[LORA_MAX_PACKET_SIZE];
  int len = pkt.pack(buf);

  loraSendPacket(buf,len);
  loraIdle();
}

void loraTDMNextSlot()
{
  current_slot_number++;
  if(current_slot_number == TDM_NUM_SLOTS)
    current_slot_number = 0;

  if(current_slot_number == TDM_THIS_SLOT_ID)
  {
    tdm_state = TDM_STATE_TRANSMIT;
  }
  else
  {
    tdm_state = TDM_STATE_RECEIVE;
  }
}

/*
void loraTDMTask(void *args)
{

  while(true)
  {
    TDMEventType tdm_event;
    if(xQueueReceive(qTDMEvent,&tdm_event,(TickType_t)100))
    {
      if(tdm_event == TDM_EVENT_SLOT_END)
      {
        if(current_slot_number == TDM_NUM_SLOTS-1)
          current_slot_number = 0;
        else
          current_slot_number++;
        ESP_LOGI(TAG,"Start Slot %i",current_slot_number);

        if(current_slot_number == TDM_THIS_SLOT_ID)
        {
          SyncMessage msg = {};
          msg.slot_number = TDM_THIS_SLOT_ID;

          long airtime = loraCalculateAirtime(5,LORA_SF,true, 0, LORA_CR_DEN, LORA_BW);
          uint64_t next_alarm;
          uint64_t now_counter;
          timer_get_alarm_value(TIMER_GROUP_0,TIMER_0,&next_alarm);
          timer_get_counter_value(TIMER_GROUP_0,TIMER_0,&now_counter);
          msg.micros_to_slot_end = next_alarm - now_counter - airtime;
          ESP_LOGI(TAG,"Slot Ends in %d",msg.micros_to_slot_end);
          uint8_t buf[LORA_MAX_MESSAGE_LEN];
          int len = msg.pack(buf);
          loraSendPacket(buf,len);
        }
      }
    }
  }
}
*/

void loraTDMConfigureRadio()
{
  loraSetFrequency(LORA_FREQ);
  loraSetSignalBandwidth(LORA_BW);
  loraSetSpreadingFactor(LORA_SF);
  loraSetTxPower(LORA_TX_PWR);
  loraExplicitHeaderMode();
  loraSetCodingRate4(LORA_CR_DEN);
  loraEnableCrc();
}
/*
 * Sync Packet: send at the start of each slot, contains window and syncronization details
 * Startup: Receive until we see a SYNC packet (perhaps multiple that align?)
 * If no sync packets seen in TDM_LISTEN_TIMEOUT, then assume we are the first node alive
 * If first node alive, set our own slot timing and start transmitting.
 * If we receive a sync packet, we can begin tdm

 * Sync packets contain uint16_t time to end of slot adjusted by transmitter for airtime
 * Sync packets also contain TDM_SLOT_COUNT, so we can detect bad timing
 * Sync packets contain tdmSlotID, which is our id in the system, must be unique

 */
