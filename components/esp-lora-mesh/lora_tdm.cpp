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

void IRAM_ATTR dio0_isr(void *para)
{
  TDMEventType tdm_event = TDM_EVENT_DIO_IRQ;
  xQueueSend(qTDMEvent,&tdm_event,(TickType_t)0);
}

void IRAM_ATTR slot_timer_isr(void *para)
{
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
    loraTDMWaitForNextSlot();

  switch(tdm_state)
  {
  case TDM_STATE_LISTEN:
    loraTDMListen();
    break;

  case TDM_STATE_LOCKING:
    loraTDMLocking();
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
  loraReceive();
  TDMEventType tdm_event;
  if(xQueueReceive(qTDMEvent,&tdm_event,TDM_LISTEN_MS / portTICK_PERIOD_MS))
  {
    if(tdm_event == TDM_EVENT_DIO_IRQ)
      {
        // Clear IRQs so we dont miss the next one
        uint8_t buf[LORA_MAX_MESSAGE_LEN];
        int len = loraReadPacket(buf,LORA_MAX_MESSAGE_LEN);
        SyncMessage msg;
        msg.unpack(buf);
        ESP_LOGI(TAG,"Rx SyncMessage: Slot %u",msg.slot_number);
        timer_set_counter_value(TIMER_GROUP_0,TIMER_0,msg.micros_to_slot_end);
        timer_start(TIMER_GROUP_0,TIMER_0);

        // any sync message we rx means we must be in someone elses slot, go to rx
        current_slot_number = msg.slot_number;
        tdm_state = TDM_STATE_LOCKING;
      }
  }
  else
  {
    // We didnt hear anything, we have to start our own tdm timing and reenter the loop
    ESP_LOGI(TAG,"TDM Listen Timeout, setting our own timing");
    timer_start(TIMER_GROUP_0,TIMER_0);
    current_slot_number = 0;
    tdm_state = TDM_STATE_LOCKING;
  }
  loraIdle();
}

void loraTDMLocking()
{
  ESP_LOGI(TAG,"TDM Locking on");
  loraTDMNextSlot();
}

void loraTDMReceive()
{
  loraReceive();
  TDMEventType tdm_event;
  if(xQueueReceive(qTDMEvent,&tdm_event,(TDM_USABLE_SLOT/1000) / portTICK_PERIOD_MS))
    {
      if(tdm_event == TDM_EVENT_DIO_IRQ)
        {
          // Clear IRQs so we dont miss the next one
          uint8_t buf[LORA_MAX_MESSAGE_LEN];
          int len = loraReadPacket(buf,LORA_MAX_MESSAGE_LEN);
          SyncMessage msg;
          msg.unpack(buf);
          ESP_LOGI(TAG,"Rx SyncMessage: Slot %u",msg.slot_number);
        }
    }
  loraIdle();
  loraTDMNextSlot();
}

void loraTDMTransmit()
{
  ESP_LOGI(TAG,"This is our TX Window, Transmitting");
  loraTDMNextSlot();
}

void loraTDMNextSlot()
{
  current_slot_number++;
  if(current_slot_number == TDM_NUM_SLOTS)
    current_slot_number = 0;

  if(current_slot_number == TDM_THIS_SLOT_ID)
  {
    tdm_state = TDM_STATE_TRANSMIT;
    ESP_LOGI(TAG,"TDM Tx Slot %i",current_slot_number);
  }
  else
  {
    tdm_state = TDM_STATE_RECEIVE;
    ESP_LOGI(TAG,"TDM Rx Slot %i",current_slot_number);
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
