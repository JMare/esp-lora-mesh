#include <lora_tdm.h>
#include <lora_driver.h>
#include <protocol.h>

#include "driver/timer.h"


static const char* TAG = "LORA TDM";

uint32_t slot_ends[TDM_NUM_SLOTS];

QueueHandle_t qTDMEvent=NULL;

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

  timer_start(TIMER_GROUP_0,TIMER_0);
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

void loraTDMConfigureRadio()
{
  loraSetFrequency(LORA_FREQ);
  loraSetSignalBandwidth(LORA_BW);
  loraSetSpreadingFactor(LORA_SF);
  loraSetTxPower(LORA_TX_PWR);
  loraExplicitHeaderMode();
  loraSetCodingRate4(LORA_CR_DEN);
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
