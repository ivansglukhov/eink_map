#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>

namespace mapbuttons {
enum Key : uint8_t { None, Up, Down, Right, Left, ZoomIn, ZoomOut, Invert, Exit, ToggleFollow, ToggleTrack, FullRefresh, ToggleWifi, BatteryProfile };
struct Event { Key key; uint32_t epoch; };
static QueueHandle_t queue = nullptr;
static std::atomic<uint32_t> epoch{0}, dropped{0};
constexpr int ADC_PIN = 2;
constexpr uint32_t DEBOUNCE_MS = 45, REPEAT_DELAY_MS = 600;
constexpr uint32_t REPEAT_MS = 250, LONG_PRESS_MS = 1000, DOUBLE_CLICK_MS = 400;

inline Key readKey() {
  int a[7];
  for (int i=0; i<7; ++i) {
    a[i] = analogRead(ADC_PIN);
    for (int j=i; j>0 && a[j]<a[j-1]; --j) { int t=a[j]; a[j]=a[j-1]; a[j-1]=t; }
  }
  const int v=a[3];
  // Confirmed SW1..SW7: 0,1915,872,482,272,135,43.
  if (v < 22) return Up;
  if (v < 89) return Invert;
  if (v < 204) return ZoomOut;
  if (v < 377) return ZoomIn;
  if (v < 677) return Left;
  if (v < 1394) return Right;
  if (v < 3000) return Down;
  return None;
}
inline void push(Key key, uint32_t pressEpoch) {
  Event e{key,pressEpoch};
  if (xQueueSend(queue,&e,0)!=pdTRUE) ++dropped;
}
inline void task(void*) {
  Key candidate=None, stable=None;
  uint32_t changed=millis(), pressed=0, repeated=0, pressEpoch=0;
  Key pendingClick=None;
  uint32_t pendingAt=0, pendingEpoch=0;
  bool longSent=false;
  TickType_t wake=xTaskGetTickCount();
  for (;;) {
    const uint32_t now=millis();
    const Key raw=readKey();
    if (raw!=candidate) { candidate=raw; changed=now; }
    if (candidate!=stable && now-changed>=DEBOUNCE_MS) {
      if (stable==Invert && !longSent) {
        if (pendingClick==Invert && now-pendingAt<=DOUBLE_CLICK_MS) {
          push(FullRefresh,pressEpoch); pendingClick=None;
        } else {
          if (pendingClick!=None) push(pendingClick,pendingEpoch);
          pendingClick=Invert; pendingAt=now; pendingEpoch=pressEpoch;
        }
      }
      if (stable==Up && !longSent) {
        if (pendingClick==Up && now-pendingAt<=DOUBLE_CLICK_MS) {
          push(ToggleWifi,pressEpoch); pendingClick=None;
        } else {
          if (pendingClick!=None) push(pendingClick,pendingEpoch);
          pendingClick=Up; pendingAt=now; pendingEpoch=pressEpoch;
        }
      }
      if (stable==ZoomOut && !longSent) push(ZoomOut,pressEpoch);
      if (stable==Down && !longSent) push(Down,pressEpoch);
      stable=candidate; pressed=now; repeated=now; longSent=false;
      pressEpoch=epoch.load();
      if (stable!=None && stable!=Invert && stable!=Up && stable!=Down && stable!=ZoomOut) push(stable,pressEpoch);
    }
    if (stable==Invert && !longSent && candidate==stable && now-pressed>=LONG_PRESS_MS) {
      push(Exit,pressEpoch); longSent=true;
    }
    if (stable==Up && !longSent && candidate==stable && now-pressed>=LONG_PRESS_MS) {
      push(ToggleFollow,pressEpoch); longSent=true;
    }
    if (stable==Down && !longSent && candidate==stable && now-pressed>=LONG_PRESS_MS) {
      push(ToggleTrack,pressEpoch); longSent=true;
    }
    if (stable==ZoomOut && !longSent && candidate==stable && now-pressed>=LONG_PRESS_MS) {
      push(BatteryProfile,pressEpoch); longSent=true;
    }
    if (pendingClick!=None && now-pendingAt>DOUBLE_CLICK_MS) {
      push(pendingClick,pendingEpoch); pendingClick=None;
    }
    if (stable>=Up && stable<=Left && stable!=Up && stable!=Down && !longSent && candidate==stable && now-pressed>=REPEAT_DELAY_MS && now-repeated>=REPEAT_MS) {
      push(stable,pressEpoch); repeated=now;
    }
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(15));
  }
}
inline bool begin() {
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN,ADC_11db);
  queue=xQueueCreate(256,sizeof(Event));
  if(!queue) return false;
  if(xTaskCreatePinnedToCore(task,"map-buttons",3072,nullptr,1,nullptr,0)==pdPASS) return true;
  vQueueDelete(queue);
  queue=nullptr;
  return false;
}
inline void newContext() { ++epoch; }
inline bool pop(Key& key) {
  Event e;
  while (xQueueReceive(queue,&e,0)==pdTRUE) {
    if (e.epoch==epoch.load()) { key=e.key; return true; }
  }
  return false;
}
} // namespace mapbuttons
