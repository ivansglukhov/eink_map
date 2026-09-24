// Open this file in Arduino IDE. Keep the adjacent .h files in this folder.
// GeoJSON parsing and polygon scan conversion use bounded heap buffers, but a
// larger loop task avoids exhausting the Arduino-ESP32 default 8 KiB stack.
#if defined(SET_LOOP_TASK_STACK_SIZE)
SET_LOOP_TASK_STACK_SIZE(24576)
#endif
#include "MapApp.h"

void setup() { mapapp::setup(); }
void loop() { mapapp::loop(); }
