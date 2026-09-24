#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD_MMC.h>
#include <GxEPD2_3C.h>
#include <U8g2_for_Adafruit_GFX.h>
#include "Buttons.h"
#include "MapIndex.h"
#include "MapRender.h"
#include "Route.h"
#include "GpsTrack.h"
#include "secrets.h"

namespace mapapp {
constexpr int EPD_CS=18, EPD_DC=21, EPD_RST=38, EPD_BUSY=42;
constexpr int EPD_PWR=7, EPD_SCK=6, EPD_MOSI=3;
constexpr int SCREEN_W=800, SCREEN_H=480, MAP_W=772, PANEL_W=28, MAP_H=456;
constexpr uint32_t PROGRESS_INTERVAL_MS=4000;
constexpr int MAX_MAPS=256, ROWS=12;
constexpr double GPS_LAT=56.926029, GPS_LON=35.314591;
constexpr uint32_t GPS_BAUD=38400;
constexpr uint32_t GPS_WAKE_INTERVAL_MS=10000;
constexpr uint32_t GPS_FIX_TIMEOUT_MS=5000;
constexpr int BATTERY_ADC_PIN=4;
// Calibrated against 4.19 V measured at the battery terminals.
constexpr float BATTERY_SCALE=3.32F, BATTERY_EMPTY_V=3.30F, BATTERY_FULL_V=4.20F;
constexpr uint32_t BATTERY_SAMPLE_MS=30000;
constexpr uint32_t BATTERY_PROFILE_SAMPLE_MS=60000;
static GxEPD2_3C<GxEPD2_750c_GDEW075Z08,480> display(
    GxEPD2_750c_GDEW075Z08(EPD_CS,EPD_DC,EPD_RST,EPD_BUSY));
static GFXcanvas1 canvas(SCREEN_W,SCREEN_H);
static GFXcanvas1 baseCanvas(SCREEN_W,SCREEN_H);
static GFXcanvas1 redCanvas(SCREEN_W,SCREEN_H);
static uint8_t* invertedBitmap=nullptr;
static HardwareSerial gpsSerial(1);
constexpr size_t BITMAP_BYTES=((SCREEN_W+7)/8)*SCREEN_H;
static U8G2_FOR_ADAFRUIT_GFX textFont, statusFont;
static mapindex::Index index;
static maprender::View view;
static maprender::Renderer renderer;
static maproute::Route route;
static gpstrack::Track gpsTrack;
static String maps[MAX_MAPS], currentPath, message, lastStage, lastMapLogStage;
static int mapCount=0, selection=0;
static bool inMap=false, inverted=false, fatal=false, displayFault=false;
static bool systemMenu=false, wifiEnabled=false;
static int systemSelection=0;
static bool rerenderRequested=false, leaveMapRequested=false;
static bool redLayerActive=false, redRefreshRequested=false, clearRedRequested=false;
static bool routeRunning=false;
static uint32_t routeRenderAt=0;
static double routeDistanceMeters=0.0;
static bool gpsFix=false, gpsAwake=false;
static uint8_t gpsSatellites=0;
static double gpsLat=GPS_LAT, gpsLon=GPS_LON;
static bool hasLastGps=false, gpsPointReceivedThisWake=false;
static bool gpsDataChanged=false;
static uint32_t gpsRenderAt=0;
static String gpsLine;
static bool gpsCapturingNmea=false;
constexpr int NMEA_HISTORY_LINES=32, NMEA_SCREEN_LINES=27;
static String nmeaHistory[NMEA_HISTORY_LINES];
static int nmeaHead=0, nmeaCount=0;
static bool nmeaScreen=false, nmeaDirty=false;
static uint32_t nmeaScreenAt=0;
static bool followEnabled=true;
static bool startTestOnOpen=false;
static uint32_t gpsLastFixAt=0;
static uint32_t gpsAwakeAt=0, gpsNextWakeAt=0;
static bool batteryValid=false;
static float batteryVoltage=0.0F;
static uint8_t batteryPercent=0;
static uint32_t batterySampleAt=0;
static uint8_t batteryHistory[100];
static uint8_t batteryHistoryCount=0;
static uint32_t batteryProfileSampleAt=0;
static bool batteryScreen=false;
static maproute::Point lastRenderedMarker{};
static bool lastRenderedMarkerValid=false;
constexpr double ROUTE_SPEED_KMH=5.0;
constexpr uint32_t ROUTE_TICK_MS=5000;
constexpr double ROUTE_STEP_METERS=100.0;
static WebServer web(80);
static File uploadFile;
static String trackWarning;
static bool cancelled=false, mapListTruncated=false;
static uint32_t lastProgress=0, refreshMs=0, rendered=0;
static uint8_t lastMapLogPercent=255;

inline void busyCallback(const void*) { delay(1); }
inline uint16_t ink() { return inverted?GxEPD_WHITE:GxEPD_BLACK; }
inline uint16_t paper() { return inverted?GxEPD_BLACK:GxEPD_WHITE; }
inline void writeText(U8G2_FOR_ADAFRUIT_GFX& font,int x,int baseline,const String& text) {
  font.setCursor(x,baseline); font.print(text);
}
inline String fitText(U8G2_FOR_ADAFRUIT_GFX& font,String s,int width) {
  if (font.getUTF8Width(s.c_str())<=width) return s;
  while (s.length() && font.getUTF8Width((s+"...").c_str())>width) {
    size_t end=s.length()-1;
    while (end>0 && (uint8_t(s[end])&0xC0)==0x80) --end;
    s.remove(end);
  }
  return s+"...";
}
inline String nmeaField(const String& line,int field) {
  int start=0;
  for(int i=0;i<=line.length();++i) {
    if(i==line.length() || line[i]==',') {
      if(field==0) return line.substring(start,i);
      --field; start=i+1;
    }
  }
  return String();
}
inline int hexDigit(char c) {
  if(c>='0'&&c<='9') return c-'0';
  if(c>='A'&&c<='F') return c-'A'+10;
  if(c>='a'&&c<='f') return c-'a'+10;
  return -1;
}
inline bool validNmeaChecksum(const String& line) {
  if(!line.startsWith("$") || line.length()<7) return false;
  int star=line.indexOf('*');
  if(star<0 || star+2>=line.length()) return false;
  uint8_t checksum=0;
  for(int i=1;i<star;++i) checksum^=(uint8_t)line[i];
  int hi=hexDigit(line[star+1]),lo=hexDigit(line[star+2]);
  return hi>=0 && lo>=0 && checksum==(uint8_t)((hi<<4)|lo);
}
inline bool nmeaCoordinate(const String& value,const String& hemi,bool latitude,double& out) {
  (void)latitude;
  if(value.length()<4 || hemi.length()<1) return false;
  double raw=value.toDouble();
  double degrees=std::floor(raw/100.0);
  double result=degrees+(raw-degrees*100.0)/60.0;
  if(hemi[0]=='S' || hemi[0]=='W') result=-result;
  out=result;
  return std::isfinite(out);
}
inline void parseNmea(const String& line) {
  if(!validNmeaChecksum(line)) return;
  if(!line.startsWith("$GP") && !line.startsWith("$GN")) return;
  const bool gga=line.startsWith("$GPGGA") || line.startsWith("$GNGGA");
  const bool rmc=line.startsWith("$GPRMC") || line.startsWith("$GNRMC");
  if(!gga && !rmc) return;
  bool oldFix=gpsFix; uint8_t oldSat=gpsSatellites; double oldLat=gpsLat, oldLon=gpsLon;
  double lat,lon;
  if(gga) {
    int quality=nmeaField(line,6).toInt();
    int satellites=(int)nmeaField(line,7).toInt();
    gpsSatellites=(uint8_t)std::min(99,std::max(0,satellites));
    if(quality>0 && nmeaCoordinate(nmeaField(line,2),nmeaField(line,3),true,lat) &&
       nmeaCoordinate(nmeaField(line,4),nmeaField(line,5),false,lon)) {
      gpsLat=lat; gpsLon=lon; gpsFix=true; hasLastGps=true; gpsPointReceivedThisWake=true; gpsLastFixAt=millis();
    } else gpsFix=false;
  } else if(rmc) {
    const bool active=nmeaField(line,2)=="A";
    if(active && nmeaCoordinate(nmeaField(line,3),nmeaField(line,4),true,lat) &&
       nmeaCoordinate(nmeaField(line,5),nmeaField(line,6),false,lon)) {
      gpsLat=lat; gpsLon=lon; gpsFix=true; hasLastGps=true; gpsPointReceivedThisWake=true; gpsLastFixAt=millis();
    } else gpsFix=false;
  }
  if(oldFix!=gpsFix || oldSat!=gpsSatellites || std::fabs(oldLat-gpsLat)>1e-7 || std::fabs(oldLon-gpsLon)>1e-7) {
    gpsDataChanged=true;
    Serial.printf("GPS %s sats=%u lat=%.7f lon=%.7f\n",gpsFix?"fix":"nofix",gpsSatellites,gpsLat,gpsLon);
  }
}
inline void storeNmea(const String& line) {
  nmeaHistory[nmeaHead]=line;
  nmeaHead=(nmeaHead+1)%NMEA_HISTORY_LINES;
  if(nmeaCount<NMEA_HISTORY_LINES) ++nmeaCount;
  nmeaDirty=true;
}
inline bool pollGps() {
  if(!gpsAwake) return false;
  bool changed=false;
  while(gpsSerial.available()) {
    char c=(char)gpsSerial.read();
    if(c=='$') { gpsLine="$"; gpsCapturingNmea=true; continue; }
    if(!gpsCapturingNmea) continue;
    if(c=='\r') continue;
    if(c=='\n') {
      if(gpsLine.length()) {
        storeNmea(gpsLine);
        parseNmea(gpsLine);
        changed|=gpsDataChanged;
        gpsDataChanged=false;
      }
      gpsLine=""; gpsCapturingNmea=false;
    }
    else if(gpsLine.length()<127) gpsLine+=c;
    else { gpsLine=""; gpsCapturingNmea=false; }
  }
  return changed;
}
inline maproute::Point gpsPoint() { return maproute::Point{gpsLon,gpsLat}; }
inline maproute::Point liveMarkerPoint() {
  return hasLastGps ? gpsPoint() : maproute::Point{view.lon,view.lat};
}
inline void sendUbx(uint8_t cls,uint8_t id,const uint8_t* payload,uint16_t length) {
  uint8_t ckA=0,ckB=0;
  auto checked=[&](uint8_t value){ gpsSerial.write(value); ckA+=value; ckB+=ckA; };
  gpsSerial.write(0xB5); gpsSerial.write(0x62);
  checked(cls); checked(id); checked((uint8_t)length); checked((uint8_t)(length>>8));
  for(uint16_t i=0;i<length;++i) checked(payload[i]);
  gpsSerial.write(ckA); gpsSerial.write(ckB); gpsSerial.flush();
}
inline void configureGpsTenSeconds() {
  // The receiver is awake only briefly; produce a 1 Hz solution while awake.
  // The application timer below controls the 10-second sampling interval.
  const uint8_t payload[]={
    0x00,0x01,0x00,0x00,
    0x01,0x00,0x21,0x30,0xE8,0x03,
    0x02,0x00,0x21,0x30,0x01,0x00,
    0x03,0x00,0x21,0x20,0x01
  };
  sendUbx(0x06,0x8A,payload,sizeof(payload));
  Serial.println("GPS CFG-VALSET requested: 1 Hz while awake; application interval 10 s");
}
inline void wakeGps() {
  if(gpsAwake) return;
  gpsSerial.begin(GPS_BAUD,SERIAL_8N1,16,17);
  gpsSerial.write(0xFF); gpsSerial.flush();
  gpsAwake=true; gpsAwakeAt=millis(); gpsPointReceivedThisWake=false;
  Serial.println("GPS wake");
}
inline void sleepGps() {
  if(!gpsAwake) return;
  uint8_t payload[16]={};
  payload[8]=0x06;   // backup + minimum consumption
  payload[12]=0x08;  // wake on UART RX edge
  sendUbx(0x02,0x41,payload,sizeof(payload));
  delay(100);
  gpsSerial.end();
  pinMode(17,OUTPUT); digitalWrite(17,HIGH);
  gpsAwake=false; gpsNextWakeAt=millis()+GPS_WAKE_INTERVAL_MS;
  Serial.println("GPS sleep");
}
inline bool sampleBattery() {
  uint32_t millivolts=0;
  for(int i=0;i<16;++i) {
    millivolts+=analogReadMilliVolts(BATTERY_ADC_PIN);
    delay(1);
  }
  const float measured=(millivolts/16.0F)*BATTERY_SCALE/1000.0F;
  const uint8_t oldPercent=batteryPercent;
  batteryVoltage=batteryValid?batteryVoltage*0.8F+measured*0.2F:measured;
  batteryPercent=(uint8_t)constrain(
      (int)std::lround((batteryVoltage-BATTERY_EMPTY_V)*100.0F/
                       (BATTERY_FULL_V-BATTERY_EMPTY_V)),0,100);
  const bool changed=!batteryValid || oldPercent!=batteryPercent;
  batteryValid=true;
  Serial.printf("BAT %.3f V %u%%\n",batteryVoltage,batteryPercent);
  return changed;
}
inline void present();
inline void sampleBatteryProfile() {
  const uint8_t value=batteryPercent;
  if(batteryHistoryCount<100) batteryHistory[batteryHistoryCount++]=value;
  else { memmove(batteryHistory,batteryHistory+1,99); batteryHistory[99]=value; }
  batteryProfileSampleAt=millis();
}
inline void drawBatteryProfile() {
  canvas.fillScreen(1);
  writeText(textFont,18,26,"ПРОФИЛЬ БАТАРЕИ");
  writeText(textFont,610,26,String(batteryPercent)+"% "+String(batteryVoltage,2)+"V");
  canvas.drawRect(42,48,710,350,0);
  for(int i=0;i<=4;++i) { int y=48+i*87; canvas.drawFastHLine(43,y,708,0); writeText(textFont,8,y+5,String(100-i*25)+"%"); }
  int ox=-1,oy=-1;
  for(uint8_t i=0;i<batteryHistoryCount;++i) {
    int x=50+i*7; int y=48+350-(int)batteryHistory[i]*350/100;
    if(ox>=0) canvas.drawLine(ox,oy,x,y,0);
    canvas.fillCircle(x,y,2,0); ox=x; oy=y;
  }
  writeText(textFont,18,430,"100 точек, шаг 1 минута, расстояние 7 px");
  writeText(textFont,18,473,"SW4: назад");
  present();
  batteryProfileSampleAt=millis();
}
inline void drawIpBottom(int leftLimit=0) {
  String ip=wifiEnabled && WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : "WiFi: выкл";
  ip=fitText(textFont,ip,SCREEN_W-leftLimit-2);
  int width=textFont.getUTF8Width(ip.c_str());
  int x=max(leftLimit,SCREEN_W-width-2);
  if(x+width>SCREEN_W-1) x=leftLimit;
  writeText(textFont,x,473,ip);
}
inline void scanMaps();
inline String mapFooterText() {
  if(trackWarning.length()) return "TRACK: "+trackWarning;
  String footer="GPS: ";
  footer+=(gpsFix?"fix ":"nofix ");
  footer+=String(gpsSatellites)+" | F:"+(followEnabled?"on":"off")+" T:"+(gpsTrack.recording()?"on":"off")+" | BAT: ";
  footer+=(batteryValid?String(batteryPercent)+"%":"--%");
  footer+=" | Пройдено: "+String(routeDistanceMeters/1000.0,1)+" км | Z"+String(view.zoom);
  footer+=" | "+String(rendered)+" объектов | "+currentPath.substring(6);
  if(renderer.skippedGeometries()) footer+=" | пропущено: "+String(renderer.skippedGeometries());
  return footer;
}
inline void stopTrackOnError() {
  const String reason=gpsTrack.error().length() ? gpsTrack.error() : "Не удалось сохранить точку";
  const double completedDistance=gpsTrack.distanceMetres();
  const String path=gpsTrack.path();
  gpsTrack.abort();
  gpsTrack.clearPoints();
  routeDistanceMeters=completedDistance;
  trackWarning="остановлен — "+reason;
  message="TRACK: "+trackWarning;
  Serial.printf("TRACK STOPPED: %s; partial=%s\n",reason.c_str(),path.c_str());
  if(inMap) rerenderRequested=true;
}
inline bool toggleTrackRecording() {
  bool redrawMap=false;
  if(!gpsTrack.recording()) {
    if(routeRunning) {
      routeRunning=false;
      route.clear();
      redrawMap=true;
      Serial.println("Test route stopped: TRACK ON");
    }
    routeDistanceMeters=0.0;
    if(!gpsTrack.start(SD_MMC)) { message=gpsTrack.error(); Serial.println(message); return false; }
    trackWarning="";
    if(gpsFix && !gpsTrack.append(gpsLon,gpsLat) && gpsTrack.error().length()) {
      stopTrackOnError();
      return true;
    }
    Serial.print("TRACK ON: "); Serial.println(gpsTrack.path());
  } else {
    const String path=gpsTrack.path();
    const double completedDistance=gpsTrack.distanceMetres();
    const bool stopped=gpsTrack.stop();
    if(!stopped) stopTrackOnError();
    gpsTrack.clearPoints();
    routeDistanceMeters=completedDistance;
    redrawMap=true;
    if(!inMap) scanMaps();
    Serial.print(stopped ? "TRACK OFF: " : "TRACK FAILED: "); Serial.println(path);
  }
  if(inMap && redrawMap) rerenderRequested=true;
  return redrawMap;
}
inline void refreshFooterOnly() {
  // Update the distance counter without rebuilding the map image.
  statusFont.setForegroundColor(ink());
  statusFont.setBackgroundColor(paper());
  display.fillRect(0,MAP_H,SCREEN_W,SCREEN_H-MAP_H,paper());
  display.drawFastHLine(0,MAP_H,MAP_W,ink());
  String ip=wifiEnabled && WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : "WiFi: выкл";
  ip=fitText(statusFont,ip,138);
  int ipX=SCREEN_W-statusFont.getUTF8Width(ip.c_str())-2;
  writeText(statusFont,8,473,fitText(statusFont,mapFooterText(),ipX-18));
  writeText(statusFont,ipX,473,ip);
  display.displayWindowBW(0,MAP_H,SCREEN_W,SCREEN_H-MAP_H);
}
inline void drawIcon(int index,int cx,int cy) {
  const int c=0;
  if(index==0 || index==1 || index==2 || index==3) {
    int dx=(index==2)-(index==3), dy=(index==1)-(index==0);
    canvas.drawLine(cx-dx*10,cy-dy*10,cx+dx*10,cy+dy*10,c);
    canvas.drawLine(cx+dx*10,cy+dy*10,cx+dx*5-dy*4,cy+dy*5+dx*4,c);
    canvas.drawLine(cx+dx*10,cy+dy*10,cx+dx*5+dy*4,cy+dy*5-dx*4,c);
  } else if(index==4) {
    canvas.drawFastHLine(cx-9,cy,19,c); canvas.drawFastVLine(cx,cy-9,19,c);
  } else if(index==5) {
    canvas.drawFastHLine(cx-9,cy,19,c);
  } else {
    canvas.drawCircle(cx,cy,9,c); canvas.fillCircle(cx,cy,3,c);
  }
}
inline bool refresh() {
  if (displayFault) return false;
  const uint32_t started=millis();
  // Includes transfer and old-frame synchronization. Never use hibernate here.
  display.displayWindowBW(0,0,SCREEN_W,SCREEN_H);
  refreshMs=millis()-started;
  if (refreshMs>=20000 || digitalRead(EPD_BUSY)==LOW) {
    displayFault=true;
    Serial.println("EPD ERROR: BUSY timeout. Check wiring/driver; reset to retry.");
    return false;
  }
  display.powerOff(); // controller RAM remains powered through EPD_PWR
  Serial.printf("EPD B/W: %lu ms\n",(unsigned long)refreshMs);
  return true;
}
inline void present() {
  display.setFullWindow();
  // drawBitmap writes foreground bits only; clear the controller buffer first
  // so inversion also changes the panel and icon background consistently.
  display.fillScreen(paper());
  display.drawBitmap(0,0,canvas.getBuffer(),SCREEN_W,SCREEN_H,paper(),ink());
  refresh();
}
inline void presentColor() {
  display.setFullWindow();
  const uint8_t* black=canvas.getBuffer();
  if(inverted) {
    if(!invertedBitmap) invertedBitmap=(uint8_t*)ps_malloc(BITMAP_BYTES);
    if(invertedBitmap) {
      for(size_t i=0;i<BITMAP_BYTES;++i) invertedBitmap[i]=~black[i];
      black=invertedBitmap;
    }
  }
  display.writeImage(black,redCanvas.getBuffer(),0,0,SCREEN_W,SCREEN_H);
  // Keep GxEPD2's internal black plane synchronized for subsequent local
  // marker-only displayWindowBW() updates.
  display.fillScreen(paper());
  display.drawBitmap(0,0,canvas.getBuffer(),SCREEN_W,SCREEN_H,paper(),ink());
  // display(false) would upload GxEPD2's internal (white) color buffer again.
  // We already uploaded both planes explicitly, so refresh the controller RAM.
  display.refresh(false);
  display.powerOff();
}
inline void status(const String& title,const String& detail,const String& tail="") {
  Serial.printf("%s | %s | %s\n",title.c_str(),detail.c_str(),tail.c_str());
  display.fillScreen(paper());
  statusFont.setForegroundColor(ink());
  statusFont.setBackgroundColor(paper());
  writeText(statusFont,24,40,fitText(statusFont,title,750));
  display.drawFastHLine(24,54,752,ink());
  writeText(statusFont,24,92,fitText(statusFont,detail,750));
  writeText(statusFont,24,126,fitText(statusFont,tail,750));
  writeText(statusFont,24,440,"Удержание SW7: вернуться к списку карт");
  refresh();
  lastProgress=millis();
}
inline bool cancelCheck() {
  mapbuttons::Key key;
  while (mapbuttons::pop(key)) {
    if (key==mapbuttons::Exit) cancelled=true;
  }
  return cancelled || displayFault;
}
inline void pollRenderButtons() {
  mapbuttons::Key key;
  while(mapbuttons::pop(key)) {
    if(key==mapbuttons::Exit) { leaveMapRequested=true; return; }
    if(key==mapbuttons::ToggleFollow) { followEnabled=!followEnabled; continue; }
    if(key==mapbuttons::ToggleTrack) { toggleTrackRecording(); return; }
    if(key==mapbuttons::Invert) { inverted=!inverted; redRefreshRequested=true; clearRedRequested=false; rerenderRequested=true; continue; }
    if(key==mapbuttons::Up) view.panPixels(0,-MAP_H/4.0);
    else if(key==mapbuttons::Down) view.panPixels(0,MAP_H/4.0);
    else if(key==mapbuttons::Right) view.panPixels(MAP_W/4.0,0);
    else if(key==mapbuttons::Left) view.panPixels(-MAP_W/4.0,0);
    else if(key==mapbuttons::ZoomIn) view.zoomBy(1);
    else if(key==mapbuttons::ZoomOut) view.zoomBy(-1);
    else continue;
    if(redLayerActive) { clearRedRequested=true; redRefreshRequested=false; }
    rerenderRequested=true;
  }
}
inline void progress(const char* stage,uint32_t done,uint32_t total,uint32_t count) {
  const uint8_t percent=total ? uint8_t(uint64_t(done)*100/total) : 0;
  // Serial is intentionally concise: map progress is useful for diagnosis,
  // whereas raw GNSS and display-driver traces obscure the actual failure.
  if(lastMapLogStage!=stage || percent==100 || lastMapLogPercent==255 ||
     percent>=uint8_t(lastMapLogPercent+10)) {
    Serial.printf("MAP INDEX: %s %u%% objects=%lu bytes=%lu/%lu\n",stage,
                  percent,(unsigned long)count,(unsigned long)done,(unsigned long)total);
    lastMapLogStage=stage;
    lastMapLogPercent=percent;
  }
  // Callbacks may happen per feature. E-paper updates only at stage changes
  // or >=4 seconds AFTER the previous refresh has completed.
  if (cancelCheck()) return;
  const bool changed=lastStage!=stage;
  if (!changed && millis()-lastProgress<PROGRESS_INTERVAL_MS) return;
  lastStage=stage;
  String detail=String(stage)+"  "+String(total?uint32_t(uint64_t(done)*100/total):0)+"%";
  if(String(stage)=="Building index") {
    canvas.fillScreen(1);
    writeText(textFont,24,34,"Индексация карты");
    writeText(textFont,24,68,"Минимальный LOD");
    canvas.drawRect(24,92,752,320,0);
    canvas.drawRect(40,108,720,288,0);
    canvas.drawFastHLine(40,252,720,0);
    canvas.drawFastVLine(400,108,288,0);
    canvas.fillRect(40,420,total?680UL*done/total:0,14,0);
    writeText(textFont,24,460,detail+" | Объектов: "+String(count));
    present();
    lastProgress=millis();
  } else status("Загрузка карты",detail,"Объектов: "+String(count));
}
inline void legend(bool browser) {
  canvas.fillRect(MAP_W,0,PANEL_W,SCREEN_H,1);
  // Do not continue the panel border through the bottom status row: the IP
  // address is allowed to use that row all the way to the right edge.
  canvas.drawFastVLine(MAP_W,0,MAP_H,0);
  for(int i=0;i<7;++i) drawIcon(i,MAP_W+PANEL_W/2,60+i*35);
}
inline void scanMaps();
inline String safeName(String name) {
  name.replace("\\","/"); int slash=name.lastIndexOf('/'); if(slash>=0) name=name.substring(slash+1);
  String out; for(size_t i=0;i<name.length();++i) { char c=name[i]; if(isalnum(c)||c=='.'||c=='_'||c=='-') out+=c; }
  return out;
}
inline String indexName(const String& geo) { String x="/maps/"+geo; int dot=x.lastIndexOf('.'); if(dot>=0)x.remove(dot); return x+".gidx"; }
inline void wifiPage() {
  String html="<html><meta charset='utf-8'><title>EINK maps</title><body><h2>EINK maps</h2>";
  html+="<p>IP: "+(wifiEnabled?WiFi.localIP().toString():"off")+"</p><form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='file'><button>Upload</button></form><ul>";
  File d=SD_MMC.open("/maps"); if(d&&d.isDirectory()){ for(File f=d.openNextFile();f;f=d.openNextFile()){ if(!f.isDirectory()){String n=f.name(); html+="<li>"+n+" ("+String(f.size())+") <a href='/download?name="+n.substring(n.lastIndexOf('/')+1)+"'>download</a> <a href='/delete?name="+n.substring(n.lastIndexOf('/')+1)+"'>delete</a></li>";} f.close(); } d.close(); }
  html+="</ul></body></html>"; web.send(200,"text/html; charset=utf-8",html);
}
inline void handleUpload() {
  HTTPUpload& u=web.upload();
  if(u.status==UPLOAD_FILE_START){ String n=safeName(u.filename); if(!n.endsWith(".geojson")) n+=".geojson"; uploadFile=SD_MMC.open("/maps/"+n,FILE_WRITE); }
  else if(u.status==UPLOAD_FILE_WRITE && uploadFile) uploadFile.write(u.buf,u.currentSize);
  else if(u.status==UPLOAD_FILE_END){ if(uploadFile) uploadFile.close(); }
}
inline void startWifi() {
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASS);
  uint32_t started=millis(); while(WiFi.status()!=WL_CONNECTED && millis()-started<15000) delay(250);
  Serial.printf("WiFi %s IP=%s\n",WiFi.status()==WL_CONNECTED?"connected":"offline",WiFi.localIP().toString().c_str());
  web.on("/",HTTP_GET,wifiPage);
  web.on("/upload",HTTP_POST,[](){ web.send(303,"text/plain","/" ); scanMaps(); },handleUpload);
  web.on("/delete",HTTP_GET,[](){ String n=safeName(web.arg("name")); if(n.length()){SD_MMC.remove("/maps/"+n); SD_MMC.remove(indexName(n));} web.sendHeader("Location","/"); web.send(303); scanMaps(); });
  web.on("/download",HTTP_GET,[](){ String n=safeName(web.arg("name")); File f=SD_MMC.open("/maps/"+n); if(!f){web.send(404,"text/plain","not found");return;} web.streamFile(f,"application/octet-stream"); f.close(); });
  web.begin();
}
inline void stopWifi() { web.stop(); WiFi.disconnect(true,true); WiFi.mode(WIFI_OFF); wifiEnabled=false; }
inline void systemScreen() {
  canvas.fillScreen(1); writeText(textFont,18,28,"СИСТЕМА");
  const char* rows[]={"WiFi","Диагностика NMEA","Тестовый маршрут","Назад"};
  for(int i=0;i<4;++i){int y=80+i*45;if(i==systemSelection)canvas.drawRect(10,y-22,620,34,0);writeText(textFont,30,y,rows[i]);}
  writeText(textFont,18,285,wifiEnabled?(WiFi.status()==WL_CONNECTED?"WiFi подключен":"WiFi подключение..."):"WiFi выключен");
  if(wifiEnabled&&WiFi.status()==WL_CONNECTED) writeText(textFont,18,315,"IP: "+WiFi.localIP().toString());
  writeText(textFont,18,350,"FOLLOW: "+String(followEnabled?"ON":"OFF")+"  TRACK: "+String(gpsTrack.recording()?"ON":"OFF"));
  if(message.length()) writeText(textFont,18,385,fitText(textFont,message,700));
  legend(true); present();
}
inline void scanMaps() {
  mapCount=0; selection=0; mapListTruncated=false; message="";
  File dir=SD_MMC.open("/maps");
  if (!dir || !dir.isDirectory()) { message="Создайте папку /maps на SD"; return; }
  for (File f=dir.openNextFile();f;f=dir.openNextFile()) {
    if (!f.isDirectory()) {
      String name=f.name();
      const int slash=name.lastIndexOf('/'); if(slash>=0) name=name.substring(slash+1);
      String lower=name; lower.toLowerCase();
      if (lower.endsWith(".geojson")) {
        if(mapCount<MAX_MAPS) maps[mapCount++]=name;
        else mapListTruncated=true;
      }
    }
    f.close(); delay(1);
  }
  dir.close();
  for(int i=1;i<mapCount;++i) {
    String value=maps[i]; int j=i;
    while(j>0 && maps[j-1].compareTo(value)>0) { maps[j]=maps[j-1]; --j; }
    maps[j]=value;
  }
  if(!mapCount) message="Нет /maps/*.geojson";
  else if(mapListTruncated) message="Показаны первые 256 файлов";
}
inline void drawBrowser() {
  canvas.fillScreen(1);
  writeText(textFont,18,25,"КАРТЫ НА SD");
  writeText(textFont,18,48,"Выберите файл и нажмите SW3 (вправо)");
  const int first=(selection/ROWS)*ROWS;
  for(int row=0;row<ROWS && first+row<mapCount;++row) {
    const int n=first+row,y=68+row*28;
    if(n==selection) canvas.drawRect(12,y-15,MAP_W-24,25,0);
    writeText(textFont,24,y+2,fitText(textFont,maps[n],MAP_W-50));
  }
  writeText(textFont,18,431,fitText(textFont,message,610));
  canvas.drawFastHLine(0,MAP_H,MAP_W,0);
  writeText(textFont,18,473,String(mapCount?selection+1:0)+" / "+String(mapCount)+" файлов | F:"+(followEnabled?"on":"off")+" T:"+(gpsTrack.recording()?"on":"off"));
  legend(true); drawIpBottom(320); present();
}
inline void drawNmeaScreen() {
  canvas.fillScreen(1);
  String title="RAW NMEA | GPS: ";
  title+=(gpsFix?"fix ":"nofix ");
  title+=String(gpsSatellites);
  writeText(textFont,6,20,title);
  canvas.drawFastHLine(0,28,SCREEN_W,0);
  const int shown=min(nmeaCount,NMEA_SCREEN_LINES);
  int first=(nmeaHead-shown+NMEA_HISTORY_LINES)%NMEA_HISTORY_LINES;
  for(int i=0;i<shown;++i) {
    const String& line=nmeaHistory[(first+i)%NMEA_HISTORY_LINES];
    writeText(textFont,4,45+i*15,fitText(textFont,line,SCREEN_W-8));
  }
  canvas.drawFastHLine(0,MAP_H,SCREEN_W,0);
  writeText(textFont,6,473,"SW4 или удержание SW7: назад");
  present();
  nmeaDirty=false;
  nmeaScreenAt=millis();
}
inline void toBrowser(bool preserveMessage=false) {
  // scanMaps() clears transient messages. Preserve an opening/indexing error
  // so a failed map does not silently return to the browser.
  const String browserMessage=preserveMessage ? message : "";
  index.close(); inMap=false; nmeaScreen=false; cancelled=false;
  scanMaps();
  if(browserMessage.length()) message=browserMessage;
  mapbuttons::newContext(); drawBrowser();
}
inline int layer(uint32_t flags) {
  if(flags&mapindex::Road) return 3;
  if(flags&mapindex::Building) return 1;
  if(flags&mapindex::Water) return 0;
  return 2;
}
inline bool renderMap();
inline void systemScreen();
inline void drawNmeaScreen();
inline void drawBrowser();
inline void fitViewToBounds(const mapindex::BBox& b) {
  if(!std::isfinite(b.minLon) || !std::isfinite(b.maxLon) || b.maxLon<=b.minLon ||
     !std::isfinite(b.minLat) || !std::isfinite(b.maxLat) || b.maxLat<=b.minLat) {
    view=maprender::View(); return;
  }
  view.lon=(b.minLon+b.maxLon)*0.5; view.zoom=maprender::kMinZoom;
  for(int z=maprender::kMaxZoom; z>=maprender::kMinZoom; --z) {
    const double world=std::ldexp(256.0,z);
    const double xs=(b.maxLon-b.minLon)*world/360.0;
    const double ys=std::fabs(maprender::worldY(b.minLat,world)-maprender::worldY(b.maxLat,world));
    if(xs<=MAP_W-20 && ys<=MAP_H-20) { view.zoom=z; break; }
  }
  const double world=view.worldSize();
  const double cy=(maprender::worldY(b.minLat,world)+maprender::worldY(b.maxLat,world))*0.5;
  view.lat=maprender::latitudeFromY(cy,world); view.normalize();
}
inline void fullRefresh() {
  if(batteryScreen) { drawBatteryProfile(); return; }
  if(inMap) { redRefreshRequested=true; clearRedRequested=false; rerenderRequested=false; renderMap(); }
  else if(systemMenu) systemScreen(); else if(nmeaScreen) drawNmeaScreen(); else drawBrowser();
}
inline bool visibleAtLod(uint32_t flags) {
  if(view.zoom<=8) return (flags&mapindex::Water) || (flags&mapindex::MajorRoad);
  if(view.zoom<=11) return !(flags&mapindex::Building) || (flags&mapindex::MajorRoad);
  return true;
}
inline bool renderMap() {
  if(nmeaScreen) return true;
  Serial.printf("MAP RENDER: start zoom=%d center=%.7f,%.7f records=%lu\n",
                view.zoom,view.lon,view.lat,(unsigned long)index.header().count);
  canvas.fillScreen(1);
  const bool colorFrame=redRefreshRequested || clearRedRequested;
  if(colorFrame) redCanvas.fillScreen(1);
  if(redRefreshRequested) {
    renderer.begin(redCanvas,view);
    renderer.setPass(maprender::Pass::Water);
    renderer.setSolidWater(true);
    if(!index.rewind()) { message=index.error(); return false; }
    mapindex::Record waterRecord;
    mapindex::BBox waterViewport;
    view.bounds(waterViewport.minLon,waterViewport.minLat,waterViewport.maxLon,waterViewport.maxLat);
    mapindex::PsramAllocator waterAllocator;
    JsonDocument waterDoc(&waterAllocator);
    while(index.next(waterRecord)) {
      if(!(waterRecord.flags&mapindex::Water) || !waterRecord.bbox.intersects(waterViewport)) continue;
      if(!index.load(waterRecord,waterDoc)) { message=index.error(); return false; }
      renderer.feature(waterDoc.as<JsonObjectConst>()); waterDoc.clear();
    }
    // Remove red pixels under settlement labels. The black text itself is
    // rendered later into the main bitmap, so only its white backing is used
    // in the color bitmap.
    renderer.setPass(maprender::Pass::Labels);
    renderer.setLabelEraseOnly(true);
    if(!index.rewind()) { message=index.error(); return false; }
    mapindex::Record labelRecord;
    while(index.next(labelRecord)) {
      if(!(labelRecord.flags&mapindex::Place) || !labelRecord.bbox.intersects(waterViewport)) continue;
      if(!index.load(labelRecord,waterDoc)) { message=index.error(); return false; }
      renderer.feature(waterDoc.as<JsonObjectConst>()); waterDoc.clear();
    }
    renderer.finishLabels();
    renderer.setLabelEraseOnly(false);
    renderer.setSolidWater(false);
  }
  renderer.begin(canvas,view);
  mapindex::BBox viewport;
  view.bounds(viewport.minLon,viewport.minLat,viewport.maxLon,viewport.maxLat);
  mapindex::PsramAllocator allocator;
  JsonDocument doc(&allocator);
  rendered=0;
  const uint32_t started=millis();
  const maprender::Pass passes[]={maprender::Pass::Water,maprender::Pass::Building,maprender::Pass::Other,maprender::Pass::Road};
  bool budgetExceeded=false;
  for(int pass=0;pass<4;++pass) {
    if(!index.rewind()) { message=index.error(); return false; }
    mapindex::Record record;
    uint32_t scanned=0;
    while(index.next(record)) {
      if(millis()-started>8000) { budgetExceeded=true; Serial.println("RENDER: time budget reached; keeping partial LOD"); break; }
      ++scanned;
      // Preserve all map navigation events while loading/rendering/refreshing.
      // Only opening/indexing uses cancelCheck(), which discards navigation.
      if(layer(record.flags)!=pass || !visibleAtLod(record.flags) || !record.bbox.intersects(viewport)) continue;
      if(!index.load(record,doc)) { message=index.error(); return false; }
      renderer.setPass(passes[pass]);
      renderer.feature(doc.as<JsonObjectConst>());
      renderer.setPass(maprender::Pass::Labels);
      renderer.feature(doc.as<JsonObjectConst>());
      doc.clear(); ++rendered;
      if(rendered>=3000) { budgetExceeded=true; Serial.println("RENDER: feature budget reached; keeping LOD"); break; }
      pollGps();
      pollRenderButtons();
      // A pending color frame must reach its final upload once, otherwise a
      // queued navigation event would leave clearRedRequested latched and
      // every following frame would repeat the slow full refresh.
      if((leaveMapRequested || rerenderRequested || nmeaScreen) && !colorFrame) return true;
      if(millis()-lastProgress>=PROGRESS_INTERVAL_MS && millis()-started>=PROGRESS_INTERVAL_MS) {
        // Show the accumulated geometry while the remaining layers are read.
        // The next refresh replaces this partial frame with the completed map.
        legend(false);
        present();
        lastProgress=millis();
        Serial.printf("PARTIAL MAP: layer=%d objects=%lu\n",pass+1,(unsigned long)rendered);
      }
      delay(1);
    }
    if(budgetExceeded) break;
    if(index.error().length()) { message=index.error(); return false; }
  }
  if(nmeaScreen) return true;
  // Draw the recorded GPS trail below labels and the current marker.
  for(size_t i=1;i<gpsTrack.count();++i) {
    const gpstrack::Point& a=gpsTrack.point(i-1);
    const gpstrack::Point& b=gpsTrack.point(i);
    renderer.drawTrackSegment(canvas,a.lon,a.lat,b.lon,b.lat);
  }
  // Keep route test marks below settlement labels.
  if(route.valid()) {
    for(int km=1;km<=route.markCount();++km) {
      maproute::Point mark,tangent;
      if(route.markAt(km,mark,tangent)) renderer.drawRouteMark(mark.lon,mark.lat,tangent.lon,tangent.lat,km);
    }
  }
  maproute::Point marker=routeRunning&&route.valid()?route.current():liveMarkerPoint();
  // Save the completed map without the marker. This background is used for
  // small differential updates when either real or simulated GPS moves.
  renderer.finishLabels();
  memcpy(baseCanvas.getBuffer(),canvas.getBuffer(),((MAP_W+7)/8)*MAP_H);
  const bool drawGpsMarker=route.valid() || hasLastGps;
  if(drawGpsMarker) { renderer.drawMarker(marker.lon,marker.lat); lastRenderedMarker=marker; lastRenderedMarkerValid=true; }
  else lastRenderedMarkerValid=false;
  canvas.fillRect(0,MAP_H,MAP_W,SCREEN_H-MAP_H,1);
  canvas.drawFastHLine(0,MAP_H,MAP_W,0);
  legend(false);
  // The footer may use the whole bottom row. Reserve only the pixels that
  // the actual IP/Wi-Fi label occupies, rather than a fixed 160-pixel gap.
  String ip=wifiEnabled && WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : "WiFi: выкл";
  int ipX=SCREEN_W-textFont.getUTF8Width(ip.c_str())-2;
  writeText(textFont,8,473,fitText(textFont,mapFooterText(),ipX-18));
  writeText(textFont,ipX,473,ip);
  if(colorFrame) {
    presentColor();
    redLayerActive=redRefreshRequested;
    redRefreshRequested=false;
    clearRedRequested=false;
  } else present();
  Serial.printf("MAP RENDER: done features=%lu elapsed=%lu ms psram=%u\n",
                (unsigned long)rendered,(unsigned long)(millis()-started),ESP.getFreePsram());
  return true;
}
inline void openSelected() {
  if(!mapCount) return;
  currentPath="/maps/"+maps[selection];
  cancelled=false; mapbuttons::newContext(); lastStage="";
  lastMapLogStage=""; lastMapLogPercent=255;
  File selected=SD_MMC.open(currentPath,FILE_READ);
  const uint32_t selectedBytes=selected ? uint32_t(selected.size()) : 0;
  selected.close();
  Serial.printf("MAP OPEN: file=%s bytes=%lu\n",currentPath.c_str(),(unsigned long)selectedBytes);
  status("Открытие карты",maps[selection],"Проверка SD и индекса");
  if(!index.open(SD_MMC,currentPath,progress,cancelCheck)) {
    message=cancelled?"Загрузка отменена":index.error();
    Serial.printf("MAP ERROR: open failed: %s\n",message.c_str()); toBrowser(true); return;
  }
  const mapindex::Header& header=index.header();
  Serial.printf("MAP READY: records=%lu skipped=%lu bounds=%.7f,%.7f..%.7f,%.7f\n",
                (unsigned long)header.count,(unsigned long)header.skippedFeatures,
                header.bbox.minLon,header.bbox.minLat,header.bbox.maxLon,header.bbox.maxLat);
  view=maprender::View();
  fitViewToBounds(index.header().bbox);
  inMap=true; rerenderRequested=false; leaveMapRequested=false; mapbuttons::newContext();
  routeRunning=false; route.clear();
  if(!gpsTrack.recording()) routeDistanceMeters=0.0;
  if(startTestOnOpen) {
    startTestOnOpen=false;
    if(route.load(SD_MMC,"/maps/route.geojson")) {
      route.reset(); routeRunning=true; routeRenderAt=millis();
      maproute::Point start=route.current(); view.lon=start.lon; view.lat=start.lat;
    } else message=route.error();
  }
  do {
    rerenderRequested=false;
    if(!renderMap()) { Serial.printf("MAP ERROR: render failed: %s\n",message.c_str()); toBrowser(true); return; }
  } while(rerenderRequested && !leaveMapRequested);
  if(nmeaScreen) { mapbuttons::newContext(); drawNmeaScreen(); return; }
  if(leaveMapRequested) { toBrowser(); }
}
inline void refreshRouteMarkerOnly(maproute::Point oldPoint,maproute::Point newPoint) {
  int ox,oy,nx,ny;
  if(!lastRenderedMarkerValid || !renderer.screenPoint(oldPoint.lon,oldPoint.lat,ox,oy) || !renderer.screenPoint(newPoint.lon,newPoint.lat,nx,ny)) { rerenderRequested=true; return; }
  int x0=max(0,min(ox,nx)-18), y0=max(0,min(oy,ny)-18), x1=min(MAP_W-1,max(ox,nx)+18), y1=min(MAP_H-1,max(oy,ny)+18);
  if(x0>x1 || y0>y1) { lastRenderedMarker=newPoint; return; }
  x0-=x0%8; int w=((x1-x0+8)/8)*8; if(x0+w>MAP_W) w=MAP_W-x0; int h=y1-y0+1;
  // Restore the old marker from the saved map and draw only the new marker.
  for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) display.drawPixel(x,y,baseCanvas.getPixel(x,y)?paper():ink());
  if(nx>=-15 && nx<MAP_W+15 && ny>=-15 && ny<MAP_H+15) {
    display.fillCircle(nx,ny,15,paper()); display.fillCircle(nx,ny,11,ink()); display.fillCircle(nx,ny,7,paper()); display.fillCircle(nx,ny,4,ink());
  }
  // Refresh only the changed marker square.
  display.displayWindowBW(x0,y0,w,h);
  lastRenderedMarker=newPoint;
  Serial.printf("ROUTE MARKER REFRESH x=%d y=%d old=%d,%d window=%dx%d\n",nx,ny,ox,oy,w,h);
}
inline void setup() {
  Serial.begin(115200); delay(300);
  gpsSerial.setRxBufferSize(4096);
  Serial.printf("GPS UART: RX=GPIO16 TX=GPIO17 %lu 8N1\n",(unsigned long)GPS_BAUD);
  wakeGps(); delay(100); configureGpsTenSeconds();
  pinMode(EPD_PWR,OUTPUT); digitalWrite(EPD_PWR,HIGH); delay(100);
  SPI.begin(EPD_SCK,-1,EPD_MOSI,EPD_CS);
  // A zero diagnostics baud rate suppresses GxEPD2's PowerOn/Update/PowerOff
  // trace, leaving only application-level diagnostics in Serial.
  display.init(0,true,2,false);
  display.epd2.setBusyCallback(busyCallback);
  display.setRotation(0); display.setFullWindow(); display.setTextWrap(false);
  statusFont.begin(display); statusFont.setFont(u8g2_font_6x13_t_cyrillic); statusFont.setFontMode(1);
  // A white FULL refresh is essential: current + previous B/W reference planes
  // agree before the first differential update. Red is never used afterwards.
  display.fillScreen(GxEPD_WHITE);
  const uint32_t started=millis(); display.display(false);
  if(millis()-started>=20000 || digitalRead(EPD_BUSY)==LOW) {
    displayFault=true; fatal=true; Serial.println("EPD initial full refresh failed (BUSY)."); return;
  }
  if(!canvas.getBuffer()) { status("Ошибка памяти","Нет памяти для экрана"); fatal=true; return; }
  textFont.begin(canvas); textFont.setFont(u8g2_font_6x13_t_cyrillic); textFont.setFontMode(1);
  textFont.setForegroundColor(0); textFont.setBackgroundColor(1);
  if(!mapbuttons::begin()) { status("Ошибка","Не удалось запустить кнопки"); fatal=true; return; }
  analogSetPinAttenuation(BATTERY_ADC_PIN,ADC_11db);
  sampleBattery(); batterySampleAt=millis();
  if(!psramFound()) { status("Ошибка PSRAM","В Arduino IDE выберите PSRAM: OPI PSRAM"); fatal=true; return; }
  Serial.printf("Flash=%u PSRAM=%u free=%u\n",ESP.getFlashChipSize(),ESP.getPsramSize(),ESP.getFreePsram());
  status("Запуск","Подключение SD_MMC, 1 bit");
  if(!SD_MMC.setPins(41,39,40) || !SD_MMC.begin("/sdcard",true)) {
    status("Ошибка SD","Проверьте SD, FAT32 и GPIO 41/39/40","После исправления нажмите RESET"); fatal=true; return;
  }
  scanMaps(); mapbuttons::newContext(); drawBrowser();
}
inline void loop() {
  if(fatal || displayFault) { delay(100); return; }
  if(wifiEnabled) web.handleClient();
  if(!gpsAwake && (int32_t)(millis()-gpsNextWakeAt)>=0) wakeGps();
  bool gpsChanged=pollGps();
  if(gpsAwake && (gpsPointReceivedThisWake || millis()-gpsAwakeAt>=GPS_FIX_TIMEOUT_MS)) sleepGps();
  if(gpsFix && millis()-gpsLastFixAt>25000) {
    gpsFix=false; gpsChanged=true;
    Serial.println("GPS nofix: timeout");
  }
  bool trackPointAdded=false;
  gpstrack::Point previousTrack{};
  if(gpsChanged && gpsFix && gpsTrack.recording()) {
    if(gpsTrack.count()) previousTrack=gpsTrack.point(gpsTrack.count()-1);
    trackPointAdded=gpsTrack.append(gpsLon,gpsLat);
    routeDistanceMeters=gpsTrack.distanceMetres();
    if(!trackPointAdded && gpsTrack.error().length()) stopTrackOnError();
  }
  if(millis()-batterySampleAt>=BATTERY_SAMPLE_MS) {
    batterySampleAt=millis();
    const bool batteryChanged=sampleBattery();
    if(batteryChanged && inMap && !nmeaScreen && !rerenderRequested) refreshFooterOnly();
  }
  if(batteryScreen) {
    if(millis()-batteryProfileSampleAt>=BATTERY_PROFILE_SAMPLE_MS) { sampleBatteryProfile(); drawBatteryProfile(); }
    mapbuttons::Key profileKey;
    while(mapbuttons::pop(profileKey)) {
      if(profileKey==mapbuttons::Left || profileKey==mapbuttons::Exit) { batteryScreen=false; mapbuttons::newContext(); if(inMap) renderMap(); else drawBrowser(); return; }
      if(profileKey==mapbuttons::FullRefresh) fullRefresh();
    }
    delay(20); return;
  }
  if(nmeaScreen) {
    mapbuttons::Key nmeaKey;
    while(mapbuttons::pop(nmeaKey)) {
      if(nmeaKey==mapbuttons::Left || nmeaKey==mapbuttons::Exit) {
        nmeaScreen=false;
        systemMenu=true;
        mapbuttons::newContext();
        systemScreen();
        return;
      }
    }
    if(nmeaDirty && millis()-nmeaScreenAt>=750) drawNmeaScreen();
    delay(10);
    return;
  }
  if(inMap && !routeRunning && gpsChanged && millis()-gpsRenderAt>=500) {
    gpsRenderAt=millis();
    const maproute::Point oldPoint=lastRenderedMarker;
    const maproute::Point p=liveMarkerPoint();
    if(trackPointAdded && gpsTrack.count()>1) {
      renderer.drawTrackSegment(baseCanvas,previousTrack.lon,previousTrack.lat,p.lon,p.lat);
    }
    bool moveViewport=false;
    if(gpsFix && followEnabled) {
      int px,py;
      if(renderer.screenPoint(p.lon,p.lat,px,py)) {
        int targetX=px,targetY=py;
        if(px<MAP_W/10) targetX=MAP_W*9/10;
        else if(px>MAP_W*9/10) targetX=MAP_W/10;
        if(py<MAP_H/10) targetY=MAP_H*9/10;
        else if(py>MAP_H*9/10) targetY=MAP_H/10;
        if(targetX!=px || targetY!=py) {
          view.panPixels(px-targetX,py-targetY);
          moveViewport=true;
        }
      }
    }
    if(moveViewport) rerenderRequested=true;
    else if(redLayerActive) { clearRedRequested=true; redRefreshRequested=false; rerenderRequested=true; }
    else if(lastRenderedMarkerValid && (gpsFix || oldPoint.lon!=p.lon || oldPoint.lat!=p.lat)) {
      refreshRouteMarkerOnly(oldPoint,p); refreshFooterOnly(); rerenderRequested=false;
    } else {
      refreshFooterOnly();
    }
  }
  if(inMap && routeRunning && route.valid() && millis()-routeRenderAt>=ROUTE_TICK_MS) {
    maproute::Point oldPoint=route.current();
    route.advanceMeters(ROUTE_STEP_METERS); routeRenderAt=millis();
    routeDistanceMeters+=ROUTE_STEP_METERS;
    maproute::Point p=route.current();
    Serial.printf("ROUTE step=%.1f m lon=%.7f lat=%.7f\n",ROUTE_STEP_METERS,p.lon,p.lat);
    bool moveViewport=false;
    if(followEnabled) {
      int px,py;
      if(renderer.screenPoint(p.lon,p.lat,px,py)) {
        int targetX=px,targetY=py;
        if(px<MAP_W/10) targetX=MAP_W*9/10;
        else if(px>MAP_W*9/10) targetX=MAP_W/10;
        if(py<MAP_H/10) targetY=MAP_H*9/10;
        else if(py>MAP_H*9/10) targetY=MAP_H/10;
        if(targetX!=px || targetY!=py) { view.panPixels(px-targetX,py-targetY); moveViewport=true; }
      }
    }
    if(moveViewport) rerenderRequested=true;
    else if(redLayerActive) { clearRedRequested=true; redRefreshRequested=false; rerenderRequested=true; }
    else { refreshRouteMarkerOnly(oldPoint,p); refreshFooterOnly(); rerenderRequested=false; }
  }
  bool dirty=rerenderRequested;
  mapbuttons::Key key;
  while(mapbuttons::pop(key)) {
    if(key==mapbuttons::FullRefresh) { fullRefresh(); dirty=false; continue; }
    if(key==mapbuttons::ToggleWifi) {
      if(wifiEnabled) stopWifi(); else { wifiEnabled=true; startWifi(); }
      if(systemMenu) systemScreen(); else if(inMap) refreshFooterOnly(); else drawBrowser();
      continue;
    }
    if(key==mapbuttons::BatteryProfile) { batteryScreen=true; mapbuttons::newContext(); sampleBatteryProfile(); drawBatteryProfile(); return; }
    if(key==mapbuttons::Exit) {
      if(inMap) { message=""; toBrowser(); return; }
      if(!systemMenu) { systemMenu=true; systemSelection=0; systemScreen(); return; }
    }
    if(key==mapbuttons::ToggleFollow) {
      if(!inMap) continue;
      followEnabled=!followEnabled;
      refreshFooterOnly(); continue;
    }
    if(key==mapbuttons::ToggleTrack) {
      if(systemMenu) continue;
      const bool redrawMap=toggleTrackRecording();
      if(inMap) {
        if(redrawMap) dirty=true;
        else refreshFooterOnly();
        continue;
      }
      drawBrowser(); return;
    }
    if(key==mapbuttons::Invert) { inverted=!inverted; redRefreshRequested=true; clearRedRequested=false; dirty=true; continue; }
    if(systemMenu) {
      if(key==mapbuttons::Up) systemSelection=(systemSelection+3)%4;
      else if(key==mapbuttons::Down) systemSelection=(systemSelection+1)%4;
      else if(key==mapbuttons::Left) { systemMenu=false; drawBrowser(); return; }
      else if(key==mapbuttons::Right && systemSelection==0) { if(wifiEnabled) stopWifi(); else { wifiEnabled=true; startWifi(); } }
      else if(key==mapbuttons::Right && systemSelection==1) { nmeaScreen=true; mapbuttons::newContext(); drawNmeaScreen(); return; }
      else if(key==mapbuttons::Right && systemSelection==2) {
        if(gpsTrack.recording()) { message="Сначала выключите TRACK"; }
        else { startTestOnOpen=true; systemMenu=false; openSelected(); return; }
      }
      else if(key==mapbuttons::Right && systemSelection==3) { systemMenu=false; drawBrowser(); return; }
      systemScreen(); return;
    } else if(inMap) {
      if(key==mapbuttons::Up) view.panPixels(0,-MAP_H/4.0);
      else if(key==mapbuttons::Down) view.panPixels(0,MAP_H/4.0);
      else if(key==mapbuttons::Right) view.panPixels(MAP_W/4.0,0);
      else if(key==mapbuttons::Left) view.panPixels(-MAP_W/4.0,0);
      else if(key==mapbuttons::ZoomIn) view.zoomBy(1);
      else if(key==mapbuttons::ZoomOut) view.zoomBy(-1);
      if(redLayerActive) { clearRedRequested=true; redRefreshRequested=false; }
      dirty=true;
    } else {
      if(key==mapbuttons::Right) { openSelected(); return; }
      if(key==mapbuttons::Left) scanMaps();
      else if(key==mapbuttons::Up && selection>0) --selection;
      else if(key==mapbuttons::Down && selection+1<mapCount) ++selection;
      else if(key==mapbuttons::ZoomIn) selection=max(0,selection-ROWS);
      else if(key==mapbuttons::ZoomOut) selection=min(max(0,mapCount-1),selection+ROWS);
      dirty=true;
    }
  }
  const uint32_t dropped=mapbuttons::dropped.exchange(0);
  if(dropped) Serial.printf("BUTTON QUEUE FULL: dropped %lu events\n",(unsigned long)dropped);
  if(dirty) {
    if(inMap) {
      rerenderRequested=false; leaveMapRequested=false;
      if(!renderMap()) { Serial.println(message); toBrowser(true); }
      else if(nmeaScreen) { mapbuttons::newContext(); drawNmeaScreen(); return; }
      else if(leaveMapRequested) { message=""; toBrowser(); }
      else while(rerenderRequested && !leaveMapRequested) {
        rerenderRequested=false;
        if(!renderMap()) { Serial.println(message); toBrowser(true); break; }
        if(nmeaScreen) { mapbuttons::newContext(); drawNmeaScreen(); return; }
      }
    }
    else drawBrowser();
  }
  delay(10);
}
} // namespace mapapp
