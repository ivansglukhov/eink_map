#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <math.h>

namespace maproute {
struct Point { double lon, lat; };

class Route {
 public:
  ~Route() { clear(); }
  void clear() { if(points_) heap_caps_free(points_); points_=nullptr; count_=0; segment_=0; direction_=1; }
  bool load(fs::FS& fs, const String& path) {
    clear(); File f=fs.open(path,FILE_READ); if(!f) { error_="Маршрут не найден"; return false; }
    PsramAllocator alloc;
    JsonDocument doc(&alloc);
    DeserializationError e=deserializeJson(doc,f); f.close();
    if(e) { error_=String("Маршрут: ")+e.c_str(); return false; }
    JsonArrayConst features=doc["features"].as<JsonArrayConst>();
    if(features.isNull()) { error_="Маршрут без features"; return false; }
    JsonArrayConst coords;
    for(JsonObjectConst feature:features) {
      JsonObjectConst geometry=feature["geometry"].as<JsonObjectConst>();
      if(!strcmp(geometry["type"]|"", "LineString")) { coords=geometry["coordinates"].as<JsonArrayConst>(); break; }
    }
    if(coords.isNull() || coords.size()<2 || coords.size()>2048) { error_="Нужен LineString из 2..2048 точек"; return false; }
    points_=static_cast<Point*>(heap_caps_malloc(coords.size()*sizeof(Point),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    if(!points_) { error_="Нет PSRAM для маршрута"; return false; }
    for(JsonVariantConst value:coords) { JsonArrayConst p=value.as<JsonArrayConst>(); if(p.size()<2) { clear(); error_="Некорректная точка маршрута"; return false; } points_[count_++]={p[0].as<double>(),p[1].as<double>()}; }
    reset(); return true;
  }
  void reset() { segment_=0; direction_=1; progress_=0; current_=points_?points_[0]:Point{}; lastMs_=millis(); }
  bool valid() const { return count_>=2; }
  const String& error() const { return error_; }
  Point current() const { return current_; }
  int markCount() const { int n=0; for(size_t i=0;i+1<count_;++i) { if(totalTo(i+1)>=1000.0*(n+1)) ++n; } return n; }
  bool markAt(int number, Point& point, Point& tangent) const {
    if(number<1) return false; double target=number*1000.0, walked=0;
    for(size_t i=0;i+1<count_;++i) { Point a=points_[i],b=points_[i+1]; double d=distance(a,b); if(walked+d>=target) { double t=(target-walked)/d; point={a.lon+(b.lon-a.lon)*t,a.lat+(b.lat-a.lat)*t}; tangent={b.lon-a.lon,b.lat-a.lat}; return true; } walked+=d; }
    return false;
  }
  void update(uint32_t now, double kmh) {
    if(!valid()) return; uint32_t elapsed=now-lastMs_; lastMs_=now; advanceMeters(kmh*1000.0*elapsed/3600000.0);
  }
  void advanceMeters(double metres) {
    if(!valid() || metres<=0) return;
    while(metres>0 && valid()) {
      Point a=direction_>0?points_[segment_]:points_[segment_+1];
      Point b=direction_>0?points_[segment_+1]:points_[segment_];
      double full=distance(a,b), remaining=full* (direction_>0 ? 1.0-progress_ : progress_);
      if(remaining<=0.001) { advanceEndpoint(); continue; }
      double step=metres<remaining?metres:remaining; double fraction=step/full;
      if(direction_>0) progress_+=fraction; else progress_-=fraction;
      double t=direction_>0?progress_:1.0-progress_;
      current_={a.lon+(b.lon-a.lon)*t,a.lat+(b.lat-a.lat)*t}; metres-=step;
      if(step>=remaining-0.001) advanceEndpoint();
    }
  }
 private:
  class PsramAllocator:public ArduinoJson::Allocator { public: void* allocate(size_t n) override{return heap_caps_malloc(n,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);} void deallocate(void*p) override{heap_caps_free(p);} void* reallocate(void*p,size_t n) override{return heap_caps_realloc(p,n,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);} };
  static double distance(Point a,Point b) { const double r=6371000.0, p=0.017453292519943295; double dlat=(b.lat-a.lat)*p,dlon=(b.lon-a.lon)*p; double q=sin(dlat/2)*sin(dlat/2)+cos(a.lat*p)*cos(b.lat*p)*sin(dlon/2)*sin(dlon/2); return 2*r*atan2(sqrt(q),sqrt(1-q)); }
  double totalTo(size_t end) const { double d=0; for(size_t i=0;i<end&&i+1<count_;++i)d+=distance(points_[i],points_[i+1]); return d; }
  void advanceEndpoint() { if(direction_>0) { if(segment_+1>=count_-1){direction_=-1; progress_=1;} else {++segment_;progress_=0;} } else { if(segment_==0){direction_=1;progress_=0;} else {--segment_;progress_=1;} } }
  Point* points_=nullptr,current_{}; size_t count_=0,segment_=0; int direction_=1; double progress_=0; uint32_t lastMs_=0; String error_;
};
}
