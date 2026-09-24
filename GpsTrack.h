#pragma once

#include <Arduino.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace gpstrack {

struct Point { double lon, lat; };

class Track {
 public:
  ~Track() { clearPoints(); }

  bool start(fs::FS& fs) {
    if(recording_) return true;
    clearPoints(); error_=""; finalPath_=""; tempPath_=""; fs_=&fs;
    for(unsigned n=1;n<=9999;++n) {
      char name[40]; snprintf(name,sizeof(name),"/maps/track_%04u.geojson",n);
      String candidate=name;
      String part=candidate+".part";
      if(!fs.exists(candidate.c_str()) && !fs.exists(part.c_str())) { finalPath_=candidate; break; }
    }
    if(!finalPath_.length()) return fail("Нет свободного имени трека");
    tempPath_=finalPath_+".part";
    file_=fs.open(tempPath_.c_str(),FILE_WRITE);
    if(!file_) return fail("Не удалось создать файл трека");
    if(!writeText("{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\",\"properties\":{},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[")) return false;
    recording_=true;
    return true;
  }

  bool append(double lon,double lat) {
    if(!recording_ || !std::isfinite(lon) || !std::isfinite(lat) ||
       lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0) return false;
    Point point{lon,lat};
    double metres=0.0;
    if(count_) {
      metres=distance(points_[count_-1],point);
      if(metres<3.0) return false;
    }
    if(!reserve(count_+1)) { fail("Нет памяти для точек трека"); return false; }
    char coordinate[64];
    const int length=snprintf(coordinate,sizeof(coordinate),count_ ? ",[%.7f,%.7f]" : "[%.7f,%.7f]",lon,lat);
    if(length<0 || length>=static_cast<int>(sizeof(coordinate)) || !writeText(coordinate)) return false;
    distanceMetres_+=metres;
    points_[count_++]=point;
    return true;
  }

  bool stop() {
    if(!recording_) { clearPoints(); return true; }
    if(!writeText("]}}]}")) return false;
    file_.close(); recording_=false;
    if(!fs_->rename(tempPath_.c_str(),finalPath_.c_str())) return fail("Не удалось завершить файл трека");
    return true;
  }

  // Preserve the incomplete .part file for recovery, but stop accepting points.
  void abort() {
    if(file_) { file_.flush(); file_.close(); }
    recording_=false;
  }

  void clearPoints() {
    if(points_) heap_caps_free(points_);
    points_=nullptr; count_=capacity_=0; distanceMetres_=0;
  }
  bool recording() const { return recording_; }
  size_t count() const { return count_; }
  const Point& point(size_t index) const { return points_[index]; }
  double distanceMetres() const { return distanceMetres_; }
  const String& path() const { return finalPath_; }
  const String& error() const { return error_; }

 private:
  bool fail(const String& error) { error_=error; return false; }
  bool writeText(const char* text) {
    if(!file_) { abort(); return fail("Файл трека недоступен"); }
    file_.clearWriteError();
    const size_t expected=std::strlen(text);
    const size_t written=file_.print(text);
    file_.flush();
    if(written!=expected || file_.getWriteError()) {
      abort();
      return fail("Ошибка записи трека на SD");
    }
    return true;
  }
  bool reserve(size_t wanted) {
    if(wanted<=capacity_) return true;
    size_t next=capacity_?capacity_*2:256;
    while(next<wanted) next*=2;
    if(next>8192) next=8192;
    if(wanted>next) return false;
    void* memory=heap_caps_realloc(points_,next*sizeof(Point),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!memory) return false;
    points_=static_cast<Point*>(memory); capacity_=next; return true;
  }
  static double distance(Point a,Point b) {
    constexpr double radians=0.017453292519943295, earth=6371000.0;
    const double dlat=(b.lat-a.lat)*radians, dlon=(b.lon-a.lon)*radians;
    const double q=std::sin(dlat/2)*std::sin(dlat/2)+std::cos(a.lat*radians)*std::cos(b.lat*radians)*std::sin(dlon/2)*std::sin(dlon/2);
    return 2*earth*std::atan2(std::sqrt(q),std::sqrt(std::max(0.0,1.0-q)));
  }

  fs::FS* fs_=nullptr;
  File file_;
  String finalPath_,tempPath_,error_;
  Point* points_=nullptr;
  size_t count_=0,capacity_=0;
  double distanceMetres_=0;
  bool recording_=false;
};

} // namespace gpstrack
