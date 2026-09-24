#pragma once

// One-bit, bounded-memory GeoJSON rendering for the left-hand map viewport.
// GFXcanvas1 convention here is 0 = black and 1 = white. The caller may invert
// the finished bitmap; gray is a spatial pattern, not a panel waveform.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>

namespace maprender {

constexpr int kWidth = 772;
constexpr int kHeight = 456;
constexpr int kMinZoom = 3;
constexpr int kMaxZoom = 20;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxLatitude = 85.0511287798066;

inline double clampNumber(double value, double lo, double hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

inline double worldY(double latitude, double worldSize) {
  const double r = clampNumber(latitude, -kMaxLatitude, kMaxLatitude) * kPi / 180.0;
  return (1.0 - std::log(std::tan(kPi / 4.0 + r / 2.0)) / kPi) * 0.5 * worldSize;
}

inline double latitudeFromY(double y, double worldSize) {
  return std::atan(std::sinh(kPi * (1.0 - 2.0 * y / worldSize))) * 180.0 / kPi;
}

struct View {
  double lon = 35.314591;
  double lat = 56.926029;
  int zoom = 13;

  void normalize() {
    if (!std::isfinite(lon)) lon = 35.314591;
    if (!std::isfinite(lat)) lat = 56.926029;
    lon = clampNumber(lon, -180.0, 180.0);
    lat = clampNumber(lat, -kMaxLatitude, kMaxLatitude);
    zoom = std::max(kMinZoom, std::min(kMaxZoom, zoom));
  }

  double worldSize() const {
    return std::ldexp(256.0, std::max(kMinZoom, std::min(kMaxZoom, zoom)));
  }

  // Positive dx moves east; positive dy moves south. There is no date-line wrap.
  void panPixels(double dx, double dy) {
    normalize();
    if (!std::isfinite(dx) || !std::isfinite(dy)) return;
    const double world = worldSize();
    lon += dx * 360.0 / world;
    lat = latitudeFromY(clampNumber(worldY(lat, world) + dy, 0.0, world), world);
    normalize();
  }

  void zoomBy(int delta) {
    zoom = std::max(kMinZoom, std::min(kMaxZoom, zoom + std::max(-20, std::min(20, delta))));
  }

  void bounds(double &minLon, double &minLat, double &maxLon, double &maxLat) const {
    View v = *this;
    v.normalize();
    const double world = v.worldSize();
    const double cy = worldY(v.lat, world);
    minLon = clampNumber(v.lon - kWidth * 180.0 / world, -180.0, 180.0);
    maxLon = clampNumber(v.lon + kWidth * 180.0 / world, -180.0, 180.0);
    minLat = latitudeFromY(clampNumber(cy + kHeight / 2.0, 0.0, world), world);
    maxLat = latitudeFromY(clampNumber(cy - kHeight / 2.0, 0.0, world), world);
  }
};

enum class Pass : uint8_t { Water, Building, Other, Road, Labels };

class Renderer {
 public:
  Renderer() = default;
  Renderer(const Renderer &) = delete;
  Renderer &operator=(const Renderer &) = delete;
  ~Renderer() { free(edges_); free(labels_); }

  // The caller clears the complete canvas and handles drawing order/UI.
  void begin(GFXcanvas1 &canvas, const View &view) {
    canvas_ = &canvas;
    view_ = view;
    view_.normalize();
    world_ = view_.worldSize();
    centerX_ = (view_.lon + 180.0) / 360.0 * world_;
    centerY_ = worldY(view_.lat, world_);
    labelCount_ = 0;
    skipped_ = 0;
    droppedLabels_ = 0;
    gpsReserved_ = false;
    pass_ = Pass::Other;
    font_.begin(canvas);
    font_.setFontMode(1);
    font_.setForegroundColor(0);
    font_.setBackgroundColor(1);
    font_.setFont(u8g2_font_6x12_t_cyrillic);
    if (!labels_) labels_ = static_cast<Label *>(allocate(sizeof(Label) * kMaxLabels));
  }

  void begin(GFXcanvas1 &canvas, double lon, double lat, int zoom) {
    View v;
    v.lon = lon;
    v.lat = lat;
    v.zoom = zoom;
    begin(canvas, v);
  }

  void setPass(Pass pass) { pass_ = pass; }
  void setSolidWater(bool solid) { solidWater_ = solid; }
  void setLabelEraseOnly(bool eraseOnly) { labelEraseOnly_ = eraseOnly; }
  uint32_t skippedGeometries() const { return skipped_; }
  uint32_t droppedLabels() const { return droppedLabels_; }

  // Geometry passes deliberately do not filter features: the index/caller
  // chooses a layer. Call again with Pass::Labels on the same parsed object.
  void feature(JsonObjectConst object) {
    if (!canvas_ || object.isNull()) return;
    const JsonObjectConst geometry = object["geometry"].as<JsonObjectConst>();
    const JsonObjectConst properties = object["properties"].as<JsonObjectConst>();
    if (geometry.isNull()) { ++skipped_; return; }
    if (pass_ == Pass::Labels) {
      collectLabel(properties, geometry);
      return;
    }
    const Style style = styleFor(properties);
    geometryDraw(geometry, style, 0);
    // Do not retain a large edge buffer while the next Feature is parsed.
    free(edges_);
    edges_ = nullptr;
    edgeCapacity_ = edgeCount_ = 0;
  }

  void drawGps(double longitude, double latitude) {
    double x, y;
    if (!canvas_ || !project(longitude, latitude, x, y) ||
        x < 0 || y < 0 || x >= kWidth || y >= kHeight) return;
    const int px = static_cast<int>(std::lround(x));
    const int py = static_cast<int>(std::lround(y));
    disk(px, py, 15, Shade::White);
    disk(px, py, 11, Shade::Black);
    disk(px, py, 7, Shade::White);
    disk(px, py, 4, Shade::Black);
    gpsRect_ = {px - 17, py - 17, px + 18, py + 18};
    gpsReserved_ = true;
  }
  bool screenPoint(double longitude,double latitude,int& x,int& y) const {
    double px,py; if(!project(longitude,latitude,px,py)) return false;
    x=(int)std::lround(px); y=(int)std::lround(py); return true;
  }
  void drawCrosshairAt(double longitude,double latitude) {
    double x,y; if(!canvas_ || !project(longitude,latitude,x,y)) return;
    int px=(int)std::lround(x), py=(int)std::lround(y);
    span(0,kWidth-1,py,Shade::Black); for(int yy=0;yy<kHeight;++yy) canvas_->drawPixel(px,yy,0);
  }
  void drawMarker(double longitude,double latitude) {
    double x,y; if(!canvas_ || !project(longitude,latitude,x,y)) return;
    int px=(int)std::lround(x), py=(int)std::lround(y);
    disk(px,py,15,Shade::White); disk(px,py,11,Shade::Black); disk(px,py,7,Shade::White); disk(px,py,4,Shade::Black);
  }

  void drawTrackSegment(GFXcanvas1& target,double lon0,double lat0,double lon1,double lat1) {
    double x0,y0,x1,y1;
    if(!project(lon0,lat0,x0,y0) || !project(lon1,lat1,x1,y1)) return;
    GFXcanvas1* saved=canvas_;
    canvas_=&target;
    Style style; style.stroke=Shade::Black; style.width=3;
    segment(x0,y0,x1,y1,style);
    canvas_=saved;
  }

  void drawRouteMark(double longitude, double latitude, double tangentLon,
                     double tangentLat, int number) {
    double x,y;
    if(!canvas_ || !project(longitude,latitude,x,y)) return;
    double tx=tangentLon, ty=-tangentLat;
    double length=std::sqrt(tx*tx+ty*ty); if(length<1e-12) return;
    tx/=length; ty/=length;
    double px=-ty*11.0, py=tx*11.0;
    // White halo keeps the kilometre tick readable over roads, rivers and
    // other dark map features.
    thinLine(x-px+tx*2.0,y-py+ty*2.0,x+px+tx*2.0,y+py+ty*2.0,Shade::White);
    thinLine(x-px-tx*2.0,y-py-ty*2.0,x+px-tx*2.0,y+py-ty*2.0,Shade::White);
    thinLine(x-px,y-py,x+px,y+py,Shade::Black);
    char label[12]; snprintf(label,sizeof(label),"%d",number);
    font_.setFont(u8g2_font_9x15_t_cyrillic);
    int width=font_.getUTF8Width(label), bx=(int)std::lround(x+px+4), by=(int)std::lround(y+py-2);
    bx=std::max(1,std::min(kWidth-width-3,bx)); by=std::max(13,std::min(kHeight-2,by));
    for(int yy=by-15;yy<=by+3;++yy) span(bx-3,bx+width+3,yy,Shade::White);
    // Draw a one-pixel white outline around the enlarged black label.
    font_.setForegroundColor(1);
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx)
      if(dx || dy) font_.drawUTF8(bx+dx,by+dy,label);
    font_.setForegroundColor(0);
    font_.drawUTF8(bx,by,label);
  }

  void finishLabels() {
    if (!canvas_ || !labels_) return;
    std::sort(labels_, labels_ + labelCount_, [](const Label &a, const Label &b) {
      if (a.priority != b.priority) return a.priority < b.priority;
      return a.sequence < b.sequence;
    });
    uint16_t accepted = 0;
    for (uint16_t i = 0; i < labelCount_; ++i) {
      Label &label = labels_[i];
      font_.setFont(u8g2_font_9x15_t_cyrillic);
      int width = font_.getUTF8Width(label.text);
      while (width > kWidth - 8 && label.text[0]) {
        removeLastUtf8(label.text);
        width = font_.getUTF8Width(label.text);
      }
      if (width <= 0) continue;
      const int textHeight = 15;
      const int x = std::max(2, std::min(kWidth - width - 3,
                      static_cast<int>(std::lround(label.x)) - width / 2));
      const int baseline = std::max(textHeight + 2, std::min(kHeight - 4,
                            static_cast<int>(std::lround(label.y)) - 3));
      const Rect rect = {x - 2, baseline - textHeight - 1, x + width + 3, baseline + 4};
      bool blocked = gpsReserved_ && intersects(rect, gpsRect_);
      for (uint16_t j = 0; !blocked && j < accepted; ++j) blocked = intersects(rect, accepted_[j]);
      if (blocked) continue;
      for (int y = rect.y0; y < rect.y1; ++y) span(rect.x0, rect.x1 - 1, y, Shade::White);
      if (!labelEraseOnly_) font_.drawUTF8(x, baseline, label.text);
      accepted_[accepted++] = rect;
    }
  }

 private:
  enum class Shade : uint8_t { Black, White, Gray25, Gray50 };
  struct Style {
    Shade stroke = Shade::Black;
    Shade fill = Shade::White;
    int width = 1;
    int pointRadius = 2;
    bool filled = false;
  };
  struct Edge { double ymin, ymax, x, slope; };
  struct Label { double x, y; uint16_t sequence; uint8_t priority; char text[128]; };
  struct Rect { int x0, y0, x1, y1; };
  static constexpr uint16_t kMaxLabels = 256;
  static constexpr size_t kMaxEdges = 32768;
  static constexpr size_t kMaxIntersections = 2048;
  GFXcanvas1 *canvas_ = nullptr;
  View view_;
  Pass pass_ = Pass::Other;
  double world_ = 0, centerX_ = 0, centerY_ = 0;
  U8G2_FOR_ADAFRUIT_GFX font_;
  Edge *edges_ = nullptr;
  size_t edgeCount_ = 0, edgeCapacity_ = 0;
  Label *labels_ = nullptr;
  uint16_t labelCount_ = 0;
  Rect accepted_[kMaxLabels];
  Rect gpsRect_ = {};
  bool gpsReserved_ = false;
  uint32_t skipped_ = 0, droppedLabels_ = 0;
  bool solidWater_ = false;
  bool labelEraseOnly_ = false;

  static void *allocate(size_t bytes) {
    void *memory = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return memory ? memory : heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
  }

  static bool same(const char *value, const char *expected) { return value && std::strcmp(value, expected) == 0; }
  static bool tag(const char *value) { return value && value[0] && !same(value, "no"); }
  static bool intersects(const Rect &a, const Rect &b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
  }

  bool project(double longitude, double latitude, double &x, double &y) const {
    if (!std::isfinite(longitude) || !std::isfinite(latitude) ||
        longitude < -180.0 || longitude > 180.0 || latitude < -90.0 || latitude > 90.0) return false;
    x = (longitude + 180.0) / 360.0 * world_ - centerX_ + kWidth / 2.0;
    y = worldY(latitude, world_) - centerY_ + kHeight / 2.0;
    return std::isfinite(x) && std::isfinite(y);
  }

  bool coordinate(JsonVariantConst value, double &x, double &y) const {
    JsonArrayConst coordinate = value.as<JsonArrayConst>();
    if (coordinate.size() < 2 || !coordinate[0].is<double>() || !coordinate[1].is<double>()) return false;
    return project(coordinate[0].as<double>(), coordinate[1].as<double>(), x, y);
  }

  Style styleFor(JsonObjectConst properties) const {
    Style result;
    const char *building = properties["building"] | "";
    const char *waterway = properties["waterway"] | "";
    const char *natural = properties["natural"] | "";
    const char *landuse = properties["landuse"] | "";
    const char *water = properties["water"] | "";
    const char *highway = properties["highway"] | "";
    const char *place = properties["place"] | "";
    const double scale = std::pow(1.4, view_.zoom - 16);
    if (tag(building)) {
      result.filled = true;
      result.fill = Shade::Gray25;
    } else if (tag(waterway) || same(natural, "water") || same(natural, "wetland") ||
               same(landuse, "reservoir") || same(landuse, "basin") || tag(water)) {
      result.filled = true;
      result.fill = Shade::Black;
      result.stroke = Shade::Black;
      const int base = same(waterway, "river") ? 9 : (same(waterway, "canal") ? 7 : 4);
      result.width = static_cast<int>(clampNumber(std::round(base * scale), 3, 30));
    } else if (tag(highway)) {
      result.stroke = Shade::Gray50;
      result.width = 3;
    }
    // Small point-only maps need a marker that remains visible on e-paper.
    // Labels continue to be rendered in the separate label pass.
    if (tag(place)) result.pointRadius = 3;
    return result;
  }

  void span(int x0, int x1, int y, Shade shade) {
    if (y < 0 || y >= kHeight || x1 < 0 || x0 >= kWidth || x0 > x1) return;
    x0 = std::max(0, x0);
    x1 = std::min(kWidth - 1, x1);
    if (shade == Shade::Black || shade == Shade::White) {
      canvas_->drawFastHLine(x0, y, x1 - x0 + 1, shade == Shade::White ? 1 : 0);
      return;
    }
    canvas_->drawFastHLine(x0, y, x1 - x0 + 1, 1);
    if (shade == Shade::Gray25 && (y & 1)) return;
    const int phase = shade == Shade::Gray50 ? (y & 1) : ((y >> 1) & 1);
    for (int x = x0 + ((x0 ^ phase) & 1); x <= x1; x += 2) canvas_->drawPixel(x, y, 0);
  }

  void disk(int cx, int cy, int radius, Shade shade) {
    for (int dy = -radius; dy <= radius; ++dy) {
      if (cy + dy < 0 || cy + dy >= kHeight) continue;
      const int half = static_cast<int>(std::sqrt(static_cast<double>(radius * radius - dy * dy)));
      span(cx - half, cx + half, cy + dy, shade);
    }
  }

  static bool clip(double &x0, double &y0, double &x1, double &y1, double margin = 0) {
    if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1)) return false;
    const double dx = x1 - x0, dy = y1 - y0;
    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {x0 + margin, kWidth - 1.0 + margin - x0,
                         y0 + margin, kHeight - 1.0 + margin - y0};
    double low = 0, high = 1;
    for (int i = 0; i < 4; ++i) {
      if (p[i] == 0) { if (q[i] < 0) return false; continue; }
      const double r = q[i] / p[i];
      if (p[i] < 0) low = std::max(low, r); else high = std::min(high, r);
      if (low > high) return false;
    }
    x1 = x0 + high * dx;
    y1 = y0 + high * dy;
    x0 += low * dx;
    y0 += low * dy;
    return true;
  }

  void thinLine(double x0, double y0, double x1, double y1, Shade shade) {
    if (!clip(x0, y0, x1, y1)) return;
    int x = static_cast<int>(std::lround(x0)), y = static_cast<int>(std::lround(y0));
    const int endX = static_cast<int>(std::lround(x1)), endY = static_cast<int>(std::lround(y1));
    if (shade == Shade::Black || shade == Shade::White) {
      canvas_->drawLine(x, y, endX, endY, shade == Shade::White ? 1 : 0);
      return;
    }
    const int dx = std::abs(endX - x), dy = -std::abs(endY - y);
    const int sx = x < endX ? 1 : -1, sy = y < endY ? 1 : -1;
    int error = dx + dy;
    for (;;) {
      span(x, x, y, shade);
      if (x == endX && y == endY) break;
      const int twice = 2 * error;
      if (twice >= dy) { error += dy; x += sx; }
      if (twice <= dx) { error += dx; y += sy; }
    }
  }

  void segment(double x0, double y0, double x1, double y1, const Style &style) {
    if (style.width <= 1) { thinLine(x0, y0, x1, y1, style.stroke); return; }
    const int radius = style.width / 2;
    if (!clip(x0, y0, x1, y1, radius)) return;
    int x = static_cast<int>(std::lround(x0)), y = static_cast<int>(std::lround(y0));
    const int endX = static_cast<int>(std::lround(x1)), endY = static_cast<int>(std::lround(y1));
    const int dx = std::abs(endX - x), dy = -std::abs(endY - y);
    const int sx = x < endX ? 1 : -1, sy = y < endY ? 1 : -1;
    int error = dx + dy;
    for (;;) {
      disk(x, y, radius, style.stroke);
      if (x == endX && y == endY) break;
      const int twice = 2 * error;
      if (twice >= dy) { error += dy; x += sx; }
      if (twice <= dx) { error += dx; y += sy; }
    }
  }

  void line(JsonArrayConst points, const Style &style, bool close) {
    double previousX = 0, previousY = 0, firstX = 0, firstY = 0;
    bool previousValid = false, firstValid = false, broken = false;
    size_t visited = 0;
    for (JsonVariantConst point : points) {
      if ((++visited & 127) == 0) yield();
      double x, y;
      if (!coordinate(point, x, y)) { previousValid = false; broken = true; continue; }
      if (!firstValid) { firstX = x; firstY = y; firstValid = true; }
      if (previousValid) segment(previousX, previousY, x, y, style);
      previousX = x; previousY = y; previousValid = true;
    }
    if (close && !broken && firstValid && previousValid) segment(previousX, previousY, firstX, firstY, style);
    if (broken || !firstValid) ++skipped_;
  }

  bool appendEdge(double x0, double y0, double x1, double y1) {
    if (y0 == y1) return true;
    if (y1 < y0) { std::swap(x0, x1); std::swap(y0, y1); }
    if (y1 <= 0 || y0 >= kHeight) return true;
    if (edgeCount_ == kMaxEdges) return false;
    if (edgeCount_ == edgeCapacity_) {
      const size_t capacity = std::min(kMaxEdges, edgeCapacity_ ? edgeCapacity_ * 2 : size_t(128));
      Edge *memory = static_cast<Edge *>(allocate(capacity * sizeof(Edge)));
      if (!memory) return false;
      if (edges_) { std::memcpy(memory, edges_, edgeCount_ * sizeof(Edge)); free(edges_); }
      edges_ = memory;
      edgeCapacity_ = capacity;
    }
    const double slope = (x1 - x0) / (y1 - y0);
    const double ymin = std::max(0.0, y0), ymax = std::min(double(kHeight), y1);
    const double x = x0 + (ymin - y0) * slope;
    if (!std::isfinite(slope) || !std::isfinite(x)) return false;
    edges_[edgeCount_++] = {ymin, ymax, x, slope};
    return true;
  }

  bool polygonEdges(JsonArrayConst rings) {
    edgeCount_ = 0;
    if (rings.isNull()) return false;
    for (JsonVariantConst ringValue : rings) {
      const JsonArrayConst ring = ringValue.as<JsonArrayConst>();
      if (ring.size() < 3) return false;
      double previousX = 0, previousY = 0, firstX = 0, firstY = 0;
      bool first = true;
      for (JsonVariantConst point : ring) {
        double x, y;
        if (!coordinate(point, x, y)) return false;
        if (first) { firstX = x; firstY = y; first = false; }
        else if (!appendEdge(previousX, previousY, x, y)) return false;
        previousX = x; previousY = y;
      }
      if (!first && !appendEdge(previousX, previousY, firstX, firstY)) return false;
    }
    return true;
  }

  bool fillPolygon(Shade shade) {
    if (!edgeCount_) return true;
    double firstY = kHeight, lastY = 0;
    for (size_t i = 0; i < edgeCount_; ++i) {
      firstY = std::min(firstY, edges_[i].ymin);
      lastY = std::max(lastY, edges_[i].ymax);
    }
    const int firstRow = std::max(0, static_cast<int>(std::ceil(firstY - 0.5)));
    const int lastRow = std::min(kHeight - 1, static_cast<int>(std::ceil(lastY - 0.5)) - 1);
    // Heap allocation avoids a 16 KiB temporary on the Arduino loop-task stack.
    double *crossings = static_cast<double *>(allocate(std::min(edgeCount_, kMaxIntersections) * sizeof(double)));
    if (!crossings) return false;
    bool okay = true;
    // Preflight the intersection cap before drawing any fill, so exhaustion
    // leaves a complete outline rather than a partly filled polygon.
    for (int y = firstRow; y <= lastRow && okay; ++y) {
      const double scanY = y + 0.5;
      size_t count = 0;
      for (size_t i = 0; i < edgeCount_; ++i) {
        const Edge &edge = edges_[i];
        if (scanY >= edge.ymin && scanY < edge.ymax && ++count > kMaxIntersections) { okay = false; break; }
      }
    }
    for (int y = firstRow; y <= lastRow && okay; ++y) {
      const double scanY = y + 0.5;
      size_t count = 0;
      for (size_t i = 0; i < edgeCount_; ++i) {
        const Edge &edge = edges_[i];
        if (scanY >= edge.ymin && scanY < edge.ymax) crossings[count++] = edge.x + (scanY - edge.ymin) * edge.slope;
      }
      std::sort(crossings, crossings + count);
      for (size_t i = 0; i + 1 < count; i += 2) {
        const double left = std::max(0.0, crossings[i]);
        const double right = std::min(double(kWidth), crossings[i + 1]);
        if (left >= right) continue;
        // Clip in floating point before converting to screen integers.
        const int x0 = static_cast<int>(std::ceil(left - 0.5));
        const int x1 = static_cast<int>(std::ceil(right - 0.5)) - 1;
        span(x0, x1, y, shade);
      }
      if ((y & 31) == 0) yield();
    }
    free(crossings);
    return okay;
  }

  void polygon(JsonArrayConst rings, const Style &style) {
    if (style.filled && (!polygonEdges(rings) || !fillPolygon(style.fill))) ++skipped_;
    Style outline = style;
    // Polygon outlines stay fine, including coastlines; waterway widths apply
    // only to linear rivers, streams and canals.
    outline.width = 1;
    for (JsonVariantConst ring : rings) line(ring.as<JsonArrayConst>(), outline, true);
  }

  void geometryDraw(JsonObjectConst geometry, const Style &style, unsigned depth) {
    if (depth > 4) { ++skipped_; return; }
    const char *type = geometry["type"] | "";
    JsonArrayConst coordinates = geometry["coordinates"].as<JsonArrayConst>();
    if (same(type, "Point")) {
      double x, y;
      if (!coordinate(geometry["coordinates"], x, y)) { ++skipped_; return; }
      if (x >= 0 && x < kWidth && y >= 0 && y < kHeight)
        disk(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)),
             style.pointRadius, style.stroke);
    } else if (same(type, "MultiPoint")) {
      for (JsonVariantConst point : coordinates) {
        double x, y;
        if (!coordinate(point, x, y)) { ++skipped_; continue; }
        if (x >= 0 && x < kWidth && y >= 0 && y < kHeight)
          disk(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)),
               style.pointRadius, style.stroke);
      }
    } else if (same(type, "LineString")) line(coordinates, style, false);
    else if (same(type, "MultiLineString")) {
      for (JsonVariantConst points : coordinates) line(points.as<JsonArrayConst>(), style, false);
    } else if (same(type, "Polygon")) polygon(coordinates, style);
    else if (same(type, "MultiPolygon")) {
      for (JsonVariantConst rings : coordinates) polygon(rings.as<JsonArrayConst>(), style);
    } else if (same(type, "GeometryCollection")) {
      for (JsonObjectConst child : geometry["geometries"].as<JsonArrayConst>()) geometryDraw(child, style, depth + 1);
    } else ++skipped_;
  }

  static void removeLastUtf8(char *text) {
    size_t length = std::strlen(text);
    if (!length) return;
    --length;
    while (length && (static_cast<uint8_t>(text[length]) & 0xC0) == 0x80) --length;
    text[length] = 0;
  }

  static void copyUtf8(char *destination, size_t capacity, const char *source) {
    size_t length = std::strlen(source);
    if (length >= capacity) {
      length = capacity - 1;
      while (length && (static_cast<uint8_t>(source[length]) & 0xC0) == 0x80) --length;
    }
    std::memcpy(destination, source, length);
    destination[length] = 0;
  }

  bool anchorBounds(JsonVariantConst coordinates, double &xmin, double &ymin,
                    double &xmax, double &ymax, unsigned depth) const {
    if (depth > 4) return false;
    JsonArrayConst array = coordinates.as<JsonArrayConst>();
    if (array.isNull()) return false;
    if (array.size() >= 2 && array[0].is<double>()) {
      double x, y;
      if (!coordinate(coordinates, x, y)) return false;
      xmin = std::min(xmin, x); ymin = std::min(ymin, y);
      xmax = std::max(xmax, x); ymax = std::max(ymax, y);
      return true;
    }
    bool found = false;
    for (JsonVariantConst child : array) found = anchorBounds(child, xmin, ymin, xmax, ymax, depth + 1) || found;
    return found;
  }

  void collectLabel(JsonObjectConst properties, JsonObjectConst geometry) {
    const char *place = properties["place"] | "";
    const char *name = properties["name:ru"] | "";
    if (!name[0]) name = properties["name"] | "";
    if (!tag(place) || !name[0]) return;
    uint8_t priority = 255;
    if (same(place, "city")) priority = 0;
    else if (same(place, "town")) priority = 1;
    else if (same(place, "village")) priority = 2;
    else if (same(place, "hamlet")) priority = 3;
    else if (same(place, "isolated_dwelling")) priority = 4;
    else if (same(place, "suburb") || same(place, "neighbourhood") || same(place, "quarter")) priority = 5;
    else if (same(place, "locality")) priority = 6;
    if (priority == 255) return;
    if (!labels_) { ++droppedLabels_; return; }
    double xmin = INFINITY, ymin = INFINITY, xmax = -INFINITY, ymax = -INFINITY;
    if (!anchorBounds(geometry["coordinates"], xmin, ymin, xmax, ymax, 0)) return;
    const double x = xmin + (xmax - xmin) * 0.5, y = ymin + (ymax - ymin) * 0.5;
    if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || x >= kWidth || y < 0 || y >= kHeight) return;
    for (uint16_t i = 0; i < labelCount_; ++i)
      if (std::strcmp(labels_[i].text, name) == 0 && std::abs(labels_[i].x - x) < 20 && std::abs(labels_[i].y - y) < 20) return;
    uint16_t target = labelCount_;
    if (labelCount_ == kMaxLabels) {
      target = 0;
      for (uint16_t i = 1; i < labelCount_; ++i)
        if (labels_[i].priority > labels_[target].priority) target = i;
      ++droppedLabels_;
      if (priority >= labels_[target].priority) return;
    } else ++labelCount_;
    Label &label = labels_[target];
    label.x = x;
    label.y = y;
    label.priority = priority;
    label.sequence = target;
    copyUtf8(label.text, sizeof(label.text), name);
  }
};

}  // namespace maprender
