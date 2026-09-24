#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

// Binary indexes are local cache files, not a portable interchange format.
// They are regenerated when the format, source bytes, or checksums change.
namespace mapindex {

using Progress = void (*)(const char*, uint32_t, uint32_t, uint32_t);
using Cancel = bool (*)();

struct BBox {
  double minLon, minLat, maxLon, maxLat;
  bool intersects(const BBox& b) const {
    return maxLon >= b.minLon && minLon <= b.maxLon &&
           maxLat >= b.minLat && minLat <= b.maxLat;
  }
};

enum Geometry : uint32_t {
  Unknown = 0, Point = 1, LineString = 2, MultiLineString = 3,
  Polygon = 4, MultiPolygon = 5, MultiPoint = 6
};
enum Flag : uint32_t { Building = 1, Road = 2, Water = 4, Place = 8, MajorRoad = 16 };

struct Record {
  uint32_t offset, length;
  BBox bbox;
  uint32_t geometry, flags;
};

struct Header {
  char magic[8];
  uint32_t version, sourceSize, sourceCrc, count, recordBytes, recordsCrc;
  uint32_t totalFeatures, skippedFeatures;
  BBox bbox;
  uint32_t headerCrc, reserved;
};
static_assert(sizeof(Record) == 48, "Unexpected index record layout");
static_assert(sizeof(Header) == 80, "Unexpected index header layout");

// ArduinoJson 7 allocates individual pools through this allocator. A failed
// allocation becomes DeserializationError::NoMemory, never a partial index.
class PsramAllocator : public ArduinoJson::Allocator {
 public:
  explicit PsramAllocator(size_t limit = 5U * 1024U * 1024U) : limit_(limit) {}
  void* allocate(size_t bytes) override {
    if (bytes > limit_ - used_ || bytes > SIZE_MAX - sizeof(Block)) return nullptr;
    auto* block = static_cast<Block*>(
        heap_caps_malloc(bytes + sizeof(Block), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!block) return nullptr;
    block->size = bytes;
    used_ += bytes;
    return block + 1;
  }
  void deallocate(void* ptr) override {
    if (!ptr) return;
    auto* block = static_cast<Block*>(ptr) - 1;
    used_ -= block->size;
    heap_caps_free(block);
  }
  void* reallocate(void* ptr, size_t bytes) override {
    if (!ptr) return allocate(bytes);
    if (!bytes) { deallocate(ptr); return nullptr; }
    auto* block = static_cast<Block*>(ptr) - 1;
    size_t previous = block->size;
    if (bytes > limit_ - (used_ - previous) || bytes > SIZE_MAX - sizeof(Block))
      return nullptr;
    auto* replacement = static_cast<Block*>(heap_caps_realloc(
        block, bytes + sizeof(Block), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!replacement) return nullptr;
    replacement->size = bytes;
    used_ = used_ - previous + bytes;
    return replacement + 1;
  }
  size_t used() const { return used_; }
 private:
  struct alignas(8) Block { size_t size; };
  size_t limit_, used_ = 0;
};

namespace detail {
inline uint32_t crc32(uint32_t crc, const void* data, size_t size) {
  static const uint32_t nibble[16] = {
    0x00000000,0x1DB71064,0x3B6E20C8,0x26D930AC,
    0x76DC4190,0x6B6B51F4,0x4DB26158,0x5005713C,
    0xEDB88320,0xF00F9344,0xD6D6A3E8,0xCB61B38C,
    0x9B64C2B0,0x86D3D2D4,0xA00AE278,0xBDBDF21C
  };
  auto* p = static_cast<const uint8_t*>(data);
  while (size--) {
    crc ^= *p++;
    crc = (crc >> 4) ^ nibble[crc & 15];
    crc = (crc >> 4) ^ nibble[crc & 15];
  }
  return crc;
}
inline BBox emptyBBox() { return {180.0, 90.0, -180.0, -90.0}; }
inline bool validBBox(const BBox& b) {
  return isfinite(b.minLon) && isfinite(b.minLat) &&
         isfinite(b.maxLon) && isfinite(b.maxLat) &&
         b.minLon <= b.maxLon && b.minLat <= b.maxLat &&
         b.minLon >= -180 && b.maxLon <= 180 && b.minLat >= -90 && b.maxLat <= 90;
}
inline void extend(BBox& target, const BBox& b) {
  if (b.minLon < target.minLon) target.minLon = b.minLon;
  if (b.minLat < target.minLat) target.minLat = b.minLat;
  if (b.maxLon > target.maxLon) target.maxLon = b.maxLon;
  if (b.maxLat > target.maxLat) target.maxLat = b.maxLat;
}

// Separately opened File handles let the scanner keep its 2 KB read-ahead
// while an individual Feature is deserialized by seeking the source handle.
class Scanner {
 public:
  explicit Scanner(File& file) : file_(file) {}
  int peek() {
    if (at_ == size_) {
      size_ = file_.read(buffer_, sizeof(buffer_));
      at_ = 0;
    }
    return at_ < size_ ? buffer_[at_] : -1;
  }
  int read() {
    int c = peek();
    if (c >= 0) { ++at_; ++position_; }
    return c;
  }
  uint32_t position() const { return position_; }
  void whitespace() {
    int c;
    while ((c = peek()) == ' ' || c == '\t' || c == '\r' || c == '\n') read();
  }
  bool take(int expected) { whitespace(); return read() == expected; }
  bool string(String* output = nullptr) {
    if (read() != '"') return false;
    if (output) *output = "";
    for (;;) {
      int c = read();
      if (c == '"') return true;
      if (c < 0x20) return false;
      if (c == '\\') {
        c = read();
        switch (c) {
          case '"': case '\\': case '/': break;
          case 'b': c = '\b'; break;
          case 'f': c = '\f'; break;
          case 'n': c = '\n'; break;
          case 'r': c = '\r'; break;
          case 't': c = '\t'; break;
          case 'u': {
            unsigned value = 0;
            for (int i = 0; i < 4; ++i) {
              int digit = read();
              if (digit >= '0' && digit <= '9') digit -= '0';
              else if (digit >= 'a' && digit <= 'f') digit = digit - 'a' + 10;
              else if (digit >= 'A' && digit <= 'F') digit = digit - 'A' + 10;
              else return false;
              value = (value << 4) | unsigned(digit);
            }
            // Only the ASCII FeatureCollection/type/features keys are needed.
            c = value < 128 ? int(value) : '?';
            break;
          }
          default: return false;
        }
      }
      if (output) {
        if (output->length() >= 128) return false;
        *output += char(c);
      }
    }
  }
  bool value(unsigned depth = 0) {
    if (depth > 32) return false;
    whitespace();
    int c = peek();
    if (c == '"') return string();
    if (c == '{' || c == '[') {
      bool object = read() == '{';
      int end = object ? '}' : ']';
      whitespace();
      if (peek() == end) { read(); return true; }
      for (;;) {
        if (object) {
          whitespace();
          if (!string() || !take(':')) return false;
        }
        if (!value(depth + 1)) return false;
        whitespace();
        c = read();
        if (c == end) return true;
        if (c != ',') return false;
      }
    }
    const char* literal = c == 't' ? "true" : c == 'f' ? "false" : c == 'n' ? "null" : nullptr;
    if (literal) {
      while (*literal) if (read() != *literal++) return false;
      return true;
    }
    if (c == '-') { read(); c = peek(); }
    if (c == '0') read();
    else {
      if (c < '1' || c > '9') return false;
      do { read(); c = peek(); } while (c >= '0' && c <= '9');
    }
    if (peek() == '.') {
      read(); c = peek();
      if (c < '0' || c > '9') return false;
      do { read(); c = peek(); } while (c >= '0' && c <= '9');
    }
    if (peek() == 'e' || peek() == 'E') {
      read(); c = peek();
      if (c == '+' || c == '-') { read(); c = peek(); }
      if (c < '0' || c > '9') return false;
      do { read(); c = peek(); } while (c >= '0' && c <= '9');
    }
    return true;
  }
 private:
  File& file_;
  uint8_t buffer_[2048];
  size_t at_ = 0, size_ = 0;
  uint32_t position_ = 0;
};

class FeatureReader {
 public:
  FeatureReader(File& file, uint32_t size) : file_(file), remaining_(size) {}
  int read() {
    if (at_ == size_) {
      if (!remaining_) return -1;
      size_t wanted = remaining_ < sizeof(buffer_) ? remaining_ : sizeof(buffer_);
      size_ = file_.read(buffer_, wanted);
      at_ = 0;
      remaining_ -= size_;
      if (!size_) return -1;
    }
    return buffer_[at_++];
  }
  size_t readBytes(char* output, size_t length) {
    size_t done = 0;
    while (done < length) {
      int c = read();
      if (c < 0) break;
      output[done++] = char(c);
    }
    return done;
  }
 private:
  File& file_;
  uint32_t remaining_;
  uint8_t buffer_[1024];
  size_t at_ = 0, size_ = 0;
};

inline bool coordinateBBox(JsonVariantConst value, unsigned depth, BBox& bbox,
                           uint32_t& positions) {
  if (!value.is<JsonArrayConst>()) return false;
  JsonArrayConst array = value.as<JsonArrayConst>();
  if (depth) {
    for (JsonVariantConst child : array)
      if (!coordinateBBox(child, depth - 1, bbox, positions)) return false;
    return true;
  }
  if (array.size() < 2 || !array[0].is<double>() || !array[1].is<double>()) return false;
  double lon = array[0].as<double>(), lat = array[1].as<double>();
  if (!isfinite(lon) || !isfinite(lat) || lon < -180 || lon > 180 || lat < -90 || lat > 90)
    return false;
  extend(bbox, {lon, lat, lon, lat});
  ++positions;
  return true;
}
inline bool tag(JsonObjectConst props, const char* key) {
  JsonVariantConst v = props[key];
  const char* text = v.as<const char*>();
  if (text) return *text && strcmp(text, "no") && strcmp(text, "false") && strcmp(text, "0");
  return v.is<bool>() && v.as<bool>();
}
}  // namespace detail

class Index {
 public:
  bool open(fs::FS& filesystem, const String& path, Progress progress = nullptr,
            Cancel cancel = nullptr) {
    close();
    error_ = "";
    fs_ = &filesystem;
    path_ = path;
    progress_ = progress;
    cancel_ = cancel;
    source_ = fs_->open(path_, FILE_READ);
    if (!source_ || source_.isDirectory()) return fail("Cannot open GeoJSON");
    if (!source_.size() || uint64_t(source_.size()) > UINT32_MAX)
      return fail("GeoJSON must be between 1 byte and 4 GB");
    sourceSize_ = uint32_t(source_.size());
    uint32_t fingerprint;
    if (!hashSource(fingerprint)) return false;
    indexPath_ = path_;
    int dot = indexPath_.lastIndexOf('.');
    if (dot >= 0) indexPath_.remove(dot);
    indexPath_ += ".gidx";
    index_ = fs_->open(indexPath_, FILE_READ);
    if (index_ && validate(fingerprint)) return rewind();
    index_.close();
    if (isCancelled()) return fail("Cancelled");
    error_ = "";  // An invalid cache is rebuilt; source errors remain fatal.
    if (!build(fingerprint)) return false;
    index_ = fs_->open(indexPath_, FILE_READ);
    if (!index_ || !validate(fingerprint)) return fail("New index could not be verified");
    return rewind();
  }
  void close() {
    source_.close();
    index_.close();
    header_ = {};
    cursor_ = 0;
    bufferedRecords_ = bufferCursor_ = 0;
    error_ = "";
  }
  bool rewind() {
    if (!index_ || !index_.seek(sizeof(Header))) return fail("Cannot seek index");
    cursor_ = 0;
    bufferedRecords_ = bufferCursor_ = 0;
    return true;
  }
  bool next(Record& record) {
    if (cursor_ >= header_.count) return false;
    if (!readRecord(record) || !validRecord(record)) return fail("Index record is damaged");
    ++cursor_;
    return true;
  }
  bool load(const Record& record, JsonDocument& document) {
    document.clear();
    if (!validRecord(record)) return fail("Invalid feature offset or bounds");
    // The DOM allocator has an independent 5 MB cap; text length is a second
    // guard against spending minutes parsing one oversized OSM relation.
    if (record.length > 4U * 1024U * 1024U)
      return fail(String("Feature exceeds 4 MB at byte ") + record.offset);
    if (!source_.seek(record.offset)) return fail("Cannot seek GeoJSON feature");
    detail::FeatureReader reader(source_, record.length);
    DeserializationError result = deserializeJson(
        document, reader, DeserializationOption::NestingLimit(32));
    if (result) {
      document.clear();
      return fail(String("Feature at byte ") + record.offset + ": " + result.c_str());
    }
    return true;
  }
  const Header& header() const { return header_; }
  const String& error() const { return error_; }
  const String& path() const { return path_; }

 private:
  static constexpr uint32_t VERSION = 2;
  bool fail(const String& message) { error_ = message; return false; }
  bool isCancelled() const { return cancel_ && cancel_(); }
  void report(const char* stage, uint32_t done, uint32_t total, uint32_t count) {
    if (progress_) progress_(stage, done, total, count);
    yield();
  }
  bool validRecord(const Record& r) const {
    return r.length && r.offset < sourceSize_ && r.length <= sourceSize_ - r.offset &&
           // MajorRoad is bit 4 (16), so all currently defined flag bits are
           // 0..4. Rejecting it made a just-built index fail verification.
           r.geometry >= Point && r.geometry <= MultiPoint && !(r.flags & ~31U) &&
           detail::validBBox(r.bbox);
  }
  bool readRecord(Record& record) {
    if (bufferCursor_ == bufferedRecords_) {
      size_t bytes = index_.read(reinterpret_cast<uint8_t*>(recordBuffer_), sizeof(recordBuffer_));
      if (!bytes || bytes % sizeof(Record)) return false;
      bufferedRecords_ = bytes / sizeof(Record);
      bufferCursor_ = 0;
    }
    record = recordBuffer_[bufferCursor_++];
    return true;
  }
  bool hashSource(uint32_t& result) {
    if (!source_.seek(0)) return fail("Cannot seek GeoJSON");
    uint8_t buffer[4096];
    uint32_t done = 0, crc = UINT32_MAX;
    report("Checking map", 0, sourceSize_, 0);
    while (done < sourceSize_) {
      if (isCancelled()) return fail("Cancelled");
      size_t wanted = sourceSize_ - done;
      if (wanted > sizeof(buffer)) wanted = sizeof(buffer);
      size_t amount = source_.read(buffer, wanted);
      if (amount != wanted) return fail("SD read failed while checking GeoJSON");
      crc = detail::crc32(crc, buffer, amount);
      done += amount;
      if ((done & 65535U) == 0 || done == sourceSize_)
        report("Checking map", done, sourceSize_, 0);
    }
    result = ~crc;
    return true;
  }
  bool validate(uint32_t fingerprint) {
    Header candidate{};
    if (index_.read(reinterpret_cast<uint8_t*>(&candidate), sizeof(candidate)) != sizeof(candidate))
      return false;
    if (memcmp(candidate.magic, "GEOIDX1", 8) || candidate.version != VERSION ||
        candidate.recordBytes != sizeof(Record) || candidate.reserved ||
        candidate.sourceSize != sourceSize_ || candidate.sourceCrc != fingerprint ||
        uint64_t(candidate.count) + candidate.skippedFeatures != candidate.totalFeatures ||
        uint64_t(sizeof(Header)) + uint64_t(candidate.count) * sizeof(Record) != index_.size())
      return false;
    uint32_t expected = candidate.headerCrc;
    candidate.headerCrc = 0;
    if (~detail::crc32(UINT32_MAX, &candidate, sizeof(candidate)) != expected) return false;
    candidate.headerCrc = expected;
    bufferedRecords_ = bufferCursor_ = 0;
    uint32_t recordsCrc = UINT32_MAX, previousEnd = 0;
    BBox bounds = detail::emptyBBox();
    for (uint32_t i = 0; i < candidate.count; ++i) {
      if (isCancelled()) return false;
      Record record{};
      if (!readRecord(record) || !validRecord(record) || record.offset < previousEnd) return false;
      previousEnd = record.offset + record.length;
      recordsCrc = detail::crc32(recordsCrc, &record, sizeof(record));
      detail::extend(bounds, record.bbox);
      if ((i & 255U) == 0) report("Checking index", i, candidate.count, i);
    }
    if (~recordsCrc != candidate.recordsCrc) return false;
    if (!candidate.count) bounds = {0, 0, 0, 0};
    if (memcmp(&bounds, &candidate.bbox, sizeof(bounds))) return false;
    header_ = candidate;
    report("Checking index", header_.count, header_.count, header_.count);
    return true;
  }
  bool describe(JsonDocument& doc, Record& record, bool& supported) {
    supported = false;
    JsonObjectConst feature = doc.as<JsonObjectConst>();
    if (feature.isNull() || strcmp(feature["type"] | "", "Feature"))
      return fail("Expected a GeoJSON Feature object");
    JsonVariantConst geometryValue = feature["geometry"];
    if (geometryValue.isNull()) return true;
    if (!geometryValue.is<JsonObjectConst>()) return fail("Feature geometry must be an object or null");
    JsonObjectConst geometry = geometryValue.as<JsonObjectConst>();
    const char* type = geometry["type"] | "";
    unsigned depth;
    if (!strcmp(type, "Point")) { record.geometry = Point; depth = 0; }
    else if (!strcmp(type, "LineString")) { record.geometry = LineString; depth = 1; }
    else if (!strcmp(type, "MultiLineString")) { record.geometry = MultiLineString; depth = 2; }
    else if (!strcmp(type, "Polygon")) { record.geometry = Polygon; depth = 2; }
    else if (!strcmp(type, "MultiPolygon")) { record.geometry = MultiPolygon; depth = 3; }
    else if (!strcmp(type, "MultiPoint")) { record.geometry = MultiPoint; depth = 1; }
    else if (!strcmp(type, "GeometryCollection")) return true;
    else return fail("Unknown GeoJSON geometry type");
    uint32_t positions = 0;
    record.bbox = detail::emptyBBox();
    if (!detail::coordinateBBox(geometry["coordinates"], depth, record.bbox, positions))
      return fail(String("Invalid coordinates at byte ") + record.offset);
    if (!positions) return true;
    JsonObjectConst properties = feature["properties"].as<JsonObjectConst>();
    if (detail::tag(properties, "building")) record.flags |= Building;
    if (detail::tag(properties, "highway")) {
      record.flags |= Road;
      const char* highway=properties["highway"] | "";
      if(!strcmp(highway,"motorway") || !strcmp(highway,"trunk") ||
         !strcmp(highway,"primary") || !strcmp(highway,"secondary")) record.flags |= MajorRoad;
    }
    if (detail::tag(properties, "waterway") || detail::tag(properties, "water") ||
        !strcmp(properties["natural"] | "", "water") ||
        !strcmp(properties["natural"] | "", "wetland") ||
        !strcmp(properties["natural"] | "", "bay")) record.flags |= Water;
    if (detail::tag(properties, "place")) record.flags |= Place;
    supported = true;
    return true;
  }
  bool build(uint32_t fingerprint) {
    String temporary = indexPath_ + ".tmp";
    String backup = indexPath_ + ".bak";
    if (fs_->exists(temporary) && !fs_->remove(temporary))
      return fail("Cannot remove interrupted index file");
    File output = fs_->open(temporary, FILE_WRITE);
    if (!output) return fail("Cannot create index: check SD free space");
    Header building{};
    memcpy(building.magic, "GEOIDX1", 8);
    building.version = VERSION;
    building.sourceSize = sourceSize_;
    building.sourceCrc = fingerprint;
    building.recordBytes = sizeof(Record);
    building.bbox = detail::emptyBBox();
    bool ok = output.write(reinterpret_cast<const uint8_t*>(&building), sizeof(building)) == sizeof(building);
    if (!ok) fail("Cannot write index header");
    File scanFile = fs_->open(path_, FILE_READ);
    if (!scanFile) { ok = false; fail("Cannot open GeoJSON scanner"); }
    if (ok) {
      detail::Scanner scanner(scanFile);
      PsramAllocator allocator;
      JsonDocument document(&allocator);
      uint32_t recordsCrc = UINT32_MAX;
      bool seenFeatures = false, seenType = false;
      ok = scanner.take('{');
      report("Building index", 0, sourceSize_, 0);
      scanner.whitespace();
      if (ok && scanner.peek() == '}') { scanner.read(); ok = false; }
      while (ok) {
        if (isCancelled()) { ok = fail("Cancelled"); break; }
        String key;
        scanner.whitespace();
        if (!scanner.string(&key) || !scanner.take(':')) { ok = false; break; }
        scanner.whitespace();
        if (key == "type") {
          String type;
          if (seenType || !scanner.string(&type) || type != "FeatureCollection") { ok = false; break; }
          seenType = true;
        } else if (key == "features") {
          if (seenFeatures || !scanner.take('[')) { ok = false; break; }
          seenFeatures = true;
          scanner.whitespace();
          if (scanner.peek() == ']') scanner.read();
          else for (;;) {
            if (isCancelled()) { ok = fail("Cancelled"); break; }
            scanner.whitespace();
            Record record{};
            record.offset = scanner.position();
            if (scanner.peek() != '{' || !scanner.value()) { ok = false; break; }
            record.length = scanner.position() - record.offset;
            // load() also verifies bbox and type: use temporary valid values
            // until the Feature has been decoded and its actual bounds computed.
            record.geometry = Point;
            record.bbox = {0, 0, 0, 0};
            if (!load(record, document)) { ok = false; break; }
            bool supported;
            if (!describe(document, record, supported)) { ok = false; break; }
            ++building.totalFeatures;
            if (supported) {
              if (output.write(reinterpret_cast<const uint8_t*>(&record), sizeof(record)) != sizeof(record)) {
                ok = fail("Index write failed: SD may be full"); break;
              }
              recordsCrc = detail::crc32(recordsCrc, &record, sizeof(record));
              detail::extend(building.bbox, record.bbox);
              ++building.count;
            } else ++building.skippedFeatures;
            document.clear();
            report("Building index", scanner.position(), sourceSize_, building.count);
            scanner.whitespace();
            int delimiter = scanner.read();
            if (delimiter == ']') break;
            if (delimiter != ',') { ok = false; break; }
          }
          if (!ok) break;
        } else if (!scanner.value()) { ok = false; break; }
        scanner.whitespace();
        int delimiter = scanner.read();
        if (delimiter == '}') break;
        if (delimiter != ',') { ok = false; break; }
      }
      scanner.whitespace();
      if (ok && (!seenType || !seenFeatures || scanner.peek() != -1 ||
                 scanner.position() != sourceSize_)) ok = false;
      if (!ok && error_.isEmpty())
        fail(String("Invalid/truncated GeoJSON near byte ") + scanner.position());
      if (ok) {
        if (!building.count) building.bbox = {0, 0, 0, 0};
        building.recordsCrc = ~recordsCrc;
        building.headerCrc = ~detail::crc32(UINT32_MAX, &building, sizeof(building));
        if (!output.seek(0) ||
            output.write(reinterpret_cast<const uint8_t*>(&building), sizeof(building)) != sizeof(building))
          ok = fail("Cannot finalize index header");
      }
    }
    output.flush();
    output.close();
    scanFile.close();
    if (!ok) { fs_->remove(temporary); return false; }
    // Validate the completed temp file before replacing any prior cache.
    index_ = fs_->open(temporary, FILE_READ);
    ok = index_ && validate(fingerprint);
    index_.close();
    if (!ok) { fs_->remove(temporary); return fail(isCancelled() ? "Cancelled" : "Temporary index verification failed"); }
    if (fs_->exists(backup) && !fs_->remove(backup)) return fail("Cannot remove old index backup");
    bool hadPrevious = fs_->exists(indexPath_);
    if (hadPrevious && !fs_->rename(indexPath_, backup)) return fail("Cannot back up old index");
    if (!fs_->rename(temporary, indexPath_)) {
      if (hadPrevious) fs_->rename(backup, indexPath_);
      return fail("Cannot publish index");
    }
    if (hadPrevious) fs_->remove(backup);
    report("Index ready", sourceSize_, sourceSize_, building.count);
    return true;
  }
  fs::FS* fs_ = nullptr;
  File source_, index_;
  String path_, indexPath_, error_;
  Header header_{};
  Record recordBuffer_[32];
  size_t bufferedRecords_ = 0, bufferCursor_ = 0;
  uint32_t sourceSize_ = 0, cursor_ = 0;
  Progress progress_ = nullptr;
  Cancel cancel_ = nullptr;
};

}  // namespace mapindex
