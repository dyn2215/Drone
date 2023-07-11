// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: SITLGps.proto

#ifndef PROTOBUF_INCLUDED_SITLGps_2eproto
#define PROTOBUF_INCLUDED_SITLGps_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_SITLGps_2eproto 

namespace protobuf_SITLGps_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_SITLGps_2eproto
namespace sensor_msgs {
namespace msgs {
class SITLGps;
class SITLGpsDefaultTypeInternal;
extern SITLGpsDefaultTypeInternal _SITLGps_default_instance_;
}  // namespace msgs
}  // namespace sensor_msgs
namespace google {
namespace protobuf {
template<> ::sensor_msgs::msgs::SITLGps* Arena::CreateMaybeMessage<::sensor_msgs::msgs::SITLGps>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace sensor_msgs {
namespace msgs {

// ===================================================================

class SITLGps : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:sensor_msgs.msgs.SITLGps) */ {
 public:
  SITLGps();
  virtual ~SITLGps();

  SITLGps(const SITLGps& from);

  inline SITLGps& operator=(const SITLGps& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SITLGps(SITLGps&& from) noexcept
    : SITLGps() {
    *this = ::std::move(from);
  }

  inline SITLGps& operator=(SITLGps&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SITLGps& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SITLGps* internal_default_instance() {
    return reinterpret_cast<const SITLGps*>(
               &_SITLGps_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(SITLGps* other);
  friend void swap(SITLGps& a, SITLGps& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SITLGps* New() const final {
    return CreateMaybeMessage<SITLGps>(NULL);
  }

  SITLGps* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<SITLGps>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const SITLGps& from);
  void MergeFrom(const SITLGps& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(SITLGps* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int64 time_usec = 1;
  bool has_time_usec() const;
  void clear_time_usec();
  static const int kTimeUsecFieldNumber = 1;
  ::google::protobuf::int64 time_usec() const;
  void set_time_usec(::google::protobuf::int64 value);

  // optional int64 time_utc_usec = 2;
  bool has_time_utc_usec() const;
  void clear_time_utc_usec();
  static const int kTimeUtcUsecFieldNumber = 2;
  ::google::protobuf::int64 time_utc_usec() const;
  void set_time_utc_usec(::google::protobuf::int64 value);

  // required double latitude_deg = 3;
  bool has_latitude_deg() const;
  void clear_latitude_deg();
  static const int kLatitudeDegFieldNumber = 3;
  double latitude_deg() const;
  void set_latitude_deg(double value);

  // required double longitude_deg = 4;
  bool has_longitude_deg() const;
  void clear_longitude_deg();
  static const int kLongitudeDegFieldNumber = 4;
  double longitude_deg() const;
  void set_longitude_deg(double value);

  // required double altitude = 5;
  bool has_altitude() const;
  void clear_altitude();
  static const int kAltitudeFieldNumber = 5;
  double altitude() const;
  void set_altitude(double value);

  // optional double eph = 6;
  bool has_eph() const;
  void clear_eph();
  static const int kEphFieldNumber = 6;
  double eph() const;
  void set_eph(double value);

  // optional double epv = 7;
  bool has_epv() const;
  void clear_epv();
  static const int kEpvFieldNumber = 7;
  double epv() const;
  void set_epv(double value);

  // optional double velocity = 8;
  bool has_velocity() const;
  void clear_velocity();
  static const int kVelocityFieldNumber = 8;
  double velocity() const;
  void set_velocity(double value);

  // optional double velocity_east = 9;
  bool has_velocity_east() const;
  void clear_velocity_east();
  static const int kVelocityEastFieldNumber = 9;
  double velocity_east() const;
  void set_velocity_east(double value);

  // optional double velocity_north = 10;
  bool has_velocity_north() const;
  void clear_velocity_north();
  static const int kVelocityNorthFieldNumber = 10;
  double velocity_north() const;
  void set_velocity_north(double value);

  // optional double velocity_up = 11;
  bool has_velocity_up() const;
  void clear_velocity_up();
  static const int kVelocityUpFieldNumber = 11;
  double velocity_up() const;
  void set_velocity_up(double value);

  // @@protoc_insertion_point(class_scope:sensor_msgs.msgs.SITLGps)
 private:
  void set_has_time_usec();
  void clear_has_time_usec();
  void set_has_time_utc_usec();
  void clear_has_time_utc_usec();
  void set_has_latitude_deg();
  void clear_has_latitude_deg();
  void set_has_longitude_deg();
  void clear_has_longitude_deg();
  void set_has_altitude();
  void clear_has_altitude();
  void set_has_eph();
  void clear_has_eph();
  void set_has_epv();
  void clear_has_epv();
  void set_has_velocity();
  void clear_has_velocity();
  void set_has_velocity_east();
  void clear_has_velocity_east();
  void set_has_velocity_north();
  void clear_has_velocity_north();
  void set_has_velocity_up();
  void clear_has_velocity_up();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::int64 time_usec_;
  ::google::protobuf::int64 time_utc_usec_;
  double latitude_deg_;
  double longitude_deg_;
  double altitude_;
  double eph_;
  double epv_;
  double velocity_;
  double velocity_east_;
  double velocity_north_;
  double velocity_up_;
  friend struct ::protobuf_SITLGps_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SITLGps

// required int64 time_usec = 1;
inline bool SITLGps::has_time_usec() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SITLGps::set_has_time_usec() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SITLGps::clear_has_time_usec() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SITLGps::clear_time_usec() {
  time_usec_ = GOOGLE_LONGLONG(0);
  clear_has_time_usec();
}
inline ::google::protobuf::int64 SITLGps::time_usec() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.time_usec)
  return time_usec_;
}
inline void SITLGps::set_time_usec(::google::protobuf::int64 value) {
  set_has_time_usec();
  time_usec_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.time_usec)
}

// optional int64 time_utc_usec = 2;
inline bool SITLGps::has_time_utc_usec() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SITLGps::set_has_time_utc_usec() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SITLGps::clear_has_time_utc_usec() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SITLGps::clear_time_utc_usec() {
  time_utc_usec_ = GOOGLE_LONGLONG(0);
  clear_has_time_utc_usec();
}
inline ::google::protobuf::int64 SITLGps::time_utc_usec() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.time_utc_usec)
  return time_utc_usec_;
}
inline void SITLGps::set_time_utc_usec(::google::protobuf::int64 value) {
  set_has_time_utc_usec();
  time_utc_usec_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.time_utc_usec)
}

// required double latitude_deg = 3;
inline bool SITLGps::has_latitude_deg() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SITLGps::set_has_latitude_deg() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SITLGps::clear_has_latitude_deg() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SITLGps::clear_latitude_deg() {
  latitude_deg_ = 0;
  clear_has_latitude_deg();
}
inline double SITLGps::latitude_deg() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.latitude_deg)
  return latitude_deg_;
}
inline void SITLGps::set_latitude_deg(double value) {
  set_has_latitude_deg();
  latitude_deg_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.latitude_deg)
}

// required double longitude_deg = 4;
inline bool SITLGps::has_longitude_deg() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SITLGps::set_has_longitude_deg() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SITLGps::clear_has_longitude_deg() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void SITLGps::clear_longitude_deg() {
  longitude_deg_ = 0;
  clear_has_longitude_deg();
}
inline double SITLGps::longitude_deg() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.longitude_deg)
  return longitude_deg_;
}
inline void SITLGps::set_longitude_deg(double value) {
  set_has_longitude_deg();
  longitude_deg_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.longitude_deg)
}

// required double altitude = 5;
inline bool SITLGps::has_altitude() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SITLGps::set_has_altitude() {
  _has_bits_[0] |= 0x00000010u;
}
inline void SITLGps::clear_has_altitude() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void SITLGps::clear_altitude() {
  altitude_ = 0;
  clear_has_altitude();
}
inline double SITLGps::altitude() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.altitude)
  return altitude_;
}
inline void SITLGps::set_altitude(double value) {
  set_has_altitude();
  altitude_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.altitude)
}

// optional double eph = 6;
inline bool SITLGps::has_eph() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void SITLGps::set_has_eph() {
  _has_bits_[0] |= 0x00000020u;
}
inline void SITLGps::clear_has_eph() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void SITLGps::clear_eph() {
  eph_ = 0;
  clear_has_eph();
}
inline double SITLGps::eph() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.eph)
  return eph_;
}
inline void SITLGps::set_eph(double value) {
  set_has_eph();
  eph_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.eph)
}

// optional double epv = 7;
inline bool SITLGps::has_epv() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void SITLGps::set_has_epv() {
  _has_bits_[0] |= 0x00000040u;
}
inline void SITLGps::clear_has_epv() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void SITLGps::clear_epv() {
  epv_ = 0;
  clear_has_epv();
}
inline double SITLGps::epv() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.epv)
  return epv_;
}
inline void SITLGps::set_epv(double value) {
  set_has_epv();
  epv_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.epv)
}

// optional double velocity = 8;
inline bool SITLGps::has_velocity() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void SITLGps::set_has_velocity() {
  _has_bits_[0] |= 0x00000080u;
}
inline void SITLGps::clear_has_velocity() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void SITLGps::clear_velocity() {
  velocity_ = 0;
  clear_has_velocity();
}
inline double SITLGps::velocity() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.velocity)
  return velocity_;
}
inline void SITLGps::set_velocity(double value) {
  set_has_velocity();
  velocity_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.velocity)
}

// optional double velocity_east = 9;
inline bool SITLGps::has_velocity_east() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void SITLGps::set_has_velocity_east() {
  _has_bits_[0] |= 0x00000100u;
}
inline void SITLGps::clear_has_velocity_east() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void SITLGps::clear_velocity_east() {
  velocity_east_ = 0;
  clear_has_velocity_east();
}
inline double SITLGps::velocity_east() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.velocity_east)
  return velocity_east_;
}
inline void SITLGps::set_velocity_east(double value) {
  set_has_velocity_east();
  velocity_east_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.velocity_east)
}

// optional double velocity_north = 10;
inline bool SITLGps::has_velocity_north() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void SITLGps::set_has_velocity_north() {
  _has_bits_[0] |= 0x00000200u;
}
inline void SITLGps::clear_has_velocity_north() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void SITLGps::clear_velocity_north() {
  velocity_north_ = 0;
  clear_has_velocity_north();
}
inline double SITLGps::velocity_north() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.velocity_north)
  return velocity_north_;
}
inline void SITLGps::set_velocity_north(double value) {
  set_has_velocity_north();
  velocity_north_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.velocity_north)
}

// optional double velocity_up = 11;
inline bool SITLGps::has_velocity_up() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void SITLGps::set_has_velocity_up() {
  _has_bits_[0] |= 0x00000400u;
}
inline void SITLGps::clear_has_velocity_up() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void SITLGps::clear_velocity_up() {
  velocity_up_ = 0;
  clear_has_velocity_up();
}
inline double SITLGps::velocity_up() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.SITLGps.velocity_up)
  return velocity_up_;
}
inline void SITLGps::set_velocity_up(double value) {
  set_has_velocity_up();
  velocity_up_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.SITLGps.velocity_up)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace sensor_msgs

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_SITLGps_2eproto
