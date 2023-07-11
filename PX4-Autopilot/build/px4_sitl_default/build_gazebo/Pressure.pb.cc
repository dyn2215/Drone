// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Pressure.proto

#include "Pressure.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace sensor_msgs {
namespace msgs {
class PressureDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Pressure>
      _instance;
} _Pressure_default_instance_;
}  // namespace msgs
}  // namespace sensor_msgs
namespace protobuf_Pressure_2eproto {
static void InitDefaultsPressure() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::sensor_msgs::msgs::_Pressure_default_instance_;
    new (ptr) ::sensor_msgs::msgs::Pressure();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::sensor_msgs::msgs::Pressure::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_Pressure =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsPressure}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Pressure.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::sensor_msgs::msgs::Pressure, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::sensor_msgs::msgs::Pressure, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::sensor_msgs::msgs::Pressure, time_usec_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::sensor_msgs::msgs::Pressure, temperature_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::sensor_msgs::msgs::Pressure, absolute_pressure_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::sensor_msgs::msgs::Pressure, pressure_altitude_),
  0,
  1,
  2,
  3,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::sensor_msgs::msgs::Pressure)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::sensor_msgs::msgs::_Pressure_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "Pressure.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\016Pressure.proto\022\020sensor_msgs.msgs\"h\n\010Pr"
      "essure\022\021\n\ttime_usec\030\001 \002(\003\022\023\n\013temperature"
      "\030\002 \002(\002\022\031\n\021absolute_pressure\030\003 \002(\002\022\031\n\021pre"
      "ssure_altitude\030\004 \002(\002"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 140);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Pressure.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_Pressure_2eproto
namespace sensor_msgs {
namespace msgs {

// ===================================================================

void Pressure::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Pressure::kTimeUsecFieldNumber;
const int Pressure::kTemperatureFieldNumber;
const int Pressure::kAbsolutePressureFieldNumber;
const int Pressure::kPressureAltitudeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Pressure::Pressure()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_Pressure_2eproto::scc_info_Pressure.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:sensor_msgs.msgs.Pressure)
}
Pressure::Pressure(const Pressure& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&time_usec_, &from.time_usec_,
    static_cast<size_t>(reinterpret_cast<char*>(&pressure_altitude_) -
    reinterpret_cast<char*>(&time_usec_)) + sizeof(pressure_altitude_));
  // @@protoc_insertion_point(copy_constructor:sensor_msgs.msgs.Pressure)
}

void Pressure::SharedCtor() {
  ::memset(&time_usec_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&pressure_altitude_) -
      reinterpret_cast<char*>(&time_usec_)) + sizeof(pressure_altitude_));
}

Pressure::~Pressure() {
  // @@protoc_insertion_point(destructor:sensor_msgs.msgs.Pressure)
  SharedDtor();
}

void Pressure::SharedDtor() {
}

void Pressure::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Pressure::descriptor() {
  ::protobuf_Pressure_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_Pressure_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Pressure& Pressure::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_Pressure_2eproto::scc_info_Pressure.base);
  return *internal_default_instance();
}


void Pressure::Clear() {
// @@protoc_insertion_point(message_clear_start:sensor_msgs.msgs.Pressure)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 15u) {
    ::memset(&time_usec_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&pressure_altitude_) -
        reinterpret_cast<char*>(&time_usec_)) + sizeof(pressure_altitude_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Pressure::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:sensor_msgs.msgs.Pressure)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required int64 time_usec = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          set_has_time_usec();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int64, ::google::protobuf::internal::WireFormatLite::TYPE_INT64>(
                 input, &time_usec_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float temperature = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {
          set_has_temperature();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &temperature_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float absolute_pressure = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {
          set_has_absolute_pressure();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &absolute_pressure_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required float pressure_altitude = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(37u /* 37 & 0xFF */)) {
          set_has_pressure_altitude();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &pressure_altitude_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:sensor_msgs.msgs.Pressure)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:sensor_msgs.msgs.Pressure)
  return false;
#undef DO_
}

void Pressure::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:sensor_msgs.msgs.Pressure)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required int64 time_usec = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt64(1, this->time_usec(), output);
  }

  // required float temperature = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->temperature(), output);
  }

  // required float absolute_pressure = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->absolute_pressure(), output);
  }

  // required float pressure_altitude = 4;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->pressure_altitude(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:sensor_msgs.msgs.Pressure)
}

::google::protobuf::uint8* Pressure::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:sensor_msgs.msgs.Pressure)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required int64 time_usec = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt64ToArray(1, this->time_usec(), target);
  }

  // required float temperature = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->temperature(), target);
  }

  // required float absolute_pressure = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->absolute_pressure(), target);
  }

  // required float pressure_altitude = 4;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->pressure_altitude(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:sensor_msgs.msgs.Pressure)
  return target;
}

size_t Pressure::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:sensor_msgs.msgs.Pressure)
  size_t total_size = 0;

  if (has_time_usec()) {
    // required int64 time_usec = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int64Size(
        this->time_usec());
  }

  if (has_temperature()) {
    // required float temperature = 2;
    total_size += 1 + 4;
  }

  if (has_absolute_pressure()) {
    // required float absolute_pressure = 3;
    total_size += 1 + 4;
  }

  if (has_pressure_altitude()) {
    // required float pressure_altitude = 4;
    total_size += 1 + 4;
  }

  return total_size;
}
size_t Pressure::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:sensor_msgs.msgs.Pressure)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x0000000f) ^ 0x0000000f) == 0) {  // All required fields are present.
    // required int64 time_usec = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int64Size(
        this->time_usec());

    // required float temperature = 2;
    total_size += 1 + 4;

    // required float absolute_pressure = 3;
    total_size += 1 + 4;

    // required float pressure_altitude = 4;
    total_size += 1 + 4;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Pressure::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:sensor_msgs.msgs.Pressure)
  GOOGLE_DCHECK_NE(&from, this);
  const Pressure* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Pressure>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:sensor_msgs.msgs.Pressure)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:sensor_msgs.msgs.Pressure)
    MergeFrom(*source);
  }
}

void Pressure::MergeFrom(const Pressure& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:sensor_msgs.msgs.Pressure)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      time_usec_ = from.time_usec_;
    }
    if (cached_has_bits & 0x00000002u) {
      temperature_ = from.temperature_;
    }
    if (cached_has_bits & 0x00000004u) {
      absolute_pressure_ = from.absolute_pressure_;
    }
    if (cached_has_bits & 0x00000008u) {
      pressure_altitude_ = from.pressure_altitude_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void Pressure::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:sensor_msgs.msgs.Pressure)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Pressure::CopyFrom(const Pressure& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:sensor_msgs.msgs.Pressure)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Pressure::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000000f) != 0x0000000f) return false;
  return true;
}

void Pressure::Swap(Pressure* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Pressure::InternalSwap(Pressure* other) {
  using std::swap;
  swap(time_usec_, other->time_usec_);
  swap(temperature_, other->temperature_);
  swap(absolute_pressure_, other->absolute_pressure_);
  swap(pressure_altitude_, other->pressure_altitude_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Pressure::GetMetadata() const {
  protobuf_Pressure_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_Pressure_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace sensor_msgs
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::sensor_msgs::msgs::Pressure* Arena::CreateMaybeMessage< ::sensor_msgs::msgs::Pressure >(Arena* arena) {
  return Arena::CreateInternal< ::sensor_msgs::msgs::Pressure >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)