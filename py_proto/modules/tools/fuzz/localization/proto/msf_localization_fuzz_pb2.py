# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/tools/fuzz/localization/proto/msf_localization_fuzz.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.drivers.gnss.proto import imu_pb2 as modules_dot_drivers_dot_gnss_dot_proto_dot_imu__pb2
from modules.drivers.gnss.proto import gnss_best_pose_pb2 as modules_dot_drivers_dot_gnss_dot_proto_dot_gnss__best__pose__pb2
from modules.drivers.gnss.proto import gnss_raw_observation_pb2 as modules_dot_drivers_dot_gnss_dot_proto_dot_gnss__raw__observation__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/tools/fuzz/localization/proto/msf_localization_fuzz.proto',
  package='apollo.tools.fuzz.localization',
  syntax='proto2',
  serialized_pb=_b('\nAmodules/tools/fuzz/localization/proto/msf_localization_fuzz.proto\x12\x1e\x61pollo.tools.fuzz.localization\x1a$modules/drivers/gnss/proto/imu.proto\x1a/modules/drivers/gnss/proto/gnss_best_pose.proto\x1a\x35modules/drivers/gnss/proto/gnss_raw_observation.proto\"\xf5\x01\n\x1aMSFLocalizationFuzzMessage\x12%\n\x03imu\x18\x01 \x02(\x0b\x32\x18.apollo.drivers.gnss.Imu\x12\x39\n\x0egnss_best_pose\x18\x02 \x02(\x0b\x32!.apollo.drivers.gnss.GnssBestPose\x12;\n\x0cgnss_rtk_obs\x18\x03 \x02(\x0b\x32%.apollo.drivers.gnss.EpochObservation\x12\x38\n\x0cgnss_rtk_eph\x18\x04 \x02(\x0b\x32\".apollo.drivers.gnss.GnssEphemeris')
  ,
  dependencies=[modules_dot_drivers_dot_gnss_dot_proto_dot_imu__pb2.DESCRIPTOR,modules_dot_drivers_dot_gnss_dot_proto_dot_gnss__best__pose__pb2.DESCRIPTOR,modules_dot_drivers_dot_gnss_dot_proto_dot_gnss__raw__observation__pb2.DESCRIPTOR,])




_MSFLOCALIZATIONFUZZMESSAGE = _descriptor.Descriptor(
  name='MSFLocalizationFuzzMessage',
  full_name='apollo.tools.fuzz.localization.MSFLocalizationFuzzMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='imu', full_name='apollo.tools.fuzz.localization.MSFLocalizationFuzzMessage.imu', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gnss_best_pose', full_name='apollo.tools.fuzz.localization.MSFLocalizationFuzzMessage.gnss_best_pose', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gnss_rtk_obs', full_name='apollo.tools.fuzz.localization.MSFLocalizationFuzzMessage.gnss_rtk_obs', index=2,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gnss_rtk_eph', full_name='apollo.tools.fuzz.localization.MSFLocalizationFuzzMessage.gnss_rtk_eph', index=3,
      number=4, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=244,
  serialized_end=489,
)

_MSFLOCALIZATIONFUZZMESSAGE.fields_by_name['imu'].message_type = modules_dot_drivers_dot_gnss_dot_proto_dot_imu__pb2._IMU
_MSFLOCALIZATIONFUZZMESSAGE.fields_by_name['gnss_best_pose'].message_type = modules_dot_drivers_dot_gnss_dot_proto_dot_gnss__best__pose__pb2._GNSSBESTPOSE
_MSFLOCALIZATIONFUZZMESSAGE.fields_by_name['gnss_rtk_obs'].message_type = modules_dot_drivers_dot_gnss_dot_proto_dot_gnss__raw__observation__pb2._EPOCHOBSERVATION
_MSFLOCALIZATIONFUZZMESSAGE.fields_by_name['gnss_rtk_eph'].message_type = modules_dot_drivers_dot_gnss_dot_proto_dot_gnss__raw__observation__pb2._GNSSEPHEMERIS
DESCRIPTOR.message_types_by_name['MSFLocalizationFuzzMessage'] = _MSFLOCALIZATIONFUZZMESSAGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MSFLocalizationFuzzMessage = _reflection.GeneratedProtocolMessageType('MSFLocalizationFuzzMessage', (_message.Message,), dict(
  DESCRIPTOR = _MSFLOCALIZATIONFUZZMESSAGE,
  __module__ = 'modules.tools.fuzz.localization.proto.msf_localization_fuzz_pb2'
  # @@protoc_insertion_point(class_scope:apollo.tools.fuzz.localization.MSFLocalizationFuzzMessage)
  ))
_sym_db.RegisterMessage(MSFLocalizationFuzzMessage)


# @@protoc_insertion_point(module_scope)
