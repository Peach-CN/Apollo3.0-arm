# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/tools/navi_generator/proto/trajectory_util_config.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/tools/navi_generator/proto/trajectory_util_config.proto',
  package='apollo.navi_generator',
  syntax='proto2',
  serialized_pb=_b('\n?modules/tools/navi_generator/proto/trajectory_util_config.proto\x12\x15\x61pollo.navi_generator\"\x90\x01\n\x18TrajectorySmootherConfig\x12X\n\x18smoother_config_filename\x18\x01 \x01(\t:6modules/planning/conf/qp_spline_smoother_config.pb.txt\x12\x1a\n\rsmooth_length\x18\x02 \x01(\x01:\x03\x32\x30\x30\"`\n\x14TrajectoryUtilConfig\x12H\n\x0fsmoother_config\x18\x01 \x01(\x0b\x32/.apollo.navi_generator.TrajectorySmootherConfig')
)




_TRAJECTORYSMOOTHERCONFIG = _descriptor.Descriptor(
  name='TrajectorySmootherConfig',
  full_name='apollo.navi_generator.TrajectorySmootherConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='smoother_config_filename', full_name='apollo.navi_generator.TrajectorySmootherConfig.smoother_config_filename', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("modules/planning/conf/qp_spline_smoother_config.pb.txt").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='smooth_length', full_name='apollo.navi_generator.TrajectorySmootherConfig.smooth_length', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(200),
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
  serialized_start=91,
  serialized_end=235,
)


_TRAJECTORYUTILCONFIG = _descriptor.Descriptor(
  name='TrajectoryUtilConfig',
  full_name='apollo.navi_generator.TrajectoryUtilConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='smoother_config', full_name='apollo.navi_generator.TrajectoryUtilConfig.smoother_config', index=0,
      number=1, type=11, cpp_type=10, label=1,
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
  serialized_start=237,
  serialized_end=333,
)

_TRAJECTORYUTILCONFIG.fields_by_name['smoother_config'].message_type = _TRAJECTORYSMOOTHERCONFIG
DESCRIPTOR.message_types_by_name['TrajectorySmootherConfig'] = _TRAJECTORYSMOOTHERCONFIG
DESCRIPTOR.message_types_by_name['TrajectoryUtilConfig'] = _TRAJECTORYUTILCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrajectorySmootherConfig = _reflection.GeneratedProtocolMessageType('TrajectorySmootherConfig', (_message.Message,), dict(
  DESCRIPTOR = _TRAJECTORYSMOOTHERCONFIG,
  __module__ = 'modules.tools.navi_generator.proto.trajectory_util_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.TrajectorySmootherConfig)
  ))
_sym_db.RegisterMessage(TrajectorySmootherConfig)

TrajectoryUtilConfig = _reflection.GeneratedProtocolMessageType('TrajectoryUtilConfig', (_message.Message,), dict(
  DESCRIPTOR = _TRAJECTORYUTILCONFIG,
  __module__ = 'modules.tools.navi_generator.proto.trajectory_util_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.TrajectoryUtilConfig)
  ))
_sym_db.RegisterMessage(TrajectoryUtilConfig)


# @@protoc_insertion_point(module_scope)
