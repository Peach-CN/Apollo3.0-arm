# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/poly_st_speed_config.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.planning.proto import st_boundary_config_pb2 as modules_dot_planning_dot_proto_dot_st__boundary__config__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/planning/proto/poly_st_speed_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n1modules/planning/proto/poly_st_speed_config.proto\x12\x0f\x61pollo.planning\x1a/modules/planning/proto/st_boundary_config.proto\"\xdb\x02\n\x11PolyStSpeedConfig\x12\x19\n\x11total_path_length\x18\x01 \x01(\x01\x12\x12\n\ntotal_time\x18\x02 \x01(\x01\x12\x17\n\x0fpreferred_accel\x18\x03 \x01(\x01\x12\x17\n\x0fpreferred_decel\x18\x04 \x01(\x01\x12\x11\n\tmax_accel\x18\x05 \x01(\x01\x12\x11\n\tmin_decel\x18\x06 \x01(\x01\x12\x1a\n\x12speed_limit_buffer\x18\x07 \x01(\x01\x12\x14\n\x0cspeed_weight\x18\x08 \x01(\x01\x12\x13\n\x0bjerk_weight\x18\t \x01(\x01\x12\x17\n\x0fobstacle_weight\x18\n \x01(\x01\x12 \n\x18unblocking_obstacle_cost\x18\x0b \x01(\x01\x12=\n\x12st_boundary_config\x18\x0c \x01(\x0b\x32!.apollo.planning.StBoundaryConfig')
  ,
  dependencies=[modules_dot_planning_dot_proto_dot_st__boundary__config__pb2.DESCRIPTOR,])




_POLYSTSPEEDCONFIG = _descriptor.Descriptor(
  name='PolyStSpeedConfig',
  full_name='apollo.planning.PolyStSpeedConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='total_path_length', full_name='apollo.planning.PolyStSpeedConfig.total_path_length', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='total_time', full_name='apollo.planning.PolyStSpeedConfig.total_time', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='preferred_accel', full_name='apollo.planning.PolyStSpeedConfig.preferred_accel', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='preferred_decel', full_name='apollo.planning.PolyStSpeedConfig.preferred_decel', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_accel', full_name='apollo.planning.PolyStSpeedConfig.max_accel', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_decel', full_name='apollo.planning.PolyStSpeedConfig.min_decel', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_limit_buffer', full_name='apollo.planning.PolyStSpeedConfig.speed_limit_buffer', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_weight', full_name='apollo.planning.PolyStSpeedConfig.speed_weight', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='jerk_weight', full_name='apollo.planning.PolyStSpeedConfig.jerk_weight', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='obstacle_weight', full_name='apollo.planning.PolyStSpeedConfig.obstacle_weight', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='unblocking_obstacle_cost', full_name='apollo.planning.PolyStSpeedConfig.unblocking_obstacle_cost', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='st_boundary_config', full_name='apollo.planning.PolyStSpeedConfig.st_boundary_config', index=11,
      number=12, type=11, cpp_type=10, label=1,
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
  serialized_start=120,
  serialized_end=467,
)

_POLYSTSPEEDCONFIG.fields_by_name['st_boundary_config'].message_type = modules_dot_planning_dot_proto_dot_st__boundary__config__pb2._STBOUNDARYCONFIG
DESCRIPTOR.message_types_by_name['PolyStSpeedConfig'] = _POLYSTSPEEDCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PolyStSpeedConfig = _reflection.GeneratedProtocolMessageType('PolyStSpeedConfig', (_message.Message,), dict(
  DESCRIPTOR = _POLYSTSPEEDCONFIG,
  __module__ = 'modules.planning.proto.poly_st_speed_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PolyStSpeedConfig)
  ))
_sym_db.RegisterMessage(PolyStSpeedConfig)


# @@protoc_insertion_point(module_scope)
