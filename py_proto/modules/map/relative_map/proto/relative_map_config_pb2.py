# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/map/relative_map/proto/relative_map_config.proto

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
  name='modules/map/relative_map/proto/relative_map_config.proto',
  package='apollo.relative_map',
  syntax='proto2',
  serialized_pb=_b('\n8modules/map/relative_map/proto/relative_map_config.proto\x12\x13\x61pollo.relative_map\"\x7f\n\x12MapGenerationParam\x12 \n\x12\x64\x65\x66\x61ult_left_width\x18\x01 \x01(\x01:\x04\x31.75\x12!\n\x13\x64\x65\x66\x61ult_right_width\x18\x02 \x01(\x01:\x04\x31.75\x12$\n\x13\x64\x65\x66\x61ult_speed_limit\x18\x03 \x01(\x01:\x07\x32\x39.0576\"\xbc\x01\n\x14NavigationLaneConfig\x12$\n\x17min_lane_marker_quality\x18\x01 \x01(\x01:\x03\x30.5\x12I\n\x0blane_source\x18\x02 \x01(\x0e\x32\x34.apollo.relative_map.NavigationLaneConfig.LaneSource\"3\n\nLaneSource\x12\x0e\n\nPERCEPTION\x10\x01\x12\x15\n\x11OFFLINE_GENERATED\x10\x02\"\x93\x01\n\x11RelativeMapConfig\x12:\n\tmap_param\x18\x01 \x01(\x0b\x32\'.apollo.relative_map.MapGenerationParam\x12\x42\n\x0fnavigation_lane\x18\x02 \x01(\x0b\x32).apollo.relative_map.NavigationLaneConfig')
)



_NAVIGATIONLANECONFIG_LANESOURCE = _descriptor.EnumDescriptor(
  name='LaneSource',
  full_name='apollo.relative_map.NavigationLaneConfig.LaneSource',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PERCEPTION', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OFFLINE_GENERATED', index=1, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=348,
  serialized_end=399,
)
_sym_db.RegisterEnumDescriptor(_NAVIGATIONLANECONFIG_LANESOURCE)


_MAPGENERATIONPARAM = _descriptor.Descriptor(
  name='MapGenerationParam',
  full_name='apollo.relative_map.MapGenerationParam',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='default_left_width', full_name='apollo.relative_map.MapGenerationParam.default_left_width', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1.75),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='default_right_width', full_name='apollo.relative_map.MapGenerationParam.default_right_width', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1.75),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='default_speed_limit', full_name='apollo.relative_map.MapGenerationParam.default_speed_limit', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(29.0576),
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
  serialized_start=81,
  serialized_end=208,
)


_NAVIGATIONLANECONFIG = _descriptor.Descriptor(
  name='NavigationLaneConfig',
  full_name='apollo.relative_map.NavigationLaneConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='min_lane_marker_quality', full_name='apollo.relative_map.NavigationLaneConfig.min_lane_marker_quality', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.5),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lane_source', full_name='apollo.relative_map.NavigationLaneConfig.lane_source', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _NAVIGATIONLANECONFIG_LANESOURCE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=211,
  serialized_end=399,
)


_RELATIVEMAPCONFIG = _descriptor.Descriptor(
  name='RelativeMapConfig',
  full_name='apollo.relative_map.RelativeMapConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='map_param', full_name='apollo.relative_map.RelativeMapConfig.map_param', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='navigation_lane', full_name='apollo.relative_map.RelativeMapConfig.navigation_lane', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
  serialized_start=402,
  serialized_end=549,
)

_NAVIGATIONLANECONFIG.fields_by_name['lane_source'].enum_type = _NAVIGATIONLANECONFIG_LANESOURCE
_NAVIGATIONLANECONFIG_LANESOURCE.containing_type = _NAVIGATIONLANECONFIG
_RELATIVEMAPCONFIG.fields_by_name['map_param'].message_type = _MAPGENERATIONPARAM
_RELATIVEMAPCONFIG.fields_by_name['navigation_lane'].message_type = _NAVIGATIONLANECONFIG
DESCRIPTOR.message_types_by_name['MapGenerationParam'] = _MAPGENERATIONPARAM
DESCRIPTOR.message_types_by_name['NavigationLaneConfig'] = _NAVIGATIONLANECONFIG
DESCRIPTOR.message_types_by_name['RelativeMapConfig'] = _RELATIVEMAPCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MapGenerationParam = _reflection.GeneratedProtocolMessageType('MapGenerationParam', (_message.Message,), dict(
  DESCRIPTOR = _MAPGENERATIONPARAM,
  __module__ = 'modules.map.relative_map.proto.relative_map_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.relative_map.MapGenerationParam)
  ))
_sym_db.RegisterMessage(MapGenerationParam)

NavigationLaneConfig = _reflection.GeneratedProtocolMessageType('NavigationLaneConfig', (_message.Message,), dict(
  DESCRIPTOR = _NAVIGATIONLANECONFIG,
  __module__ = 'modules.map.relative_map.proto.relative_map_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.relative_map.NavigationLaneConfig)
  ))
_sym_db.RegisterMessage(NavigationLaneConfig)

RelativeMapConfig = _reflection.GeneratedProtocolMessageType('RelativeMapConfig', (_message.Message,), dict(
  DESCRIPTOR = _RELATIVEMAPCONFIG,
  __module__ = 'modules.map.relative_map.proto.relative_map_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.relative_map.RelativeMapConfig)
  ))
_sym_db.RegisterMessage(RelativeMapConfig)


# @@protoc_insertion_point(module_scope)
