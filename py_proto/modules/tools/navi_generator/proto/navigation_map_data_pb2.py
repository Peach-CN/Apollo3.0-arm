# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/tools/navi_generator/proto/navigation_map_data.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.tools.navi_generator.proto import navigation_coordinate_pb2 as modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/tools/navi_generator/proto/navigation_map_data.proto',
  package='apollo.navi_generator',
  syntax='proto2',
  serialized_pb=_b('\n<modules/tools/navi_generator/proto/navigation_map_data.proto\x12\x15\x61pollo.navi_generator\x1a>modules/tools/navi_generator/proto/navigation_coordinate.proto\"7\n\x07MapStep\x12\x12\n\nstep_index\x18\x01 \x02(\x04\x12\x0b\n\x03lng\x18\x02 \x02(\x01\x12\x0b\n\x03lat\x18\x03 \x02(\x01\"\x7f\n\x08MapRoute\x12\x11\n\tnum_steps\x18\x01 \x02(\x04\x12,\n\x04step\x18\x02 \x03(\x0b\x32\x1e.apollo.navi_generator.MapStep\x12\x32\n\x04path\x18\x03 \x03(\x0b\x32$.apollo.navi_generator.NaviWGS84Corr\"P\n\tMapRoutes\x12\x13\n\x0broute_index\x18\x01 \x02(\x04\x12.\n\x05route\x18\x02 \x02(\x0b\x32\x1f.apollo.navi_generator.MapRoute\"T\n\x0cMapRoutePlan\x12\x12\n\nnum_routes\x18\x01 \x02(\x04\x12\x30\n\x06routes\x18\x02 \x03(\x0b\x32 .apollo.navi_generator.MapRoutes\"b\n\rMapRoutePlans\x12\x18\n\x10route_plan_index\x18\x01 \x02(\x04\x12\x37\n\nroute_plan\x18\x02 \x02(\x0b\x32#.apollo.navi_generator.MapRoutePlan\"\x85\x02\n\x07MapData\x12\x0c\n\x04type\x18\x01 \x02(\t\x12\x33\n\x05start\x18\x02 \x02(\x0b\x32$.apollo.navi_generator.NaviWGS84Corr\x12\x31\n\x03\x65nd\x18\x03 \x02(\x0b\x32$.apollo.navi_generator.NaviWGS84Corr\x12\x36\n\x08waypoint\x18\x04 \x03(\x0b\x32$.apollo.navi_generator.NaviWGS84Corr\x12\x11\n\tnum_plans\x18\x05 \x02(\x04\x12\x39\n\x0broute_plans\x18\x06 \x03(\x0b\x32$.apollo.navi_generator.MapRoutePlans')
  ,
  dependencies=[modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2.DESCRIPTOR,])




_MAPSTEP = _descriptor.Descriptor(
  name='MapStep',
  full_name='apollo.navi_generator.MapStep',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='step_index', full_name='apollo.navi_generator.MapStep.step_index', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lng', full_name='apollo.navi_generator.MapStep.lng', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lat', full_name='apollo.navi_generator.MapStep.lat', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
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
  serialized_start=151,
  serialized_end=206,
)


_MAPROUTE = _descriptor.Descriptor(
  name='MapRoute',
  full_name='apollo.navi_generator.MapRoute',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='num_steps', full_name='apollo.navi_generator.MapRoute.num_steps', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='step', full_name='apollo.navi_generator.MapRoute.step', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='path', full_name='apollo.navi_generator.MapRoute.path', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=208,
  serialized_end=335,
)


_MAPROUTES = _descriptor.Descriptor(
  name='MapRoutes',
  full_name='apollo.navi_generator.MapRoutes',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='route_index', full_name='apollo.navi_generator.MapRoutes.route_index', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='route', full_name='apollo.navi_generator.MapRoutes.route', index=1,
      number=2, type=11, cpp_type=10, label=2,
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
  serialized_start=337,
  serialized_end=417,
)


_MAPROUTEPLAN = _descriptor.Descriptor(
  name='MapRoutePlan',
  full_name='apollo.navi_generator.MapRoutePlan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='num_routes', full_name='apollo.navi_generator.MapRoutePlan.num_routes', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='routes', full_name='apollo.navi_generator.MapRoutePlan.routes', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=419,
  serialized_end=503,
)


_MAPROUTEPLANS = _descriptor.Descriptor(
  name='MapRoutePlans',
  full_name='apollo.navi_generator.MapRoutePlans',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='route_plan_index', full_name='apollo.navi_generator.MapRoutePlans.route_plan_index', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='route_plan', full_name='apollo.navi_generator.MapRoutePlans.route_plan', index=1,
      number=2, type=11, cpp_type=10, label=2,
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
  serialized_start=505,
  serialized_end=603,
)


_MAPDATA = _descriptor.Descriptor(
  name='MapData',
  full_name='apollo.navi_generator.MapData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='apollo.navi_generator.MapData.type', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start', full_name='apollo.navi_generator.MapData.start', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end', full_name='apollo.navi_generator.MapData.end', index=2,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='waypoint', full_name='apollo.navi_generator.MapData.waypoint', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_plans', full_name='apollo.navi_generator.MapData.num_plans', index=4,
      number=5, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='route_plans', full_name='apollo.navi_generator.MapData.route_plans', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=606,
  serialized_end=867,
)

_MAPROUTE.fields_by_name['step'].message_type = _MAPSTEP
_MAPROUTE.fields_by_name['path'].message_type = modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2._NAVIWGS84CORR
_MAPROUTES.fields_by_name['route'].message_type = _MAPROUTE
_MAPROUTEPLAN.fields_by_name['routes'].message_type = _MAPROUTES
_MAPROUTEPLANS.fields_by_name['route_plan'].message_type = _MAPROUTEPLAN
_MAPDATA.fields_by_name['start'].message_type = modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2._NAVIWGS84CORR
_MAPDATA.fields_by_name['end'].message_type = modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2._NAVIWGS84CORR
_MAPDATA.fields_by_name['waypoint'].message_type = modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2._NAVIWGS84CORR
_MAPDATA.fields_by_name['route_plans'].message_type = _MAPROUTEPLANS
DESCRIPTOR.message_types_by_name['MapStep'] = _MAPSTEP
DESCRIPTOR.message_types_by_name['MapRoute'] = _MAPROUTE
DESCRIPTOR.message_types_by_name['MapRoutes'] = _MAPROUTES
DESCRIPTOR.message_types_by_name['MapRoutePlan'] = _MAPROUTEPLAN
DESCRIPTOR.message_types_by_name['MapRoutePlans'] = _MAPROUTEPLANS
DESCRIPTOR.message_types_by_name['MapData'] = _MAPDATA
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MapStep = _reflection.GeneratedProtocolMessageType('MapStep', (_message.Message,), dict(
  DESCRIPTOR = _MAPSTEP,
  __module__ = 'modules.tools.navi_generator.proto.navigation_map_data_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.MapStep)
  ))
_sym_db.RegisterMessage(MapStep)

MapRoute = _reflection.GeneratedProtocolMessageType('MapRoute', (_message.Message,), dict(
  DESCRIPTOR = _MAPROUTE,
  __module__ = 'modules.tools.navi_generator.proto.navigation_map_data_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.MapRoute)
  ))
_sym_db.RegisterMessage(MapRoute)

MapRoutes = _reflection.GeneratedProtocolMessageType('MapRoutes', (_message.Message,), dict(
  DESCRIPTOR = _MAPROUTES,
  __module__ = 'modules.tools.navi_generator.proto.navigation_map_data_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.MapRoutes)
  ))
_sym_db.RegisterMessage(MapRoutes)

MapRoutePlan = _reflection.GeneratedProtocolMessageType('MapRoutePlan', (_message.Message,), dict(
  DESCRIPTOR = _MAPROUTEPLAN,
  __module__ = 'modules.tools.navi_generator.proto.navigation_map_data_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.MapRoutePlan)
  ))
_sym_db.RegisterMessage(MapRoutePlan)

MapRoutePlans = _reflection.GeneratedProtocolMessageType('MapRoutePlans', (_message.Message,), dict(
  DESCRIPTOR = _MAPROUTEPLANS,
  __module__ = 'modules.tools.navi_generator.proto.navigation_map_data_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.MapRoutePlans)
  ))
_sym_db.RegisterMessage(MapRoutePlans)

MapData = _reflection.GeneratedProtocolMessageType('MapData', (_message.Message,), dict(
  DESCRIPTOR = _MAPDATA,
  __module__ = 'modules.tools.navi_generator.proto.navigation_map_data_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.MapData)
  ))
_sym_db.RegisterMessage(MapData)


# @@protoc_insertion_point(module_scope)
