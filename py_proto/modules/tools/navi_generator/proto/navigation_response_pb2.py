# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/tools/navi_generator/proto/navigation_response.proto

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
  name='modules/tools/navi_generator/proto/navigation_response.proto',
  package='apollo.navi_generator',
  syntax='proto2',
  serialized_pb=_b('\n<modules/tools/navi_generator/proto/navigation_response.proto\x12\x15\x61pollo.navi_generator\x1a>modules/tools/navi_generator/proto/navigation_coordinate.proto\"R\n\x08NaviPath\x12\x12\n\nnavi_index\x18\x01 \x02(\x04\x12\x32\n\x04path\x18\x02 \x03(\x0b\x32$.apollo.navi_generator.NaviWGS84Corr\"\x89\x01\n\tNaviRoute\x12\x13\n\x0broute_index\x18\x01 \x02(\x04\x12\x11\n\tspeed_min\x18\x02 \x02(\x01\x12\x11\n\tspeed_max\x18\x03 \x02(\x01\x12\x11\n\tnum_navis\x18\x04 \x02(\x04\x12.\n\x05navis\x18\x05 \x03(\x0b\x32\x1f.apollo.navi_generator.NaviPath\"o\n\rNaviRoutePlan\x12\x12\n\nnum_routes\x18\x01 \x02(\x04\x12\x18\n\x10route_plan_index\x18\x02 \x02(\x04\x12\x30\n\x06routes\x18\x03 \x03(\x0b\x32 .apollo.navi_generator.NaviRoute\"\xc6\x01\n\x0eNaviRoutePlans\x12\x33\n\x05start\x18\x01 \x02(\x0b\x32$.apollo.navi_generator.NaviWGS84Corr\x12\x31\n\x03\x65nd\x18\x02 \x02(\x0b\x32$.apollo.navi_generator.NaviWGS84Corr\x12\x11\n\tnum_plans\x18\x03 \x02(\x04\x12\x39\n\x0broute_plans\x18\x04 \x03(\x0b\x32$.apollo.navi_generator.NaviRoutePlan\"+\n\x0bNaviSummary\x12\x0f\n\x07success\x18\x01 \x02(\x04\x12\x0b\n\x03msg\x18\x02 \x02(\t\"\x89\x01\n\x0cNaviResponse\x12\x0c\n\x04type\x18\x01 \x02(\t\x12\x32\n\x06result\x18\x02 \x02(\x0b\x32\".apollo.navi_generator.NaviSummary\x12\x37\n\x08res_data\x18\x03 \x01(\x0b\x32%.apollo.navi_generator.NaviRoutePlans')
  ,
  dependencies=[modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2.DESCRIPTOR,])




_NAVIPATH = _descriptor.Descriptor(
  name='NaviPath',
  full_name='apollo.navi_generator.NaviPath',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='navi_index', full_name='apollo.navi_generator.NaviPath.navi_index', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='path', full_name='apollo.navi_generator.NaviPath.path', index=1,
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
  serialized_start=151,
  serialized_end=233,
)


_NAVIROUTE = _descriptor.Descriptor(
  name='NaviRoute',
  full_name='apollo.navi_generator.NaviRoute',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='route_index', full_name='apollo.navi_generator.NaviRoute.route_index', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_min', full_name='apollo.navi_generator.NaviRoute.speed_min', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_max', full_name='apollo.navi_generator.NaviRoute.speed_max', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_navis', full_name='apollo.navi_generator.NaviRoute.num_navis', index=3,
      number=4, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='navis', full_name='apollo.navi_generator.NaviRoute.navis', index=4,
      number=5, type=11, cpp_type=10, label=3,
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
  serialized_start=236,
  serialized_end=373,
)


_NAVIROUTEPLAN = _descriptor.Descriptor(
  name='NaviRoutePlan',
  full_name='apollo.navi_generator.NaviRoutePlan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='num_routes', full_name='apollo.navi_generator.NaviRoutePlan.num_routes', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='route_plan_index', full_name='apollo.navi_generator.NaviRoutePlan.route_plan_index', index=1,
      number=2, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='routes', full_name='apollo.navi_generator.NaviRoutePlan.routes', index=2,
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
  serialized_start=375,
  serialized_end=486,
)


_NAVIROUTEPLANS = _descriptor.Descriptor(
  name='NaviRoutePlans',
  full_name='apollo.navi_generator.NaviRoutePlans',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='start', full_name='apollo.navi_generator.NaviRoutePlans.start', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end', full_name='apollo.navi_generator.NaviRoutePlans.end', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_plans', full_name='apollo.navi_generator.NaviRoutePlans.num_plans', index=2,
      number=3, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='route_plans', full_name='apollo.navi_generator.NaviRoutePlans.route_plans', index=3,
      number=4, type=11, cpp_type=10, label=3,
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
  serialized_start=489,
  serialized_end=687,
)


_NAVISUMMARY = _descriptor.Descriptor(
  name='NaviSummary',
  full_name='apollo.navi_generator.NaviSummary',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='success', full_name='apollo.navi_generator.NaviSummary.success', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='msg', full_name='apollo.navi_generator.NaviSummary.msg', index=1,
      number=2, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=689,
  serialized_end=732,
)


_NAVIRESPONSE = _descriptor.Descriptor(
  name='NaviResponse',
  full_name='apollo.navi_generator.NaviResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='apollo.navi_generator.NaviResponse.type', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='result', full_name='apollo.navi_generator.NaviResponse.result', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='res_data', full_name='apollo.navi_generator.NaviResponse.res_data', index=2,
      number=3, type=11, cpp_type=10, label=1,
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
  serialized_start=735,
  serialized_end=872,
)

_NAVIPATH.fields_by_name['path'].message_type = modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2._NAVIWGS84CORR
_NAVIROUTE.fields_by_name['navis'].message_type = _NAVIPATH
_NAVIROUTEPLAN.fields_by_name['routes'].message_type = _NAVIROUTE
_NAVIROUTEPLANS.fields_by_name['start'].message_type = modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2._NAVIWGS84CORR
_NAVIROUTEPLANS.fields_by_name['end'].message_type = modules_dot_tools_dot_navi__generator_dot_proto_dot_navigation__coordinate__pb2._NAVIWGS84CORR
_NAVIROUTEPLANS.fields_by_name['route_plans'].message_type = _NAVIROUTEPLAN
_NAVIRESPONSE.fields_by_name['result'].message_type = _NAVISUMMARY
_NAVIRESPONSE.fields_by_name['res_data'].message_type = _NAVIROUTEPLANS
DESCRIPTOR.message_types_by_name['NaviPath'] = _NAVIPATH
DESCRIPTOR.message_types_by_name['NaviRoute'] = _NAVIROUTE
DESCRIPTOR.message_types_by_name['NaviRoutePlan'] = _NAVIROUTEPLAN
DESCRIPTOR.message_types_by_name['NaviRoutePlans'] = _NAVIROUTEPLANS
DESCRIPTOR.message_types_by_name['NaviSummary'] = _NAVISUMMARY
DESCRIPTOR.message_types_by_name['NaviResponse'] = _NAVIRESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

NaviPath = _reflection.GeneratedProtocolMessageType('NaviPath', (_message.Message,), dict(
  DESCRIPTOR = _NAVIPATH,
  __module__ = 'modules.tools.navi_generator.proto.navigation_response_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.NaviPath)
  ))
_sym_db.RegisterMessage(NaviPath)

NaviRoute = _reflection.GeneratedProtocolMessageType('NaviRoute', (_message.Message,), dict(
  DESCRIPTOR = _NAVIROUTE,
  __module__ = 'modules.tools.navi_generator.proto.navigation_response_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.NaviRoute)
  ))
_sym_db.RegisterMessage(NaviRoute)

NaviRoutePlan = _reflection.GeneratedProtocolMessageType('NaviRoutePlan', (_message.Message,), dict(
  DESCRIPTOR = _NAVIROUTEPLAN,
  __module__ = 'modules.tools.navi_generator.proto.navigation_response_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.NaviRoutePlan)
  ))
_sym_db.RegisterMessage(NaviRoutePlan)

NaviRoutePlans = _reflection.GeneratedProtocolMessageType('NaviRoutePlans', (_message.Message,), dict(
  DESCRIPTOR = _NAVIROUTEPLANS,
  __module__ = 'modules.tools.navi_generator.proto.navigation_response_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.NaviRoutePlans)
  ))
_sym_db.RegisterMessage(NaviRoutePlans)

NaviSummary = _reflection.GeneratedProtocolMessageType('NaviSummary', (_message.Message,), dict(
  DESCRIPTOR = _NAVISUMMARY,
  __module__ = 'modules.tools.navi_generator.proto.navigation_response_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.NaviSummary)
  ))
_sym_db.RegisterMessage(NaviSummary)

NaviResponse = _reflection.GeneratedProtocolMessageType('NaviResponse', (_message.Message,), dict(
  DESCRIPTOR = _NAVIRESPONSE,
  __module__ = 'modules.tools.navi_generator.proto.navigation_response_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.NaviResponse)
  ))
_sym_db.RegisterMessage(NaviResponse)


# @@protoc_insertion_point(module_scope)