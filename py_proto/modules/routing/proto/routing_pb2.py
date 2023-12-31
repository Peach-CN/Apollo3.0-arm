# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/routing/proto/routing.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import header_pb2 as modules_dot_common_dot_proto_dot_header__pb2
from modules.common.proto import geometry_pb2 as modules_dot_common_dot_proto_dot_geometry__pb2
from modules.common.proto import error_code_pb2 as modules_dot_common_dot_proto_dot_error__code__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/routing/proto/routing.proto',
  package='apollo.routing',
  syntax='proto2',
  serialized_pb=_b('\n#modules/routing/proto/routing.proto\x12\x0e\x61pollo.routing\x1a!modules/common/proto/header.proto\x1a#modules/common/proto/geometry.proto\x1a%modules/common/proto/error_code.proto\"L\n\x0cLaneWaypoint\x12\n\n\x02id\x18\x01 \x01(\t\x12\t\n\x01s\x18\x02 \x01(\x01\x12%\n\x04pose\x18\x03 \x01(\x0b\x32\x17.apollo.common.PointENU\"9\n\x0bLaneSegment\x12\n\n\x02id\x18\x01 \x01(\t\x12\x0f\n\x07start_s\x18\x02 \x01(\x01\x12\r\n\x05\x65nd_s\x18\x03 \x01(\x01\"\xd1\x01\n\x0eRoutingRequest\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12.\n\x08waypoint\x18\x02 \x03(\x0b\x32\x1c.apollo.routing.LaneWaypoint\x12\x35\n\x10\x62lacklisted_lane\x18\x03 \x03(\x0b\x32\x1b.apollo.routing.LaneSegment\x12\x18\n\x10\x62lacklisted_road\x18\x04 \x03(\t\x12\x17\n\tbroadcast\x18\x05 \x01(\x08:\x04true\"\x1f\n\x0bMeasurement\x12\x10\n\x08\x64istance\x18\x01 \x01(\x01\"\x8c\x01\n\x07Passage\x12,\n\x07segment\x18\x01 \x03(\x0b\x32\x1b.apollo.routing.LaneSegment\x12\x10\n\x08\x63\x61n_exit\x18\x02 \x01(\x08\x12\x41\n\x10\x63hange_lane_type\x18\x03 \x01(\x0e\x32\x1e.apollo.routing.ChangeLaneType:\x07\x46ORWARD\"C\n\x0bRoadSegment\x12\n\n\x02id\x18\x01 \x01(\t\x12(\n\x07passage\x18\x02 \x03(\x0b\x32\x17.apollo.routing.Passage\"\x8c\x02\n\x0fRoutingResponse\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12)\n\x04road\x18\x02 \x03(\x0b\x32\x1b.apollo.routing.RoadSegment\x12\x30\n\x0bmeasurement\x18\x03 \x01(\x0b\x32\x1b.apollo.routing.Measurement\x12\x37\n\x0frouting_request\x18\x04 \x01(\x0b\x32\x1e.apollo.routing.RoutingRequest\x12\x13\n\x0bmap_version\x18\x05 \x01(\x0c\x12\'\n\x06status\x18\x06 \x01(\x0b\x32\x17.apollo.common.StatusPb*2\n\x0e\x43hangeLaneType\x12\x0b\n\x07\x46ORWARD\x10\x00\x12\x08\n\x04LEFT\x10\x01\x12\t\n\x05RIGHT\x10\x02')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,modules_dot_common_dot_proto_dot_geometry__pb2.DESCRIPTOR,modules_dot_common_dot_proto_dot_error__code__pb2.DESCRIPTOR,])

_CHANGELANETYPE = _descriptor.EnumDescriptor(
  name='ChangeLaneType',
  full_name='apollo.routing.ChangeLaneType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FORWARD', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LEFT', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RIGHT', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1031,
  serialized_end=1081,
)
_sym_db.RegisterEnumDescriptor(_CHANGELANETYPE)

ChangeLaneType = enum_type_wrapper.EnumTypeWrapper(_CHANGELANETYPE)
FORWARD = 0
LEFT = 1
RIGHT = 2



_LANEWAYPOINT = _descriptor.Descriptor(
  name='LaneWaypoint',
  full_name='apollo.routing.LaneWaypoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.routing.LaneWaypoint.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s', full_name='apollo.routing.LaneWaypoint.s', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pose', full_name='apollo.routing.LaneWaypoint.pose', index=2,
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
  serialized_start=166,
  serialized_end=242,
)


_LANESEGMENT = _descriptor.Descriptor(
  name='LaneSegment',
  full_name='apollo.routing.LaneSegment',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.routing.LaneSegment.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_s', full_name='apollo.routing.LaneSegment.start_s', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_s', full_name='apollo.routing.LaneSegment.end_s', index=2,
      number=3, type=1, cpp_type=5, label=1,
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
  serialized_start=244,
  serialized_end=301,
)


_ROUTINGREQUEST = _descriptor.Descriptor(
  name='RoutingRequest',
  full_name='apollo.routing.RoutingRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.routing.RoutingRequest.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='waypoint', full_name='apollo.routing.RoutingRequest.waypoint', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='blacklisted_lane', full_name='apollo.routing.RoutingRequest.blacklisted_lane', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='blacklisted_road', full_name='apollo.routing.RoutingRequest.blacklisted_road', index=3,
      number=4, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='broadcast', full_name='apollo.routing.RoutingRequest.broadcast', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
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
  serialized_start=304,
  serialized_end=513,
)


_MEASUREMENT = _descriptor.Descriptor(
  name='Measurement',
  full_name='apollo.routing.Measurement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='distance', full_name='apollo.routing.Measurement.distance', index=0,
      number=1, type=1, cpp_type=5, label=1,
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
  serialized_start=515,
  serialized_end=546,
)


_PASSAGE = _descriptor.Descriptor(
  name='Passage',
  full_name='apollo.routing.Passage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='segment', full_name='apollo.routing.Passage.segment', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='can_exit', full_name='apollo.routing.Passage.can_exit', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='change_lane_type', full_name='apollo.routing.Passage.change_lane_type', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
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
  serialized_start=549,
  serialized_end=689,
)


_ROADSEGMENT = _descriptor.Descriptor(
  name='RoadSegment',
  full_name='apollo.routing.RoadSegment',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.routing.RoadSegment.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='passage', full_name='apollo.routing.RoadSegment.passage', index=1,
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
  serialized_start=691,
  serialized_end=758,
)


_ROUTINGRESPONSE = _descriptor.Descriptor(
  name='RoutingResponse',
  full_name='apollo.routing.RoutingResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.routing.RoutingResponse.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='road', full_name='apollo.routing.RoutingResponse.road', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='measurement', full_name='apollo.routing.RoutingResponse.measurement', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='routing_request', full_name='apollo.routing.RoutingResponse.routing_request', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='map_version', full_name='apollo.routing.RoutingResponse.map_version', index=4,
      number=5, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='apollo.routing.RoutingResponse.status', index=5,
      number=6, type=11, cpp_type=10, label=1,
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
  serialized_start=761,
  serialized_end=1029,
)

_LANEWAYPOINT.fields_by_name['pose'].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINTENU
_ROUTINGREQUEST.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_ROUTINGREQUEST.fields_by_name['waypoint'].message_type = _LANEWAYPOINT
_ROUTINGREQUEST.fields_by_name['blacklisted_lane'].message_type = _LANESEGMENT
_PASSAGE.fields_by_name['segment'].message_type = _LANESEGMENT
_PASSAGE.fields_by_name['change_lane_type'].enum_type = _CHANGELANETYPE
_ROADSEGMENT.fields_by_name['passage'].message_type = _PASSAGE
_ROUTINGRESPONSE.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_ROUTINGRESPONSE.fields_by_name['road'].message_type = _ROADSEGMENT
_ROUTINGRESPONSE.fields_by_name['measurement'].message_type = _MEASUREMENT
_ROUTINGRESPONSE.fields_by_name['routing_request'].message_type = _ROUTINGREQUEST
_ROUTINGRESPONSE.fields_by_name['status'].message_type = modules_dot_common_dot_proto_dot_error__code__pb2._STATUSPB
DESCRIPTOR.message_types_by_name['LaneWaypoint'] = _LANEWAYPOINT
DESCRIPTOR.message_types_by_name['LaneSegment'] = _LANESEGMENT
DESCRIPTOR.message_types_by_name['RoutingRequest'] = _ROUTINGREQUEST
DESCRIPTOR.message_types_by_name['Measurement'] = _MEASUREMENT
DESCRIPTOR.message_types_by_name['Passage'] = _PASSAGE
DESCRIPTOR.message_types_by_name['RoadSegment'] = _ROADSEGMENT
DESCRIPTOR.message_types_by_name['RoutingResponse'] = _ROUTINGRESPONSE
DESCRIPTOR.enum_types_by_name['ChangeLaneType'] = _CHANGELANETYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LaneWaypoint = _reflection.GeneratedProtocolMessageType('LaneWaypoint', (_message.Message,), dict(
  DESCRIPTOR = _LANEWAYPOINT,
  __module__ = 'modules.routing.proto.routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.LaneWaypoint)
  ))
_sym_db.RegisterMessage(LaneWaypoint)

LaneSegment = _reflection.GeneratedProtocolMessageType('LaneSegment', (_message.Message,), dict(
  DESCRIPTOR = _LANESEGMENT,
  __module__ = 'modules.routing.proto.routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.LaneSegment)
  ))
_sym_db.RegisterMessage(LaneSegment)

RoutingRequest = _reflection.GeneratedProtocolMessageType('RoutingRequest', (_message.Message,), dict(
  DESCRIPTOR = _ROUTINGREQUEST,
  __module__ = 'modules.routing.proto.routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.RoutingRequest)
  ))
_sym_db.RegisterMessage(RoutingRequest)

Measurement = _reflection.GeneratedProtocolMessageType('Measurement', (_message.Message,), dict(
  DESCRIPTOR = _MEASUREMENT,
  __module__ = 'modules.routing.proto.routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.Measurement)
  ))
_sym_db.RegisterMessage(Measurement)

Passage = _reflection.GeneratedProtocolMessageType('Passage', (_message.Message,), dict(
  DESCRIPTOR = _PASSAGE,
  __module__ = 'modules.routing.proto.routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.Passage)
  ))
_sym_db.RegisterMessage(Passage)

RoadSegment = _reflection.GeneratedProtocolMessageType('RoadSegment', (_message.Message,), dict(
  DESCRIPTOR = _ROADSEGMENT,
  __module__ = 'modules.routing.proto.routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.RoadSegment)
  ))
_sym_db.RegisterMessage(RoadSegment)

RoutingResponse = _reflection.GeneratedProtocolMessageType('RoutingResponse', (_message.Message,), dict(
  DESCRIPTOR = _ROUTINGRESPONSE,
  __module__ = 'modules.routing.proto.routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.RoutingResponse)
  ))
_sym_db.RegisterMessage(RoutingResponse)


# @@protoc_insertion_point(module_scope)
