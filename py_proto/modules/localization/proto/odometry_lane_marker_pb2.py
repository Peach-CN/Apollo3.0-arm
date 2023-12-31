# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/localization/proto/odometry_lane_marker.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import geometry_pb2 as modules_dot_common_dot_proto_dot_geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/localization/proto/odometry_lane_marker.proto',
  package='apollo.localization',
  syntax='proto2',
  serialized_pb=_b('\n5modules/localization/proto/odometry_lane_marker.proto\x12\x13\x61pollo.localization\x1a#modules/common/proto/geometry.proto\"\x7f\n\x17OdometryLaneMarkerPoint\x12)\n\x08position\x18\x01 \x01(\x0b\x32\x17.apollo.common.PointENU\x12&\n\x06\x64irect\x18\x02 \x01(\x0b\x32\x16.apollo.common.Point3D\x12\x11\n\tcurvature\x18\x03 \x01(\x01\"q\n\x12OdometryLaneMarker\x12\x0c\n\x04type\x18\x01 \x01(\t\x12\x0f\n\x07quality\x18\x02 \x01(\x01\x12<\n\x06points\x18\x03 \x03(\x0b\x32,.apollo.localization.OdometryLaneMarkerPoint\"Z\n\x1a\x43ontourOdometryLaneMarkers\x12<\n\x0blane_marker\x18\x01 \x03(\x0b\x32\'.apollo.localization.OdometryLaneMarker\"`\n\x17OdometryLaneMarkersPack\x12\x45\n\x0clane_markers\x18\x01 \x03(\x0b\x32/.apollo.localization.ContourOdometryLaneMarkers')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_geometry__pb2.DESCRIPTOR,])




_ODOMETRYLANEMARKERPOINT = _descriptor.Descriptor(
  name='OdometryLaneMarkerPoint',
  full_name='apollo.localization.OdometryLaneMarkerPoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='position', full_name='apollo.localization.OdometryLaneMarkerPoint.position', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='direct', full_name='apollo.localization.OdometryLaneMarkerPoint.direct', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='curvature', full_name='apollo.localization.OdometryLaneMarkerPoint.curvature', index=2,
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
  serialized_start=115,
  serialized_end=242,
)


_ODOMETRYLANEMARKER = _descriptor.Descriptor(
  name='OdometryLaneMarker',
  full_name='apollo.localization.OdometryLaneMarker',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='apollo.localization.OdometryLaneMarker.type', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='quality', full_name='apollo.localization.OdometryLaneMarker.quality', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='points', full_name='apollo.localization.OdometryLaneMarker.points', index=2,
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
  serialized_start=244,
  serialized_end=357,
)


_CONTOURODOMETRYLANEMARKERS = _descriptor.Descriptor(
  name='ContourOdometryLaneMarkers',
  full_name='apollo.localization.ContourOdometryLaneMarkers',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lane_marker', full_name='apollo.localization.ContourOdometryLaneMarkers.lane_marker', index=0,
      number=1, type=11, cpp_type=10, label=3,
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
  serialized_start=359,
  serialized_end=449,
)


_ODOMETRYLANEMARKERSPACK = _descriptor.Descriptor(
  name='OdometryLaneMarkersPack',
  full_name='apollo.localization.OdometryLaneMarkersPack',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lane_markers', full_name='apollo.localization.OdometryLaneMarkersPack.lane_markers', index=0,
      number=1, type=11, cpp_type=10, label=3,
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
  serialized_start=451,
  serialized_end=547,
)

_ODOMETRYLANEMARKERPOINT.fields_by_name['position'].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINTENU
_ODOMETRYLANEMARKERPOINT.fields_by_name['direct'].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINT3D
_ODOMETRYLANEMARKER.fields_by_name['points'].message_type = _ODOMETRYLANEMARKERPOINT
_CONTOURODOMETRYLANEMARKERS.fields_by_name['lane_marker'].message_type = _ODOMETRYLANEMARKER
_ODOMETRYLANEMARKERSPACK.fields_by_name['lane_markers'].message_type = _CONTOURODOMETRYLANEMARKERS
DESCRIPTOR.message_types_by_name['OdometryLaneMarkerPoint'] = _ODOMETRYLANEMARKERPOINT
DESCRIPTOR.message_types_by_name['OdometryLaneMarker'] = _ODOMETRYLANEMARKER
DESCRIPTOR.message_types_by_name['ContourOdometryLaneMarkers'] = _CONTOURODOMETRYLANEMARKERS
DESCRIPTOR.message_types_by_name['OdometryLaneMarkersPack'] = _ODOMETRYLANEMARKERSPACK
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

OdometryLaneMarkerPoint = _reflection.GeneratedProtocolMessageType('OdometryLaneMarkerPoint', (_message.Message,), dict(
  DESCRIPTOR = _ODOMETRYLANEMARKERPOINT,
  __module__ = 'modules.localization.proto.odometry_lane_marker_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.OdometryLaneMarkerPoint)
  ))
_sym_db.RegisterMessage(OdometryLaneMarkerPoint)

OdometryLaneMarker = _reflection.GeneratedProtocolMessageType('OdometryLaneMarker', (_message.Message,), dict(
  DESCRIPTOR = _ODOMETRYLANEMARKER,
  __module__ = 'modules.localization.proto.odometry_lane_marker_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.OdometryLaneMarker)
  ))
_sym_db.RegisterMessage(OdometryLaneMarker)

ContourOdometryLaneMarkers = _reflection.GeneratedProtocolMessageType('ContourOdometryLaneMarkers', (_message.Message,), dict(
  DESCRIPTOR = _CONTOURODOMETRYLANEMARKERS,
  __module__ = 'modules.localization.proto.odometry_lane_marker_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.ContourOdometryLaneMarkers)
  ))
_sym_db.RegisterMessage(ContourOdometryLaneMarkers)

OdometryLaneMarkersPack = _reflection.GeneratedProtocolMessageType('OdometryLaneMarkersPack', (_message.Message,), dict(
  DESCRIPTOR = _ODOMETRYLANEMARKERSPACK,
  __module__ = 'modules.localization.proto.odometry_lane_marker_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.OdometryLaneMarkersPack)
  ))
_sym_db.RegisterMessage(OdometryLaneMarkersPack)


# @@protoc_insertion_point(module_scope)
