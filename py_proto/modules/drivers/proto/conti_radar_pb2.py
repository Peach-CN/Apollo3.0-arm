# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/drivers/proto/conti_radar.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import header_pb2 as modules_dot_common_dot_proto_dot_header__pb2
from modules.drivers.radar.conti_radar.proto import conti_radar_conf_pb2 as modules_dot_drivers_dot_radar_dot_conti__radar_dot_proto_dot_conti__radar__conf__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/drivers/proto/conti_radar.proto',
  package='apollo.drivers',
  syntax='proto2',
  serialized_pb=_b('\n\'modules/drivers/proto/conti_radar.proto\x12\x0e\x61pollo.drivers\x1a!modules/common/proto/header.proto\x1a>modules/drivers/radar/conti_radar/proto/conti_radar_conf.proto\"m\n\x15\x43lusterListStatus_600\x12\x0f\n\x04near\x18\x01 \x01(\x05:\x01\x30\x12\x0e\n\x03\x66\x61r\x18\x02 \x01(\x05:\x01\x30\x12\x18\n\x0cmeas_counter\x18\x03 \x01(\x05:\x02-1\x12\x19\n\x11interface_version\x18\x04 \x01(\x05\"c\n\x14ObjectListStatus_60A\x12\x16\n\x0bnof_objects\x18\x01 \x01(\x05:\x01\x30\x12\x18\n\x0cmeas_counter\x18\x02 \x01(\x05:\x02-1\x12\x19\n\x11interface_version\x18\x03 \x01(\x05\"\xe6\x01\n\x0eRadarState_201\x12\x14\n\x0cmax_distance\x18\x01 \x01(\r\x12\x13\n\x0bradar_power\x18\x02 \x01(\r\x12;\n\x0boutput_type\x18\x03 \x01(\x0e\x32&.apollo.drivers.conti_radar.OutputType\x12?\n\rrcs_threshold\x18\x04 \x01(\x0e\x32(.apollo.drivers.conti_radar.RcsThreshold\x12\x14\n\x0csend_quality\x18\x05 \x01(\x08\x12\x15\n\rsend_ext_info\x18\x06 \x01(\x08\"\xc1\x04\n\rContiRadarObs\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x16\n\x0e\x63lusterortrack\x18\x02 \x01(\x08\x12\x13\n\x0bobstacle_id\x18\x03 \x01(\x05\x12\x16\n\x0elongitude_dist\x18\x04 \x02(\x01\x12\x14\n\x0clateral_dist\x18\x05 \x02(\x01\x12\x15\n\rlongitude_vel\x18\x06 \x02(\x01\x12\x13\n\x0blateral_vel\x18\x07 \x02(\x01\x12\x0b\n\x03rcs\x18\x08 \x01(\x01\x12\x0f\n\x07\x64ynprop\x18\t \x01(\x05\x12\x1a\n\x12longitude_dist_rms\x18\n \x01(\x01\x12\x18\n\x10lateral_dist_rms\x18\x0b \x01(\x01\x12\x19\n\x11longitude_vel_rms\x18\x0c \x01(\x01\x12\x17\n\x0flateral_vel_rms\x18\r \x01(\x01\x12\x11\n\tprobexist\x18\x0e \x01(\x01\x12\x12\n\nmeas_state\x18\x0f \x01(\x05\x12\x17\n\x0flongitude_accel\x18\x10 \x01(\x01\x12\x15\n\rlateral_accel\x18\x11 \x01(\x01\x12\x17\n\x0foritation_angle\x18\x12 \x01(\x01\x12\x1b\n\x13longitude_accel_rms\x18\x13 \x01(\x01\x12\x19\n\x11lateral_accel_rms\x18\x14 \x01(\x01\x12\x1b\n\x13oritation_angle_rms\x18\x15 \x01(\x01\x12\x0e\n\x06length\x18\x16 \x01(\x01\x12\r\n\x05width\x18\x17 \x01(\x01\x12\x16\n\x0eobstacle_class\x18\x18 \x01(\x05\"\x9f\x02\n\nContiRadar\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12/\n\x08\x63ontiobs\x18\x02 \x03(\x0b\x32\x1d.apollo.drivers.ContiRadarObs\x12\x33\n\x0bradar_state\x18\x03 \x01(\x0b\x32\x1e.apollo.drivers.RadarState_201\x12\x42\n\x13\x63luster_list_status\x18\x04 \x01(\x0b\x32%.apollo.drivers.ClusterListStatus_600\x12@\n\x12object_list_status\x18\x05 \x01(\x0b\x32$.apollo.drivers.ObjectListStatus_60A')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,modules_dot_drivers_dot_radar_dot_conti__radar_dot_proto_dot_conti__radar__conf__pb2.DESCRIPTOR,])




_CLUSTERLISTSTATUS_600 = _descriptor.Descriptor(
  name='ClusterListStatus_600',
  full_name='apollo.drivers.ClusterListStatus_600',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='near', full_name='apollo.drivers.ClusterListStatus_600.near', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='far', full_name='apollo.drivers.ClusterListStatus_600.far', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='meas_counter', full_name='apollo.drivers.ClusterListStatus_600.meas_counter', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='interface_version', full_name='apollo.drivers.ClusterListStatus_600.interface_version', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=158,
  serialized_end=267,
)


_OBJECTLISTSTATUS_60A = _descriptor.Descriptor(
  name='ObjectListStatus_60A',
  full_name='apollo.drivers.ObjectListStatus_60A',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='nof_objects', full_name='apollo.drivers.ObjectListStatus_60A.nof_objects', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='meas_counter', full_name='apollo.drivers.ObjectListStatus_60A.meas_counter', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='interface_version', full_name='apollo.drivers.ObjectListStatus_60A.interface_version', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=269,
  serialized_end=368,
)


_RADARSTATE_201 = _descriptor.Descriptor(
  name='RadarState_201',
  full_name='apollo.drivers.RadarState_201',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='max_distance', full_name='apollo.drivers.RadarState_201.max_distance', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='radar_power', full_name='apollo.drivers.RadarState_201.radar_power', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='output_type', full_name='apollo.drivers.RadarState_201.output_type', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='rcs_threshold', full_name='apollo.drivers.RadarState_201.rcs_threshold', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='send_quality', full_name='apollo.drivers.RadarState_201.send_quality', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='send_ext_info', full_name='apollo.drivers.RadarState_201.send_ext_info', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=371,
  serialized_end=601,
)


_CONTIRADAROBS = _descriptor.Descriptor(
  name='ContiRadarObs',
  full_name='apollo.drivers.ContiRadarObs',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.drivers.ContiRadarObs.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='clusterortrack', full_name='apollo.drivers.ContiRadarObs.clusterortrack', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='obstacle_id', full_name='apollo.drivers.ContiRadarObs.obstacle_id', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_dist', full_name='apollo.drivers.ContiRadarObs.longitude_dist', index=3,
      number=4, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lateral_dist', full_name='apollo.drivers.ContiRadarObs.lateral_dist', index=4,
      number=5, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_vel', full_name='apollo.drivers.ContiRadarObs.longitude_vel', index=5,
      number=6, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lateral_vel', full_name='apollo.drivers.ContiRadarObs.lateral_vel', index=6,
      number=7, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='rcs', full_name='apollo.drivers.ContiRadarObs.rcs', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dynprop', full_name='apollo.drivers.ContiRadarObs.dynprop', index=8,
      number=9, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_dist_rms', full_name='apollo.drivers.ContiRadarObs.longitude_dist_rms', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lateral_dist_rms', full_name='apollo.drivers.ContiRadarObs.lateral_dist_rms', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_vel_rms', full_name='apollo.drivers.ContiRadarObs.longitude_vel_rms', index=11,
      number=12, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lateral_vel_rms', full_name='apollo.drivers.ContiRadarObs.lateral_vel_rms', index=12,
      number=13, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='probexist', full_name='apollo.drivers.ContiRadarObs.probexist', index=13,
      number=14, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='meas_state', full_name='apollo.drivers.ContiRadarObs.meas_state', index=14,
      number=15, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_accel', full_name='apollo.drivers.ContiRadarObs.longitude_accel', index=15,
      number=16, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lateral_accel', full_name='apollo.drivers.ContiRadarObs.lateral_accel', index=16,
      number=17, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='oritation_angle', full_name='apollo.drivers.ContiRadarObs.oritation_angle', index=17,
      number=18, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_accel_rms', full_name='apollo.drivers.ContiRadarObs.longitude_accel_rms', index=18,
      number=19, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lateral_accel_rms', full_name='apollo.drivers.ContiRadarObs.lateral_accel_rms', index=19,
      number=20, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='oritation_angle_rms', full_name='apollo.drivers.ContiRadarObs.oritation_angle_rms', index=20,
      number=21, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='length', full_name='apollo.drivers.ContiRadarObs.length', index=21,
      number=22, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='apollo.drivers.ContiRadarObs.width', index=22,
      number=23, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='obstacle_class', full_name='apollo.drivers.ContiRadarObs.obstacle_class', index=23,
      number=24, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=604,
  serialized_end=1181,
)


_CONTIRADAR = _descriptor.Descriptor(
  name='ContiRadar',
  full_name='apollo.drivers.ContiRadar',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.drivers.ContiRadar.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='contiobs', full_name='apollo.drivers.ContiRadar.contiobs', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='radar_state', full_name='apollo.drivers.ContiRadar.radar_state', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cluster_list_status', full_name='apollo.drivers.ContiRadar.cluster_list_status', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='object_list_status', full_name='apollo.drivers.ContiRadar.object_list_status', index=4,
      number=5, type=11, cpp_type=10, label=1,
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
  serialized_start=1184,
  serialized_end=1471,
)

_RADARSTATE_201.fields_by_name['output_type'].enum_type = modules_dot_drivers_dot_radar_dot_conti__radar_dot_proto_dot_conti__radar__conf__pb2._OUTPUTTYPE
_RADARSTATE_201.fields_by_name['rcs_threshold'].enum_type = modules_dot_drivers_dot_radar_dot_conti__radar_dot_proto_dot_conti__radar__conf__pb2._RCSTHRESHOLD
_CONTIRADAROBS.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_CONTIRADAR.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_CONTIRADAR.fields_by_name['contiobs'].message_type = _CONTIRADAROBS
_CONTIRADAR.fields_by_name['radar_state'].message_type = _RADARSTATE_201
_CONTIRADAR.fields_by_name['cluster_list_status'].message_type = _CLUSTERLISTSTATUS_600
_CONTIRADAR.fields_by_name['object_list_status'].message_type = _OBJECTLISTSTATUS_60A
DESCRIPTOR.message_types_by_name['ClusterListStatus_600'] = _CLUSTERLISTSTATUS_600
DESCRIPTOR.message_types_by_name['ObjectListStatus_60A'] = _OBJECTLISTSTATUS_60A
DESCRIPTOR.message_types_by_name['RadarState_201'] = _RADARSTATE_201
DESCRIPTOR.message_types_by_name['ContiRadarObs'] = _CONTIRADAROBS
DESCRIPTOR.message_types_by_name['ContiRadar'] = _CONTIRADAR
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ClusterListStatus_600 = _reflection.GeneratedProtocolMessageType('ClusterListStatus_600', (_message.Message,), dict(
  DESCRIPTOR = _CLUSTERLISTSTATUS_600,
  __module__ = 'modules.drivers.proto.conti_radar_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.ClusterListStatus_600)
  ))
_sym_db.RegisterMessage(ClusterListStatus_600)

ObjectListStatus_60A = _reflection.GeneratedProtocolMessageType('ObjectListStatus_60A', (_message.Message,), dict(
  DESCRIPTOR = _OBJECTLISTSTATUS_60A,
  __module__ = 'modules.drivers.proto.conti_radar_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.ObjectListStatus_60A)
  ))
_sym_db.RegisterMessage(ObjectListStatus_60A)

RadarState_201 = _reflection.GeneratedProtocolMessageType('RadarState_201', (_message.Message,), dict(
  DESCRIPTOR = _RADARSTATE_201,
  __module__ = 'modules.drivers.proto.conti_radar_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.RadarState_201)
  ))
_sym_db.RegisterMessage(RadarState_201)

ContiRadarObs = _reflection.GeneratedProtocolMessageType('ContiRadarObs', (_message.Message,), dict(
  DESCRIPTOR = _CONTIRADAROBS,
  __module__ = 'modules.drivers.proto.conti_radar_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.ContiRadarObs)
  ))
_sym_db.RegisterMessage(ContiRadarObs)

ContiRadar = _reflection.GeneratedProtocolMessageType('ContiRadar', (_message.Message,), dict(
  DESCRIPTOR = _CONTIRADAR,
  __module__ = 'modules.drivers.proto.conti_radar_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.ContiRadar)
  ))
_sym_db.RegisterMessage(ContiRadar)


# @@protoc_insertion_point(module_scope)
