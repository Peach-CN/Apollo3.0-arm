# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common/adapters/proto/adapter_config.proto

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
  name='modules/common/adapters/proto/adapter_config.proto',
  package='apollo.common.adapter',
  syntax='proto2',
  serialized_pb=_b('\n2modules/common/adapters/proto/adapter_config.proto\x12\x15\x61pollo.common.adapter\"\x9f\x0e\n\rAdapterConfig\x12>\n\x04type\x18\x01 \x02(\x0e\x32\x30.apollo.common.adapter.AdapterConfig.MessageType\x12\x37\n\x04mode\x18\x02 \x02(\x0e\x32).apollo.common.adapter.AdapterConfig.Mode\x12!\n\x15message_history_limit\x18\x03 \x01(\x05:\x02\x31\x30\x12\x14\n\x05latch\x18\x04 \x01(\x08:\x05\x66\x61lse\x12\r\n\x05topic\x18\x05 \x01(\t\"\x94\x0c\n\x0bMessageType\x12\x0f\n\x0bPOINT_CLOUD\x10\x01\x12\x15\n\x11VLP16_POINT_CLOUD\x10:\x12\x07\n\x03GPS\x10\x02\x12\x07\n\x03IMU\x10\x03\x12\x0b\n\x07\x43HASSIS\x10\x04\x12\x10\n\x0cLOCALIZATION\x10\x05\x12\x17\n\x13PLANNING_TRAJECTORY\x10\x06\x12\x0b\n\x07MONITOR\x10\x07\x12\x07\n\x03PAD\x10\x08\x12\x13\n\x0f\x43ONTROL_COMMAND\x10\t\x12\x0e\n\nPREDICTION\x10\n\x12\x18\n\x14PERCEPTION_OBSTACLES\x10\x0b\x12\x1b\n\x17TRAFFIC_LIGHT_DETECTION\x10\x0c\x12\x12\n\x0e\x43HASSIS_DETAIL\x10\r\x12\x10\n\x08\x44\x45\x43ISION\x10\x0e\x1a\x02\x08\x01\x12\n\n\x06\x43\x41NBUS\x10\x0f\x12\x13\n\x0fROUTING_REQUEST\x10\x10\x12\x14\n\x10ROUTING_RESPONSE\x10\x11\x12\x15\n\x11RELATIVE_ODOMETRY\x10\x12\x12\x0c\n\x08INS_STAT\x10\x13\x12\x13\n\x0bHMI_COMMAND\x10\x14\x1a\x02\x08\x01\x12\x0c\n\x08MOBILEYE\x10\x15\x12\r\n\tDELPHIESR\x10\x16\x12\x14\n\x10\x43OMPRESSED_IMAGE\x10\x17\x12\x11\n\rSYSTEM_STATUS\x10\x18\x12\x0e\n\nINS_STATUS\x10\x19\x12\x0f\n\x0bGNSS_STATUS\x10\x1a\x12\x0f\n\x0b\x43ONTI_RADAR\x10\x1b\x12\x0f\n\x0bIMAGE_SHORT\x10\x1c\x12\x0e\n\nIMAGE_LONG\x10\x1d\x12\x0f\n\x0b\x44RIVE_EVENT\x10\x1e\x12\x10\n\x0cGNSS_RTK_OBS\x10\x1f\x12\x10\n\x0cGNSS_RTK_EPH\x10 \x12\x12\n\x0eGNSS_BEST_POSE\x10!\x12\x19\n\x15LOCALIZATION_MSF_GNSS\x10\"\x12\x1a\n\x16LOCALIZATION_MSF_LIDAR\x10#\x12\x1d\n\x19LOCALIZATION_MSF_SINS_PVA\x10$\x12\x0b\n\x07RAW_IMU\x10%\x12\x1b\n\x17LOCALIZATION_MSF_STATUS\x10&\x12\x0f\n\x0bSTATIC_INFO\x10\'\x12\x10\n\x0cRELATIVE_MAP\x10(\x12\x0e\n\nNAVIGATION\x10)\x12\x14\n\x10ULTRASONIC_RADAR\x10*\x12\x11\n\rAUDIO_CAPTURE\x10+\x12\x0f\n\x0bIMAGE_FRONT\x10-\x12\x17\n\x13PANDORA_POINT_CLOUD\x10.\x12\x1e\n\x1aPANDORA_CAMERA_FRONT_COLOR\x10/\x12\x1d\n\x19PANDORA_CAMERA_RIGHT_GRAY\x10\x30\x12\x1c\n\x18PANDORA_CAMERA_LEFT_GRAY\x10\x31\x12\x1d\n\x19PANDORA_CAMERA_FRONT_GRAY\x10\x32\x12\x1c\n\x18PANDORA_CAMERA_BACK_GRAY\x10\x33\x12\x18\n\x14PERCEPTION_LANE_MASK\x10\x34\x12\x0c\n\x08GUARDIAN\x10\x35\x12\x11\n\rGNSS_RAW_DATA\x10\x36\x12\x11\n\rSTREAM_STATUS\x10\x37\x12\x10\n\x0cGNSS_HEADING\x10\x38\x12\r\n\tRTCM_DATA\x10\x39\x12\x11\n\rRACOBIT_RADAR\x10;\x12\x15\n\x11POINT_CLOUD_DENSE\x10<\x12\x19\n\x15POINT_CLOUD_DENSE_RAW\x10=\x12\x17\n\x13VELODYNE_SCAN_DENSE\x10>\x12\x18\n\x14POINT_CLOUD_SPARSE_1\x10?\x12\x1c\n\x18POINT_CLOUD_SPARSE_RAW_1\x10@\x12\x1a\n\x16VELODYNE_SCAN_SPARSE_1\x10\x41\x12\x18\n\x14POINT_CLOUD_SPARSE_2\x10\x42\x12\x1c\n\x18POINT_CLOUD_SPARSE_RAW_2\x10\x43\x12\x1a\n\x16VELODYNE_SCAN_SPARSE_2\x10\x44\x12\x18\n\x14POINT_CLOUD_SPARSE_3\x10\x45\x12\x1c\n\x18POINT_CLOUD_SPARSE_RAW_3\x10\x46\x12\x1a\n\x16VELODYNE_SCAN_SPARSE_3\x10G\x12\x15\n\x11\x43\x41MERA_IMAGE_LONG\x10H\x12\x16\n\x12\x43\x41MERA_IMAGE_SHORT\x10I\x12\x10\n\x0cPLANNING_PAD\x10J\"6\n\x04Mode\x12\x10\n\x0cRECEIVE_ONLY\x10\x00\x12\x10\n\x0cPUBLISH_ONLY\x10\x01\x12\n\n\x06\x44UPLEX\x10\x02\"\\\n\x14\x41\x64\x61pterManagerConfig\x12\x34\n\x06\x63onfig\x18\x01 \x03(\x0b\x32$.apollo.common.adapter.AdapterConfig\x12\x0e\n\x06is_ros\x18\x02 \x02(\x08')
)



_ADAPTERCONFIG_MESSAGETYPE = _descriptor.EnumDescriptor(
  name='MessageType',
  full_name='apollo.common.adapter.AdapterConfig.MessageType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VLP16_POINT_CLOUD', index=1, number=58,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GPS', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IMU', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHASSIS', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOCALIZATION', index=5, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PLANNING_TRAJECTORY', index=6, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MONITOR', index=7, number=7,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PAD', index=8, number=8,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CONTROL_COMMAND', index=9, number=9,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PREDICTION', index=10, number=10,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PERCEPTION_OBSTACLES', index=11, number=11,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRAFFIC_LIGHT_DETECTION', index=12, number=12,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHASSIS_DETAIL', index=13, number=13,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DECISION', index=14, number=14,
      options=_descriptor._ParseOptions(descriptor_pb2.EnumValueOptions(), _b('\010\001')),
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CANBUS', index=15, number=15,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ROUTING_REQUEST', index=16, number=16,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ROUTING_RESPONSE', index=17, number=17,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RELATIVE_ODOMETRY', index=18, number=18,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_STAT', index=19, number=19,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HMI_COMMAND', index=20, number=20,
      options=_descriptor._ParseOptions(descriptor_pb2.EnumValueOptions(), _b('\010\001')),
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MOBILEYE', index=21, number=21,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DELPHIESR', index=22, number=22,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMPRESSED_IMAGE', index=23, number=23,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SYSTEM_STATUS', index=24, number=24,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_STATUS', index=25, number=25,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GNSS_STATUS', index=26, number=26,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CONTI_RADAR', index=27, number=27,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IMAGE_SHORT', index=28, number=28,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IMAGE_LONG', index=29, number=29,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRIVE_EVENT', index=30, number=30,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GNSS_RTK_OBS', index=31, number=31,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GNSS_RTK_EPH', index=32, number=32,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GNSS_BEST_POSE', index=33, number=33,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOCALIZATION_MSF_GNSS', index=34, number=34,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOCALIZATION_MSF_LIDAR', index=35, number=35,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOCALIZATION_MSF_SINS_PVA', index=36, number=36,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RAW_IMU', index=37, number=37,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOCALIZATION_MSF_STATUS', index=38, number=38,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STATIC_INFO', index=39, number=39,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RELATIVE_MAP', index=40, number=40,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NAVIGATION', index=41, number=41,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ULTRASONIC_RADAR', index=42, number=42,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='AUDIO_CAPTURE', index=43, number=43,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IMAGE_FRONT', index=44, number=45,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANDORA_POINT_CLOUD', index=45, number=46,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANDORA_CAMERA_FRONT_COLOR', index=46, number=47,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANDORA_CAMERA_RIGHT_GRAY', index=47, number=48,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANDORA_CAMERA_LEFT_GRAY', index=48, number=49,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANDORA_CAMERA_FRONT_GRAY', index=49, number=50,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANDORA_CAMERA_BACK_GRAY', index=50, number=51,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PERCEPTION_LANE_MASK', index=51, number=52,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GUARDIAN', index=52, number=53,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GNSS_RAW_DATA', index=53, number=54,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STREAM_STATUS', index=54, number=55,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GNSS_HEADING', index=55, number=56,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RTCM_DATA', index=56, number=57,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RACOBIT_RADAR', index=57, number=59,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_DENSE', index=58, number=60,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_DENSE_RAW', index=59, number=61,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VELODYNE_SCAN_DENSE', index=60, number=62,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_SPARSE_1', index=61, number=63,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_SPARSE_RAW_1', index=62, number=64,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VELODYNE_SCAN_SPARSE_1', index=63, number=65,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_SPARSE_2', index=64, number=66,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_SPARSE_RAW_2', index=65, number=67,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VELODYNE_SCAN_SPARSE_2', index=66, number=68,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_SPARSE_3', index=67, number=69,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='POINT_CLOUD_SPARSE_RAW_3', index=68, number=70,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VELODYNE_SCAN_SPARSE_3', index=69, number=71,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CAMERA_IMAGE_LONG', index=70, number=72,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CAMERA_IMAGE_SHORT', index=71, number=73,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PLANNING_PAD', index=72, number=74,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=289,
  serialized_end=1845,
)
_sym_db.RegisterEnumDescriptor(_ADAPTERCONFIG_MESSAGETYPE)

_ADAPTERCONFIG_MODE = _descriptor.EnumDescriptor(
  name='Mode',
  full_name='apollo.common.adapter.AdapterConfig.Mode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='RECEIVE_ONLY', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PUBLISH_ONLY', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DUPLEX', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1847,
  serialized_end=1901,
)
_sym_db.RegisterEnumDescriptor(_ADAPTERCONFIG_MODE)


_ADAPTERCONFIG = _descriptor.Descriptor(
  name='AdapterConfig',
  full_name='apollo.common.adapter.AdapterConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='apollo.common.adapter.AdapterConfig.type', index=0,
      number=1, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mode', full_name='apollo.common.adapter.AdapterConfig.mode', index=1,
      number=2, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='message_history_limit', full_name='apollo.common.adapter.AdapterConfig.message_history_limit', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=10,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latch', full_name='apollo.common.adapter.AdapterConfig.latch', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='topic', full_name='apollo.common.adapter.AdapterConfig.topic', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _ADAPTERCONFIG_MESSAGETYPE,
    _ADAPTERCONFIG_MODE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=78,
  serialized_end=1901,
)


_ADAPTERMANAGERCONFIG = _descriptor.Descriptor(
  name='AdapterManagerConfig',
  full_name='apollo.common.adapter.AdapterManagerConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='config', full_name='apollo.common.adapter.AdapterManagerConfig.config', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='is_ros', full_name='apollo.common.adapter.AdapterManagerConfig.is_ros', index=1,
      number=2, type=8, cpp_type=7, label=2,
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
  serialized_start=1903,
  serialized_end=1995,
)

_ADAPTERCONFIG.fields_by_name['type'].enum_type = _ADAPTERCONFIG_MESSAGETYPE
_ADAPTERCONFIG.fields_by_name['mode'].enum_type = _ADAPTERCONFIG_MODE
_ADAPTERCONFIG_MESSAGETYPE.containing_type = _ADAPTERCONFIG
_ADAPTERCONFIG_MODE.containing_type = _ADAPTERCONFIG
_ADAPTERMANAGERCONFIG.fields_by_name['config'].message_type = _ADAPTERCONFIG
DESCRIPTOR.message_types_by_name['AdapterConfig'] = _ADAPTERCONFIG
DESCRIPTOR.message_types_by_name['AdapterManagerConfig'] = _ADAPTERMANAGERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AdapterConfig = _reflection.GeneratedProtocolMessageType('AdapterConfig', (_message.Message,), dict(
  DESCRIPTOR = _ADAPTERCONFIG,
  __module__ = 'modules.common.adapters.proto.adapter_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.common.adapter.AdapterConfig)
  ))
_sym_db.RegisterMessage(AdapterConfig)

AdapterManagerConfig = _reflection.GeneratedProtocolMessageType('AdapterManagerConfig', (_message.Message,), dict(
  DESCRIPTOR = _ADAPTERMANAGERCONFIG,
  __module__ = 'modules.common.adapters.proto.adapter_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.common.adapter.AdapterManagerConfig)
  ))
_sym_db.RegisterMessage(AdapterManagerConfig)


_ADAPTERCONFIG_MESSAGETYPE.values_by_name["DECISION"].has_options = True
_ADAPTERCONFIG_MESSAGETYPE.values_by_name["DECISION"]._options = _descriptor._ParseOptions(descriptor_pb2.EnumValueOptions(), _b('\010\001'))
_ADAPTERCONFIG_MESSAGETYPE.values_by_name["HMI_COMMAND"].has_options = True
_ADAPTERCONFIG_MESSAGETYPE.values_by_name["HMI_COMMAND"]._options = _descriptor._ParseOptions(descriptor_pb2.EnumValueOptions(), _b('\010\001'))
# @@protoc_insertion_point(module_scope)
