# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/data/proto/task.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.data.proto import static_info_pb2 as modules_dot_data_dot_proto_dot_static__info__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/data/proto/task.proto',
  package='apollo.data',
  syntax='proto2',
  serialized_pb=_b('\n\x1dmodules/data/proto/task.proto\x12\x0b\x61pollo.data\x1a$modules/data/proto/static_info.proto\"/\n\x08MapPoint\x12\x10\n\x08latitude\x18\x01 \x01(\x01\x12\x11\n\tlongitude\x18\x02 \x01(\x01\"\xaa\x02\n\x06Rosbag\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0c\n\x04size\x18\x02 \x01(\x04\x12\x0f\n\x07version\x18\x03 \x01(\x05\x12\x12\n\nstart_time\x18\x04 \x01(\x01\x12\x10\n\x08\x65nd_time\x18\x05 \x01(\x01\x12\x11\n\tmsg_count\x18\x06 \x01(\x05\x12/\n\x06topics\x18\x07 \x03(\x0b\x32\x1f.apollo.data.Rosbag.TopicsEntry\x1a?\n\x05Topic\x12\x10\n\x08msg_type\x18\x01 \x01(\t\x12\x11\n\tmsg_count\x18\x02 \x01(\x05\x12\x11\n\tfrequency\x18\x03 \x01(\x01\x1aH\n\x0bTopicsEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12(\n\x05value\x18\x02 \x01(\x0b\x32\x19.apollo.data.Rosbag.Topic:\x02\x38\x01\"T\n\rDisengagement\x12\x0c\n\x04time\x18\x01 \x01(\x01\x12\'\n\x08location\x18\x02 \x01(\x0b\x32\x15.apollo.data.MapPoint\x12\x0c\n\x04\x64\x65sc\x18\x03 \x01(\t\"\xb7\x03\n\x04Task\x12\n\n\x02id\x18\x01 \x01(\t\x12\x12\n\nstart_time\x18\x02 \x01(\x01\x12\x10\n\x08\x65nd_time\x18\x03 \x01(\x01\x12-\n\tloop_type\x18\x04 \x01(\x0e\x32\x1a.apollo.data.Task.LoopType\x12%\n\x04info\x18\x05 \x01(\x0b\x32\x17.apollo.data.StaticInfo\x12!\n\x04\x62\x61gs\x18\x06 \x03(\x0b\x32\x13.apollo.data.Rosbag\x12\x0e\n\x06topics\x18\x07 \x03(\t\x12\x32\n\x0e\x64isengagements\x18\x08 \x03(\x0b\x32\x1a.apollo.data.Disengagement\x12\'\n\x08map_path\x18\t \x03(\x0b\x32\x15.apollo.data.MapPoint\x12/\n\x07mileage\x18\n \x03(\x0b\x32\x1e.apollo.data.Task.MileageEntry\x1a.\n\x0cMileageEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\x01:\x02\x38\x01\"6\n\x08LoopType\x12\x0b\n\x07UNKNOWN\x10\x00\x12\r\n\tOPEN_LOOP\x10\x01\x12\x0e\n\nCLOSE_LOOP\x10\x02')
  ,
  dependencies=[modules_dot_data_dot_proto_dot_static__info__pb2.DESCRIPTOR,])



_TASK_LOOPTYPE = _descriptor.EnumDescriptor(
  name='LoopType',
  full_name='apollo.data.Task.LoopType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OPEN_LOOP', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CLOSE_LOOP', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=906,
  serialized_end=960,
)
_sym_db.RegisterEnumDescriptor(_TASK_LOOPTYPE)


_MAPPOINT = _descriptor.Descriptor(
  name='MapPoint',
  full_name='apollo.data.MapPoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='latitude', full_name='apollo.data.MapPoint.latitude', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='apollo.data.MapPoint.longitude', index=1,
      number=2, type=1, cpp_type=5, label=1,
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
  serialized_start=84,
  serialized_end=131,
)


_ROSBAG_TOPIC = _descriptor.Descriptor(
  name='Topic',
  full_name='apollo.data.Rosbag.Topic',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='msg_type', full_name='apollo.data.Rosbag.Topic.msg_type', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='msg_count', full_name='apollo.data.Rosbag.Topic.msg_count', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frequency', full_name='apollo.data.Rosbag.Topic.frequency', index=2,
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
  serialized_start=295,
  serialized_end=358,
)

_ROSBAG_TOPICSENTRY = _descriptor.Descriptor(
  name='TopicsEntry',
  full_name='apollo.data.Rosbag.TopicsEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.data.Rosbag.TopicsEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.data.Rosbag.TopicsEntry.value', index=1,
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
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=360,
  serialized_end=432,
)

_ROSBAG = _descriptor.Descriptor(
  name='Rosbag',
  full_name='apollo.data.Rosbag',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.data.Rosbag.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='size', full_name='apollo.data.Rosbag.size', index=1,
      number=2, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='version', full_name='apollo.data.Rosbag.version', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_time', full_name='apollo.data.Rosbag.start_time', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_time', full_name='apollo.data.Rosbag.end_time', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='msg_count', full_name='apollo.data.Rosbag.msg_count', index=5,
      number=6, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='topics', full_name='apollo.data.Rosbag.topics', index=6,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_ROSBAG_TOPIC, _ROSBAG_TOPICSENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=134,
  serialized_end=432,
)


_DISENGAGEMENT = _descriptor.Descriptor(
  name='Disengagement',
  full_name='apollo.data.Disengagement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='time', full_name='apollo.data.Disengagement.time', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='location', full_name='apollo.data.Disengagement.location', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='desc', full_name='apollo.data.Disengagement.desc', index=2,
      number=3, type=9, cpp_type=9, label=1,
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
  serialized_start=434,
  serialized_end=518,
)


_TASK_MILEAGEENTRY = _descriptor.Descriptor(
  name='MileageEntry',
  full_name='apollo.data.Task.MileageEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.data.Task.MileageEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.data.Task.MileageEntry.value', index=1,
      number=2, type=1, cpp_type=5, label=1,
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
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=858,
  serialized_end=904,
)

_TASK = _descriptor.Descriptor(
  name='Task',
  full_name='apollo.data.Task',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.data.Task.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_time', full_name='apollo.data.Task.start_time', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_time', full_name='apollo.data.Task.end_time', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='loop_type', full_name='apollo.data.Task.loop_type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='info', full_name='apollo.data.Task.info', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='bags', full_name='apollo.data.Task.bags', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='topics', full_name='apollo.data.Task.topics', index=6,
      number=7, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='disengagements', full_name='apollo.data.Task.disengagements', index=7,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='map_path', full_name='apollo.data.Task.map_path', index=8,
      number=9, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mileage', full_name='apollo.data.Task.mileage', index=9,
      number=10, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_TASK_MILEAGEENTRY, ],
  enum_types=[
    _TASK_LOOPTYPE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=521,
  serialized_end=960,
)

_ROSBAG_TOPIC.containing_type = _ROSBAG
_ROSBAG_TOPICSENTRY.fields_by_name['value'].message_type = _ROSBAG_TOPIC
_ROSBAG_TOPICSENTRY.containing_type = _ROSBAG
_ROSBAG.fields_by_name['topics'].message_type = _ROSBAG_TOPICSENTRY
_DISENGAGEMENT.fields_by_name['location'].message_type = _MAPPOINT
_TASK_MILEAGEENTRY.containing_type = _TASK
_TASK.fields_by_name['loop_type'].enum_type = _TASK_LOOPTYPE
_TASK.fields_by_name['info'].message_type = modules_dot_data_dot_proto_dot_static__info__pb2._STATICINFO
_TASK.fields_by_name['bags'].message_type = _ROSBAG
_TASK.fields_by_name['disengagements'].message_type = _DISENGAGEMENT
_TASK.fields_by_name['map_path'].message_type = _MAPPOINT
_TASK.fields_by_name['mileage'].message_type = _TASK_MILEAGEENTRY
_TASK_LOOPTYPE.containing_type = _TASK
DESCRIPTOR.message_types_by_name['MapPoint'] = _MAPPOINT
DESCRIPTOR.message_types_by_name['Rosbag'] = _ROSBAG
DESCRIPTOR.message_types_by_name['Disengagement'] = _DISENGAGEMENT
DESCRIPTOR.message_types_by_name['Task'] = _TASK
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MapPoint = _reflection.GeneratedProtocolMessageType('MapPoint', (_message.Message,), dict(
  DESCRIPTOR = _MAPPOINT,
  __module__ = 'modules.data.proto.task_pb2'
  # @@protoc_insertion_point(class_scope:apollo.data.MapPoint)
  ))
_sym_db.RegisterMessage(MapPoint)

Rosbag = _reflection.GeneratedProtocolMessageType('Rosbag', (_message.Message,), dict(

  Topic = _reflection.GeneratedProtocolMessageType('Topic', (_message.Message,), dict(
    DESCRIPTOR = _ROSBAG_TOPIC,
    __module__ = 'modules.data.proto.task_pb2'
    # @@protoc_insertion_point(class_scope:apollo.data.Rosbag.Topic)
    ))
  ,

  TopicsEntry = _reflection.GeneratedProtocolMessageType('TopicsEntry', (_message.Message,), dict(
    DESCRIPTOR = _ROSBAG_TOPICSENTRY,
    __module__ = 'modules.data.proto.task_pb2'
    # @@protoc_insertion_point(class_scope:apollo.data.Rosbag.TopicsEntry)
    ))
  ,
  DESCRIPTOR = _ROSBAG,
  __module__ = 'modules.data.proto.task_pb2'
  # @@protoc_insertion_point(class_scope:apollo.data.Rosbag)
  ))
_sym_db.RegisterMessage(Rosbag)
_sym_db.RegisterMessage(Rosbag.Topic)
_sym_db.RegisterMessage(Rosbag.TopicsEntry)

Disengagement = _reflection.GeneratedProtocolMessageType('Disengagement', (_message.Message,), dict(
  DESCRIPTOR = _DISENGAGEMENT,
  __module__ = 'modules.data.proto.task_pb2'
  # @@protoc_insertion_point(class_scope:apollo.data.Disengagement)
  ))
_sym_db.RegisterMessage(Disengagement)

Task = _reflection.GeneratedProtocolMessageType('Task', (_message.Message,), dict(

  MileageEntry = _reflection.GeneratedProtocolMessageType('MileageEntry', (_message.Message,), dict(
    DESCRIPTOR = _TASK_MILEAGEENTRY,
    __module__ = 'modules.data.proto.task_pb2'
    # @@protoc_insertion_point(class_scope:apollo.data.Task.MileageEntry)
    ))
  ,
  DESCRIPTOR = _TASK,
  __module__ = 'modules.data.proto.task_pb2'
  # @@protoc_insertion_point(class_scope:apollo.data.Task)
  ))
_sym_db.RegisterMessage(Task)
_sym_db.RegisterMessage(Task.MileageEntry)


_ROSBAG_TOPICSENTRY.has_options = True
_ROSBAG_TOPICSENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_TASK_MILEAGEENTRY.has_options = True
_TASK_MILEAGEENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)
