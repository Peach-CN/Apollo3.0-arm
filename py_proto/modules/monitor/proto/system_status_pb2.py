# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/monitor/proto/system_status.proto

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
from modules.monitor.proto import monitor_conf_pb2 as modules_dot_monitor_dot_proto_dot_monitor__conf__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/monitor/proto/system_status.proto',
  package='apollo.monitor',
  syntax='proto2',
  serialized_pb=_b('\n)modules/monitor/proto/system_status.proto\x12\x0e\x61pollo.monitor\x1a!modules/common/proto/header.proto\x1a(modules/monitor/proto/monitor_conf.proto\"\x84\x03\n\x0eHardwareStatus\x12\x31\n\x07summary\x18\x01 \x01(\x0e\x32\x17.apollo.monitor.Summary:\x07UNKNOWN\x12\x0b\n\x03msg\x18\x02 \x01(\t\x12<\n\x06status\x18\x03 \x01(\x0e\x32%.apollo.monitor.HardwareStatus.Status:\x05UNDEF\x12\x14\n\x0c\x64\x65tailed_msg\x18\x06 \x01(\t\x12\x1f\n\x17gps_unstable_start_time\x18\x04 \x01(\x01\x12\x31\n\x0ctopic_status\x18\x05 \x01(\x0b\x32\x1b.apollo.monitor.TopicStatus\"\x89\x01\n\x06Status\x12\x06\n\x02OK\x10\x00\x12\r\n\tNOT_READY\x10\x01\x12\x0f\n\x0bNOT_PRESENT\x10\x02\x12\x08\n\x04WARN\x10\x06\x12\x07\n\x03\x45RR\x10\x03\x12\x18\n\x14GPS_UNSTABLE_WARNING\x10\x04\x12\x16\n\x12GPS_UNSTABLE_ERROR\x10\x05\x12\x12\n\x05UNDEF\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01\"\xb8\x01\n\x0cModuleStatus\x12\x31\n\x07summary\x18\x01 \x01(\x0e\x32\x17.apollo.monitor.Summary:\x07UNKNOWN\x12\x0b\n\x03msg\x18\x02 \x01(\t\x12\x35\n\x0eprocess_status\x18\x03 \x01(\x0b\x32\x1d.apollo.monitor.ProcessStatus\x12\x31\n\x0ctopic_status\x18\x04 \x01(\x0b\x32\x1b.apollo.monitor.TopicStatus\"\xa7\x03\n\x0cSystemStatus\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12:\n\x07modules\x18\x02 \x03(\x0b\x32).apollo.monitor.SystemStatus.ModulesEntry\x12<\n\x08hardware\x18\x03 \x03(\x0b\x32*.apollo.monitor.SystemStatus.HardwareEntry\x12\x15\n\rpassenger_msg\x18\x04 \x01(\t\x12 \n\x18safety_mode_trigger_time\x18\x05 \x01(\x01\x12\x1e\n\x16require_emergency_stop\x18\x06 \x01(\x08\x1aL\n\x0cModulesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12+\n\x05value\x18\x02 \x01(\x0b\x32\x1c.apollo.monitor.ModuleStatus:\x02\x38\x01\x1aO\n\rHardwareEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12-\n\x05value\x18\x02 \x01(\x0b\x32\x1e.apollo.monitor.HardwareStatus:\x02\x38\x01*>\n\x07Summary\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x06\n\x02OK\x10\x01\x12\x08\n\x04WARN\x10\x02\x12\t\n\x05\x45RROR\x10\x03\x12\t\n\x05\x46\x41TAL\x10\x04')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,modules_dot_monitor_dot_proto_dot_monitor__conf__pb2.DESCRIPTOR,])

_SUMMARY = _descriptor.EnumDescriptor(
  name='Summary',
  full_name='apollo.monitor.Summary',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OK', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WARN', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FATAL', index=4, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1142,
  serialized_end=1204,
)
_sym_db.RegisterEnumDescriptor(_SUMMARY)

Summary = enum_type_wrapper.EnumTypeWrapper(_SUMMARY)
UNKNOWN = 0
OK = 1
WARN = 2
ERROR = 3
FATAL = 4


_HARDWARESTATUS_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='apollo.monitor.HardwareStatus.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NOT_READY', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NOT_PRESENT', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WARN', index=3, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR', index=4, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GPS_UNSTABLE_WARNING', index=5, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GPS_UNSTABLE_ERROR', index=6, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNDEF', index=7, number=-1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=390,
  serialized_end=527,
)
_sym_db.RegisterEnumDescriptor(_HARDWARESTATUS_STATUS)


_HARDWARESTATUS = _descriptor.Descriptor(
  name='HardwareStatus',
  full_name='apollo.monitor.HardwareStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='summary', full_name='apollo.monitor.HardwareStatus.summary', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='msg', full_name='apollo.monitor.HardwareStatus.msg', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='apollo.monitor.HardwareStatus.status', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='detailed_msg', full_name='apollo.monitor.HardwareStatus.detailed_msg', index=3,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gps_unstable_start_time', full_name='apollo.monitor.HardwareStatus.gps_unstable_start_time', index=4,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='topic_status', full_name='apollo.monitor.HardwareStatus.topic_status', index=5,
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
    _HARDWARESTATUS_STATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=139,
  serialized_end=527,
)


_MODULESTATUS = _descriptor.Descriptor(
  name='ModuleStatus',
  full_name='apollo.monitor.ModuleStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='summary', full_name='apollo.monitor.ModuleStatus.summary', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='msg', full_name='apollo.monitor.ModuleStatus.msg', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='process_status', full_name='apollo.monitor.ModuleStatus.process_status', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='topic_status', full_name='apollo.monitor.ModuleStatus.topic_status', index=3,
      number=4, type=11, cpp_type=10, label=1,
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
  serialized_start=530,
  serialized_end=714,
)


_SYSTEMSTATUS_MODULESENTRY = _descriptor.Descriptor(
  name='ModulesEntry',
  full_name='apollo.monitor.SystemStatus.ModulesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.monitor.SystemStatus.ModulesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.monitor.SystemStatus.ModulesEntry.value', index=1,
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
  serialized_start=983,
  serialized_end=1059,
)

_SYSTEMSTATUS_HARDWAREENTRY = _descriptor.Descriptor(
  name='HardwareEntry',
  full_name='apollo.monitor.SystemStatus.HardwareEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.monitor.SystemStatus.HardwareEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.monitor.SystemStatus.HardwareEntry.value', index=1,
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
  serialized_start=1061,
  serialized_end=1140,
)

_SYSTEMSTATUS = _descriptor.Descriptor(
  name='SystemStatus',
  full_name='apollo.monitor.SystemStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.monitor.SystemStatus.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='modules', full_name='apollo.monitor.SystemStatus.modules', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='hardware', full_name='apollo.monitor.SystemStatus.hardware', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='passenger_msg', full_name='apollo.monitor.SystemStatus.passenger_msg', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='safety_mode_trigger_time', full_name='apollo.monitor.SystemStatus.safety_mode_trigger_time', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='require_emergency_stop', full_name='apollo.monitor.SystemStatus.require_emergency_stop', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_SYSTEMSTATUS_MODULESENTRY, _SYSTEMSTATUS_HARDWAREENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=717,
  serialized_end=1140,
)

_HARDWARESTATUS.fields_by_name['summary'].enum_type = _SUMMARY
_HARDWARESTATUS.fields_by_name['status'].enum_type = _HARDWARESTATUS_STATUS
_HARDWARESTATUS.fields_by_name['topic_status'].message_type = modules_dot_monitor_dot_proto_dot_monitor__conf__pb2._TOPICSTATUS
_HARDWARESTATUS_STATUS.containing_type = _HARDWARESTATUS
_MODULESTATUS.fields_by_name['summary'].enum_type = _SUMMARY
_MODULESTATUS.fields_by_name['process_status'].message_type = modules_dot_monitor_dot_proto_dot_monitor__conf__pb2._PROCESSSTATUS
_MODULESTATUS.fields_by_name['topic_status'].message_type = modules_dot_monitor_dot_proto_dot_monitor__conf__pb2._TOPICSTATUS
_SYSTEMSTATUS_MODULESENTRY.fields_by_name['value'].message_type = _MODULESTATUS
_SYSTEMSTATUS_MODULESENTRY.containing_type = _SYSTEMSTATUS
_SYSTEMSTATUS_HARDWAREENTRY.fields_by_name['value'].message_type = _HARDWARESTATUS
_SYSTEMSTATUS_HARDWAREENTRY.containing_type = _SYSTEMSTATUS
_SYSTEMSTATUS.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_SYSTEMSTATUS.fields_by_name['modules'].message_type = _SYSTEMSTATUS_MODULESENTRY
_SYSTEMSTATUS.fields_by_name['hardware'].message_type = _SYSTEMSTATUS_HARDWAREENTRY
DESCRIPTOR.message_types_by_name['HardwareStatus'] = _HARDWARESTATUS
DESCRIPTOR.message_types_by_name['ModuleStatus'] = _MODULESTATUS
DESCRIPTOR.message_types_by_name['SystemStatus'] = _SYSTEMSTATUS
DESCRIPTOR.enum_types_by_name['Summary'] = _SUMMARY
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HardwareStatus = _reflection.GeneratedProtocolMessageType('HardwareStatus', (_message.Message,), dict(
  DESCRIPTOR = _HARDWARESTATUS,
  __module__ = 'modules.monitor.proto.system_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.monitor.HardwareStatus)
  ))
_sym_db.RegisterMessage(HardwareStatus)

ModuleStatus = _reflection.GeneratedProtocolMessageType('ModuleStatus', (_message.Message,), dict(
  DESCRIPTOR = _MODULESTATUS,
  __module__ = 'modules.monitor.proto.system_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.monitor.ModuleStatus)
  ))
_sym_db.RegisterMessage(ModuleStatus)

SystemStatus = _reflection.GeneratedProtocolMessageType('SystemStatus', (_message.Message,), dict(

  ModulesEntry = _reflection.GeneratedProtocolMessageType('ModulesEntry', (_message.Message,), dict(
    DESCRIPTOR = _SYSTEMSTATUS_MODULESENTRY,
    __module__ = 'modules.monitor.proto.system_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.monitor.SystemStatus.ModulesEntry)
    ))
  ,

  HardwareEntry = _reflection.GeneratedProtocolMessageType('HardwareEntry', (_message.Message,), dict(
    DESCRIPTOR = _SYSTEMSTATUS_HARDWAREENTRY,
    __module__ = 'modules.monitor.proto.system_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.monitor.SystemStatus.HardwareEntry)
    ))
  ,
  DESCRIPTOR = _SYSTEMSTATUS,
  __module__ = 'modules.monitor.proto.system_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.monitor.SystemStatus)
  ))
_sym_db.RegisterMessage(SystemStatus)
_sym_db.RegisterMessage(SystemStatus.ModulesEntry)
_sym_db.RegisterMessage(SystemStatus.HardwareEntry)


_SYSTEMSTATUS_MODULESENTRY.has_options = True
_SYSTEMSTATUS_MODULESENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_SYSTEMSTATUS_HARDWAREENTRY.has_options = True
_SYSTEMSTATUS_HARDWAREENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)
