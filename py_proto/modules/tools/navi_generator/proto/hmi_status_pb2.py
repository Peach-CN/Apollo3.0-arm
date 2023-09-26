# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/tools/navi_generator/proto/hmi_status.proto

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
from modules.monitor.proto import system_status_pb2 as modules_dot_monitor_dot_proto_dot_system__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/tools/navi_generator/proto/hmi_status.proto',
  package='apollo.navi_generator',
  syntax='proto2',
  serialized_pb=_b('\n3modules/tools/navi_generator/proto/hmi_status.proto\x12\x15\x61pollo.navi_generator\x1a!modules/common/proto/header.proto\x1a)modules/monitor/proto/system_status.proto\"J\n\x15HardwareStatusSummary\x12\x31\n\x07summary\x18\x01 \x01(\x0e\x32\x17.apollo.monitor.Summary:\x07UNKNOWN\"H\n\x13ModuleStatusSummary\x12\x31\n\x07summary\x18\x01 \x01(\x0e\x32\x17.apollo.monitor.Summary:\x07UNKNOWN\"\x8f\x03\n\tHMIStatus\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12>\n\x07modules\x18\x02 \x03(\x0b\x32-.apollo.navi_generator.HMIStatus.ModulesEntry\x12@\n\x08hardware\x18\x03 \x03(\x0b\x32..apollo.navi_generator.HMIStatus.HardwareEntry\x12\x1e\n\x0c\x63urrent_mode\x18\x04 \x01(\t:\x08Standard\x1aZ\n\x0cModulesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\x39\n\x05value\x18\x02 \x01(\x0b\x32*.apollo.navi_generator.ModuleStatusSummary:\x02\x38\x01\x1a]\n\rHardwareEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12;\n\x05value\x18\x02 \x01(\x0b\x32,.apollo.navi_generator.HardwareStatusSummary:\x02\x38\x01')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,modules_dot_monitor_dot_proto_dot_system__status__pb2.DESCRIPTOR,])




_HARDWARESTATUSSUMMARY = _descriptor.Descriptor(
  name='HardwareStatusSummary',
  full_name='apollo.navi_generator.HardwareStatusSummary',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='summary', full_name='apollo.navi_generator.HardwareStatusSummary.summary', index=0,
      number=1, type=14, cpp_type=8, label=1,
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
  serialized_start=156,
  serialized_end=230,
)


_MODULESTATUSSUMMARY = _descriptor.Descriptor(
  name='ModuleStatusSummary',
  full_name='apollo.navi_generator.ModuleStatusSummary',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='summary', full_name='apollo.navi_generator.ModuleStatusSummary.summary', index=0,
      number=1, type=14, cpp_type=8, label=1,
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
  serialized_start=232,
  serialized_end=304,
)


_HMISTATUS_MODULESENTRY = _descriptor.Descriptor(
  name='ModulesEntry',
  full_name='apollo.navi_generator.HMIStatus.ModulesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.navi_generator.HMIStatus.ModulesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.navi_generator.HMIStatus.ModulesEntry.value', index=1,
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
  serialized_start=521,
  serialized_end=611,
)

_HMISTATUS_HARDWAREENTRY = _descriptor.Descriptor(
  name='HardwareEntry',
  full_name='apollo.navi_generator.HMIStatus.HardwareEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.navi_generator.HMIStatus.HardwareEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.navi_generator.HMIStatus.HardwareEntry.value', index=1,
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
  serialized_start=613,
  serialized_end=706,
)

_HMISTATUS = _descriptor.Descriptor(
  name='HMIStatus',
  full_name='apollo.navi_generator.HMIStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.navi_generator.HMIStatus.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='modules', full_name='apollo.navi_generator.HMIStatus.modules', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='hardware', full_name='apollo.navi_generator.HMIStatus.hardware', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_mode', full_name='apollo.navi_generator.HMIStatus.current_mode', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("Standard").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_HMISTATUS_MODULESENTRY, _HMISTATUS_HARDWAREENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=307,
  serialized_end=706,
)

_HARDWARESTATUSSUMMARY.fields_by_name['summary'].enum_type = modules_dot_monitor_dot_proto_dot_system__status__pb2._SUMMARY
_MODULESTATUSSUMMARY.fields_by_name['summary'].enum_type = modules_dot_monitor_dot_proto_dot_system__status__pb2._SUMMARY
_HMISTATUS_MODULESENTRY.fields_by_name['value'].message_type = _MODULESTATUSSUMMARY
_HMISTATUS_MODULESENTRY.containing_type = _HMISTATUS
_HMISTATUS_HARDWAREENTRY.fields_by_name['value'].message_type = _HARDWARESTATUSSUMMARY
_HMISTATUS_HARDWAREENTRY.containing_type = _HMISTATUS
_HMISTATUS.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_HMISTATUS.fields_by_name['modules'].message_type = _HMISTATUS_MODULESENTRY
_HMISTATUS.fields_by_name['hardware'].message_type = _HMISTATUS_HARDWAREENTRY
DESCRIPTOR.message_types_by_name['HardwareStatusSummary'] = _HARDWARESTATUSSUMMARY
DESCRIPTOR.message_types_by_name['ModuleStatusSummary'] = _MODULESTATUSSUMMARY
DESCRIPTOR.message_types_by_name['HMIStatus'] = _HMISTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HardwareStatusSummary = _reflection.GeneratedProtocolMessageType('HardwareStatusSummary', (_message.Message,), dict(
  DESCRIPTOR = _HARDWARESTATUSSUMMARY,
  __module__ = 'modules.tools.navi_generator.proto.hmi_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.HardwareStatusSummary)
  ))
_sym_db.RegisterMessage(HardwareStatusSummary)

ModuleStatusSummary = _reflection.GeneratedProtocolMessageType('ModuleStatusSummary', (_message.Message,), dict(
  DESCRIPTOR = _MODULESTATUSSUMMARY,
  __module__ = 'modules.tools.navi_generator.proto.hmi_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.ModuleStatusSummary)
  ))
_sym_db.RegisterMessage(ModuleStatusSummary)

HMIStatus = _reflection.GeneratedProtocolMessageType('HMIStatus', (_message.Message,), dict(

  ModulesEntry = _reflection.GeneratedProtocolMessageType('ModulesEntry', (_message.Message,), dict(
    DESCRIPTOR = _HMISTATUS_MODULESENTRY,
    __module__ = 'modules.tools.navi_generator.proto.hmi_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.navi_generator.HMIStatus.ModulesEntry)
    ))
  ,

  HardwareEntry = _reflection.GeneratedProtocolMessageType('HardwareEntry', (_message.Message,), dict(
    DESCRIPTOR = _HMISTATUS_HARDWAREENTRY,
    __module__ = 'modules.tools.navi_generator.proto.hmi_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.navi_generator.HMIStatus.HardwareEntry)
    ))
  ,
  DESCRIPTOR = _HMISTATUS,
  __module__ = 'modules.tools.navi_generator.proto.hmi_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.navi_generator.HMIStatus)
  ))
_sym_db.RegisterMessage(HMIStatus)
_sym_db.RegisterMessage(HMIStatus.ModulesEntry)
_sym_db.RegisterMessage(HMIStatus.HardwareEntry)


_HMISTATUS_MODULESENTRY.has_options = True
_HMISTATUS_MODULESENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_HMISTATUS_HARDWAREENTRY.has_options = True
_HMISTATUS_HARDWAREENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)