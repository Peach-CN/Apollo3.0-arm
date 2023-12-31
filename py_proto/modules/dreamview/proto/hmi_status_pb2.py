# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/dreamview/proto/hmi_status.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.monitor.proto import system_status_pb2 as modules_dot_monitor_dot_proto_dot_system__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/dreamview/proto/hmi_status.proto',
  package='apollo.dreamview',
  syntax='proto2',
  serialized_pb=_b('\n(modules/dreamview/proto/hmi_status.proto\x12\x10\x61pollo.dreamview\x1a)modules/monitor/proto/system_status.proto\"\x8e\x01\n\tHMIStatus\x12\x33\n\rsystem_status\x18\x01 \x01(\x0b\x32\x1c.apollo.monitor.SystemStatus\x12\x13\n\x0b\x63urrent_map\x18\x02 \x01(\t\x12\x17\n\x0f\x63urrent_vehicle\x18\x03 \x01(\t\x12\x1e\n\x0c\x63urrent_mode\x18\x04 \x01(\t:\x08Standard')
  ,
  dependencies=[modules_dot_monitor_dot_proto_dot_system__status__pb2.DESCRIPTOR,])




_HMISTATUS = _descriptor.Descriptor(
  name='HMIStatus',
  full_name='apollo.dreamview.HMIStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='system_status', full_name='apollo.dreamview.HMIStatus.system_status', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_map', full_name='apollo.dreamview.HMIStatus.current_map', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_vehicle', full_name='apollo.dreamview.HMIStatus.current_vehicle', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='current_mode', full_name='apollo.dreamview.HMIStatus.current_mode', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("Standard").decode('utf-8'),
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
  serialized_start=106,
  serialized_end=248,
)

_HMISTATUS.fields_by_name['system_status'].message_type = modules_dot_monitor_dot_proto_dot_system__status__pb2._SYSTEMSTATUS
DESCRIPTOR.message_types_by_name['HMIStatus'] = _HMISTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HMIStatus = _reflection.GeneratedProtocolMessageType('HMIStatus', (_message.Message,), dict(
  DESCRIPTOR = _HMISTATUS,
  __module__ = 'modules.dreamview.proto.hmi_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIStatus)
  ))
_sym_db.RegisterMessage(HMIStatus)


# @@protoc_insertion_point(module_scope)
