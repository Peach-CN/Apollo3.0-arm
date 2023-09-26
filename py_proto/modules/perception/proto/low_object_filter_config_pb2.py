# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/proto/low_object_filter_config.proto

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
  name='modules/perception/proto/low_object_filter_config.proto',
  package='apollo.perception.low_object_filter_config',
  syntax='proto2',
  serialized_pb=_b('\n7modules/perception/proto/low_object_filter_config.proto\x12*apollo.perception.low_object_filter_config\"\x9b\x01\n\x0cModelConfigs\x12\x1d\n\x04name\x18\x01 \x01(\t:\x0fLowObjectFilter\x12\x16\n\x07version\x18\x02 \x01(\t:\x05\x31.0.0\x12$\n\x17object_height_threshold\x18\x03 \x01(\x01:\x03\x30.1\x12.\n object_position_height_threshold\x18\x04 \x01(\x01:\x04-1.6')
)




_MODELCONFIGS = _descriptor.Descriptor(
  name='ModelConfigs',
  full_name='apollo.perception.low_object_filter_config.ModelConfigs',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.low_object_filter_config.ModelConfigs.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("LowObjectFilter").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='version', full_name='apollo.perception.low_object_filter_config.ModelConfigs.version', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("1.0.0").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='object_height_threshold', full_name='apollo.perception.low_object_filter_config.ModelConfigs.object_height_threshold', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='object_position_height_threshold', full_name='apollo.perception.low_object_filter_config.ModelConfigs.object_position_height_threshold', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(-1.6),
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
  serialized_start=104,
  serialized_end=259,
)

DESCRIPTOR.message_types_by_name['ModelConfigs'] = _MODELCONFIGS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ModelConfigs = _reflection.GeneratedProtocolMessageType('ModelConfigs', (_message.Message,), dict(
  DESCRIPTOR = _MODELCONFIGS,
  __module__ = 'modules.perception.proto.low_object_filter_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.low_object_filter_config.ModelConfigs)
  ))
_sym_db.RegisterMessage(ModelConfigs)


# @@protoc_insertion_point(module_scope)