# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/lib/config_manager/proto/perception_config_schema.proto

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
  name='modules/perception/lib/config_manager/proto/perception_config_schema.proto',
  package='apollo.perception.config_manager',
  syntax='proto2',
  serialized_pb=_b('\nJmodules/perception/lib/config_manager/proto/perception_config_schema.proto\x12 apollo.perception.config_manager\"*\n\x0bKeyValueInt\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\r\n\x05value\x18\x02 \x02(\x05\"-\n\x0eKeyValueString\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\r\n\x05value\x18\x02 \x02(\x0c\"-\n\x0eKeyValueDouble\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\r\n\x05value\x18\x02 \x02(\x01\",\n\rKeyValueFloat\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\r\n\x05value\x18\x02 \x02(\x02\"+\n\x0cKeyValueBool\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\r\n\x05value\x18\x02 \x02(\x08\"0\n\x10KeyValueArrayInt\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\x0e\n\x06values\x18\x02 \x03(\x05\"3\n\x13KeyValueArrayString\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\x0e\n\x06values\x18\x02 \x03(\x0c\"3\n\x13KeyValueArrayDouble\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\x0e\n\x06values\x18\x02 \x03(\x01\"2\n\x12KeyValueArrayFloat\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\x0e\n\x06values\x18\x02 \x03(\x02\"1\n\x11KeyValueArrayBool\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\x0e\n\x06values\x18\x02 \x03(\x08\"\xb2\x06\n\x10ModelConfigProto\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\x0f\n\x07version\x18\x02 \x01(\t\x12\x45\n\x0einteger_params\x18\x03 \x03(\x0b\x32-.apollo.perception.config_manager.KeyValueInt\x12G\n\rstring_params\x18\x04 \x03(\x0b\x32\x30.apollo.perception.config_manager.KeyValueString\x12G\n\rdouble_params\x18\x05 \x03(\x0b\x32\x30.apollo.perception.config_manager.KeyValueDouble\x12\x45\n\x0c\x66loat_params\x18\x06 \x03(\x0b\x32/.apollo.perception.config_manager.KeyValueFloat\x12\x43\n\x0b\x62ool_params\x18\x07 \x03(\x0b\x32..apollo.perception.config_manager.KeyValueBool\x12P\n\x14\x61rray_integer_params\x18\x08 \x03(\x0b\x32\x32.apollo.perception.config_manager.KeyValueArrayInt\x12R\n\x13\x61rray_string_params\x18\t \x03(\x0b\x32\x35.apollo.perception.config_manager.KeyValueArrayString\x12R\n\x13\x61rray_double_params\x18\n \x03(\x0b\x32\x35.apollo.perception.config_manager.KeyValueArrayDouble\x12P\n\x12\x61rray_float_params\x18\x0b \x03(\x0b\x32\x34.apollo.perception.config_manager.KeyValueArrayFloat\x12N\n\x11\x61rray_bool_params\x18\x0c \x03(\x0b\x32\x33.apollo.perception.config_manager.KeyValueArrayBool\"b\n\x15MultiModelConfigProto\x12I\n\rmodel_configs\x18\x01 \x03(\x0b\x32\x32.apollo.perception.config_manager.ModelConfigProto\"5\n\x18ModelConfigFileListProto\x12\x19\n\x11model_config_path\x18\x01 \x03(\t')
)




_KEYVALUEINT = _descriptor.Descriptor(
  name='KeyValueInt',
  full_name='apollo.perception.config_manager.KeyValueInt',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueInt.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.perception.config_manager.KeyValueInt.value', index=1,
      number=2, type=5, cpp_type=1, label=2,
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
  serialized_start=112,
  serialized_end=154,
)


_KEYVALUESTRING = _descriptor.Descriptor(
  name='KeyValueString',
  full_name='apollo.perception.config_manager.KeyValueString',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueString.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.perception.config_manager.KeyValueString.value', index=1,
      number=2, type=12, cpp_type=9, label=2,
      has_default_value=False, default_value=_b(""),
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
  serialized_end=201,
)


_KEYVALUEDOUBLE = _descriptor.Descriptor(
  name='KeyValueDouble',
  full_name='apollo.perception.config_manager.KeyValueDouble',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueDouble.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.perception.config_manager.KeyValueDouble.value', index=1,
      number=2, type=1, cpp_type=5, label=2,
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
  serialized_start=203,
  serialized_end=248,
)


_KEYVALUEFLOAT = _descriptor.Descriptor(
  name='KeyValueFloat',
  full_name='apollo.perception.config_manager.KeyValueFloat',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueFloat.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.perception.config_manager.KeyValueFloat.value', index=1,
      number=2, type=2, cpp_type=6, label=2,
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
  serialized_start=250,
  serialized_end=294,
)


_KEYVALUEBOOL = _descriptor.Descriptor(
  name='KeyValueBool',
  full_name='apollo.perception.config_manager.KeyValueBool',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueBool.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.perception.config_manager.KeyValueBool.value', index=1,
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
  serialized_start=296,
  serialized_end=339,
)


_KEYVALUEARRAYINT = _descriptor.Descriptor(
  name='KeyValueArrayInt',
  full_name='apollo.perception.config_manager.KeyValueArrayInt',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueArrayInt.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='values', full_name='apollo.perception.config_manager.KeyValueArrayInt.values', index=1,
      number=2, type=5, cpp_type=1, label=3,
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
  serialized_start=341,
  serialized_end=389,
)


_KEYVALUEARRAYSTRING = _descriptor.Descriptor(
  name='KeyValueArrayString',
  full_name='apollo.perception.config_manager.KeyValueArrayString',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueArrayString.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='values', full_name='apollo.perception.config_manager.KeyValueArrayString.values', index=1,
      number=2, type=12, cpp_type=9, label=3,
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
  serialized_start=391,
  serialized_end=442,
)


_KEYVALUEARRAYDOUBLE = _descriptor.Descriptor(
  name='KeyValueArrayDouble',
  full_name='apollo.perception.config_manager.KeyValueArrayDouble',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueArrayDouble.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='values', full_name='apollo.perception.config_manager.KeyValueArrayDouble.values', index=1,
      number=2, type=1, cpp_type=5, label=3,
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
  serialized_start=444,
  serialized_end=495,
)


_KEYVALUEARRAYFLOAT = _descriptor.Descriptor(
  name='KeyValueArrayFloat',
  full_name='apollo.perception.config_manager.KeyValueArrayFloat',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueArrayFloat.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='values', full_name='apollo.perception.config_manager.KeyValueArrayFloat.values', index=1,
      number=2, type=2, cpp_type=6, label=3,
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
  serialized_start=497,
  serialized_end=547,
)


_KEYVALUEARRAYBOOL = _descriptor.Descriptor(
  name='KeyValueArrayBool',
  full_name='apollo.perception.config_manager.KeyValueArrayBool',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.KeyValueArrayBool.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='values', full_name='apollo.perception.config_manager.KeyValueArrayBool.values', index=1,
      number=2, type=8, cpp_type=7, label=3,
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
  serialized_start=549,
  serialized_end=598,
)


_MODELCONFIGPROTO = _descriptor.Descriptor(
  name='ModelConfigProto',
  full_name='apollo.perception.config_manager.ModelConfigProto',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.config_manager.ModelConfigProto.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='version', full_name='apollo.perception.config_manager.ModelConfigProto.version', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='integer_params', full_name='apollo.perception.config_manager.ModelConfigProto.integer_params', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='string_params', full_name='apollo.perception.config_manager.ModelConfigProto.string_params', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='double_params', full_name='apollo.perception.config_manager.ModelConfigProto.double_params', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='float_params', full_name='apollo.perception.config_manager.ModelConfigProto.float_params', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='bool_params', full_name='apollo.perception.config_manager.ModelConfigProto.bool_params', index=6,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='array_integer_params', full_name='apollo.perception.config_manager.ModelConfigProto.array_integer_params', index=7,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='array_string_params', full_name='apollo.perception.config_manager.ModelConfigProto.array_string_params', index=8,
      number=9, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='array_double_params', full_name='apollo.perception.config_manager.ModelConfigProto.array_double_params', index=9,
      number=10, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='array_float_params', full_name='apollo.perception.config_manager.ModelConfigProto.array_float_params', index=10,
      number=11, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='array_bool_params', full_name='apollo.perception.config_manager.ModelConfigProto.array_bool_params', index=11,
      number=12, type=11, cpp_type=10, label=3,
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
  serialized_start=601,
  serialized_end=1419,
)


_MULTIMODELCONFIGPROTO = _descriptor.Descriptor(
  name='MultiModelConfigProto',
  full_name='apollo.perception.config_manager.MultiModelConfigProto',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='model_configs', full_name='apollo.perception.config_manager.MultiModelConfigProto.model_configs', index=0,
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
  serialized_start=1421,
  serialized_end=1519,
)


_MODELCONFIGFILELISTPROTO = _descriptor.Descriptor(
  name='ModelConfigFileListProto',
  full_name='apollo.perception.config_manager.ModelConfigFileListProto',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='model_config_path', full_name='apollo.perception.config_manager.ModelConfigFileListProto.model_config_path', index=0,
      number=1, type=9, cpp_type=9, label=3,
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
  serialized_start=1521,
  serialized_end=1574,
)

_MODELCONFIGPROTO.fields_by_name['integer_params'].message_type = _KEYVALUEINT
_MODELCONFIGPROTO.fields_by_name['string_params'].message_type = _KEYVALUESTRING
_MODELCONFIGPROTO.fields_by_name['double_params'].message_type = _KEYVALUEDOUBLE
_MODELCONFIGPROTO.fields_by_name['float_params'].message_type = _KEYVALUEFLOAT
_MODELCONFIGPROTO.fields_by_name['bool_params'].message_type = _KEYVALUEBOOL
_MODELCONFIGPROTO.fields_by_name['array_integer_params'].message_type = _KEYVALUEARRAYINT
_MODELCONFIGPROTO.fields_by_name['array_string_params'].message_type = _KEYVALUEARRAYSTRING
_MODELCONFIGPROTO.fields_by_name['array_double_params'].message_type = _KEYVALUEARRAYDOUBLE
_MODELCONFIGPROTO.fields_by_name['array_float_params'].message_type = _KEYVALUEARRAYFLOAT
_MODELCONFIGPROTO.fields_by_name['array_bool_params'].message_type = _KEYVALUEARRAYBOOL
_MULTIMODELCONFIGPROTO.fields_by_name['model_configs'].message_type = _MODELCONFIGPROTO
DESCRIPTOR.message_types_by_name['KeyValueInt'] = _KEYVALUEINT
DESCRIPTOR.message_types_by_name['KeyValueString'] = _KEYVALUESTRING
DESCRIPTOR.message_types_by_name['KeyValueDouble'] = _KEYVALUEDOUBLE
DESCRIPTOR.message_types_by_name['KeyValueFloat'] = _KEYVALUEFLOAT
DESCRIPTOR.message_types_by_name['KeyValueBool'] = _KEYVALUEBOOL
DESCRIPTOR.message_types_by_name['KeyValueArrayInt'] = _KEYVALUEARRAYINT
DESCRIPTOR.message_types_by_name['KeyValueArrayString'] = _KEYVALUEARRAYSTRING
DESCRIPTOR.message_types_by_name['KeyValueArrayDouble'] = _KEYVALUEARRAYDOUBLE
DESCRIPTOR.message_types_by_name['KeyValueArrayFloat'] = _KEYVALUEARRAYFLOAT
DESCRIPTOR.message_types_by_name['KeyValueArrayBool'] = _KEYVALUEARRAYBOOL
DESCRIPTOR.message_types_by_name['ModelConfigProto'] = _MODELCONFIGPROTO
DESCRIPTOR.message_types_by_name['MultiModelConfigProto'] = _MULTIMODELCONFIGPROTO
DESCRIPTOR.message_types_by_name['ModelConfigFileListProto'] = _MODELCONFIGFILELISTPROTO
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

KeyValueInt = _reflection.GeneratedProtocolMessageType('KeyValueInt', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEINT,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueInt)
  ))
_sym_db.RegisterMessage(KeyValueInt)

KeyValueString = _reflection.GeneratedProtocolMessageType('KeyValueString', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUESTRING,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueString)
  ))
_sym_db.RegisterMessage(KeyValueString)

KeyValueDouble = _reflection.GeneratedProtocolMessageType('KeyValueDouble', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEDOUBLE,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueDouble)
  ))
_sym_db.RegisterMessage(KeyValueDouble)

KeyValueFloat = _reflection.GeneratedProtocolMessageType('KeyValueFloat', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEFLOAT,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueFloat)
  ))
_sym_db.RegisterMessage(KeyValueFloat)

KeyValueBool = _reflection.GeneratedProtocolMessageType('KeyValueBool', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEBOOL,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueBool)
  ))
_sym_db.RegisterMessage(KeyValueBool)

KeyValueArrayInt = _reflection.GeneratedProtocolMessageType('KeyValueArrayInt', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEARRAYINT,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueArrayInt)
  ))
_sym_db.RegisterMessage(KeyValueArrayInt)

KeyValueArrayString = _reflection.GeneratedProtocolMessageType('KeyValueArrayString', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEARRAYSTRING,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueArrayString)
  ))
_sym_db.RegisterMessage(KeyValueArrayString)

KeyValueArrayDouble = _reflection.GeneratedProtocolMessageType('KeyValueArrayDouble', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEARRAYDOUBLE,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueArrayDouble)
  ))
_sym_db.RegisterMessage(KeyValueArrayDouble)

KeyValueArrayFloat = _reflection.GeneratedProtocolMessageType('KeyValueArrayFloat', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEARRAYFLOAT,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueArrayFloat)
  ))
_sym_db.RegisterMessage(KeyValueArrayFloat)

KeyValueArrayBool = _reflection.GeneratedProtocolMessageType('KeyValueArrayBool', (_message.Message,), dict(
  DESCRIPTOR = _KEYVALUEARRAYBOOL,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.KeyValueArrayBool)
  ))
_sym_db.RegisterMessage(KeyValueArrayBool)

ModelConfigProto = _reflection.GeneratedProtocolMessageType('ModelConfigProto', (_message.Message,), dict(
  DESCRIPTOR = _MODELCONFIGPROTO,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.ModelConfigProto)
  ))
_sym_db.RegisterMessage(ModelConfigProto)

MultiModelConfigProto = _reflection.GeneratedProtocolMessageType('MultiModelConfigProto', (_message.Message,), dict(
  DESCRIPTOR = _MULTIMODELCONFIGPROTO,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.MultiModelConfigProto)
  ))
_sym_db.RegisterMessage(MultiModelConfigProto)

ModelConfigFileListProto = _reflection.GeneratedProtocolMessageType('ModelConfigFileListProto', (_message.Message,), dict(
  DESCRIPTOR = _MODELCONFIGFILELISTPROTO,
  __module__ = 'modules.perception.lib.config_manager.proto.perception_config_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.config_manager.ModelConfigFileListProto)
  ))
_sym_db.RegisterMessage(ModelConfigFileListProto)


# @@protoc_insertion_point(module_scope)