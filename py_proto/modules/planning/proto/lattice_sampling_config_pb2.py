# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/lattice_sampling_config.proto

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
  name='modules/planning/proto/lattice_sampling_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n4modules/planning/proto/lattice_sampling_config.proto\x12\x0f\x61pollo.planning\";\n\x0cLonCondition\x12\x0c\n\x01s\x18\x01 \x01(\x01:\x01\x30\x12\r\n\x02\x64s\x18\x02 \x01(\x01:\x01\x30\x12\x0e\n\x03\x64\x64s\x18\x03 \x01(\x01:\x01\x30\";\n\x0cLatCondition\x12\x0c\n\x01l\x18\x01 \x01(\x01:\x01\x30\x12\r\n\x02\x64l\x18\x02 \x01(\x01:\x01\x30\x12\x0e\n\x03\x64\x64l\x18\x03 \x01(\x01:\x01\x30\"E\n\tTStrategy\x12\x11\n\tt_markers\x18\x01 \x03(\x01\x12\x13\n\x06t_step\x18\x02 \x01(\x01:\x03\x30.5\x12\x10\n\x08strategy\x18\x03 \x01(\t\"E\n\tSStrategy\x12\x11\n\ts_markers\x18\x01 \x03(\x01\x12\x13\n\x06s_step\x18\x02 \x01(\x01:\x03\x30.5\x12\x10\n\x08strategy\x18\x03 \x01(\t\"{\n\x0fLonSampleConfig\x12\x38\n\x11lon_end_condition\x18\x01 \x01(\x0b\x32\x1d.apollo.planning.LonCondition\x12.\n\nt_strategy\x18\x02 \x01(\x0b\x32\x1a.apollo.planning.TStrategy\"{\n\x0fLatSampleConfig\x12\x38\n\x11lat_end_condition\x18\x01 \x01(\x0b\x32\x1d.apollo.planning.LatCondition\x12.\n\ns_strategy\x18\x02 \x01(\x0b\x32\x1a.apollo.planning.SStrategy\"\x91\x01\n\x15LatticeSamplingConfig\x12;\n\x11lon_sample_config\x18\x01 \x01(\x0b\x32 .apollo.planning.LonSampleConfig\x12;\n\x11lat_sample_config\x18\x02 \x01(\x0b\x32 .apollo.planning.LatSampleConfig')
)




_LONCONDITION = _descriptor.Descriptor(
  name='LonCondition',
  full_name='apollo.planning.LonCondition',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='s', full_name='apollo.planning.LonCondition.s', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ds', full_name='apollo.planning.LonCondition.ds', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dds', full_name='apollo.planning.LonCondition.dds', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
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
  serialized_start=73,
  serialized_end=132,
)


_LATCONDITION = _descriptor.Descriptor(
  name='LatCondition',
  full_name='apollo.planning.LatCondition',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='l', full_name='apollo.planning.LatCondition.l', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dl', full_name='apollo.planning.LatCondition.dl', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ddl', full_name='apollo.planning.LatCondition.ddl', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
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
  serialized_start=134,
  serialized_end=193,
)


_TSTRATEGY = _descriptor.Descriptor(
  name='TStrategy',
  full_name='apollo.planning.TStrategy',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='t_markers', full_name='apollo.planning.TStrategy.t_markers', index=0,
      number=1, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='t_step', full_name='apollo.planning.TStrategy.t_step', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.5),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='strategy', full_name='apollo.planning.TStrategy.strategy', index=2,
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
  serialized_start=195,
  serialized_end=264,
)


_SSTRATEGY = _descriptor.Descriptor(
  name='SStrategy',
  full_name='apollo.planning.SStrategy',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='s_markers', full_name='apollo.planning.SStrategy.s_markers', index=0,
      number=1, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s_step', full_name='apollo.planning.SStrategy.s_step', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.5),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='strategy', full_name='apollo.planning.SStrategy.strategy', index=2,
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
  serialized_start=266,
  serialized_end=335,
)


_LONSAMPLECONFIG = _descriptor.Descriptor(
  name='LonSampleConfig',
  full_name='apollo.planning.LonSampleConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lon_end_condition', full_name='apollo.planning.LonSampleConfig.lon_end_condition', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='t_strategy', full_name='apollo.planning.LonSampleConfig.t_strategy', index=1,
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
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=337,
  serialized_end=460,
)


_LATSAMPLECONFIG = _descriptor.Descriptor(
  name='LatSampleConfig',
  full_name='apollo.planning.LatSampleConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lat_end_condition', full_name='apollo.planning.LatSampleConfig.lat_end_condition', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s_strategy', full_name='apollo.planning.LatSampleConfig.s_strategy', index=1,
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
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=462,
  serialized_end=585,
)


_LATTICESAMPLINGCONFIG = _descriptor.Descriptor(
  name='LatticeSamplingConfig',
  full_name='apollo.planning.LatticeSamplingConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lon_sample_config', full_name='apollo.planning.LatticeSamplingConfig.lon_sample_config', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lat_sample_config', full_name='apollo.planning.LatticeSamplingConfig.lat_sample_config', index=1,
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
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=588,
  serialized_end=733,
)

_LONSAMPLECONFIG.fields_by_name['lon_end_condition'].message_type = _LONCONDITION
_LONSAMPLECONFIG.fields_by_name['t_strategy'].message_type = _TSTRATEGY
_LATSAMPLECONFIG.fields_by_name['lat_end_condition'].message_type = _LATCONDITION
_LATSAMPLECONFIG.fields_by_name['s_strategy'].message_type = _SSTRATEGY
_LATTICESAMPLINGCONFIG.fields_by_name['lon_sample_config'].message_type = _LONSAMPLECONFIG
_LATTICESAMPLINGCONFIG.fields_by_name['lat_sample_config'].message_type = _LATSAMPLECONFIG
DESCRIPTOR.message_types_by_name['LonCondition'] = _LONCONDITION
DESCRIPTOR.message_types_by_name['LatCondition'] = _LATCONDITION
DESCRIPTOR.message_types_by_name['TStrategy'] = _TSTRATEGY
DESCRIPTOR.message_types_by_name['SStrategy'] = _SSTRATEGY
DESCRIPTOR.message_types_by_name['LonSampleConfig'] = _LONSAMPLECONFIG
DESCRIPTOR.message_types_by_name['LatSampleConfig'] = _LATSAMPLECONFIG
DESCRIPTOR.message_types_by_name['LatticeSamplingConfig'] = _LATTICESAMPLINGCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LonCondition = _reflection.GeneratedProtocolMessageType('LonCondition', (_message.Message,), dict(
  DESCRIPTOR = _LONCONDITION,
  __module__ = 'modules.planning.proto.lattice_sampling_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.LonCondition)
  ))
_sym_db.RegisterMessage(LonCondition)

LatCondition = _reflection.GeneratedProtocolMessageType('LatCondition', (_message.Message,), dict(
  DESCRIPTOR = _LATCONDITION,
  __module__ = 'modules.planning.proto.lattice_sampling_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.LatCondition)
  ))
_sym_db.RegisterMessage(LatCondition)

TStrategy = _reflection.GeneratedProtocolMessageType('TStrategy', (_message.Message,), dict(
  DESCRIPTOR = _TSTRATEGY,
  __module__ = 'modules.planning.proto.lattice_sampling_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.TStrategy)
  ))
_sym_db.RegisterMessage(TStrategy)

SStrategy = _reflection.GeneratedProtocolMessageType('SStrategy', (_message.Message,), dict(
  DESCRIPTOR = _SSTRATEGY,
  __module__ = 'modules.planning.proto.lattice_sampling_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.SStrategy)
  ))
_sym_db.RegisterMessage(SStrategy)

LonSampleConfig = _reflection.GeneratedProtocolMessageType('LonSampleConfig', (_message.Message,), dict(
  DESCRIPTOR = _LONSAMPLECONFIG,
  __module__ = 'modules.planning.proto.lattice_sampling_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.LonSampleConfig)
  ))
_sym_db.RegisterMessage(LonSampleConfig)

LatSampleConfig = _reflection.GeneratedProtocolMessageType('LatSampleConfig', (_message.Message,), dict(
  DESCRIPTOR = _LATSAMPLECONFIG,
  __module__ = 'modules.planning.proto.lattice_sampling_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.LatSampleConfig)
  ))
_sym_db.RegisterMessage(LatSampleConfig)

LatticeSamplingConfig = _reflection.GeneratedProtocolMessageType('LatticeSamplingConfig', (_message.Message,), dict(
  DESCRIPTOR = _LATTICESAMPLINGCONFIG,
  __module__ = 'modules.planning.proto.lattice_sampling_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.LatticeSamplingConfig)
  ))
_sym_db.RegisterMessage(LatticeSamplingConfig)


# @@protoc_insertion_point(module_scope)
