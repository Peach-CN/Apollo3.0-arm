# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/onboard/proto/dag_config.proto

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
  name='modules/perception/onboard/proto/dag_config.proto',
  package='apollo.perception',
  syntax='proto2',
  serialized_pb=_b('\n1modules/perception/onboard/proto/dag_config.proto\x12\x11\x61pollo.perception\"\x9f\x06\n\tDAGConfig\x12\x42\n\x0esubnode_config\x18\x01 \x02(\x0b\x32*.apollo.perception.DAGConfig.SubnodeConfig\x12<\n\x0b\x65\x64ge_config\x18\x02 \x02(\x0b\x32\'.apollo.perception.DAGConfig.EdgeConfig\x12\x42\n\x0b\x64\x61ta_config\x18\x03 \x02(\x0b\x32-.apollo.perception.DAGConfig.SharedDataConfig\x1a|\n\x07Subnode\x12\n\n\x02id\x18\x01 \x02(\x05\x12\x0c\n\x04name\x18\x02 \x02(\t\x12\x0f\n\x07reserve\x18\x03 \x01(\t\x12\x46\n\x04type\x18\x04 \x01(\x0e\x32(.apollo.perception.DAGConfig.SubnodeType:\x0eSUBNODE_NORMAL\x1aG\n\rSubnodeConfig\x12\x36\n\x08subnodes\x18\x01 \x03(\x0b\x32$.apollo.perception.DAGConfig.Subnode\x1a!\n\x05\x45vent\x12\n\n\x02id\x18\x01 \x02(\x05\x12\x0c\n\x04name\x18\x02 \x01(\t\x1aj\n\x04\x45\x64ge\x12\n\n\x02id\x18\x01 \x02(\x05\x12\x11\n\tfrom_node\x18\x02 \x02(\x05\x12\x0f\n\x07to_node\x18\x03 \x02(\x05\x12\x32\n\x06\x65vents\x18\x04 \x03(\x0b\x32\".apollo.perception.DAGConfig.Event\x1a>\n\nEdgeConfig\x12\x30\n\x05\x65\x64ges\x18\x01 \x03(\x0b\x32!.apollo.perception.DAGConfig.Edge\x1a&\n\nSharedData\x12\n\n\x02id\x18\x01 \x02(\x05\x12\x0c\n\x04name\x18\x02 \x02(\t\x1aJ\n\x10SharedDataConfig\x12\x36\n\x05\x64\x61tas\x18\x01 \x03(\x0b\x32\'.apollo.perception.DAGConfig.SharedData\"B\n\x0bSubnodeType\x12\x0e\n\nSUBNODE_IN\x10\x01\x12\x0f\n\x0bSUBNODE_OUT\x10\x02\x12\x12\n\x0eSUBNODE_NORMAL\x10\x03')
)



_DAGCONFIG_SUBNODETYPE = _descriptor.EnumDescriptor(
  name='SubnodeType',
  full_name='apollo.perception.DAGConfig.SubnodeType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SUBNODE_IN', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SUBNODE_OUT', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SUBNODE_NORMAL', index=2, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=806,
  serialized_end=872,
)
_sym_db.RegisterEnumDescriptor(_DAGCONFIG_SUBNODETYPE)


_DAGCONFIG_SUBNODE = _descriptor.Descriptor(
  name='Subnode',
  full_name='apollo.perception.DAGConfig.Subnode',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.perception.DAGConfig.Subnode.id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.DAGConfig.Subnode.name', index=1,
      number=2, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reserve', full_name='apollo.perception.DAGConfig.Subnode.reserve', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='apollo.perception.DAGConfig.Subnode.type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=3,
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
  serialized_start=284,
  serialized_end=408,
)

_DAGCONFIG_SUBNODECONFIG = _descriptor.Descriptor(
  name='SubnodeConfig',
  full_name='apollo.perception.DAGConfig.SubnodeConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='subnodes', full_name='apollo.perception.DAGConfig.SubnodeConfig.subnodes', index=0,
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
  serialized_start=410,
  serialized_end=481,
)

_DAGCONFIG_EVENT = _descriptor.Descriptor(
  name='Event',
  full_name='apollo.perception.DAGConfig.Event',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.perception.DAGConfig.Event.id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.DAGConfig.Event.name', index=1,
      number=2, type=9, cpp_type=9, label=1,
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
  serialized_start=483,
  serialized_end=516,
)

_DAGCONFIG_EDGE = _descriptor.Descriptor(
  name='Edge',
  full_name='apollo.perception.DAGConfig.Edge',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.perception.DAGConfig.Edge.id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='from_node', full_name='apollo.perception.DAGConfig.Edge.from_node', index=1,
      number=2, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='to_node', full_name='apollo.perception.DAGConfig.Edge.to_node', index=2,
      number=3, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='events', full_name='apollo.perception.DAGConfig.Edge.events', index=3,
      number=4, type=11, cpp_type=10, label=3,
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
  serialized_start=518,
  serialized_end=624,
)

_DAGCONFIG_EDGECONFIG = _descriptor.Descriptor(
  name='EdgeConfig',
  full_name='apollo.perception.DAGConfig.EdgeConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='edges', full_name='apollo.perception.DAGConfig.EdgeConfig.edges', index=0,
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
  serialized_start=626,
  serialized_end=688,
)

_DAGCONFIG_SHAREDDATA = _descriptor.Descriptor(
  name='SharedData',
  full_name='apollo.perception.DAGConfig.SharedData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.perception.DAGConfig.SharedData.id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.DAGConfig.SharedData.name', index=1,
      number=2, type=9, cpp_type=9, label=2,
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
  serialized_start=690,
  serialized_end=728,
)

_DAGCONFIG_SHAREDDATACONFIG = _descriptor.Descriptor(
  name='SharedDataConfig',
  full_name='apollo.perception.DAGConfig.SharedDataConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='datas', full_name='apollo.perception.DAGConfig.SharedDataConfig.datas', index=0,
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
  serialized_start=730,
  serialized_end=804,
)

_DAGCONFIG = _descriptor.Descriptor(
  name='DAGConfig',
  full_name='apollo.perception.DAGConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='subnode_config', full_name='apollo.perception.DAGConfig.subnode_config', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='edge_config', full_name='apollo.perception.DAGConfig.edge_config', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='data_config', full_name='apollo.perception.DAGConfig.data_config', index=2,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_DAGCONFIG_SUBNODE, _DAGCONFIG_SUBNODECONFIG, _DAGCONFIG_EVENT, _DAGCONFIG_EDGE, _DAGCONFIG_EDGECONFIG, _DAGCONFIG_SHAREDDATA, _DAGCONFIG_SHAREDDATACONFIG, ],
  enum_types=[
    _DAGCONFIG_SUBNODETYPE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=73,
  serialized_end=872,
)

_DAGCONFIG_SUBNODE.fields_by_name['type'].enum_type = _DAGCONFIG_SUBNODETYPE
_DAGCONFIG_SUBNODE.containing_type = _DAGCONFIG
_DAGCONFIG_SUBNODECONFIG.fields_by_name['subnodes'].message_type = _DAGCONFIG_SUBNODE
_DAGCONFIG_SUBNODECONFIG.containing_type = _DAGCONFIG
_DAGCONFIG_EVENT.containing_type = _DAGCONFIG
_DAGCONFIG_EDGE.fields_by_name['events'].message_type = _DAGCONFIG_EVENT
_DAGCONFIG_EDGE.containing_type = _DAGCONFIG
_DAGCONFIG_EDGECONFIG.fields_by_name['edges'].message_type = _DAGCONFIG_EDGE
_DAGCONFIG_EDGECONFIG.containing_type = _DAGCONFIG
_DAGCONFIG_SHAREDDATA.containing_type = _DAGCONFIG
_DAGCONFIG_SHAREDDATACONFIG.fields_by_name['datas'].message_type = _DAGCONFIG_SHAREDDATA
_DAGCONFIG_SHAREDDATACONFIG.containing_type = _DAGCONFIG
_DAGCONFIG.fields_by_name['subnode_config'].message_type = _DAGCONFIG_SUBNODECONFIG
_DAGCONFIG.fields_by_name['edge_config'].message_type = _DAGCONFIG_EDGECONFIG
_DAGCONFIG.fields_by_name['data_config'].message_type = _DAGCONFIG_SHAREDDATACONFIG
_DAGCONFIG_SUBNODETYPE.containing_type = _DAGCONFIG
DESCRIPTOR.message_types_by_name['DAGConfig'] = _DAGCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DAGConfig = _reflection.GeneratedProtocolMessageType('DAGConfig', (_message.Message,), dict(

  Subnode = _reflection.GeneratedProtocolMessageType('Subnode', (_message.Message,), dict(
    DESCRIPTOR = _DAGCONFIG_SUBNODE,
    __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig.Subnode)
    ))
  ,

  SubnodeConfig = _reflection.GeneratedProtocolMessageType('SubnodeConfig', (_message.Message,), dict(
    DESCRIPTOR = _DAGCONFIG_SUBNODECONFIG,
    __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig.SubnodeConfig)
    ))
  ,

  Event = _reflection.GeneratedProtocolMessageType('Event', (_message.Message,), dict(
    DESCRIPTOR = _DAGCONFIG_EVENT,
    __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig.Event)
    ))
  ,

  Edge = _reflection.GeneratedProtocolMessageType('Edge', (_message.Message,), dict(
    DESCRIPTOR = _DAGCONFIG_EDGE,
    __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig.Edge)
    ))
  ,

  EdgeConfig = _reflection.GeneratedProtocolMessageType('EdgeConfig', (_message.Message,), dict(
    DESCRIPTOR = _DAGCONFIG_EDGECONFIG,
    __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig.EdgeConfig)
    ))
  ,

  SharedData = _reflection.GeneratedProtocolMessageType('SharedData', (_message.Message,), dict(
    DESCRIPTOR = _DAGCONFIG_SHAREDDATA,
    __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig.SharedData)
    ))
  ,

  SharedDataConfig = _reflection.GeneratedProtocolMessageType('SharedDataConfig', (_message.Message,), dict(
    DESCRIPTOR = _DAGCONFIG_SHAREDDATACONFIG,
    __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig.SharedDataConfig)
    ))
  ,
  DESCRIPTOR = _DAGCONFIG,
  __module__ = 'modules.perception.onboard.proto.dag_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.DAGConfig)
  ))
_sym_db.RegisterMessage(DAGConfig)
_sym_db.RegisterMessage(DAGConfig.Subnode)
_sym_db.RegisterMessage(DAGConfig.SubnodeConfig)
_sym_db.RegisterMessage(DAGConfig.Event)
_sym_db.RegisterMessage(DAGConfig.Edge)
_sym_db.RegisterMessage(DAGConfig.EdgeConfig)
_sym_db.RegisterMessage(DAGConfig.SharedData)
_sym_db.RegisterMessage(DAGConfig.SharedDataConfig)


# @@protoc_insertion_point(module_scope)
