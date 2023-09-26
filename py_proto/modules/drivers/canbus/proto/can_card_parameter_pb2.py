# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/drivers/canbus/proto/can_card_parameter.proto

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
  name='modules/drivers/canbus/proto/can_card_parameter.proto',
  package='apollo.drivers.canbus',
  syntax='proto2',
  serialized_pb=_b('\n5modules/drivers/canbus/proto/can_card_parameter.proto\x12\x15\x61pollo.drivers.canbus\"\xce\x03\n\x10\x43\x41NCardParameter\x12\x43\n\x05\x62rand\x18\x01 \x01(\x0e\x32\x34.apollo.drivers.canbus.CANCardParameter.CANCardBrand\x12\x41\n\x04type\x18\x02 \x01(\x0e\x32\x33.apollo.drivers.canbus.CANCardParameter.CANCardType\x12H\n\nchannel_id\x18\x03 \x01(\x0e\x32\x34.apollo.drivers.canbus.CANCardParameter.CANChannelId\"Z\n\x0c\x43\x41NCardBrand\x12\x0c\n\x08\x46\x41KE_CAN\x10\x00\x12\x0b\n\x07\x45SD_CAN\x10\x01\x12\x12\n\x0eSOCKET_CAN_RAW\x10\x02\x12\x0e\n\nHERMES_CAN\x10\x03\x12\x0b\n\x07ZLG_CAN\x10\x04\")\n\x0b\x43\x41NCardType\x12\x0c\n\x08PCI_CARD\x10\x00\x12\x0c\n\x08USB_CARD\x10\x01\"a\n\x0c\x43\x41NChannelId\x12\x13\n\x0f\x43HANNEL_ID_ZERO\x10\x00\x12\x12\n\x0e\x43HANNEL_ID_ONE\x10\x01\x12\x12\n\x0e\x43HANNEL_ID_TWO\x10\x02\x12\x14\n\x10\x43HANNEL_ID_THREE\x10\x03')
)



_CANCARDPARAMETER_CANCARDBRAND = _descriptor.EnumDescriptor(
  name='CANCardBrand',
  full_name='apollo.drivers.canbus.CANCardParameter.CANCardBrand',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FAKE_CAN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ESD_CAN', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SOCKET_CAN_RAW', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HERMES_CAN', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ZLG_CAN', index=4, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=311,
  serialized_end=401,
)
_sym_db.RegisterEnumDescriptor(_CANCARDPARAMETER_CANCARDBRAND)

_CANCARDPARAMETER_CANCARDTYPE = _descriptor.EnumDescriptor(
  name='CANCardType',
  full_name='apollo.drivers.canbus.CANCardParameter.CANCardType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PCI_CARD', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='USB_CARD', index=1, number=1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=403,
  serialized_end=444,
)
_sym_db.RegisterEnumDescriptor(_CANCARDPARAMETER_CANCARDTYPE)

_CANCARDPARAMETER_CANCHANNELID = _descriptor.EnumDescriptor(
  name='CANChannelId',
  full_name='apollo.drivers.canbus.CANCardParameter.CANChannelId',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='CHANNEL_ID_ZERO', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHANNEL_ID_ONE', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHANNEL_ID_TWO', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHANNEL_ID_THREE', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=446,
  serialized_end=543,
)
_sym_db.RegisterEnumDescriptor(_CANCARDPARAMETER_CANCHANNELID)


_CANCARDPARAMETER = _descriptor.Descriptor(
  name='CANCardParameter',
  full_name='apollo.drivers.canbus.CANCardParameter',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='brand', full_name='apollo.drivers.canbus.CANCardParameter.brand', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='apollo.drivers.canbus.CANCardParameter.type', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='channel_id', full_name='apollo.drivers.canbus.CANCardParameter.channel_id', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _CANCARDPARAMETER_CANCARDBRAND,
    _CANCARDPARAMETER_CANCARDTYPE,
    _CANCARDPARAMETER_CANCHANNELID,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=81,
  serialized_end=543,
)

_CANCARDPARAMETER.fields_by_name['brand'].enum_type = _CANCARDPARAMETER_CANCARDBRAND
_CANCARDPARAMETER.fields_by_name['type'].enum_type = _CANCARDPARAMETER_CANCARDTYPE
_CANCARDPARAMETER.fields_by_name['channel_id'].enum_type = _CANCARDPARAMETER_CANCHANNELID
_CANCARDPARAMETER_CANCARDBRAND.containing_type = _CANCARDPARAMETER
_CANCARDPARAMETER_CANCARDTYPE.containing_type = _CANCARDPARAMETER
_CANCARDPARAMETER_CANCHANNELID.containing_type = _CANCARDPARAMETER
DESCRIPTOR.message_types_by_name['CANCardParameter'] = _CANCARDPARAMETER
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CANCardParameter = _reflection.GeneratedProtocolMessageType('CANCardParameter', (_message.Message,), dict(
  DESCRIPTOR = _CANCARDPARAMETER,
  __module__ = 'modules.drivers.canbus.proto.can_card_parameter_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.canbus.CANCardParameter)
  ))
_sym_db.RegisterMessage(CANCardParameter)


# @@protoc_insertion_point(module_scope)