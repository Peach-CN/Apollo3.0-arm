# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/tools/fuzz/control/proto/control_fuzz.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.monitor_log.proto import monitor_log_pb2 as modules_dot_common_dot_monitor__log_dot_proto_dot_monitor__log__pb2
from modules.control.proto import pad_msg_pb2 as modules_dot_control_dot_proto_dot_pad__msg__pb2
from modules.canbus.proto import chassis_pb2 as modules_dot_canbus_dot_proto_dot_chassis__pb2
from modules.localization.proto import localization_pb2 as modules_dot_localization_dot_proto_dot_localization__pb2
from modules.planning.proto import planning_pb2 as modules_dot_planning_dot_proto_dot_planning__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/tools/fuzz/control/proto/control_fuzz.proto',
  package='apollo.tools.fuzz.control',
  syntax='proto2',
  serialized_pb=_b('\n3modules/tools/fuzz/control/proto/control_fuzz.proto\x12\x19\x61pollo.tools.fuzz.control\x1a\x32modules/common/monitor_log/proto/monitor_log.proto\x1a#modules/control/proto/pad_msg.proto\x1a\"modules/canbus/proto/chassis.proto\x1a-modules/localization/proto/localization.proto\x1a%modules/planning/proto/planning.proto\"\xb6\x02\n\x18\x43ontrolModuleFuzzMessage\x12>\n\x0fmonitor_message\x18\x01 \x02(\x0b\x32%.apollo.common.monitor.MonitorMessage\x12/\n\x0bpad_message\x18\x02 \x02(\x0b\x32\x1a.apollo.control.PadMessage\x12\'\n\x07\x63hassis\x18\x03 \x02(\x0b\x32\x16.apollo.canbus.Chassis\x12H\n\x15localization_estimate\x18\x04 \x02(\x0b\x32).apollo.localization.LocalizationEstimate\x12\x36\n\x0e\x61\x64\x63_trajectory\x18\x05 \x02(\x0b\x32\x1e.apollo.planning.ADCTrajectory')
  ,
  dependencies=[modules_dot_common_dot_monitor__log_dot_proto_dot_monitor__log__pb2.DESCRIPTOR,modules_dot_control_dot_proto_dot_pad__msg__pb2.DESCRIPTOR,modules_dot_canbus_dot_proto_dot_chassis__pb2.DESCRIPTOR,modules_dot_localization_dot_proto_dot_localization__pb2.DESCRIPTOR,modules_dot_planning_dot_proto_dot_planning__pb2.DESCRIPTOR,])




_CONTROLMODULEFUZZMESSAGE = _descriptor.Descriptor(
  name='ControlModuleFuzzMessage',
  full_name='apollo.tools.fuzz.control.ControlModuleFuzzMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='monitor_message', full_name='apollo.tools.fuzz.control.ControlModuleFuzzMessage.monitor_message', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pad_message', full_name='apollo.tools.fuzz.control.ControlModuleFuzzMessage.pad_message', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='chassis', full_name='apollo.tools.fuzz.control.ControlModuleFuzzMessage.chassis', index=2,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='localization_estimate', full_name='apollo.tools.fuzz.control.ControlModuleFuzzMessage.localization_estimate', index=3,
      number=4, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='adc_trajectory', full_name='apollo.tools.fuzz.control.ControlModuleFuzzMessage.adc_trajectory', index=4,
      number=5, type=11, cpp_type=10, label=2,
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
  serialized_start=294,
  serialized_end=604,
)

_CONTROLMODULEFUZZMESSAGE.fields_by_name['monitor_message'].message_type = modules_dot_common_dot_monitor__log_dot_proto_dot_monitor__log__pb2._MONITORMESSAGE
_CONTROLMODULEFUZZMESSAGE.fields_by_name['pad_message'].message_type = modules_dot_control_dot_proto_dot_pad__msg__pb2._PADMESSAGE
_CONTROLMODULEFUZZMESSAGE.fields_by_name['chassis'].message_type = modules_dot_canbus_dot_proto_dot_chassis__pb2._CHASSIS
_CONTROLMODULEFUZZMESSAGE.fields_by_name['localization_estimate'].message_type = modules_dot_localization_dot_proto_dot_localization__pb2._LOCALIZATIONESTIMATE
_CONTROLMODULEFUZZMESSAGE.fields_by_name['adc_trajectory'].message_type = modules_dot_planning_dot_proto_dot_planning__pb2._ADCTRAJECTORY
DESCRIPTOR.message_types_by_name['ControlModuleFuzzMessage'] = _CONTROLMODULEFUZZMESSAGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ControlModuleFuzzMessage = _reflection.GeneratedProtocolMessageType('ControlModuleFuzzMessage', (_message.Message,), dict(
  DESCRIPTOR = _CONTROLMODULEFUZZMESSAGE,
  __module__ = 'modules.tools.fuzz.control.proto.control_fuzz_pb2'
  # @@protoc_insertion_point(class_scope:apollo.tools.fuzz.control.ControlModuleFuzzMessage)
  ))
_sym_db.RegisterMessage(ControlModuleFuzzMessage)


# @@protoc_insertion_point(module_scope)
