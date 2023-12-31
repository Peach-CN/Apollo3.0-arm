# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/proto/yolo_camera_detector_config.proto

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
  name='modules/perception/proto/yolo_camera_detector_config.proto',
  package='apollo.perception.yolo_camera_detector_config',
  syntax='proto2',
  serialized_pb=_b('\n:modules/perception/proto/yolo_camera_detector_config.proto\x12-apollo.perception.yolo_camera_detector_config\"\x92\x01\n\x0cModelConfigs\x12 \n\x04name\x18\x01 \x01(\t:\x12YoloCameraDetector\x12\x16\n\x07version\x18\x02 \x01(\t:\x05\x31.0.0\x12H\n\tyolo_root\x18\x03 \x01(\t:5/apollo/modules/perception/model/yolo_camera_detector')
)




_MODELCONFIGS = _descriptor.Descriptor(
  name='ModelConfigs',
  full_name='apollo.perception.yolo_camera_detector_config.ModelConfigs',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.perception.yolo_camera_detector_config.ModelConfigs.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("YoloCameraDetector").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='version', full_name='apollo.perception.yolo_camera_detector_config.ModelConfigs.version', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("1.0.0").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='yolo_root', full_name='apollo.perception.yolo_camera_detector_config.ModelConfigs.yolo_root', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("/apollo/modules/perception/model/yolo_camera_detector").decode('utf-8'),
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
  serialized_start=110,
  serialized_end=256,
)

DESCRIPTOR.message_types_by_name['ModelConfigs'] = _MODELCONFIGS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ModelConfigs = _reflection.GeneratedProtocolMessageType('ModelConfigs', (_message.Message,), dict(
  DESCRIPTOR = _MODELCONFIGS,
  __module__ = 'modules.perception.proto.yolo_camera_detector_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.yolo_camera_detector_config.ModelConfigs)
  ))
_sym_db.RegisterMessage(ModelConfigs)


# @@protoc_insertion_point(module_scope)
