# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/dreamview/proto/audio_capture.proto

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
  name='modules/dreamview/proto/audio_capture.proto',
  package='apollo.dreamview',
  syntax='proto2',
  serialized_pb=_b('\n+modules/dreamview/proto/audio_capture.proto\x12\x10\x61pollo.dreamview\"9\n\x0c\x41udioCapture\x12\x15\n\rconnection_id\x18\x01 \x01(\x04\x12\x12\n\nwav_stream\x18\x02 \x01(\x0c')
)




_AUDIOCAPTURE = _descriptor.Descriptor(
  name='AudioCapture',
  full_name='apollo.dreamview.AudioCapture',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='connection_id', full_name='apollo.dreamview.AudioCapture.connection_id', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='wav_stream', full_name='apollo.dreamview.AudioCapture.wav_stream', index=1,
      number=2, type=12, cpp_type=9, label=1,
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
  serialized_start=65,
  serialized_end=122,
)

DESCRIPTOR.message_types_by_name['AudioCapture'] = _AUDIOCAPTURE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AudioCapture = _reflection.GeneratedProtocolMessageType('AudioCapture', (_message.Message,), dict(
  DESCRIPTOR = _AUDIOCAPTURE,
  __module__ = 'modules.dreamview.proto.audio_capture_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.AudioCapture)
  ))
_sym_db.RegisterMessage(AudioCapture)


# @@protoc_insertion_point(module_scope)
