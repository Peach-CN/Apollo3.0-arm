# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/planning_status.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.planning.proto import decision_pb2 as modules_dot_planning_dot_proto_dot_decision__pb2
from modules.common.proto import drive_state_pb2 as modules_dot_common_dot_proto_dot_drive__state__pb2
from modules.common.proto import geometry_pb2 as modules_dot_common_dot_proto_dot_geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/planning/proto/planning_status.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n,modules/planning/proto/planning_status.proto\x12\x0f\x61pollo.planning\x1a%modules/planning/proto/decision.proto\x1a&modules/common/proto/drive_state.proto\x1a#modules/common/proto/geometry.proto\"\xbf\x01\n\x10\x43hangeLaneStatus\x12\x38\n\x06status\x18\x01 \x01(\x0e\x32(.apollo.planning.ChangeLaneStatus.Status\x12\x0f\n\x07path_id\x18\x02 \x01(\t\x12\x11\n\ttimestamp\x18\x03 \x01(\x01\"M\n\x06Status\x12\x12\n\x0eIN_CHANGE_LANE\x10\x01\x12\x16\n\x12\x43HANGE_LANE_FAILED\x10\x02\x12\x17\n\x13\x43HANGE_LANE_SUCCESS\x10\x03\"3\n\tStopTimer\x12\x13\n\x0bobstacle_id\x18\x01 \x01(\t\x12\x11\n\tstop_time\x18\x02 \x01(\x01\"X\n\x0f\x43rosswalkStatus\x12\x14\n\x0c\x63rosswalk_id\x18\x01 \x01(\t\x12/\n\x0bstop_timers\x18\x02 \x03(\x0b\x32\x1a.apollo.planning.StopTimer\"\xbd\x03\n\x0ePullOverStatus\x12\x1b\n\x0cin_pull_over\x18\x01 \x01(\x08:\x05\x66\x61lse\x12\x36\n\x06status\x18\x02 \x01(\x0e\x32&.apollo.planning.PullOverStatus.Status\x12\x32\n\x11inlane_dest_point\x18\x03 \x01(\x0b\x32\x17.apollo.common.PointENU\x12,\n\x0bstart_point\x18\x04 \x01(\x0b\x32\x17.apollo.common.PointENU\x12+\n\nstop_point\x18\x05 \x01(\x0b\x32\x17.apollo.common.PointENU\x12\x1a\n\x12stop_point_heading\x18\x06 \x01(\x01\x12\x36\n\x06reason\x18\x07 \x01(\x0e\x32&.apollo.planning.PullOverStatus.Reason\x12\x17\n\x0fstatus_set_time\x18\x08 \x01(\x01\"\x19\n\x06Reason\x12\x0f\n\x0b\x44\x45STINATION\x10\x01\"?\n\x06Status\x12\x0b\n\x07UNKNOWN\x10\x01\x12\x10\n\x0cIN_OPERATION\x10\x02\x12\x08\n\x04\x44ONE\x10\x03\x12\x0c\n\x08\x44ISABLED\x10\x04\".\n\x0fReroutingStatus\x12\x1b\n\x13last_rerouting_time\x18\x01 \x01(\x01\"\x86\x01\n\x10RightOfWayStatus\x12\x41\n\x08junction\x18\x01 \x03(\x0b\x32/.apollo.planning.RightOfWayStatus.JunctionEntry\x1a/\n\rJunctionEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\x08:\x02\x38\x01\"\xee\x01\n\x0eSidePassStatus\x12\x36\n\x06status\x18\x01 \x01(\x0e\x32&.apollo.planning.SidePassStatus.Status\x12\x17\n\x0fwait_start_time\x18\x02 \x01(\x01\x12\x18\n\x10pass_obstacle_id\x18\x03 \x01(\t\x12\x37\n\tpass_side\x18\x04 \x01(\x0e\x32$.apollo.planning.ObjectSidePass.Type\"8\n\x06Status\x12\x0b\n\x07UNKNOWN\x10\x00\x12\t\n\x05\x44RIVE\x10\x01\x12\x08\n\x04WAIT\x10\x02\x12\x0c\n\x08SIDEPASS\x10\x03\"\xd5\x02\n\x0eStopSignStatus\x12\x14\n\x0cstop_sign_id\x18\x01 \x01(\t\x12\x36\n\x06status\x18\x02 \x01(\x0e\x32&.apollo.planning.StopSignStatus.Status\x12\x17\n\x0fstop_start_time\x18\x03 \x01(\x01\x12N\n\x13lane_watch_vehicles\x18\x04 \x03(\x0b\x32\x31.apollo.planning.StopSignStatus.LaneWatchVehicles\x1a<\n\x11LaneWatchVehicles\x12\x0f\n\x07lane_id\x18\x01 \x01(\t\x12\x16\n\x0ewatch_vehicles\x18\x02 \x03(\t\"N\n\x06Status\x12\x0b\n\x07UNKNOWN\x10\x00\x12\t\n\x05\x44RIVE\x10\x01\x12\x08\n\x04STOP\x10\x02\x12\x08\n\x04WAIT\x10\x03\x12\t\n\x05\x43REEP\x10\x04\x12\r\n\tSTOP_DONE\x10\x05\":\n\x11\x44\x65stinationStatus\x12%\n\x16has_passed_destination\x18\x01 \x01(\x08:\x05\x66\x61lse\"\xf4\x03\n\x0ePlanningStatus\x12\x36\n\x0b\x63hange_lane\x18\x01 \x01(\x0b\x32!.apollo.planning.ChangeLaneStatus\x12\x33\n\tcrosswalk\x18\x02 \x01(\x0b\x32 .apollo.planning.CrosswalkStatus\x12\x32\n\rengage_advice\x18\x03 \x01(\x0b\x32\x1b.apollo.common.EngageAdvice\x12\x33\n\trerouting\x18\x04 \x01(\x0b\x32 .apollo.planning.ReroutingStatus\x12\x37\n\x0cright_of_way\x18\x05 \x01(\x0b\x32!.apollo.planning.RightOfWayStatus\x12\x32\n\tside_pass\x18\x06 \x01(\x0b\x32\x1f.apollo.planning.SidePassStatus\x12\x32\n\tstop_sign\x18\x07 \x01(\x0b\x32\x1f.apollo.planning.StopSignStatus\x12\x37\n\x0b\x64\x65stination\x18\x08 \x01(\x0b\x32\".apollo.planning.DestinationStatus\x12\x32\n\tpull_over\x18\t \x01(\x0b\x32\x1f.apollo.planning.PullOverStatus')
  ,
  dependencies=[modules_dot_planning_dot_proto_dot_decision__pb2.DESCRIPTOR,modules_dot_common_dot_proto_dot_drive__state__pb2.DESCRIPTOR,modules_dot_common_dot_proto_dot_geometry__pb2.DESCRIPTOR,])



_CHANGELANESTATUS_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='apollo.planning.ChangeLaneStatus.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='IN_CHANGE_LANE', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_LANE_FAILED', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_LANE_SUCCESS', index=2, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=296,
  serialized_end=373,
)
_sym_db.RegisterEnumDescriptor(_CHANGELANESTATUS_STATUS)

_PULLOVERSTATUS_REASON = _descriptor.EnumDescriptor(
  name='Reason',
  full_name='apollo.planning.PullOverStatus.Reason',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='DESTINATION', index=0, number=1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=874,
  serialized_end=899,
)
_sym_db.RegisterEnumDescriptor(_PULLOVERSTATUS_REASON)

_PULLOVERSTATUS_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='apollo.planning.PullOverStatus.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IN_OPERATION', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DONE', index=2, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DISABLED', index=3, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=901,
  serialized_end=964,
)
_sym_db.RegisterEnumDescriptor(_PULLOVERSTATUS_STATUS)

_SIDEPASSSTATUS_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='apollo.planning.SidePassStatus.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRIVE', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WAIT', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SIDEPASS', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1334,
  serialized_end=1390,
)
_sym_db.RegisterEnumDescriptor(_SIDEPASSSTATUS_STATUS)

_STOPSIGNSTATUS_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='apollo.planning.StopSignStatus.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DRIVE', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STOP', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WAIT', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CREEP', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STOP_DONE', index=5, number=5,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1656,
  serialized_end=1734,
)
_sym_db.RegisterEnumDescriptor(_STOPSIGNSTATUS_STATUS)


_CHANGELANESTATUS = _descriptor.Descriptor(
  name='ChangeLaneStatus',
  full_name='apollo.planning.ChangeLaneStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='apollo.planning.ChangeLaneStatus.status', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='path_id', full_name='apollo.planning.ChangeLaneStatus.path_id', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='apollo.planning.ChangeLaneStatus.timestamp', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _CHANGELANESTATUS_STATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=182,
  serialized_end=373,
)


_STOPTIMER = _descriptor.Descriptor(
  name='StopTimer',
  full_name='apollo.planning.StopTimer',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='obstacle_id', full_name='apollo.planning.StopTimer.obstacle_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_time', full_name='apollo.planning.StopTimer.stop_time', index=1,
      number=2, type=1, cpp_type=5, label=1,
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
  serialized_start=375,
  serialized_end=426,
)


_CROSSWALKSTATUS = _descriptor.Descriptor(
  name='CrosswalkStatus',
  full_name='apollo.planning.CrosswalkStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='crosswalk_id', full_name='apollo.planning.CrosswalkStatus.crosswalk_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_timers', full_name='apollo.planning.CrosswalkStatus.stop_timers', index=1,
      number=2, type=11, cpp_type=10, label=3,
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
  serialized_start=428,
  serialized_end=516,
)


_PULLOVERSTATUS = _descriptor.Descriptor(
  name='PullOverStatus',
  full_name='apollo.planning.PullOverStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='in_pull_over', full_name='apollo.planning.PullOverStatus.in_pull_over', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='apollo.planning.PullOverStatus.status', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='inlane_dest_point', full_name='apollo.planning.PullOverStatus.inlane_dest_point', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_point', full_name='apollo.planning.PullOverStatus.start_point', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_point', full_name='apollo.planning.PullOverStatus.stop_point', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_point_heading', full_name='apollo.planning.PullOverStatus.stop_point_heading', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reason', full_name='apollo.planning.PullOverStatus.reason', index=6,
      number=7, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status_set_time', full_name='apollo.planning.PullOverStatus.status_set_time', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _PULLOVERSTATUS_REASON,
    _PULLOVERSTATUS_STATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=519,
  serialized_end=964,
)


_REROUTINGSTATUS = _descriptor.Descriptor(
  name='ReroutingStatus',
  full_name='apollo.planning.ReroutingStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='last_rerouting_time', full_name='apollo.planning.ReroutingStatus.last_rerouting_time', index=0,
      number=1, type=1, cpp_type=5, label=1,
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
  serialized_start=966,
  serialized_end=1012,
)


_RIGHTOFWAYSTATUS_JUNCTIONENTRY = _descriptor.Descriptor(
  name='JunctionEntry',
  full_name='apollo.planning.RightOfWayStatus.JunctionEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.planning.RightOfWayStatus.JunctionEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.planning.RightOfWayStatus.JunctionEntry.value', index=1,
      number=2, type=8, cpp_type=7, label=1,
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
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1102,
  serialized_end=1149,
)

_RIGHTOFWAYSTATUS = _descriptor.Descriptor(
  name='RightOfWayStatus',
  full_name='apollo.planning.RightOfWayStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='junction', full_name='apollo.planning.RightOfWayStatus.junction', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_RIGHTOFWAYSTATUS_JUNCTIONENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1015,
  serialized_end=1149,
)


_SIDEPASSSTATUS = _descriptor.Descriptor(
  name='SidePassStatus',
  full_name='apollo.planning.SidePassStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='apollo.planning.SidePassStatus.status', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='wait_start_time', full_name='apollo.planning.SidePassStatus.wait_start_time', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pass_obstacle_id', full_name='apollo.planning.SidePassStatus.pass_obstacle_id', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pass_side', full_name='apollo.planning.SidePassStatus.pass_side', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SIDEPASSSTATUS_STATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1152,
  serialized_end=1390,
)


_STOPSIGNSTATUS_LANEWATCHVEHICLES = _descriptor.Descriptor(
  name='LaneWatchVehicles',
  full_name='apollo.planning.StopSignStatus.LaneWatchVehicles',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lane_id', full_name='apollo.planning.StopSignStatus.LaneWatchVehicles.lane_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='watch_vehicles', full_name='apollo.planning.StopSignStatus.LaneWatchVehicles.watch_vehicles', index=1,
      number=2, type=9, cpp_type=9, label=3,
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
  serialized_start=1594,
  serialized_end=1654,
)

_STOPSIGNSTATUS = _descriptor.Descriptor(
  name='StopSignStatus',
  full_name='apollo.planning.StopSignStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='stop_sign_id', full_name='apollo.planning.StopSignStatus.stop_sign_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='apollo.planning.StopSignStatus.status', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_start_time', full_name='apollo.planning.StopSignStatus.stop_start_time', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lane_watch_vehicles', full_name='apollo.planning.StopSignStatus.lane_watch_vehicles', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_STOPSIGNSTATUS_LANEWATCHVEHICLES, ],
  enum_types=[
    _STOPSIGNSTATUS_STATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1393,
  serialized_end=1734,
)


_DESTINATIONSTATUS = _descriptor.Descriptor(
  name='DestinationStatus',
  full_name='apollo.planning.DestinationStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='has_passed_destination', full_name='apollo.planning.DestinationStatus.has_passed_destination', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
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
  serialized_start=1736,
  serialized_end=1794,
)


_PLANNINGSTATUS = _descriptor.Descriptor(
  name='PlanningStatus',
  full_name='apollo.planning.PlanningStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='change_lane', full_name='apollo.planning.PlanningStatus.change_lane', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='crosswalk', full_name='apollo.planning.PlanningStatus.crosswalk', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='engage_advice', full_name='apollo.planning.PlanningStatus.engage_advice', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='rerouting', full_name='apollo.planning.PlanningStatus.rerouting', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_of_way', full_name='apollo.planning.PlanningStatus.right_of_way', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='side_pass', full_name='apollo.planning.PlanningStatus.side_pass', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_sign', full_name='apollo.planning.PlanningStatus.stop_sign', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='destination', full_name='apollo.planning.PlanningStatus.destination', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pull_over', full_name='apollo.planning.PlanningStatus.pull_over', index=8,
      number=9, type=11, cpp_type=10, label=1,
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
  serialized_start=1797,
  serialized_end=2297,
)

_CHANGELANESTATUS.fields_by_name['status'].enum_type = _CHANGELANESTATUS_STATUS
_CHANGELANESTATUS_STATUS.containing_type = _CHANGELANESTATUS
_CROSSWALKSTATUS.fields_by_name['stop_timers'].message_type = _STOPTIMER
_PULLOVERSTATUS.fields_by_name['status'].enum_type = _PULLOVERSTATUS_STATUS
_PULLOVERSTATUS.fields_by_name['inlane_dest_point'].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINTENU
_PULLOVERSTATUS.fields_by_name['start_point'].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINTENU
_PULLOVERSTATUS.fields_by_name['stop_point'].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINTENU
_PULLOVERSTATUS.fields_by_name['reason'].enum_type = _PULLOVERSTATUS_REASON
_PULLOVERSTATUS_REASON.containing_type = _PULLOVERSTATUS
_PULLOVERSTATUS_STATUS.containing_type = _PULLOVERSTATUS
_RIGHTOFWAYSTATUS_JUNCTIONENTRY.containing_type = _RIGHTOFWAYSTATUS
_RIGHTOFWAYSTATUS.fields_by_name['junction'].message_type = _RIGHTOFWAYSTATUS_JUNCTIONENTRY
_SIDEPASSSTATUS.fields_by_name['status'].enum_type = _SIDEPASSSTATUS_STATUS
_SIDEPASSSTATUS.fields_by_name['pass_side'].enum_type = modules_dot_planning_dot_proto_dot_decision__pb2._OBJECTSIDEPASS_TYPE
_SIDEPASSSTATUS_STATUS.containing_type = _SIDEPASSSTATUS
_STOPSIGNSTATUS_LANEWATCHVEHICLES.containing_type = _STOPSIGNSTATUS
_STOPSIGNSTATUS.fields_by_name['status'].enum_type = _STOPSIGNSTATUS_STATUS
_STOPSIGNSTATUS.fields_by_name['lane_watch_vehicles'].message_type = _STOPSIGNSTATUS_LANEWATCHVEHICLES
_STOPSIGNSTATUS_STATUS.containing_type = _STOPSIGNSTATUS
_PLANNINGSTATUS.fields_by_name['change_lane'].message_type = _CHANGELANESTATUS
_PLANNINGSTATUS.fields_by_name['crosswalk'].message_type = _CROSSWALKSTATUS
_PLANNINGSTATUS.fields_by_name['engage_advice'].message_type = modules_dot_common_dot_proto_dot_drive__state__pb2._ENGAGEADVICE
_PLANNINGSTATUS.fields_by_name['rerouting'].message_type = _REROUTINGSTATUS
_PLANNINGSTATUS.fields_by_name['right_of_way'].message_type = _RIGHTOFWAYSTATUS
_PLANNINGSTATUS.fields_by_name['side_pass'].message_type = _SIDEPASSSTATUS
_PLANNINGSTATUS.fields_by_name['stop_sign'].message_type = _STOPSIGNSTATUS
_PLANNINGSTATUS.fields_by_name['destination'].message_type = _DESTINATIONSTATUS
_PLANNINGSTATUS.fields_by_name['pull_over'].message_type = _PULLOVERSTATUS
DESCRIPTOR.message_types_by_name['ChangeLaneStatus'] = _CHANGELANESTATUS
DESCRIPTOR.message_types_by_name['StopTimer'] = _STOPTIMER
DESCRIPTOR.message_types_by_name['CrosswalkStatus'] = _CROSSWALKSTATUS
DESCRIPTOR.message_types_by_name['PullOverStatus'] = _PULLOVERSTATUS
DESCRIPTOR.message_types_by_name['ReroutingStatus'] = _REROUTINGSTATUS
DESCRIPTOR.message_types_by_name['RightOfWayStatus'] = _RIGHTOFWAYSTATUS
DESCRIPTOR.message_types_by_name['SidePassStatus'] = _SIDEPASSSTATUS
DESCRIPTOR.message_types_by_name['StopSignStatus'] = _STOPSIGNSTATUS
DESCRIPTOR.message_types_by_name['DestinationStatus'] = _DESTINATIONSTATUS
DESCRIPTOR.message_types_by_name['PlanningStatus'] = _PLANNINGSTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ChangeLaneStatus = _reflection.GeneratedProtocolMessageType('ChangeLaneStatus', (_message.Message,), dict(
  DESCRIPTOR = _CHANGELANESTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.ChangeLaneStatus)
  ))
_sym_db.RegisterMessage(ChangeLaneStatus)

StopTimer = _reflection.GeneratedProtocolMessageType('StopTimer', (_message.Message,), dict(
  DESCRIPTOR = _STOPTIMER,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.StopTimer)
  ))
_sym_db.RegisterMessage(StopTimer)

CrosswalkStatus = _reflection.GeneratedProtocolMessageType('CrosswalkStatus', (_message.Message,), dict(
  DESCRIPTOR = _CROSSWALKSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.CrosswalkStatus)
  ))
_sym_db.RegisterMessage(CrosswalkStatus)

PullOverStatus = _reflection.GeneratedProtocolMessageType('PullOverStatus', (_message.Message,), dict(
  DESCRIPTOR = _PULLOVERSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PullOverStatus)
  ))
_sym_db.RegisterMessage(PullOverStatus)

ReroutingStatus = _reflection.GeneratedProtocolMessageType('ReroutingStatus', (_message.Message,), dict(
  DESCRIPTOR = _REROUTINGSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.ReroutingStatus)
  ))
_sym_db.RegisterMessage(ReroutingStatus)

RightOfWayStatus = _reflection.GeneratedProtocolMessageType('RightOfWayStatus', (_message.Message,), dict(

  JunctionEntry = _reflection.GeneratedProtocolMessageType('JunctionEntry', (_message.Message,), dict(
    DESCRIPTOR = _RIGHTOFWAYSTATUS_JUNCTIONENTRY,
    __module__ = 'modules.planning.proto.planning_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.planning.RightOfWayStatus.JunctionEntry)
    ))
  ,
  DESCRIPTOR = _RIGHTOFWAYSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.RightOfWayStatus)
  ))
_sym_db.RegisterMessage(RightOfWayStatus)
_sym_db.RegisterMessage(RightOfWayStatus.JunctionEntry)

SidePassStatus = _reflection.GeneratedProtocolMessageType('SidePassStatus', (_message.Message,), dict(
  DESCRIPTOR = _SIDEPASSSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.SidePassStatus)
  ))
_sym_db.RegisterMessage(SidePassStatus)

StopSignStatus = _reflection.GeneratedProtocolMessageType('StopSignStatus', (_message.Message,), dict(

  LaneWatchVehicles = _reflection.GeneratedProtocolMessageType('LaneWatchVehicles', (_message.Message,), dict(
    DESCRIPTOR = _STOPSIGNSTATUS_LANEWATCHVEHICLES,
    __module__ = 'modules.planning.proto.planning_status_pb2'
    # @@protoc_insertion_point(class_scope:apollo.planning.StopSignStatus.LaneWatchVehicles)
    ))
  ,
  DESCRIPTOR = _STOPSIGNSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.StopSignStatus)
  ))
_sym_db.RegisterMessage(StopSignStatus)
_sym_db.RegisterMessage(StopSignStatus.LaneWatchVehicles)

DestinationStatus = _reflection.GeneratedProtocolMessageType('DestinationStatus', (_message.Message,), dict(
  DESCRIPTOR = _DESTINATIONSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.DestinationStatus)
  ))
_sym_db.RegisterMessage(DestinationStatus)

PlanningStatus = _reflection.GeneratedProtocolMessageType('PlanningStatus', (_message.Message,), dict(
  DESCRIPTOR = _PLANNINGSTATUS,
  __module__ = 'modules.planning.proto.planning_status_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PlanningStatus)
  ))
_sym_db.RegisterMessage(PlanningStatus)


_RIGHTOFWAYSTATUS_JUNCTIONENTRY.has_options = True
_RIGHTOFWAYSTATUS_JUNCTIONENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)
