load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fr_vehicle_factory",
    srcs = [
        "fr_vehicle_factory.cc",
    ],
    hdrs = [
        "fr_vehicle_factory.h",
    ],
    deps = [
        ":fr_controller",
        ":fr_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "fr_message_manager",
    srcs = [
        "fr_message_manager.cc",
    ],
    hdrs = [
        "fr_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/fr/protocol:canbus_fr_protocol",
    ],
)

cc_library(
    name = "fr_controller",
    srcs = [
        "fr_controller.cc",
    ],
    hdrs = [
        "fr_controller.h",
    ],
    deps = [
        ":fr_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/fr/protocol:canbus_fr_protocol",
    ],
)

cpplint()
