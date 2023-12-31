load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_parser",
    deps = [
        ":data_parser",
        ":novatel_parser",
        ":newtonm2_parser",
        ":rtcm_parsers",
        ":zhd_parser",
        ":marvelmind_parser",
    ],
)

cc_library(
    name = "data_parser",
    srcs = ["data_parser.cc"],
    hdrs = [
        "data_parser.h",
        "parser.h",
    ],
    deps = [
        ":novatel_parser",
        ":zhd_parser",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss:gnss_gflags",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/util:gnss_util",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "@proj4",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "novatel_parser",
    srcs = ["novatel_parser.cc"],
    hdrs = [
        "novatel_messages.h",
        "parser.h",
        "rtcm_decode.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/third_party:rtcm",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "newtonm2_parser",
    srcs = ["newtonm2_parser.cc"],
    hdrs = [
        "novatel_messages.h",
        "parser.h",
        "newtonm2_parser.h",
        "rtcm_decode.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss:gnss_gflags",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/third_party:rtcm",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "rtcm_parsers",
    srcs = [
        "rtcm3_parser.cc",
        "rtcm_parser.cc",
    ],
    hdrs = [
        "parser.h",
        "rtcm3_parser.h",
        "rtcm_decode.h",
        "rtcm_parser.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss:gnss_gflags",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/third_party:rtcm",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "zhd_parser",
    srcs = [
        "zhd_parser.cc",
    ],
    hdrs = [
        "parser.h",
        "zhd_messages.h",
        "rtcm_decode.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/third_party:rtcm",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "marvelmind_parser",
    srcs = [
        "marvelmind_parser.cpp",
    ],
    hdrs = [
        "parser.h",
        "marvelmind_messages.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss:gnss_gflags",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common",
    ],
)

cpplint()
