load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_proto",
    deps = [
        ":ins_proto",
        ":gnss_raw_observation_proto",
        ":gnss_best_pose_proto",
        ":config_proto",
        ":imu_proto",
        ":gnss_status_proto",
        ":gnss_proto_proto",
        ":zhd_gps_proto",
    ],
)


cc_proto_library(
    name = "ins_proto",
    deps = [
        ":ins_proto_lib",
    ],
)

proto_library(
    name = "ins_proto_lib",
    srcs = [
        "ins.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_raw_observation_proto",
    deps = [
        ":gnss_raw_observation_lib",
    ],
)

proto_library(
    name = "gnss_raw_observation_lib",
    srcs = ["gnss_raw_observation.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_best_pose_proto",
    deps = [
        ":gnss_best_pose_lib",
    ],
)

proto_library(
    name = "gnss_best_pose_lib",
    srcs = ["gnss_best_pose.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "config_proto",
    deps = [
        ":config_lib",
    ],
)

proto_library(
    name = "config_lib",
    srcs = ["config.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
    ],
)

cc_proto_library(
    name = "imu_proto",
    deps = [
        ":imu_lib",
    ],
)

proto_library(
    name = "imu_lib",
    srcs = ["imu.proto"],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_status_proto",
    deps = [
        ":gnss_status_proto_lib",
    ],
)

proto_library(
    name = "gnss_status_proto_lib",
    srcs = [
        "gnss_status.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_proto_proto",
    deps = [
        ":gnss_proto_lib",
    ],
)

proto_library(
    name = "gnss_proto_lib",
    srcs = [
        "gnss.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "heading_proto",
    deps = [
        ":heading_proto_lib",
    ],
)

proto_library(
    name = "heading_proto_lib",
    srcs = [
        "heading.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "zhd_gps_proto",
    deps = [
        ":zhd_gps_lib",
    ],
)

proto_library(
    name = "zhd_gps_lib",
    srcs = [
        "zhd_gps.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
