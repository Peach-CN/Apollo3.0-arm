load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "rosbag_message_extractor",
    srcs = ["rosbag_message_extractor.cc"],
    deps = [
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "@ros//:ros_common",
        "@eigen",
        "@proj4",
    ],
)

cpplint()
