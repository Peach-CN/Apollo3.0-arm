load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "perception_input_validator",
    srcs = ["perception_input_validator.cc"],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "@ros//:ros_common",
        "@opencv2//:core",
        "@opencv2//:imgproc",
    ],
)

cc_binary(
    name = "perception_input_publisher",
    srcs = ["perception_input_publisher.cc"],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//external:gflags",
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "@ros//:ros_common",
        "@opencv2//:core",
        "@opencv2//:imgproc",
    ],
)

cpplint()

