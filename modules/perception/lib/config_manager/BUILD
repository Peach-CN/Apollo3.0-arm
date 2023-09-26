load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = [
        "calibration_config_manager.cc",
        "config_manager.cc",
    ],
    hdrs = [
        "calibration_config_manager.h",
        "config_manager.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/perception/common",
        "//modules/perception/cuda_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/camera/common",
        "//modules/perception/lib/config_manager/proto:perception_config_schema_proto",
        "@cuda",
        "@opencv2//:imgproc",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
