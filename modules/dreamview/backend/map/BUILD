load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "map_service",
    srcs = [
        "map_service.cc",
        "map_generator.cc",
    ],
    hdrs = [
        "map_service.h",
        "map_generator.h",
    ],
    linkopts = [
        "-lboost_thread",
    ],
    deps = [
        "//modules/common/util:json_util",
        "//modules/common/monitor_log",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/proto:reference_line_smoother_config_proto",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/reference_line:spiral_reference_line_smoother",
        "//third_party/json",
        "@glog//:glog",
        "@yaml_cpp//:yaml",
    ],
)

cc_test(
    name = "map_service_test",
    size = "small",
    srcs = [
        "map_service_test.cc",
    ],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":map_service",
        "@gtest//:main",
    ],
)

cpplint()
