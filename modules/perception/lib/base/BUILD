load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "registerer.cc",
        "thread.cc",
	"file_util.cc",
    ],
    hdrs = [
        "concurrent_queue.h",
        "mutex.h",
        "noncopyable.h",
        "registerer.h",
        "singleton.h",
        "thread.h",
        "file_util.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "@gtest",
    ],
)

cc_test(
    name = "perception_lib_base_test",
    size = "small",
    srcs = [
        "registerer_test.cc",
    ],
    data = ["//modules/perception:perception_data"],
    deps = [
        ":base",
        "@gtest//:main",
    ],
)

cpplint()
