load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "gcc_linux_x86_64",
    values = {"cpu": "haswell"},
#    visibility = ["//visibility:public"],
)

config_setting(
    name = "aarch64",
    values = {"cpu": "arm"},
#    visibility = ["//visibility:public"],
)

cc_library(
    name = "zlg_can_client",
    srcs = [
        "zlg_can_client.cc",
    ],
    hdrs = [
        "zlg_can_client.h",
        "controlcan.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_client/socket:socket_can_client_raw",
    ] + select ({
        ":gcc_linux_x86_64": [
            "//third_party/can_card_library/zlg_can:zlg_can",
        ],
        ":aarch64": [
            "//third_party/can_card_library/zlg_can:zlg_can_aarch64",
        ],
    }),
)

cc_test(
    name = "zlg_can_client_test",
    size = "small",
    srcs = [
        "zlg_can_client_test.cc",
    ],
    deps = [
        "//modules/common:log",
        "//modules/drivers/canbus/can_client/zlg:zlg_can_client",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest//:main",
    ],
)

cpplint()
