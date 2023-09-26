syntax = "proto2"

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "perception_config_schema_proto",
    deps = [
        ":perception_config_schema_proto_lib",
    ],
)

proto_library(
    name = "perception_config_schema_proto_lib",
    srcs = [
        "perception_config_schema.proto",
    ],
)

cpplint()
