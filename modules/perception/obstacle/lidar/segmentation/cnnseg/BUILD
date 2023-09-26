load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cnnseg",
    srcs = ["cnn_segmentation.cc"],
    hdrs = ["cnn_segmentation.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_cluster2d",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_feature_generator",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg/proto:cnnseg_proto",
        "//modules/perception/proto:cnn_segmentation_config_lib_proto",
        "@caffe//:lib",
    ],
)

cc_library(
    name = "cnnseg_util",
    hdrs = ["util.h"],
)

cc_library(
    name = "cnnseg_feature_generator",
    srcs = ["feature_generator.cc"],
    hdrs = ["feature_generator.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg/proto:cnnseg_proto",
        "@caffe//:lib",
        "@eigen",
        "@cuda",
    ],
)

cc_library(
    name = "cnnseg_cluster2d",
    hdrs = ["cluster2d.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/util:disjoint_set",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
        "@caffe//:lib",
    ],
)

cc_library(
    name = "fast_rt_net",
    hdrs = [
        "fast_cluster2d.h",
        "fast_cnn_segmentation.h",
        "fast_feature_generator.h",
    ],
    deps = [
        "//modules/perception/obstacle/camera/common:cnn_adapter",
        "//modules/perception/obstacle/lidar/segmentation/spp_common:spp_common",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg/proto:fast_cnnseg_proto",
        "//third_party/rt_net:rt_net",
    ],
)

cc_test(
    name = "cnn_segmentation_test",
    size = "small",
    srcs = [
        "cnn_segmentation_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        "cnnseg",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/base",
        "@caffe//:lib",
        "@gtest//:main",
        "@opencv2//:core",
        "@pcl",
    ],
)

cpplint()
