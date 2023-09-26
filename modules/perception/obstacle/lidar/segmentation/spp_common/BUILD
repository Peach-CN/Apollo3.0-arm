load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spp_common",
    srcs = [
        "segment_graph.cc",
        "spp_cell_matrix.cc",
	    "spp_cloud_mask.cc",
	    "spp_cluster.cc",
	    "spp_cluster_list.cc",
	    "spp_dynamic_map.cc",
	    "spp_engine.cc",
	    "spp_label_map.cc",
	    "spp_seg_cc_2d.cc",
	    "spp_seg_mst.cc",
	    "spp_seg_mst_seq.cc",
	    "spp_sequence.cc",
	    "timer.cc",
        "disjoint.cc",
    ],
    hdrs = [
        "segment_graph.h",
        "spp_cell_matrix.h",
        "spp_cloud_mask.h",
        "spp_cluster.h",
        "spp_cluster_list.h",
        "spp_configure.h",
        "spp_dynamic_map.h",
	    "spp_engine.h",
	    "spp_label_map.h",
	    "spp_mono_inc_vector.h",
	    "spp_seg_cc_2d.h",
	    "spp_seg_mst.h",
	    "spp_seg_mst_seq.h",
	    "spp_sequence.h",
	    "spp_vector_pool.h",
	    "timer.h",
        "disjoint.h",
        "i_alloc.h",
        "i_basic.h",
        "i_constant.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common:pcl_util",
        "//modules/common/util:disjoint_set",
        "//modules/common/util",
        "@eigen",
        "@gtest",
    ],
)

cpplint()
