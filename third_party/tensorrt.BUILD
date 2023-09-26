cc_library(
    name = "tensorrt",
    linkopts = [
        "-Xlinker -rpath-link=/usr/lib/aarch64-linux-gnu/tegra",
        "-Xlinker -rpath-link=/usr/lib/aarch64-linux-gnu/tegra-egl",
        "-Xlinker -rpath-link=/usr/lib/aarch64-linux-gnu",
        "-L/usr/lib/aarch64-linux-gnu",
        "-L/usr/lib/aarch64-linux-gnu/tegra",
        "-L/usr/lib/aarch64-linux-gnu/tegra-egl",
        "-lnvcaffe_parser",
        "-lnvinfer",
        "-lnvinfer_plugin",
        "-lnvparsers",
    ],

    includes = [".","/usr/include/tensorrt",],
    visibility = ["//visibility:public"],
)

