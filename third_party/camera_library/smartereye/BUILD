package(default_visibility = ["//visibility:public"])

licenses(["notice"])
cc_library(
    name = "smartereye",
    srcs = glob([
        "lib/libRealtimeDatabase.so.1.0.0",
        "lib/libMessageBus.so.1.0.0",
        "lib/libSatp.so.1.0.2",
        "lib/libQt5Core.so.5.12.0",
        "lib/libStereoCamera.so.1.0.0",
        "lib/libQt5Network.so.5.12.0",
        "lib/libImageUtils.so.1.0.0",
        "lib/libRtdbService.so.1.0.0",
        "lib/libQAsioSocket.so.1.0.0",
        "lib/libSatpExt.so.1.0.2",
        "lib/libQt5Gui.so.5.12.0",
    ]),
    hdrs = glob(["include/*.h"]),
    copts = [
        "-Iinclude",
    ],
    includes = [
        "include",
    ],
    linkopts = [
        "-Wl,-rpath=/apollo/third_party/camera_library/smartereye/lib",
        "-Wl,-rpath-link=/apollo/third_party/camera_library/smartereye/lib",
        "-Llib",
    ],
    visibility = ["//visibility:public"],
)
