cc_library(
    name = "flow_cuda",
    srcs = glob(['*.cu.cc']),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    copts = ["-x",  "cuda", "-std=c++11"],
    deps = [
            "//tools:cuda",
            "//library/ray_tracing",
            "@eigen//:eigen",
            "@openscenegraph//:openscenegraph",
           ],
)

cc_library(
    name = "flow",
    srcs = glob(['*.c', '*.cpp'], exclude=["*.cu.cc"]),
    hdrs = glob(["*.h", '*.hpp'], exclude=["*.cu.h"]),
    visibility = ["//visibility:public"],
    linkopts = ["-lpthread"],
    deps = [
            ":flow_cuda",
            "//library/kitti",
            "//library/scan_matching",
            "//thirdparty/perls-math",
            "//thirdparty/perls-common",
            "@openscenegraph//:openscenegraph",
           ],
)
