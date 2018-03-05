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
           ],
)

cc_library(
    name = "flow",
    srcs = glob(['*.c'], exclude=["*.cu.cc"]),
    hdrs = glob(["*.h"], exclude=["*.cu.h"]),
    visibility = ["//visibility:public"],
    linkopts = ["-lpthread"],
    deps = [
            ":flow_cuda",
           ],
)
