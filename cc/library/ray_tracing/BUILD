cc_library(
    name = "ray_tracing_cuda",
    srcs = glob(['*.cu.cc']),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    copts = ["-x",  "cuda", "-std=c++11"],
    deps = [
            "//tools:cuda",
            "//library/sensors",
            "//library/gpu_util",
            "//library/util",
            "@eigen//:eigen",
           ],
)

cc_library(
    name = "ray_tracing",
    srcs = glob(['*.cc'], exclude=["*.cu.cc"]),
    hdrs = glob(["*.h"], exclude=["*.cu.h"]),
    visibility = ["//visibility:public"],
    linkopts = ["-lpthread"],
    deps = [
            ":ray_tracing_cuda",
           ],
)
