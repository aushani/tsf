cc_library(
    name = "ray_tracing_cuda",
    srcs = glob(['*.cu.cc']),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    copts = ["-x",  "cuda", "-std=c++11", "-arch=compute_61,code=sm_61", "-O3", "-DVERBOSE"],
    deps = [
            "//tools:cuda",
            "//library/sensors",
            "//library/gpu_util",
            "//library/util",
            "@eigen//:eigen",
            "@boost//:boost_headers",
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
