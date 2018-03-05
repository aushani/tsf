cc_library(
    name = "gpu_util",
    srcs = glob(['*.cu.cc']),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    copts = ["-x",  "cuda", "-std=c++11"],
    deps = [
            "//tools:cuda",
            "//library/util",
           ],
)
