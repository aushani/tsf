cc_library(
    name = "perls-math",
    srcs = glob(["*.cc", "*.h"]),
    visibility = ["//visibility:public"],
    linkopts = [ "-lm" ],
    deps = [
            "//thirdparty/perls-common",
            "@gsl//:gsl",
           ],
    copts = ["-fpermissive"],
)
