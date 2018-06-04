cc_library(
    name = "perls-math",
    srcs = glob(["*.cc", "*.h"]),
    visibility = ["//visibility:public"],
    linkopts = [ "-lm" ],
    deps = [
            "@gsl//:gsl",
           ],
    copts = ["-fpermissive"],
)
