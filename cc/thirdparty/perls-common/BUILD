cc_library(
    name = "perls-common",
    #srcs = glob(["*.cc", ".h"]),
    srcs = [
            "timestamp.h", "timestamp.cc",
            "timeutil.h", "timeutil.cc",
            "error.h"
           ],
    visibility = ["//visibility:public"],
    copts = ["-fpermissive"],
    deps = [
            "@glib2//:glib2",
           ],
)
