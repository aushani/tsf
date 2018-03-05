cc_library(
    name = "nodes",
    srcs = glob(['*.cpp']),
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
            "//library/viewer/nodes",
            "//library/kitti/nodes",
            "//library/kitti",
            "//library/flow",
            "@openscenegraph//:openscenegraph",
            "@eigen//:eigen",
            ],
)
