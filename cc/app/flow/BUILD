load("//tools/build_rules:qt.bzl", "qt4_cc_library")

qt4_cc_library(
    name = "viewer-lib",
    src = "viewer.cpp",
    hdr = "viewer.hpp",
    normal_hdrs = [
                    "flow_app.hpp",
                  ],
    linkopts = ["-lpthread"],
    deps = [
            "@qt4//:qt4_core",
            "@qt4//:qt4_gui",
            "@qt4//:qt4_opengl",
            "@qt4//:qt4_core_prefixstripped",
            "@qt4//:qt4_gui_prefixstripped",
            "@qt4//:qt4_opengl_prefixstripped",
            "//library/viewer/util",
            "//library/viewer/ga",
            "//library/viewer/qt",
            "//library/viewer/nodes",
            "//library/kitti/nodes",
            "//library/kitti",
            "//library/flow",
            "//app/flow/nodes",
           ],
)

cc_binary(
    name = "viewer",
    srcs = glob(['*.cpp', '*.hpp']),
    visibility = ["//visibility:public"],
    deps = [
            ":viewer-lib",
            "//library/viewer/nodes",
            "//library/viewer/util",
            "//library/viewer/ga",
            "//library/viewer/qt",
            "//library/kitti/nodes",
            "//library/kitti",
            "//library/flow",
            "//library/util",
            "//library/gpu_util",
            "//app/flow/nodes",
            "//thirdparty/perls-common",
            "//thirdparty/perls-math",
            "@openscenegraph//:openscenegraph",
            "@eigen//:eigen",
            ],
)
