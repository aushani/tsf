load("//tools/build_rules:qt.bzl", "qt4_cc_library")

qt4_cc_library(
    name = "qt-lib",
    src = "action_button.cpp",
    hdr = "action_button.hpp",
    normal_hdrs = [
                  ],
    linkopts = ["-lpthread"],
    deps = [
            "@qt4//:qt4_core",
            "@qt4//:qt4_gui",
            "@qt4//:qt4_opengl",
            "@qt4//:qt4_core_prefixstripped",
            "@qt4//:qt4_gui_prefixstripped",
            "@qt4//:qt4_opengl_prefixstripped",
           ],
)

cc_library(
    name = "qt",
    srcs = glob(['*.cpp']),
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
            "qt-lib",
            "@openscenegraph//:openscenegraph",
            "@qt4//:qt4_core",
            "@qt4//:qt4_gui",
            "@qt4//:qt4_opengl",
            ],
)
