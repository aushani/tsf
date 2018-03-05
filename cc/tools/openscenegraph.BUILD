# not portable, see https://stackoverflow.com/questions/34984290/building-opencv-code-using-bazel

cc_library(
    name = "openscenegraph",
    srcs = glob(["lib64/libosg*.so"]),
    hdrs = glob(["include/osg*/**"]),
    visibility = ["//visibility:public"],
    deps = [
            "@qt4//:qt4_core",
            "@qt4//:qt4_core_prefixstripped",
            "@qt4//:qt4_gui",
            "@qt4//:qt4_gui_prefixstripped",
            "@qt4//:qt4_opengl",
            "@qt4//:qt4_opengl_prefixstripped"
           ],
)
