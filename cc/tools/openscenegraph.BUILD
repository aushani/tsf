cc_library(
    name = "openscenegraph",
    srcs = glob(["lib64/libosg*.so"]),
    hdrs = glob(["include/**"]),
    visibility = ["//visibility:public"],
    deps = [
            #":osgPlugins",
            "@qt4//:qt4_core",
            "@qt4//:qt4_core_prefixstripped",
            "@qt4//:qt4_gui",
            "@qt4//:qt4_gui_prefixstripped",
            "@qt4//:qt4_opengl",
            "@qt4//:qt4_opengl_prefixstripped"
           ],
    strip_include_prefix = "include",
)
