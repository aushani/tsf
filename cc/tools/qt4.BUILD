# osg isn't consistent.... nead both
cc_library(
    name = "qt4_core_prefixstripped",
    hdrs = glob(["QtCore/**"]),
    includes = ["."],
    linkopts = [
        "-lQtCore",
    ],
    visibility = ["//visibility:public"],
    strip_include_prefix = "QtCore",
)

cc_library(
    name = "qt4_core",
    hdrs = glob(["QtCore/**"]),
    includes = ["."],
    linkopts = [
        "-lQtCore",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "qt4_gui_prefixstripped",
    hdrs = glob(["QtGui/**"]),
    includes = ["."],
    deps = [":qt4_core"],
    linkopts = [
        "-lQtGui",
    ],
    visibility = ["//visibility:public"],
    strip_include_prefix = "QtGui",
)

cc_library(
    name = "qt4_gui",
    hdrs = glob(["QtGui/**"]),
    includes = ["."],
    deps = [":qt4_core"],
    linkopts = [
        "-lQtGui",
    ],
    visibility = ["//visibility:public"],
)

# osg isn't consistent.... nead both
cc_library(
    name = "qt4_opengl_prefixstripped",
    hdrs = glob(["QtOpenGL/**"]),
    includes = ["."],
    deps = [":qt4_core", ":qt4_gui"],
    linkopts = [
        "-lQtOpenGL",
    ],
    visibility = ["//visibility:public"],
    strip_include_prefix = "QtOpenGL",
)

cc_library(
    name = "qt4_opengl",
    hdrs = glob(["QtOpenGL/**"]),
    includes = ["."],
    deps = [":qt4_core", ":qt4_gui"],
    linkopts = [
        "-lQtOpenGL",
    ],
    visibility = ["//visibility:public"],
)
