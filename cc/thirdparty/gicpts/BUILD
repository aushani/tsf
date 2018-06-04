cc_library(
    name = "gicpts",
    srcs = glob(["*.cpp", "*.h"], exclude=["test_gicp.cpp", "scan2ascii.cpp"]),
    visibility = ["//visibility:public"],
    linkopts = [ "-lm" ],
    deps = [
            "//thirdparty/ann",
            "@gsl//:gsl",
           ]
)

cc_binary(
    name = "test_gicpts",
    srcs = ["test_gicp.cpp"],
    visibility = ["//visibility:public"],
    deps = [
            "//thirdparty/ann",
            "//thirdparty/gicpts:gicpts",
            "@boost//:boost_programoptions",
            "@gsl//:gsl",
           ]
)

cc_binary(
    name = "scan2ascii",
    srcs = ["scan2ascii.cpp"],
    visibility = ["//visibility:public"],
    deps = [
            "//thirdparty/ann",
            "//thirdparty/gicpts:gicpts",
            "@boost//:boost_programoptions",
            "@gsl//:gsl",
           ]
)
