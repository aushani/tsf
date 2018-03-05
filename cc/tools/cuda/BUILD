package(default_visibility = ["//visibility:public"])

cc_toolchain_suite(
    name = "cuda",
    toolchains = {
        "local|compiler": ":cc-compiler-local",
        "darwin|compiler": ":cc-compiler-darwin",
    },
)

filegroup(
    name = "wrapper",
    srcs = ["crosstool_nvcc_wrapper"],
)

cc_toolchain(
    name = "cc-compiler-local",
    all_files = ":wrapper",
    compiler_files = ":wrapper",
    cpu = "local",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":wrapper",
    objcopy_files = ":empty",
    static_runtime_libs = [":empty"],
    strip_files = ":empty",
    supports_param_files = 0,
)

cc_toolchain(
    name = "cc-compiler-darwin",
    all_files = ":empty",
    compiler_files = ":empty",
    cpu = "darwin",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":empty",
    objcopy_files = ":empty",
    static_runtime_libs = [":empty"],
    strip_files = ":empty",
    supports_param_files = 0,
)

filegroup(
    name = "empty",
    srcs = [],
)

sh_binary(
    name = "protoc_wrapper",
    srcs = ["protoc_wrapper.sh"],
)
