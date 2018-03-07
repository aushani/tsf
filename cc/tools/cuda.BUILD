cc_library(
    name = "cuda",
    hdrs = glob(["include/**"]),
    data = ["bin/nvcc"],
    includes = [
        "include",
    ],
    linkopts = [
        "-Wl,-rpath=/usr/local/cuda/lib64",
        "-L/usr/local/cuda/lib64",
        #"-lnppc",
        #"-lnppi",
        #"-lnpps",
        "-lcufft",
        "-lcudart",
        "-lm",
    ],
    visibility = ["//visibility:public"],
)
