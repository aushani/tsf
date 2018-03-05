cc_library(
  name = "glib2_a",
  hdrs = glob([
              "include/glib-2.0/**/*.h",
              ]),
  linkopts = ["-lglib-2.0"],
  visibility = ["//visibility:public"],
  strip_include_prefix = "include/glib-2.0",
)

cc_library(
  name = "glib2_b",
  hdrs = glob([
              "lib/x86_64-linux-gnu/glib-2.0/include/**/*.h"
              ]),
  linkopts = ["-lglib-2.0"],
  visibility = ["//visibility:public"],
  strip_include_prefix = "lib/x86_64-linux-gnu/glib-2.0/include/",
)

cc_library(
  name = "glib2",
  visibility = ["//visibility:public"],
  deps = [ "glib2_a", "glib2_b"],
)
