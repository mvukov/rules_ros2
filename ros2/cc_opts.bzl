""" Defines commonly used C/C++ options.
"""

C_COPTS = ["-std=c11"]

# Export dynamic symbols for pluginlib/class_loader with LLVM toolchain.
CPP_LINKOPTS = select({
    "@rules_cc//cc/compiler:clang": ["-rdynamic"],
    "//conditions:default": [],
})
