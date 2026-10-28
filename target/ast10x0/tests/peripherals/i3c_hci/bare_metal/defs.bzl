# Licensed under the Apache-2.0 license
# SPDX-License-Identifier: Apache-2.0
"""Kernel-free I3C HCI firmware: ELF plus a raw SRAM binary."""

load("@pigweed//pw_build:binary_tools.bzl", "pw_elf_to_bin")
load("@rules_rust//rust:defs.bzl", "rust_binary")
load("//target/ast10x0:defs.bzl", "TARGET_COMPATIBLE_WITH")

def bare_metal_image(name, crate_features = []):
    """Build a test with standalone startup; Pigweed only supplies objcopy tooling."""
    linker = "//target/ast10x0/tests/peripherals/i3c_hci/bare_metal:runtime.ld"
    rust_binary(
        name = name + ".elf",
        srcs = ["target.rs"],
        crate_name = name,
        edition = "2024",
        crate_features = crate_features,
        compile_data = [linker],
        rustc_flags = [
            "-Cpanic=abort",
            "-Copt-level=s",
            "-Clink-arg=-T$(location " + linker + ")",
        ],
        target_compatible_with = TARGET_COMPATIBLE_WITH,
        deps = [
            "//target/ast10x0/tests/peripherals/i3c_hci/bare_metal:runtime",
            "//target/ast10x0/peripherals:i3c_hci",
            "//target/ast10x0/peripherals:scu_pinctrl",
            "@ast1080_pac",
            "@rust_crates//:cortex-m",
        ],
    )
    pw_elf_to_bin(
        name = name,
        bin_out = name + ".bin",
        elf_input = ":" + name + ".elf",
        target_compatible_with = TARGET_COMPATIBLE_WITH,
    )
