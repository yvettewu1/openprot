# Licensed under the Apache-2.0 license
# SPDX-License-Identifier: Apache-2.0

"""Module extension to fetch the AST1080 PAC."""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def _ast1080_pac_impl(module_ctx):
    git_repository(
        name = "ast1080_pac",
        remote = "https://github.com/AspeedTech-BMC/ast1080-pac.git",
        commit = "a6ec42cd62baf017a555592854e18bc670d44189",
        build_file = "@@//third_party:ast1080_pac.BUILD.bazel",
    )
    return module_ctx.extension_metadata(
        reproducible = True,
        root_module_direct_deps = ["ast1080_pac"],
        root_module_direct_dev_deps = [],
    )

ast1080_pac_ext = module_extension(
    implementation = _ast1080_pac_impl,
)
