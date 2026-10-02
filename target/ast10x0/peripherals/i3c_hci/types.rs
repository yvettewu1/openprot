// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Core AST1080 MIPI I3C HCI types.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum InitMode {
    Primary = 0,
    Secondary = 1,
    Target = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TargetRole {
    pub secondary: bool,
    pub target: bool,
}
