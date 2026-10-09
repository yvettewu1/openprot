// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! MIPI I3C HCI and ASPEED AST1080 register constants.

pub const MAX_BUSES: usize = 8;

pub const HC_CONTROL_BUS_ENABLE: u32 = 1 << 31;
pub const HC_CONTROL_RESUME: u32 = 1 << 30;
pub const HC_CONTROL_ABORT: u32 = 1 << 29;
pub const HC_CONTROL_HALT_ON_CMD_TIMEOUT: u32 = 1 << 12;
pub const HC_CONTROL_HOT_JOIN_CTRL: u32 = 1 << 8;
pub const HC_CONTROL_I2C_TARGET_PRESENT: u32 = 1 << 7;
pub const HC_CONTROL_PIO_MODE: u32 = 1 << 6;
pub const HC_CONTROL_DATA_BIG_ENDIAN: u32 = 1 << 4;
pub const HC_CONTROL_IBA_INCLUDE: u32 = 1;

pub const MASTER_DYNAMIC_ADDR_VALID: u32 = 1 << 31;
pub const MASTER_DYNAMIC_ADDR_SHIFT: u32 = 16;
pub const MASTER_DYNAMIC_ADDR_MASK: u32 = 0x7f << MASTER_DYNAMIC_ADDR_SHIFT;

pub const HC_CAP_SG_DC_EN: u32 = 1 << 30;
pub const HC_CAP_SG_IBI_EN: u32 = 1 << 29;
pub const HC_CAP_SG_CR_EN: u32 = 1 << 28;
pub const HC_CAP_DIRECT_COMMANDS_EN: u32 = 1 << 18;
pub const HC_CAP_AUTO_COMMAND: u32 = 1 << 3;

pub const BUS_RESET: u32 = 1 << 31;
pub const IBI_QUEUE_RST: u32 = 1 << 5;
pub const RX_FIFO_RST: u32 = 1 << 4;
pub const TX_FIFO_RST: u32 = 1 << 3;
pub const RESP_QUEUE_RST: u32 = 1 << 2;
pub const CMD_QUEUE_RST: u32 = 1 << 1;
pub const SOFT_RST: u32 = 1;

pub const STATE_CURRENT_MASTER: u32 = 1 << 2;

pub const INTR_HC_CMD_SEQ_UFLOW_STAT: u32 = 1 << 12;
pub const INTR_HC_SEQ_CANCEL: u32 = 1 << 11;
pub const INTR_HC_INTERNAL_ERR: u32 = 1 << 10;

pub const ASPEED_I3C_CTRL_STOP_QUEUE_PT: u32 = 1 << 31;
pub const ASPEED_I3C_CTRL_INIT: u32 = 1 << 4;
pub const ASPEED_I3C_CTRL_INIT_MODE_MASK: u32 = 0x3;

pub const ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_VALID: u32 = 1 << 23;
pub const ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_SHIFT: u32 = 16;
pub const ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_MASK: u32 =
    0x7f << ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_SHIFT;
pub const ASPEED_I3C_STS_MODE_PURE_SLV: u32 = 1 << 8;
pub const ASPEED_I3C_STS_MODE_SECONDARY_SLV_TO_MST: u32 = 1 << 7;
pub const ASPEED_I3C_STS_MODE_SECONDARY_MST_TO_SLV: u32 = 1 << 6;
pub const ASPEED_I3C_STS_MODE_SECONDARY_SLV: u32 = 1 << 5;
pub const ASPEED_I3C_STS_MODE_SECONDARY_MST: u32 = 1 << 4;
pub const ASPEED_I3C_STS_MODE_PRIMARY_SLV_TO_MST: u32 = 1 << 3;
pub const ASPEED_I3C_STS_MODE_PRIMARY_MST_TO_SLV: u32 = 1 << 2;
pub const ASPEED_I3C_STS_MODE_PRIMARY_SLV: u32 = 1 << 1;
pub const ASPEED_I3C_STS_MODE_PRIMARY_MST: u32 = 1;

pub const ASPEED_I3C_IBI_TERMINATE_EN: u32 = 1 << 16;
pub const ASPEED_I3C_IBI_TERMINATE_LEN_MASK: u32 = 0xffff;

pub const ASPEED_I3C_SLV_CHAR_CTRL_DCR_SHIFT: u32 = 16;
pub const ASPEED_I3C_SLV_CHAR_CTRL_BCR_SHIFT: u32 = 8;
pub const ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_EN: u32 = 1 << 7;
pub const ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_MASK: u32 = 0x7f;

pub const ASPEED_I3C_SLV_CAP_CTRL_ACCEPT_CR: u32 = 1 << 16;
pub const ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ: u32 = 1 << 10;
pub const ASPEED_I3C_SLV_CAP_CTRL_MR_REQ: u32 = 1 << 9;
pub const ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ: u32 = 1 << 8;
pub const ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT: u32 = 1 << 6;
pub const ASPEED_I3C_SLV_CAP_CTRL_MR_WAIT: u32 = 1 << 5;
pub const ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT: u32 = 1 << 4;

pub const ASPEED_I3C_SLV_STS1_HJ_EN: u32 = 1 << 6;
pub const ASPEED_I3C_SLV_STS1_CR_EN: u32 = 1 << 5;
pub const ASPEED_I3C_SLV_STS1_IBI_EN: u32 = 1 << 4;
pub const ASPEED_I3C_SLV_STS1_HJ_DONE: u32 = 1 << 2;
pub const ASPEED_I3C_SLV_STS1_CR_DONE: u32 = 1 << 1;
pub const ASPEED_I3C_SLV_STS1_IBI_DONE: u32 = 1;

pub const ASPEED_INTR_SUM_INHOUSE: u32 = 1 << 3;
pub const ASPEED_INTR_SUM_RHS: u32 = 1 << 2;
pub const ASPEED_INTR_SUM_PIO: u32 = 1 << 1;
pub const ASPEED_INTR_SUM_CAP: u32 = 1;

pub const MIPI_I3C_HCI_RESET_TIMEOUT_US: u32 = 10_000;
pub const MIPI_I3C_HCI_CORE_IRQS: u32 =
    INTR_HC_CMD_SEQ_UFLOW_STAT | INTR_HC_SEQ_CANCEL | INTR_HC_INTERNAL_ERR;

pub const HC_CAP_CMD_SIZE_SHIFT: u32 = 20;
pub const HC_CAP_CMD_SIZE_MASK: u32 = 0x3 << HC_CAP_CMD_SIZE_SHIFT;

pub const DAT_ENTRY_SIZE_SHIFT: u32 = 28;
pub const DAT_ENTRY_SIZE_MASK: u32 = 0xf << DAT_ENTRY_SIZE_SHIFT;
pub const DAT_TABLE_SIZE_SHIFT: u32 = 12;
pub const DAT_TABLE_SIZE_MASK: u32 = 0x7f << DAT_TABLE_SIZE_SHIFT;
pub const DAT_TABLE_OFFSET_MASK: u32 = 0x0fff;

pub const DCT_ENTRY_SIZE_SHIFT: u32 = 28;
pub const DCT_ENTRY_SIZE_MASK: u32 = 0xf << DCT_ENTRY_SIZE_SHIFT;
pub const DCT_TABLE_INDEX_SHIFT: u32 = 19;
pub const DCT_TABLE_INDEX_MASK: u32 = 0x1f << DCT_TABLE_INDEX_SHIFT;
pub const DCT_TABLE_SIZE_SHIFT: u32 = 12;
pub const DCT_TABLE_SIZE_MASK: u32 = 0x7f << DCT_TABLE_SIZE_SHIFT;
pub const DCT_TABLE_OFFSET_MASK: u32 = 0x0fff;

pub const RING_HEADERS_OFFSET_MASK: u32 = 0xffff;
pub const PIO_REGS_OFFSET_MASK: u32 = 0xffff;
pub const EXT_CAPS_OFFSET_MASK: u32 = 0xffff;
