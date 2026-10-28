// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Bare-metal UART12/BMC-UART hello. No kernel, allocator, PAC, or peripheral library.
//! The loader must map the binary at address zero and configure UART12/BMC-UART clocks,
//! pinmux and baud rate. The UART divisor and line format are preserved.

#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

global_asm!(
    r#"
    .syntax unified
    .thumb

    .section .vector_table,"a",%progbits
    .balign 1024
    .word __stack_top
    .word Reset
    .rept 254
    .word DefaultHandler
    .endr

    .section .text.Reset,"ax",%progbits
    .global Reset
    .type Reset,%function
    .thumb_func
Reset:
    cpsid i
    ldr r0, =__stack_top
    msr msp, r0
    movs r0, #0
    msr control, r0
    isb

    // Disable an inherited SysTick and point VTOR at our vector table.
    ldr r1, =0xe000e010
    str r0, [r1]
    ldr r1, =0xe000ed08
    str r0, [r1]
    dsb
    isb

    ldr r1, =__bss_start
    ldr r2, =__bss_end
.Lzero_bss:
    cmp r1, r2
    bhs .Lstart
    str r0, [r1], #4
    b .Lzero_bss
.Lstart:
    bl hello_main
    b DefaultHandler
    .ltorg

    .section .text.DefaultHandler,"ax",%progbits
    .global DefaultHandler
    .type DefaultHandler,%function
    .thumb_func
DefaultHandler:
    b DefaultHandler
"#
);

// AST1080 memory map: BMC-UART occupies 0x74C33C00..0x74C33CFF.
// The ast1080_pac calls this register block Uart5 (not hardware UART5).
const BMC_UART_BASE: usize = 0x74c3_3c00;
const UART_THR: *mut u32 = BMC_UART_BASE as *mut u32;
const UART_IER: *mut u32 = (BMC_UART_BASE + 0x04) as *mut u32;
const UART_LCR: *mut u32 = (BMC_UART_BASE + 0x0c) as *mut u32;
const UART_LSR: *const u32 = (BMC_UART_BASE + 0x14) as *const u32;

#[unsafe(no_mangle)]
pub extern "C" fn hello_main() -> ! {
    // SAFETY: These are UART12/BMC-UART MMIO registers. This image has exclusive access;
    // interrupts are disabled and UART clocks/pins are supplied by the loader.
    unsafe {
        // Clear DLAB so offset zero selects THR rather than the baud divisor.
        write_volatile(UART_LCR, read_volatile(UART_LCR) & !0x80);
        write_volatile(UART_IER, 0);
        for &byte in b"Hello AST1080!\r\n" {
            while read_volatile(UART_LSR) & (1 << 5) == 0 {}
            write_volatile(UART_THR, u32::from(byte));
        }
        // Wait for both the FIFO and shift register to drain.
        while read_volatile(UART_LSR) & (1 << 6) == 0 {}
    }
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
