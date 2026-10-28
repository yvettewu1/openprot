// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0

//! Standalone SRAM startup, IRQ vectors and plain-text UART12 logging.
#![no_std]

pub mod i3c_setup;

use core::arch::global_asm;
use core::fmt::{self, Write};
use core::panic::PanicInfo;
// Supplies the driver's critical-section implementation without a kernel.
use cortex_m as _;

global_asm!(
    r#"
    .syntax unified
    .thumb

    .section .vector_table,"a",%progbits
    .balign 2048
    .word __stack_top
    .word Reset
    .rept 110
    .word DefaultHandler
    .endr
    .word interrupt_handler_96
    .word interrupt_handler_97
    .rept 398
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
    // Mask and clear inherited NVIC sources before enabling CPU interrupts.
    ldr r1, =0xe000e180
    ldr r2, =0xe000e280
    movs r3, #16
    mvns r0, r0
.Lclear_nvic:
    str r0, [r1], #4
    str r0, [r2], #4
    subs r3, r3, #1
    bne .Lclear_nvic
    movs r0, #0
    msr basepri, r0
    msr faultmask, r0
    dsb
    isb
    cpsie i
    bl test_main
    b DefaultHandler
    .ltorg

    .section .text.DefaultHandler,"ax",%progbits
    .global DefaultHandler
    .type DefaultHandler,%function
    .thumb_func
DefaultHandler:
    b DefaultHandler

    .weak interrupt_handler_96
    .thumb_set interrupt_handler_96, DefaultHandler
    .weak interrupt_handler_97
    .thumb_set interrupt_handler_97, DefaultHandler
"#
);

// AST1080 UART12/BMC-UART; the PAC names this instance Uart5.
// The loader must configure UART clocks, pinmux, baud and line format.
pub fn write_all(buf: &[u8]) {
    // SAFETY: Tests own this UART; no ISR logs or accesses the console.
    let uart = unsafe { &*ast1080_pac::Uart5::ptr() };
    uart.uartlcr().modify(|_, w| w.dlab().clear_bit());
    uart.uartier().write(|w| unsafe { w.bits(0) });
    for &byte in buf {
        while !uart.uartlsr().read().thre().bit() {}
        uart.uartthr().write(|w| unsafe { w.bits(u32::from(byte)) });
    }
}

struct Console;
impl Write for Console {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        write_all(text.as_bytes());
        Ok(())
    }
}

pub fn log(level: &str, args: fmt::Arguments<'_>) {
    let _ = write!(Console, "[{}] {}\r\n", level, args);
}

#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => { $crate::log("INF", core::format_args!($($arg)*)) };
}

#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => { $crate::log("ERR", core::format_args!($($arg)*)) };
}

#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    cortex_m::interrupt::disable();
    log("PANIC", format_args!("{}", info));
    write_all(b"TEST_RESULT:FAIL\r\n");
    loop {
        core::hint::spin_loop();
    }
}
