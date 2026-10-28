/* Licensed under the Apache-2.0 license
 * SPDX-License-Identifier: Apache-2.0
 */
ENTRY(Reset)

/* Retain the existing target's SRAM-at-zero mapping. */
MEMORY
{
    SRAM (rwx) : ORIGIN = 0x00000000, LENGTH = 640K
}
__stack_top = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .vector_table ORIGIN(SRAM) :
    {
        KEEP(*(.vector_table))
    } > SRAM

    .text 0x00000500 :
    {
        KEEP(*(.text.Reset))
        *(.text .text.*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > SRAM

    .ARM.exidx : { *(.ARM.exidx*) } > SRAM
    .data :
    {
        . = ALIGN(4);
        *(.data .data.*)
        . = ALIGN(4);
    } > SRAM
    /* The complete binary is loaded into SRAM: .data is already in place. */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss .bss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > SRAM

    /DISCARD/ : { *(.eh_frame*) }
}
ASSERT(SIZEOF(.vector_table) <= 0x500, "Vector table overlaps code")
ASSERT(__bss_end <= __stack_top - 4096, "Reserve at least 4 KiB for the stack")
