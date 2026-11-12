/* Licensed under the Apache-2.0 license
 * SPDX-License-Identifier: Apache-2.0
 */
ENTRY(Reset)

/* Retain the existing target's SRAM-at-zero mapping. */
MEMORY
{
    SRAM (rwx) : ORIGIN = 0x00000000, LENGTH = 640K
    DMA_NC (rw) : ORIGIN = 0x000A0000, LENGTH = 128K
}
__stack_top = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .vector_table ORIGIN(SRAM) :
    {
        KEEP(*(.vector_table))
    } > SRAM

    .text 0x00000800 :
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

    /* Reserved CPU-address window, not inherently non-cacheable on G2.
     * prepare_dma_memory() disables cacheability for its 512 KiB SCU area.
     * NOLOAD memory is initialized explicitly before ring enable. */
    .dma_nc (NOLOAD) : ALIGN(64)
    {
        __dma_nc_start = .;
        KEEP(*(.dma_nc))
        . = ALIGN(64);
        __dma_nc_end = .;
    } > DMA_NC
    /DISCARD/ : { *(.eh_frame*) }
}
ASSERT(SIZEOF(.vector_table) <= 0x800, "Vector table overlaps code")
ASSERT(__bss_end <= __stack_top - 16384, "Reserve at least 16 KiB for the stack")
