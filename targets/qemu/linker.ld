/* linker.ld - Memory map for the TI Stellaris LM3S6965 (ARM Cortex-M3),
   the board QEMU emulates with -M lm3s6965evb. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);   /* main stack starts at the top of RAM */

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))              /* vector table must be at address 0 */
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        __init_array_start = .;
        KEEP(*(.init_array*))
        __init_array_end = .;
        . = ALIGN(4);
    } > FLASH

    .data :
    {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH
    _sidata = LOADADDR(.data);         /* where .data's initial values live in flash */

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
