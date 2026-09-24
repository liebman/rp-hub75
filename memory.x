MEMORY {
    /*
     * Pimoroni Interstate 75 W (RP2350): 4 MiB of external QSPI flash. The
     * MicroPython build on the same board gives 2 MiB to littlefs and about
     * 1.3 MiB to firmware, which is consistent with 4 MiB.
     *
     * Declaring less than the real size is harmless, so confirm with
     * `picotool info` (and tighten this) at the first flash.
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4096K
    /*
     * RAM consists of 8 banks, SRAM0-SRAM7, with a striped mapping. This is
     * usually good for performance, as it distributes load on those banks
     * evenly.
     */
    RAM : ORIGIN = 0x20000000, LENGTH = 512K
    /*
     * RAM banks 8 and 9 use a direct mapping. They can be used to have
     * memory areas dedicated for some specific job, improving predictability
     * of access times.
     */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    /* ### Boot ROM info
     *
     * Goes after .vector_table, to keep it in the first 4K of flash where the
     * Boot ROM (and picotool) can find it.
     */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
        /* .text starts after this block, so keep its size 8-byte aligned */
        . = ALIGN(8);
    } > FLASH

} INSERT AFTER .vector_table;

/* move .text to start /after/ the boot info */
_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    /* ### Picotool 'Binary Info' Entries
     *
     * Picotool looks through this block (as we have pointers to it in our
     * header) to find interesting information, so `picotool info` can describe
     * the binary.
     */
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    /* ### Boot ROM extra info
     *
     * Goes after everything in our program, so it can contain a signature.
     */
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
        __flash_binary_end = .;
    } > FLASH

} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
