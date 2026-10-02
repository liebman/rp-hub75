MEMORY {
    /*
     * The Raspberry Pi RP2040 (Pico / Interstate 75 W): 2 MiB of external QSPI
     * flash and 264 KiB of SRAM.
     *
     * The first 256 bytes hold the second-stage bootloader, which the ROM
     * executes from flash before anything else; the `rp2040-boot2` crate provides
     * its code as `BOOT2_FIRMWARE`, and the section below places it there.
     */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    /*
     * SRAM0-SRAM3 are striped and addressable as one 256 KiB block.
     */
    RAM : ORIGIN = 0x20000000, LENGTH = 256K
    /*
     * SRAM4 and SRAM5 use a direct mapping, for memory areas dedicated to one
     * job.
     */
    SRAM4 : ORIGIN = 0x20040000, LENGTH = 4K
    SRAM5 : ORIGIN = 0x20041000, LENGTH = 4K
}

/* The second-stage bootloader the application places here (the `BOOT2` static in
 * the examples): `rp2040-boot2` supplies the code, and this keeps the symbol even
 * though nothing calls it. */
EXTERN(BOOT2)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
