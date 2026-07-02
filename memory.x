/* Linker script for the STM32F103C8T6.
 *
 * The datasheet rates the C8 at 64K flash, but virtually all "blue pill" clones
 * carry the full 128K die (the CB part), which is what an unoptimized debug build
 * needs. If your specific chip is a genuine 64K part, revert FLASH to 64K (and the
 * probe-rs `--chip` back to STM32F103C8Tx in .cargo/config.toml), and build with
 * the default opt-level=1 profile instead of the opt-level=0 debug overrides. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM : ORIGIN = 0x20000000, LENGTH = 20K
}
