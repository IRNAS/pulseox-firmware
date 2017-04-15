MEMORY
{
  rom (rx) : ORIGIN = 0x08000000, LENGTH = 16K
  ram (xrw) : ORIGIN = 0x20000000, LENGTH = 4K
}

INCLUDE libopencm3_stm32f0.ld

