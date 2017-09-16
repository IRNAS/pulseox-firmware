enable_language(ASM)

set(SOURCES
  qfplib/qfplib.s
  clock.c
  adc.c
  battery.c
  uart.c
  measurement.c
  lcd.c
  gfx.c
  gui.c
  main.c
)

add_executable(firmware.elf ${SOURCES})
target_link_libraries(firmware.elf ${TARGET_PLATFORM_LIBRARY})
add_dependencies(firmware.elf libopencm3)
add_bin_from_elf(firmware.bin firmware.elf)
