set(OUTPUT_NAME swc-pico-forage_feeder)

add_executable(
  ${OUTPUT_NAME}
  ForageFeeder.cpp
)

FILE(GLOB FreeRTOS_src ../freeRTOS/*.c)

add_library( FreeRTOS STATIC
	${FreeRTOS_src}
	../freeRTOS/portable/GCC/ARM_CM0/port.c
	../freeRTOS/portable/MemMang/heap_4.c
)

target_include_directories(FreeRTOS PUBLIC
    ../freeRTOS/include
    ../freeRTOS/config
    ../freeRTOS/portable/GCC/ARM_CM0
    )


# Pull in pico libraries that we need
target_link_libraries(${OUTPUT_NAME}
  FreeRTOS
  breakout_colourlcd240x240
  pico_stdlib
  pico_multicore
  hardware_pwm
  )

# create map/bin/hex file etc.
pico_add_extra_outputs(${OUTPUT_NAME})
