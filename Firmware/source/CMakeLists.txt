set(OUTPUT_NAME swc-pico-forage_feeder)

add_executable(
  ${OUTPUT_NAME}
  ForageFeeder.cpp
)

pico_generate_pio_header(${OUTPUT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio_quad_encoder.pio)

if(NOT DEFINED FREERTOS_SOURCE_DIR)
  set(FREERTOS_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../freeRTOS)
endif()

FILE(GLOB FreeRTOS_src ${FREERTOS_SOURCE_DIR}/*.c)

add_library(FreeRTOS STATIC
    ${FreeRTOS_src}
    ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM0/port.c
    ${FREERTOS_SOURCE_DIR}/portable/MemMang/heap_4.c
)

target_include_directories(FreeRTOS PUBLIC
    ${FREERTOS_SOURCE_DIR}/include
    ${FREERTOS_SOURCE_DIR}/config
    ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM0
)


# Pull in pico libraries that we need
target_link_libraries(${OUTPUT_NAME}
  FreeRTOS
  # pico_display
  pico_unique_id
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  pid
  pio_quad_encoder
  )

# create map/bin/hex file etc.
pico_add_extra_outputs(${OUTPUT_NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)
