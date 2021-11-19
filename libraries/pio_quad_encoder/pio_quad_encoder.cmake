add_library(pio_quad_encoder INTERFACE)

target_sources(pio_quad_encoder INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/pio_quad_encoder.cpp)

target_include_directories(pio_quad_encoder INTERFACE ${CMAKE_CURRENT_LIST_DIR})