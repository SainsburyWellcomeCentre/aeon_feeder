add_library(pid INTERFACE)

target_sources(pid INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/pid.cpp)

target_include_directories(pid INTERFACE ${CMAKE_CURRENT_LIST_DIR})