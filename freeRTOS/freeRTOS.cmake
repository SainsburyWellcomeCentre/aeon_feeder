set(LIB_NAME FreeRTOS)

FILE(GLOB FreeRTOS_src *.c)

add_library(${LIB_NAME} INTERFACE
	${FreeRTOS_src}
	portable/GCC/ARM_CM0/port.c
	portable/MemMang/heap_4.c
)

target_include_directories(${LIB_NAME} INTERFACE
    include
    config
    portable/GCC/ARM_CM0
    )


target_link_libraries(${LIB_NAME} INTERFACE)
