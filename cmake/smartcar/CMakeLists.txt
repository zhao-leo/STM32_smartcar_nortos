cmake_minimum_required(VERSION 3.22)

project(smartcar)
add_library(smartcar INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(smartcar INTERFACE 
	USE_HAL_DRIVER
	STM32F103xB
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(smartcar INTERFACE
    ../../Utils/Inc
)

target_sources(smartcar INTERFACE
    ../../Utils/Src/ssd1306.c
    ../../Utils/Src/ssd1306_fonts.c

)

target_link_directories(smartcar INTERFACE)

target_link_libraries(smartcar INTERFACE)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


